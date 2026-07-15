#!/usr/bin/env python3
"""Decode Mazak SmoothAi EtherNet/IP Class-1 I/O from pcap/pcapng captures.

Usage:
  mazak_eip_decode.py FILE.pcapng                 # connection summary
  mazak_eip_decode.py FILE.pcapng --timeline      # bit-level change log
  mazak_eip_decode.py FILE.pcapng --dir TO        # only machine→us direction
  mazak_eip_decode.py FILE.pcapng --bytes 0,12,13 # restrict to these bytes
  mazak_eip_decode.py FILE.pcapng --no-heartbeat  # hide DO000/DI000 toggle noise

Live capture:
  tshark -i eth0 -f 'host 192.168.10.1' -w - \\
    | mazak_eip_decode.py - --timeline --no-heartbeat
"""

from __future__ import annotations

import argparse
import os
import socket
import struct
import sys
from dataclasses import dataclass

import dpkt

# ---------------------------------------------------------------------------
# Signal map
# ---------------------------------------------------------------------------

# T->O = machine→us (statuses / DO / SO)
# O->T = us→machine (commands / DI / SI)
#
# DO105 is a 4-bit service code at ctrl bits 4-7, shifting DO106+ up by 3.
# The control word occupies assembly bytes 12-13.

CTRL = 12 * 8  # bit offset of the control word


def _do_ctrl(sig: int) -> int:
    """Return the bit offset for DO signal *sig* (101-111) in the assembly."""
    if sig <= 104:
        return CTRL + (sig - 101)
    if sig == 105:
        return CTRL + 4
    return CTRL + 8 + (sig - 106)


def _di_ctrl(sig: int) -> int:
    """Return the bit offset for DI signal *sig* (101-111) in the assembly."""
    return CTRL + (sig - 101)


def _b(byte: int, bit: int = 0) -> int:
    """Bit offset = byte*8 + bit."""
    return byte * 8 + bit


# Each entry: bit_offset -> (short_name, description)
TO_SIGNALS: dict[int, tuple[str, str]] = {
    _b(0, 0): ("DO000", "comms-check (heartbeat out)"),
    _b(0, 1): ("DO001", "machine ready"),
    _b(0, 2): ("DO002", "robot stop request (normally ON; OFF=stop robot)"),
    _b(0, 3): ("DO003", "operating panel AT RETRACT position (normally ON)"),
    _b(0, 4): ("DO004", "machine alarm"),
    _b(0, 5): ("DO005", "(no allocation)"),
    _b(0, 6): ("DO006", "auto power shut-off request received"),
    _b(0, 7): ("DO007", "(std block bit7)"),
    _b(1, 0): ("DO008", "fixture1 clamp complete (is_engaged)"),
    _b(1, 1): ("DO009", "fixture1 unclamp complete (is_disengaged)"),
    _do_ctrl(101): ("DO101", "work-number search finished"),
    _do_ctrl(102): ("DO102", "cycle-start permission"),
    _do_ctrl(103): ("DO103", "machine operating (is_processing)"),
    _do_ctrl(104): ("DO104", "(ctrl bit3)"),
    _do_ctrl(106): ("DO106", "robot service request"),
    _do_ctrl(107): ("DO107", "side door open finished"),
    _do_ctrl(108): ("DO108", "side door close finished"),
    _do_ctrl(109): ("DO109", "robot access permitted"),
    _do_ctrl(110): ("DO110", "front door open finished"),
    _do_ctrl(111): ("DO111", "front door close finished"),
}

OT_SIGNALS: dict[int, tuple[str, str]] = {
    _b(0, 0): ("DI000", "comms-check (heartbeat in)"),
    _b(0, 1): ("DI001", "robot ready (ON=enable robot interface)"),
    _b(0, 2): ("DI002", "machine stop request (ON=run; OFF=stop spindle/axes/doors)"),
    _b(0, 3): ("DI003", "robot operating"),
    _b(0, 4): ("DI004", "robot alarm status"),
    _b(1, 0): ("DI008", "fixture1 clamp command (engage)"),
    _b(1, 1): ("DI009", "fixture1 unclamp command (disengage)"),
    _di_ctrl(101): ("DI101", "work-number search start"),
    _di_ctrl(102): ("DI102", "cycle start command"),
    _di_ctrl(106): ("DI106", "robot service finished"),
    _di_ctrl(107): ("DI107", "side door open command"),
    _di_ctrl(108): ("DI108", "side door close command"),
    _di_ctrl(109): ("DI109", "robot clear"),
    _di_ctrl(110): ("DI110", "front door open command"),
    _di_ctrl(111): ("DI111", "front door close command"),
}

# Named sub-ranges
DO105_BITS = 0xF0   # ctrl byte[12] bits 4-7 = 4-bit service code
HEARTBEAT_BIT = _b(0, 0)

WORK_NUMBER_OFFSET = 44
WORK_NUMBER_LEN = 32
WORK_NUMBER_BYTES = frozenset(
    range(WORK_NUMBER_OFFSET, WORK_NUMBER_OFFSET + WORK_NUMBER_LEN)
)

ASSEMBLY_LEN = 110
EIP_IO_PORT = 2222
EIP_TCP_PORT = 44818
FWD_OPEN_SERVICES = (0x54, 0x5B)


# ---------------------------------------------------------------------------
# Data types
# ---------------------------------------------------------------------------

@dataclass(frozen=True, slots=True)
class Frame:
    """One decoded implicit I/O frame (assembly data, direction, metadata)."""
    num: int
    t: float              # seconds from first decoded frame
    direction: str        # "TO" (machine→us) or "OT" (us→machine)
    cip_seq: int          # CIP sequence counter from the connected-data item
    data: bytes           # the 110-byte assembly (seq count + header stripped)
    run_idle: int | None = None  # O→T 32-bit run/idle header, None for T→O


@dataclass(frozen=True, slots=True)
class ForwardOpen:
    """Connection profile parsed from a ForwardOpen request."""
    machine_ip: str
    o2t_size: int
    t2o_size: int
    o2t_conn_id: int
    t2o_conn_id: int
    rpi_o2t_us: int
    rpi_t2o_us: int

    @property
    def assembly_len(self) -> int:
        return min(self.o2t_size, self.t2o_size) - 2  # 2B CIP sequence count

    def header_len(self, direction: str) -> int:
        size = self.o2t_size if direction == "OT" else self.t2o_size
        return (size - 2) - self.assembly_len


# ---------------------------------------------------------------------------
# Packet / stream helpers
# ---------------------------------------------------------------------------

class _Prepend:
    """Wrap a non-seekable stream with leading bytes already read."""

    def __init__(self, fh, head: bytes):
        self._fh = fh
        self._buf = head
        self._pos = 0

    def read(self, n: int = -1) -> bytes:
        if n is None or n < 0:
            out = self._buf + self._fh.read()
            self._buf = b""
        else:
            out = b""
            if self._buf:
                out = self._buf[:n]
                self._buf = self._buf[n:]
                n -= len(out)
            if n > 0:
                out += self._fh.read(n)
        self._pos += len(out)
        return out

    def tell(self) -> int:
        return self._pos


def _packets(source: str):
    """Yield (frame_no, ts, ethernet_frame) for every packet in a pcap file.

    ``source`` is a path or ``"-"`` for stdin (live capture from tshark etc.).
    """
    if source == "-":
        raw = sys.stdin.buffer
        head = raw.read(4)
        fh: object = _Prepend(raw, head)
        magic = head
    else:
        fh = open(source, "rb")
        magic = fh.read(4)
        fh.seek(0)
    try:
        reader = (
            dpkt.pcapng.Reader(fh)
            if magic == b"\x0a\x0d\x0d\x0a"
            else dpkt.pcap.Reader(fh)
        )
        for n, (ts, buf) in enumerate(reader, start=1):
            try:
                eth = dpkt.ethernet.Ethernet(buf)
            except Exception:
                continue
            yield n, ts, eth
    finally:
        if not isinstance(fh, _Prepend):
            fh.close()


# ---------------------------------------------------------------------------
# ForwardOpen parser
# ---------------------------------------------------------------------------

def parse_forward_open(ip) -> ForwardOpen | None:
    """Parse a ForwardOpen request out of a TCP/IP packet, or return None."""
    tcp = ip.data
    if not isinstance(tcp, dpkt.tcp.TCP):
        return None
    d = bytes(tcp.data)
    if len(d) < 24 + 6:
        return None
    cmd = struct.unpack_from("<H", d, 0)[0]
    if cmd != 0x6F:  # SendRRData
        return None
    cpf = d[24 + 6:]  # skip ENIP header + interface handle + timeout
    try:
        off = 2                           # item count
        off += 4                          # null address item
        _dtype, dlen = struct.unpack_from("<HH", cpf, off)
        off += 4
        cip = cpf[off:off + dlen]
        if not cip or cip[0] not in FWD_OPEN_SERVICES:
            return None
        large = cip[0] == 0x5B
        psize = cip[1]
        p = 2 + psize * 2
        body = cip[p:]
        o2t_cid, t2o_cid = struct.unpack_from("<II", body, 2)
        q = 2 + 4 + 4 + 2 + 2 + 4 + 1 + 3
        rpi_o2t = struct.unpack_from("<I", body, q)[0]
        q += 4
        if large:
            o2t_np = struct.unpack_from("<I", body, q)[0]
            q += 4
            o2t_size = o2t_np & 0xFFFF
        else:
            o2t_np = struct.unpack_from("<H", body, q)[0]
            q += 2
            o2t_size = o2t_np & 0x01FF
        rpi_t2o = struct.unpack_from("<I", body, q)[0]
        q += 4
        if large:
            t2o_np = struct.unpack_from("<I", body, q)[0]
            t2o_size = t2o_np & 0xFFFF
        else:
            t2o_np = struct.unpack_from("<H", body, q)[0]
            t2o_size = t2o_np & 0x01FF
    except struct.error:
        return None
    return ForwardOpen(
        machine_ip=socket.inet_ntoa(ip.dst),
        o2t_size=o2t_size,
        t2o_size=t2o_size,
        o2t_conn_id=o2t_cid,
        t2o_conn_id=t2o_cid,
        rpi_o2t_us=rpi_o2t,
        rpi_t2o_us=rpi_t2o,
    )


# ---------------------------------------------------------------------------
# I/O frame parser
# ---------------------------------------------------------------------------

def parse_io(payload: bytes):
    """Return (conn_id, cip_seq, body) from a UDP I/O CPF payload, or None.

    *body* is the connected-data item after the 2-byte CIP sequence count
    (still includes the run/idle header if present).
    """
    if len(payload) < 6:
        return None
    item_count = struct.unpack_from("<H", payload, 0)[0]
    off = 2
    conn_id: int | None = None
    conn_data: bytes | None = None
    for _ in range(item_count):
        if off + 4 > len(payload):
            break
        type_id, length = struct.unpack_from("<HH", payload, off)
        off += 4
        item = payload[off:off + length]
        off += length
        if type_id == 0x8002 and len(item) >= 4:
            conn_id = struct.unpack_from("<I", item, 0)[0]
        elif type_id == 0x00B1:
            conn_data = item
    if conn_data is None or len(conn_data) < 2:
        return None
    cip_seq = struct.unpack_from("<H", conn_data, 0)[0]
    return conn_id, cip_seq, conn_data[2:]


# ---------------------------------------------------------------------------
# Decoder
# ---------------------------------------------------------------------------

class MazakDecoder:
    """Streaming decoder for Mazak EtherNet/IP implicit I/O.

    Learns the connection profile from ForwardOpen (TCP) packets inline,
    then classifies and slices every I/O (UDP) packet.  Works identically
    on files and live streams.
    """

    def __init__(self, machine_ip: str | None = None) -> None:
        self.fo: ForwardOpen | None = None
        self._machine_ip_override = machine_ip
        self.machine_ip: str | None = machine_ip

    def _resolve_direction(self, src: str, body: bytes) -> str:
        body_total = len(body) + 2  # add back the 2B seq count
        if self._machine_ip_override:
            return "TO" if src == self._machine_ip_override else "OT"
        if self.fo and self.fo.o2t_size != self.fo.t2o_size:
            if body_total == self.fo.o2t_size:
                return "OT"
            if body_total == self.fo.t2o_size:
                return "TO"
        if self.machine_ip:
            return "TO" if src == self.machine_ip else "OT"
        # Before ForwardOpen: O->T has the run/idle header so it is longer.
        return "OT" if len(body) > ASSEMBLY_LEN else "TO"

    def _slice(self, direction: str, body: bytes) -> tuple[bytes, int | None]:
        if self.fo:
            alen = self.fo.assembly_len
            hdr = max(self.fo.header_len(direction), 0)
        else:
            alen = ASSEMBLY_LEN
            hdr = max(len(body) - ASSEMBLY_LEN, 0)
        run_idle: int | None = (
            struct.unpack_from("<I", body, 0)[0] if hdr >= 4 else None
        )
        return body[hdr:hdr + alen], run_idle

    def frames(self, source: str):
        """Yield ``Frame`` objects for every I/O packet in *source*."""
        t0: float | None = None
        for n, ts, eth in _packets(source):
            ip = eth.data
            if not isinstance(ip, dpkt.ip.IP):
                continue
            l4 = ip.data
            if isinstance(l4, dpkt.tcp.TCP):
                if self.fo is None and EIP_TCP_PORT in (l4.dport, l4.sport):
                    fo = parse_forward_open(ip)
                    if fo:
                        self.fo = fo
                        if not self._machine_ip_override:
                            self.machine_ip = fo.machine_ip
                continue
            if not isinstance(l4, dpkt.udp.UDP):
                continue
            if EIP_IO_PORT not in (l4.sport, l4.dport):
                continue
            parsed = parse_io(bytes(l4.data))
            if parsed is None:
                continue
            _conn_id, cip_seq, body = parsed
            src = socket.inet_ntoa(ip.src)
            if self.machine_ip is None and not self._machine_ip_override:
                self.machine_ip = src if len(body) <= ASSEMBLY_LEN else None
            direction = self._resolve_direction(src, body)
            if self.machine_ip is None and direction == "TO":
                self.machine_ip = src
            data, run_idle = self._slice(direction, body)
            if t0 is None:
                t0 = ts
            yield Frame(n, ts - t0, direction, cip_seq, data, run_idle)


# ---------------------------------------------------------------------------
# Decode helpers
# ---------------------------------------------------------------------------

def decode_word_number(data: bytes) -> str:
    """Extract the ASCII work number / program at byte 44 (32B field)."""
    chunk = data[WORK_NUMBER_OFFSET:WORK_NUMBER_OFFSET + WORK_NUMBER_LEN]
    return chunk.split(b"\x00", 1)[0].decode("ascii", "ignore")


def signal_name(direction: str, bit: int) -> str:
    """Return the signal name (e.g. 'DO002') for a bit offset, or empty."""
    table = TO_SIGNALS if direction == "TO" else OT_SIGNALS
    return table.get(bit, ("", ""))[0]


def signal_description(direction: str, bit: int) -> str:
    """Return the signal description for a bit offset, or empty."""
    table = TO_SIGNALS if direction == "TO" else OT_SIGNALS
    return table.get(bit, ("", ""))[1]


def bit_label(direction: str, bit: int) -> str:
    """Full label for a bit: 'DO002 robot stop request...' or empty."""
    name, desc = (TO_SIGNALS if direction == "TO" else OT_SIGNALS).get(
        bit, ("", "")
    )
    return f"{name} {desc}" if name else ""


# ---------------------------------------------------------------------------
# Summary output
# ---------------------------------------------------------------------------

def _print_profile(dec: MazakDecoder) -> None:
    fo = dec.fo
    if fo is None:
        print(
            "connection profile: NO ForwardOpen in capture -- "
            "assuming 110B assembly, run/idle header inferred by length"
        )
        return
    print("connection profile: from ForwardOpen")
    print(f"  machine (target) IP: {fo.machine_ip}")
    print(f"  assembly: {fo.assembly_len} bytes")
    print(
        f"  O->T (us->machine):  size {fo.o2t_size}B = 2 seq"
        f" + {fo.header_len('OT')} hdr + {fo.assembly_len} data"
        f" | conn 0x{fo.o2t_conn_id:08X} | RPI {fo.rpi_o2t_us / 1000:.0f}ms"
    )
    print(
        f"  T->O (machine->us):  size {fo.t2o_size}B = 2 seq"
        f" + {fo.header_len('TO')} hdr + {fo.assembly_len} data"
        f" | conn 0x{fo.t2o_conn_id:08X} | RPI {fo.rpi_t2o_us / 1000:.0f}ms"
    )


def summary(path: str, *, machine_ip: str | None = None) -> None:
    """Print a one-line connection summary for *path*."""
    dec = MazakDecoder(machine_ip)
    frames = list(dec.frames(path))
    to = [f for f in frames if f.direction == "TO"]
    ot = [f for f in frames if f.direction == "OT"]
    print(f"file: {path}")
    _print_profile(dec)
    print(f"machine IP: {dec.machine_ip}")
    print(
        f"frames: {len(frames)}  (T->O machine->us: {len(to)},"
        f" O->T us->machine: {len(ot)})"
    )
    if to:
        dur = frames[-1].t - frames[0].t
        print(f"duration: {dur:.2f}s")
        print(
            f"first  T->O byte0=0x{to[0].data[0]:02X}"
            f" ctrl=0x{to[0].data[12]:02X}{to[0].data[13]:02X}"
            f" work#='{decode_word_number(to[0].data)}'"
        )
        print(
            f"last   T->O byte0=0x{to[-1].data[0]:02X}"
            f" ctrl=0x{to[-1].data[12]:02X}{to[-1].data[13]:02X}"
            f" work#='{decode_word_number(to[-1].data)}'"
        )


# ---------------------------------------------------------------------------
# Timeline output
# ---------------------------------------------------------------------------

def _format_signal_line(
    byte_idx: int, bitpos: int, state: str, old: int, new: int,
    direction: str,
) -> str:
    gbit = byte_idx * 8 + bitpos
    label = bit_label(direction, gbit)
    return (
        f"           byte{byte_idx:2d} bit{bitpos} -> {state}"
        f"  (0x{old:02X}->0x{new:02X}) {label}"
    )


def timeline(
    path: str,
    *,
    only_dir: str | None = None,
    only_bytes: set[int] | None = None,
    hide_heartbeat: bool = False,
    machine_ip: str | None = None,
) -> None:
    """Print a change-log timeline of all I/O frames in *path*.

    Only bits/bytes that change between consecutive frames in the same
    direction are reported.  The first frame of each direction is the
    baseline.
    """
    dec = MazakDecoder(machine_ip)
    print(f"# {path}")
    print(
        "# T->O = machine->us (statuses) | O->T = us->machine (commands)"
    )
    print(
        "# 'initial' = first packet seen in a direction (baseline);"
        " later lines show only changes."
    )

    prev: dict[str, bytes | None] = {"TO": None, "OT": None}
    announced = False

    for f in dec.frames(path):
        if not announced and dec.fo is not None:
            fo = dec.fo
            print(
                f"# ForwardOpen: machine={fo.machine_ip}"
                f" assembly={fo.assembly_len}B"
                f" O->T hdr={fo.header_len('OT')}"
                f" T->O hdr={fo.header_len('TO')}"
            )
            announced = True

        if only_dir and f.direction != only_dir:
            continue

        tag = "T->O" if f.direction == "TO" else "O->T"
        p = prev[f.direction]

        if p is None:
            prev[f.direction] = f.data
            print(
                f"\n[{f.t:7.3f}] f{f.num} {tag} initial:"
                f" byte0=0x{f.data[0]:02X}"
                f" ctrl={f.data[12]:02X}{f.data[13]:02X}"
                f" work#='{decode_word_number(f.data)}'"
            )
            continue

        lines: list[str] = []
        for i in range(max(len(p), len(f.data))):
            if i in WORK_NUMBER_BYTES:
                continue
            if only_bytes and i not in only_bytes:
                continue
            old = p[i] if i < len(p) else 0
            new = f.data[i] if i < len(f.data) else 0
            if old != new:
                changed = old ^ new
                for bitpos in range(8):
                    if not (changed & (1 << bitpos)):
                        continue
                    gbit = i * 8 + bitpos
                    if hide_heartbeat and gbit == HEARTBEAT_BIT:
                        continue
                    state = "1" if (new & (1 << bitpos)) else "0"
                    lines.append(
                        _format_signal_line(
                            i, bitpos, state, old, new, f.direction
                        )
                    )

        # Work-number / program changes as a single decoded line.
        if not only_bytes or bool(only_bytes & WORK_NUMBER_BYTES):
            old_wn = decode_word_number(p)
            new_wn = decode_word_number(f.data)
            if old_wn != new_wn:
                lines.append(
                    f"           byte{WORK_NUMBER_OFFSET} work#/program:"
                    f" '{old_wn}' -> '{new_wn}'"
                )

        if lines:
            print(f"[{f.t:7.3f}] f{f.num} {tag}:")
            print("\n".join(lines))

        prev[f.direction] = f.data


# ---------------------------------------------------------------------------
# Diff (compare two captures)
# ---------------------------------------------------------------------------

def diff(path_a: str, path_b: str, *, machine_ip: str | None = None) -> None:
    """Compare the first T->O frame of two captures side-by-side."""

    def _load(p: str) -> tuple[str, bytes, bytes, str]:
        dec = MazakDecoder(machine_ip)
        frames = list(dec.frames(p))
        to = [f for f in frames if f.direction == "TO"]
        if not to:
            return os.path.basename(p), b"", b"", "(no TO frames)"
        return (
            os.path.basename(p),
            to[0].data,
            to[-1].data,
            decode_word_number(to[0].data),
        )

    name_a, first_a, last_a, wn_a = _load(path_a)
    name_b, first_b, last_b, wn_b = _load(path_b)

    if not first_a or not first_b:
        print("One or both captures have no T->O frames.")
        return

    print(f"--- {name_a}")
    print(f"+++ {name_b}")
    print()
    print(f"  work#: '{wn_a}' vs '{wn_b}'")
    print()

    max_bytes = 14  # first 14 bytes is enough for signal-level comparison
    header = "  byte  " + " ".join(f"{i:4d}" for i in range(max_bytes))
    print(header)
    print("  " + "-" * (7 + max_bytes * 5))
    print(f"  {name_a:5s}: " + " ".join(f"0x{b:02X}" for b in first_a[:max_bytes]))
    print(f"  {name_b:5s}: " + " ".join(f"0x{b:02X}" for b in first_b[:max_bytes]))

    diffs: list[str] = []
    for i in range(min(len(first_a), len(first_b))):
        if first_a[i] != first_b[i]:
            for bitpos in range(8):
                abit = (first_a[i] >> bitpos) & 1
                bbit = (first_b[i] >> bitpos) & 1
                if abit != bbit:
                    gbit = i * 8 + bitpos
                    label = bit_label("TO", gbit) or "(unnamed)"
                    diffs.append(
                        f"  byte[{i:2d}] bit{bitpos}:"
                        f" {name_a}={abit} {name_b}={bbit}  {label}"
                    )
    if diffs:
        print()
        print("  Bit-level differences in first frame:")
        print("\n".join(diffs))

    # Also show what changed within each capture (first vs last).
    for name, first, last in [(name_a, first_a, last_a), (name_b, first_b, last_b)]:
        own: list[str] = []
        for i in range(len(first)):
            if first[i] != last[i]:
                old, new = first[i], last[i]
                for bitpos in range(8):
                    if ((old ^ new) >> bitpos) & 1:
                        gbit = i * 8 + bitpos
                        label = bit_label("TO", gbit) or "(unnamed)"
                        state = "1 -> 0" if ((old >> bitpos) & 1) else "0 -> 1"
                        own.append(
                            f"    byte[{i:2d}] bit{bitpos}: {state} {label}"
                        )
        if own:
            print(f"\n  {name} internal changes (first→last frame):")
            print("\n".join(own))
        else:
            print(f"\n  {name}: no internal changes")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

GUIDE = """\
HOW TO READ mazak_eip_decode OUTPUT
===================================

Two directions, named from our (robot/scanner) point of view:
  T->O   machine -> us   statuses   (the manuals' DO/SO outputs)
  O->T   us -> machine   commands   (the manuals' DI/SI inputs)

SUMMARY (no --timeline)
  Prints the connection profile -- read from the ForwardOpen if the capture
  contains it (TCP 44818), else guessed. Shows per-direction sizes broken into
  [2B sequence count] + [run/idle header] + [assembly], plus machine IP, the
  O2T/T2O connection IDs, and the RPI. Then first/last byte0 and work number.

TIMELINE (--timeline)
  A change log, NOT a packet dump. The first packet seen in each direction is
  shown in full as a baseline; every later line shows only what CHANGED vs the
  previous packet in the same direction.

  Baseline line:
    [  0.004] f9 T->O initial: byte0=0x06 ctrl=0100 work#='4'
      [  0.004]    relative time (s) from the first decoded I/O packet
      f9           frame number in the capture
      T->O         direction (here machine->us)
      initial      first sighting of this direction (baseline snapshot)
      byte0=0x06   std output block (assembly byte 0), hex
      ctrl=0100    control word (assembly bytes 12-13), hex
      work#='4'    work number (32B ASCII at assembly byte 44)

  Change line:
    [  0.304] f16 T->O:
               byte 0 bit3 -> 1  (0x06->0x0E) DO003 operating panel AT RETRACT...
      byte 0       byte offset into the 110-byte assembly
      bit3         bit position within the byte (0 = LSB)
      -> 1         new state of that bit (1=on, 0=off)
      (0x06->0x0E) old -> new value of the whole byte
      DO003 ...    decoded signal name

  bit index = byte*8 + bit. Std signals live in bytes 0-2; the 101-111 control
  word is at bytes 12-13 (DO side is bit-shifted by the 4-bit DO105 service
  code -- the tool already accounts for this).

  The work number / program is a 32-byte ASCII field at byte 44; its changes
  are shown decoded as one line rather than per-bit.

NOISE
  DO000/DI000 (byte0 bit0) is the comms-check heartbeat and toggles every
  cycle. Use --no-heartbeat to hide it.

FILTERS
  --dir TO|OT        one direction only
  --bytes 0,12,13    only these byte offsets
  --signal DO101,DI101  signal names to watch (maps to --bytes automatically)
  --machine-ip IP    force which host is the machine (overrides detection)

DIFF (--diff FILE)
  Compare two captures side-by-side at the bit level.

LIVE
  Pass '-' to read pcap/pcapng from stdin, e.g.
    tshark -i eth0 -f 'host <machine>' -w - | mazak_eip_decode.py - --timeline
  Start the capture before the scanner connects so the ForwardOpen is included.
"""


def _parse_bytes_arg(raw: str | None) -> set[int] | None:
    if raw is None:
        return None
    return {int(x) for x in raw.split(",")}


def _parse_signal_arg(raw: str | None) -> set[int] | None:
    if raw is None:
        return None
    to_rev = {name: bit for bit, (name, _) in TO_SIGNALS.items()}
    ot_rev = {name: bit for bit, (name, _) in OT_SIGNALS.items()}
    bytes_needed: set[int] = set()
    for token in raw.split(","):
        token = token.strip().upper()
        bit = to_rev.get(token) or ot_rev.get(token)
        if bit is None:
            print(f"warning: unknown signal {token!r}, ignoring", file=sys.stderr)
            continue
        bytes_needed.add(bit // 8)
    return bytes_needed


def main() -> None:
    ap = argparse.ArgumentParser(
        description=(
            "Decode Mazak SmoothAi EtherNet/IP implicit I/O from pcap/pcapng."
            " Use --guide for a walkthrough of the output format."
        ),
    )
    ap.add_argument("file", nargs="?", help="pcap/pcapng file, or '-' for stdin")
    ap.add_argument("--guide", action="store_true",
                    help="print a guide to reading the output, then exit")
    ap.add_argument("--timeline", action="store_true",
                    help="print a bit-level change log")
    ap.add_argument("--dir", choices=["TO", "OT"], default=None,
                    help="only show one direction")
    ap.add_argument("--bytes", default=None,
                    help="comma-separated list of byte offsets to show")
    ap.add_argument("--signal", default=None,
                    help="comma-separated signal names (e.g. DO101,DI101,DO102); "
                    "maps to --bytes automatically")
    ap.add_argument("--machine-ip", default=None,
                    help="force which host is the machine")
    ap.add_argument("--no-heartbeat", action="store_true",
                    help="hide the DO000/DI000 comms-check toggle")
    ap.add_argument("--diff", metavar="FILE", default=None,
                    help="compare with another capture")
    args = ap.parse_args()

    if args.guide:
        print(GUIDE)
        return

    if not args.file:
        ap.error(
            "a capture file (or '-' for stdin) is required (or use --guide)"
        )

    only_bytes = _parse_bytes_arg(args.bytes)
    signal_bytes = _parse_signal_arg(args.signal)
    if signal_bytes is not None:
        only_bytes = (only_bytes or set()) | signal_bytes

    if args.diff:
        diff(args.file, args.diff, machine_ip=args.machine_ip)
    elif args.timeline:
        timeline(
            args.file,
            only_dir=args.dir,
            only_bytes=only_bytes,
            hide_heartbeat=args.no_heartbeat,
            machine_ip=args.machine_ip,
        )
    else:
        summary(args.file, machine_ip=args.machine_ip)


if __name__ == "__main__":
    main()
