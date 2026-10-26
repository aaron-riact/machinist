"""HSES file control: files to and from a controller, their list, and deletion.

Reference: *DX200 Options Instructions for High-Speed Ethernet Server
Function* (HW1481977), chapters 3.2.3-3.2.6 and 3.4. A file travels in
blocks of up to 479 bytes, numbered from 1, the last with 0x8000_0000
added, and each block is acknowledged by the side that receives it:

- save (service 0x16): the host asks for a file; the controller sends its
  blocks, the host acknowledges each, and the controller answers nothing
  to the last acknowledgement
- list (0x32): as save, the blocks being the matching names, each ending
  in CR LF
- load (0x15): the host asks to send a file; the controller acknowledges
  the request (block 0), then each block, and keeps the file after the last
- delete (0x09): the host names a file; the controller acknowledges

The manual's figures put file control on UDP 10041 in two places and on
10040 in two others; this serves it on its own port, 10041 by default.
"""

from __future__ import annotations

import fnmatch
from dataclasses import dataclass, field

from ....transport.yaskawa_hses import (
    ANSWER_SERVICE,
    LAST_BLOCK,
    MAX_DATA,
    AddedStatus,
    Answer,
    Division,
    FrameError,
    Request,
    Status,
    encode_answer,
    parse_request,
    request_id_of,
)
from .controller import JOB_SUFFIX, YaskawaController

DELETE, LOAD, SAVE, LIST = 0x09, 0x15, 0x16, 0x32
#: What a list request's pattern lists (3.2.5): no pattern and *.* list jobs.
JOB_PATTERN = "*" + JOB_SUFFIX
LIST_PATTERNS = {"": JOB_PATTERN, "*.*": JOB_PATTERN}

Address = tuple[str, int]


@dataclass(slots=True)
class _Sending:
    """A file or list going to the host, block by block."""

    service: int
    blocks: list[bytes]
    #: The number of the block sent last.
    sent: int = 0


@dataclass(slots=True)
class _Receiving:
    """A file coming from the host."""

    name: str
    received: list[bytes] = field(default_factory=list)


@dataclass(slots=True)
class HsesFileServer:
    """HSES file control for a :class:`YaskawaController`, one transfer per host."""

    controller: YaskawaController
    _transfers: dict[Address, _Sending | _Receiving] = field(default_factory=dict)

    def handle(self, datagram: bytes, sender: Address) -> bytes | None:
        try:
            packet = parse_request(datagram)
        except FrameError as exc:
            return _refuse(Division.FILE, request_id_of(datagram), 0, exc.added_status)
        if packet.division is not Division.FILE:
            return encode_answer(Answer(
                division=packet.division, request_id=packet.request_id,
                service=packet.service + ANSWER_SERVICE, status=Status.UNDEFINED_COMMAND,
            ))
        if not packet.ack:
            self._transfers.pop(sender, None)  # a new request ends any transfer before it
            return self._request(packet, sender)
        transfer = self._transfers.get(sender)
        if isinstance(transfer, _Sending):
            return self._acknowledged(packet, sender, transfer)
        if isinstance(transfer, _Receiving):
            return self._received(packet, sender, transfer)
        return _refuse(Division.FILE, packet.request_id, packet.service,
                       AddedStatus.BLOCK_NUMBER_ERROR)

    # --- requests ---------------------------------------------------------------

    def _request(self, request: Request, sender: Address) -> bytes | None:
        services = {DELETE: self._delete, SAVE: self._save, LIST: self._list, LOAD: self._load}
        handler = services.get(request.service)
        if handler is None:
            return encode_answer(Answer(
                division=Division.FILE, request_id=request.request_id,
                service=request.service + ANSWER_SERVICE, status=Status.UNDEFINED_COMMAND,
            ))
        if request.data.endswith(b"\0"):
            # A real YRC1000 refuses a NUL-terminated name, though the name is there.
            return _refuse(Division.FILE, request.request_id, request.service,
                           AddedStatus.DATA_SIZE_ZERO)
        return handler(request, sender, request.data.decode("ascii", "replace"))

    def _delete(self, request: Request, sender: Address, name: str) -> bytes:
        if name not in self.controller.files:
            return _refuse(Division.FILE, request.request_id, DELETE, AddedStatus.FILE_NOT_FOUND)
        del self.controller.files[name]
        self.controller.note("hses file", action="deleted", name=name)
        return _answer(request, DELETE, block=LAST_BLOCK)

    def _save(self, request: Request, sender: Address, name: str) -> bytes:
        if name not in self.controller.files:
            return _refuse(Division.FILE, request.request_id, SAVE, AddedStatus.FILE_NOT_FOUND)
        return self._start_sending(request, sender, self.controller.files[name])

    def _list(self, request: Request, sender: Address, name: str) -> bytes:
        pattern = LIST_PATTERNS.get(name, name)
        names = sorted(n for n in self.controller.files if fnmatch.fnmatch(n, pattern))
        return self._start_sending(request, sender, "".join(f"{n}\r\n" for n in names).encode())

    def _load(self, request: Request, sender: Address, name: str) -> bytes:
        if not name:
            return _refuse(Division.FILE, request.request_id, LOAD, AddedStatus.FILE_NOT_FOUND)
        self._transfers[sender] = _Receiving(name)
        return _answer(request, LOAD, block=0)

    # --- sending (save, list) -----------------------------------------------------

    def _start_sending(self, request: Request, sender: Address, content: bytes) -> bytes:
        blocks = [content[i : i + MAX_DATA] for i in range(0, len(content), MAX_DATA)] or [b""]
        sending = _Sending(service=request.service, blocks=blocks)
        self._transfers[sender] = sending
        return self._next_block(request.request_id, sending)

    def _next_block(self, request_id: int, sending: _Sending) -> bytes:
        sending.sent += 1
        last = sending.sent == len(sending.blocks)
        return encode_answer(Answer(
            division=Division.FILE, request_id=request_id,
            service=sending.service + ANSWER_SERVICE, data=sending.blocks[sending.sent - 1],
            block=sending.sent + (LAST_BLOCK if last else 0),
        ))

    def _acknowledged(self, ack: Request, sender: Address, sending: _Sending) -> bytes | None:
        last = sending.sent == len(sending.blocks)
        expected = sending.sent + (LAST_BLOCK if last else 0)
        if ack.block != expected:
            del self._transfers[sender]
            return _refuse(Division.FILE, ack.request_id, ack.service,
                           AddedStatus.BLOCK_NUMBER_ERROR)
        if last:
            del self._transfers[sender]
            return None  # the host's last ACK is answered with nothing
        return self._next_block(ack.request_id, sending)

    # --- receiving (load) ---------------------------------------------------------

    def _received(self, data: Request, sender: Address, receiving: _Receiving) -> bytes:
        number = data.block & ~LAST_BLOCK
        if number != len(receiving.received) + 1 or len(data.data) > MAX_DATA:
            del self._transfers[sender]
            return _refuse(Division.FILE, data.request_id, LOAD, AddedStatus.BLOCK_NUMBER_ERROR)
        receiving.received.append(data.data)
        if data.block & LAST_BLOCK:
            self.controller.files[receiving.name] = b"".join(receiving.received)
            self.controller.note("hses file", action="loaded", name=receiving.name,
                                 size=len(self.controller.files[receiving.name]))
            del self._transfers[sender]
        return _answer(data, LOAD, block=data.block)


def _answer(request: Request, service: int, *, block: int) -> bytes:
    return encode_answer(Answer(
        division=Division.FILE, request_id=request.request_id,
        service=service + ANSWER_SERVICE, block=block,
    ))


def _refuse(division: Division, request_id: int, service: int, added: AddedStatus) -> bytes:
    return encode_answer(Answer(
        division=division, request_id=request_id, service=service + ANSWER_SERVICE,
        status=Status.VENDOR, added_status=added,
    ))
