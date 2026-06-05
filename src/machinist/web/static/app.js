/*
 * Machinist web UI — front-end controller.
 *
 * Plain ES modules + a handful of custom elements (light-DOM Web Components),
 * no framework, no build step. State arrives two ways:
 *
 *   • a 4 Hz poll of GET /api/state    → smooth live joints / signals / cycle
 *   • an EventSource on /api/events    → the scrolling event log + connection
 *     liveness (SSE is push, so it doubles as a heartbeat)
 *
 * Commands (button clicks, signal toggles, the command bar) POST to
 * /api/command and reuse the exact verbs the Textual TUI accepts.
 */

const api = {
  async state() {
    const res = await fetch("/api/state", { cache: "no-store" });
    if (!res.ok) throw new Error(`state ${res.status}`);
    return res.json();
  },
  async command(command) {
    const res = await fetch("/api/command", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ command }),
    });
    return res.json();
  },
};

const fmt = (n) => (Number.isFinite(n) ? n.toFixed(3) : String(n));

/* ---- shared app state ------------------------------------------------- */
const store = {
  devices: [],
  selected: null,
  byName(name) {
    return this.devices.find((d) => d.name === name) ?? null;
  },
};

/* ---- <device-list> ---------------------------------------------------- */
class DeviceList extends HTMLElement {
  connectedCallback() {
    this._tpl = document.getElementById("device-card-tpl");
    this.addEventListener("click", (e) => {
      const card = e.target.closest(".card");
      if (card) selectDevice(card.dataset.name);
    });
  }

  render(devices, selected) {
    if (this.childElementCount !== devices.length) this.replaceChildren();
    devices.forEach((d, i) => {
      let card = this.children[i];
      if (!card) {
        card = this._tpl.content.firstElementChild.cloneNode(true);
        this.append(card);
      }
      card.dataset.name = d.name;
      card.dataset.state = d.lifecycle;
      card.setAttribute("aria-current", String(d.name === selected));
      card.querySelector(".card-name").textContent = d.name;
      card.querySelector(".card-kind").textContent = d.kind;
      card.querySelector(".card-state").textContent = d.lifecycle;
    });
  }
}

/* ---- <device-detail> -------------------------------------------------- */
class DeviceDetail extends HTMLElement {
  render(device) {
    if (!device) {
      this.innerHTML = `<p class="empty">No device selected.</p>`;
      return;
    }
    this.innerHTML = "";
    this.append(this._head(device));
    if (device.arm) this.append(this._arm(device.arm));
    if (device.machine) this.append(this._machine(device.machine));
    if (device.ethernetip) this.append(this._ethernetip(device.ethernetip));
    if (device.signals) this.append(this._signals(device));
  }

  _head(d) {
    const head = el("div", "detail-head");
    head.innerHTML = `
      <h2>${esc(d.name)}</h2>
      <span class="kind">${esc(d.kind)}</span>
      <span class="pill" data-state="${esc(d.lifecycle)}">${esc(d.lifecycle)}</span>
      <span class="endpoint">${esc(d.endpoint)}</span>`;
    const actions = el("div", "actions");
    if (d.arm) {
      actions.append(
        button("E-Stop", "danger", () => run(`estop ${d.name}`)),
        button("Reset", "go", () => run(`reset ${d.name}`)),
        button(
          d.arm.servo_on ? "Servo off" : "Servo on",
          "",
          () => run(`servo ${d.name} ${d.arm.servo_on ? "off" : "on"}`),
        ),
      );
    }
    (d.programs ?? []).forEach((p) =>
      actions.append(button(`▶ ${p}`, "", () => run(`run ${d.name} ${p}`))),
    );
    if (actions.childElementCount) head.append(actions);
    return head;
  }

  _arm(arm) {
    const top = el("div", "tiles split");
    const summary = tile("Arm", `
      <dl class="kv">
        <dt>mode</dt><dd class="${arm.estopped ? "bad" : "good"}">${esc(arm.mode)}</dd>
        <dt>servo</dt><dd>${arm.servo_on ? "on" : "off"}</dd>
        <dt>e-stop</dt><dd class="${arm.estopped ? "bad" : "good"}">${arm.estopped ? "ENGAGED" : "clear"}</dd>
        <dt>command</dt><dd>${esc(arm.command ?? "none")}</dd>
      </dl>`);
    const joints = tile("Joints", `<div class="bars">${arm.joints.map(bar).join("")}</div>`);
    top.append(summary, joints);

    const bottom = el("div", "tiles");
    bottom.append(
      tile("Pose (x y z rx ry rz)", `<div class="bars">${arm.pose.map(bar).join("")}</div>`),
    );
    return frag(top, bottom);
  }

  _machine(m) {
    const pos = `${fmt(m.position.x)}  ${fmt(m.position.y)}  ${fmt(m.position.z)}`;
    const doors = Object.entries(m.doors)
      .map(([n, open]) => `${n}:${open ? "open" : "shut"}`)
      .join("  ") || "none";
    return tile("Machine", `
      <dl class="kv">
        <dt>cycle</dt><dd class="${m.cycle === "running" ? "good" : ""}">${esc(m.cycle)}</dd>
        <dt>program</dt><dd>${esc(m.program || "none")}</dd>
        <dt>spindle</dt><dd>${m.spindle_rpm} rpm</dd>
        <dt>feed</dt><dd>${m.feed}</dd>
        <dt>tool</dt><dd>T${m.tool}</dd>
        <dt>parts</dt><dd>${m.parts}</dd>
        <dt>xyz</dt><dd>${esc(pos)}</dd>
        <dt>doors</dt><dd>${esc(doors)}</dd>
      </dl>`);
  }

  _signals(d) {
    const inputs = d.signals.filter((s) => s.direction === "input");
    const outputs = d.signals.filter((s) => s.direction === "output");
    const tiles = el("div", "tiles split");
    tiles.append(
      this._sigTile("Inputs", d.name, inputs, true),
      this._sigTile("Outputs", d.name, outputs, false),
    );
    return tiles;
  }

  _ethernetip(e) {
    const summary = tile("EtherNet/IP", `
      <dl class="kv">
        <dt>mode</dt><dd>${esc(e.mode)}</dd>
        <dt>transport</dt><dd class="${e.transport_ready ? "good" : "bad"}">${e.transport_ready ? "ready" : "offline"}</dd>
        <dt>peer</dt><dd class="${e.peer_connected ? "good" : ""}">${e.peer_connected ? "connected" : "waiting"}</dd>
      </dl>
      <div class="packet-hex">
        <div><span>IN</span> ${esc(e.input_block_hex)}</div>
        <div><span>OUT</span> ${esc(e.output_block_hex)}</div>
      </div>`);
    const tables = el("div", "tiles split");
    tables.append(
      tile("Input packet fields", fieldTable(e.input_fields)),
      tile("Output packet fields", fieldTable(e.output_fields)),
    );
    return frag(summary, tables, tile("Derived state", fieldTable(e.derived_fields)));
  }

  _sigTile(title, device, signals, clickable) {
    const t = tile(title, "");
    if (!signals.length) {
      t.insertAdjacentHTML("beforeend", `<p class="empty">none</p>`);
      return t;
    }
    const grid = el("div", "signals");
    for (const s of signals) {
      const b = document.createElement(clickable ? "button" : "div");
      b.className = "sig";
      b.dataset.on = String(s.value);
      b.dataset.dir = s.direction;
      b.innerHTML = `<span class="led"></span><span class="sig-name">${esc(s.name)}</span><span class="dir">${s.direction}</span>`;
      if (clickable) {
        b.type = "button";
        b.addEventListener("click", () => run(`set ${device}.${s.name} ${s.value ? 0 : 1}`));
      }
      grid.append(b);
    }
    t.append(grid);
    return t;
  }
}

/* ---- <event-log> ------------------------------------------------------ */
class EventLog extends HTMLElement {
  static MAX = 400;

  push(ev) {
    const atBottom = this.scrollHeight - this.scrollTop - this.clientHeight < 40;
    const row = el("div", "row");
    row.dataset.kind = ev.kind;
    const payload = Object.entries(ev.payload || {})
      .map(([k, v]) => `${k}=${v}`)
      .join(" ");
    row.innerHTML = `<span class="t">${ev.timestamp.toFixed(3).slice(-9)}</span>` +
      `<span class="d">${esc(ev.device)}</span>` +
      `<span class="k">${esc(ev.kind)}</span>` +
      `<span class="p">${esc(payload)}</span>`;
    this.append(row);
    while (this.childElementCount > EventLog.MAX) this.firstElementChild.remove();
    if (atBottom) this.scrollTop = this.scrollHeight;
  }
}

/* ---- <command-bar> ---------------------------------------------------- */
class CommandBar extends HTMLElement {
  connectedCallback() {
    this.innerHTML = `
      <form class="cmd-form">
        <span class="prompt">◇</span>
        <input name="cmd" autocomplete="off" spellcheck="false"
               placeholder="command — e.g. estop arm1 · set io1.o5 1 · run mill O0001 · help" />
        <span class="hint">↵ to send</span>
      </form>`;
    this.querySelector("form").addEventListener("submit", (e) => {
      e.preventDefault();
      const input = e.target.elements.cmd;
      const value = input.value.trim();
      if (value) run(value);
      input.value = "";
    });
  }
}

customElements.define("device-list", DeviceList);
customElements.define("device-detail", DeviceDetail);
customElements.define("event-log", EventLog);
customElements.define("command-bar", CommandBar);

/* ---- DOM helpers ------------------------------------------------------ */
function el(tag, className) {
  const node = document.createElement(tag);
  if (className) node.className = className;
  return node;
}
function tile(title, html) {
  const t = el("div", "tile");
  t.innerHTML = `<h3>${esc(title)}</h3>${html}`;
  return t;
}
function button(label, variant, onClick) {
  const b = el("button", variant ? `btn ${variant}` : "btn");
  b.type = "button";
  b.textContent = label;
  b.addEventListener("click", onClick);
  return b;
}
function bar(value) {
  const v = Math.min(Math.abs(value) / Math.PI, 1) || 0;
  return `<div class="bar" style="--v:${v.toFixed(3)}"><span class="num">${fmt(value)}</span><span class="track"></span><span class="num"></span></div>`;
}
function frag(...nodes) {
  const f = document.createDocumentFragment();
  f.append(...nodes);
  return f;
}
function esc(s) {
  return String(s ?? "").replace(/[&<>"]/g, (c) =>
    ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" })[c],
  );
}
function fieldTable(fields) {
  if (!fields?.length) return `<p class="empty">none</p>`;
  return `
    <div class="packet-table-wrap">
      <table class="packet-table">
        <thead>
          <tr><th>field</th><th>offset</th><th>type</th><th>value</th></tr>
        </thead>
        <tbody>
          ${fields.map((f) => `
            <tr>
              <td><strong>${esc(f.signal)}</strong><br /><span>${esc(f.name)}</span></td>
              <td>${esc(f.offset)}</td>
              <td>${esc(f.type)}</td>
              <td>${esc(f.value)}</td>
            </tr>`).join("")}
        </tbody>
      </table>
    </div>`;
}

/* ---- references ------------------------------------------------------- */
const devicesEl = document.getElementById("devices");
const detailEl = document.getElementById("detail");
const logEl = document.getElementById("log");
const statusEl = document.querySelector(".status");
const fleetCountEl = document.getElementById("fleet-count");

/* ---- actions ---------------------------------------------------------- */
async function run(command) {
  try {
    const result = await api.command(command);
    if (!result.ok) flashStatus(result.message);
  } catch (err) {
    flashStatus(String(err));
  }
  refresh();
}

function selectDevice(name) {
  store.selected = name;
  const swap = () => {
    renderList(true);
    renderDetail(true);
  };
  if (document.startViewTransition) document.startViewTransition(swap);
  else swap();
}

/* ---- state refresh + rendering --------------------------------------- */
let lastListSig = "";
let lastDetailSig = "";
let refreshQueued = false;
let refreshRunning = false;

function renderList(force) {
  const sig =
    store.devices.map((d) => `${d.name}:${d.lifecycle}`).join("|") + `#${store.selected}`;
  if (!force && sig === lastListSig) return;
  lastListSig = sig;
  devicesEl.render(store.devices, store.selected);
}

function renderDetail(force) {
  const device = store.byName(store.selected);
  const sig = JSON.stringify(device);
  if (!force && sig === lastDetailSig) return;
  lastDetailSig = sig;
  detailEl.render(device);
}

async function refresh() {
  let snap;
  try {
    snap = await api.state();
  } catch {
    setConn("lost");
    return;
  }
  store.devices = snap.devices;
  if (!store.selected && snap.devices.length) store.selected = snap.devices[0].name;
  fleetCountEl.textContent = `${snap.devices.length} devices`;
  renderList(false);
  renderDetail(false);
}

function shouldRefreshFromEvent(ev) {
  if (ev.kind === "state" || ev.kind === "snapshot") return true;
  return ev.device === store.selected && ev.kind !== "rx" && ev.kind !== "tx";
}

function requestRefresh() {
  refreshQueued = true;
  if (refreshRunning) return;
  void drainRefreshQueue();
}

async function drainRefreshQueue() {
  refreshRunning = true;
  try {
    while (refreshQueued) {
      refreshQueued = false;
      await refresh();
    }
  } finally {
    refreshRunning = false;
  }
}

function setConn(state) {
  statusEl.dataset.conn = state;
  statusEl.querySelector(".status-text").textContent =
    state === "live" ? "live" : state === "lost" ? "disconnected" : "connecting…";
}

let statusTimer;
function flashStatus(msg) {
  const text = statusEl.querySelector(".status-text");
  text.textContent = msg;
  clearTimeout(statusTimer);
  statusTimer = setTimeout(() => setConn(statusEl.dataset.conn), 2600);
}

/* ---- live event stream ------------------------------------------------ */
function connectStream() {
  const source = new EventSource("/api/events");
  source.onopen = () => setConn("live");
  source.onerror = () => setConn("lost");
  source.onmessage = (e) => {
    const ev = JSON.parse(e.data);
    logEl.push(ev);
    if (shouldRefreshFromEvent(ev)) requestRefresh();
  };
}

/* ---- boot ------------------------------------------------------------- */
await refresh();
connectStream();
