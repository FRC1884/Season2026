// External WebUI for FRC NetworkTables (NT4)
// This script expects the WPILib NT4 browser client UMD to expose a global (e.g., NTCore or NT4).
// We handle two common globals: window.NTCore and window.NT4 with a similar interface.

(function () {
  const $ = (sel) => document.querySelector(sel);
  const logEl = $('#log');
  const statusEl = $('#status');
  const ntLibWarning = $('#ntLibWarning');

  const ctl = {
    enabled: $('#ctlEnabled'),
    face: $('#face'),
    side: $('#side'),
    level: $('#level'),
    name: $('#targetName'),
    send: $('#sendBtn'),
    trigLoad: $('#triggerLoadBtn'),
    trigRelease: $('#triggerReleaseBtn'),
  };
  const state = {
    enabled: $('#stEnabled'),
    face: $('#stFace'),
    side: $('#stSide'),
    level: $('#stLevel'),
    name: $('#stName'),
    branches: $('#branches'),
  };

  const connectBtn = $('#connectBtn');
  const disconnectBtn = $('#disconnectBtn');
  const rioHostInput = $('#rioHost');
  const clientNameInput = $('#clientName');

  const TOP = {
    root: '/CoralRegistry',
    enabled: '/CoralRegistry/AutonomousEnabled',
    face: '/CoralRegistry/TargetFace',
    side: '/CoralRegistry/TargetSide',
    level: '/CoralRegistry/TargetLevel',
    name: '/CoralRegistry/TargetName',
    control: {
      enabled: '/CoralRegistry/Control/AutonomousEnabled',
      face: '/CoralRegistry/Control/TargetFace',
      side: '/CoralRegistry/Control/TargetSide',
      level: '/CoralRegistry/Control/TargetLevel',
      name: '/CoralRegistry/Control/TargetName',
      send: '/CoralRegistry/Control/Send',
      trigLoad: '/CoralRegistry/Control/TriggerLoad',
      trigRelease: '/CoralRegistry/Control/TriggerRelease',
    },
  };

  const BRANCHES = [
    'F1_A', 'F1_B', 'F2_A', 'F2_B', 'F3_A', 'F3_B',
    'F4_A', 'F4_B', 'F5_A', 'F5_B', 'F6_A', 'F6_B',
  ];

  // NT client wrapper (adapts to available global)
  let nt = null; // { connect(url, name), close(), publish(topic, type, value), subscribe(topic, cb) }
  let connected = false;

  function log(msg) {
    const t = new Date().toLocaleTimeString();
    logEl.textContent += `[${t}] ${msg}\n`;
    logEl.scrollTop = logEl.scrollHeight;
  }

  function setStatus(text, ok) {
    statusEl.textContent = text;
    statusEl.style.color = ok ? '#33d17a' : '#98a6b3';
  }

  function detectNT() {
    // Known UMD exports: window.NTCore (preferred), window.NT4, or WPILib's window.NT4_Client
    const g = window;
    const libBase = g.NTCore || g.NT4 || null;
    // If WPILib's bare client class exists, wrap it into a lib-like shape
    const lib = libBase || (g.NT4_Client ? { NT4Client: g.NT4_Client } : null);
    if (!lib) return null;

    // Try to detect a client class or factory
    const Client = lib.NT4Client || lib.Client || lib.NTClient || lib.default?.NT4Client;
    if (!Client) return null;

    // Minimal adapter
    let client = null;
    const subs = new Map();
    return {
      connect: async (url, name) => {
        try {
          client = new Client({ url, name });
        } catch (e) {
          // fallback constructors
          try { client = new Client(url, name); } catch (_) {}
        }

        if (!client) throw new Error('NT client constructor not compatible');
        if (client.connect) await client.connect();
        connected = true;
        return true;
      },
      close: () => { try { client?.close?.(); } catch {} connected = false; },
      publish: (topic, type, value) => {
        try {
          if (client.publish) return client.publish(topic, type, value);
          if (client.setValue) return client.setValue(topic, value);
          // last resort: getTopic + publish
          const t = client.getTopic ? client.getTopic(topic, { type }) : null;
          return t?.publish?.(value);
        } catch (e) { log(`publish error ${topic}: ${e}`); }
      },
      subscribe: (topic, cb) => {
        try {
          if (client.subscribe) {
            const un = client.subscribe(topic, cb);
            subs.set(topic, un);
            return () => { try { un(); } catch {} };
          }
          // fallback: addListener
          if (client.addListener) {
            const h = client.addListener(topic, cb);
            subs.set(topic, h);
            return () => { try { client.removeListener(h); } catch {} };
          }
        } catch (e) { log(`subscribe error ${topic}: ${e}`); }
        return () => {};
      },
    };
  }

  function ensureNTLib() {
    const detected = detectNT();
    if (!detected) {
      ntLibWarning.hidden = false;
      setStatus('NT client library not loaded', false);
      return null;
    }
    ntLibWarning.hidden = true;
    return detected;
  }

  function rioUrlFromHost(host) {
    const h = (host || '').trim();
    const hh = h || 'roborio-0000-frc.local';
    return `ws://${hh}:5810/nt/`;
  }

  function bindControls() {
    ctl.enabled.addEventListener('change', () => nt?.publish(TOP.control.enabled, 'boolean', ctl.enabled.checked));
    ctl.face.addEventListener('change', () => nt?.publish(TOP.control.face, 'string', ctl.face.value));
    ctl.side.addEventListener('change', () => nt?.publish(TOP.control.side, 'string', ctl.side.value));
    ctl.level.addEventListener('change', () => nt?.publish(TOP.control.level, 'string', ctl.level.value));
    ctl.name.addEventListener('change', () => nt?.publish(TOP.control.name, 'string', ctl.name.value));

    ctl.send.addEventListener('click', () => momentary(TOP.control.send));
    ctl.trigLoad.addEventListener('click', () => momentary(TOP.control.trigLoad));
    ctl.trigRelease.addEventListener('click', () => momentary(TOP.control.trigRelease));
  }

  async function momentary(topic) {
    try {
      nt?.publish(topic, 'boolean', true);
      await new Promise((r) => setTimeout(r, 100));
      nt?.publish(topic, 'boolean', false);
    } catch (e) { log(`momentary error ${topic}: ${e}`); }
  }

  function subscribeState() {
    nt?.subscribe(TOP.enabled, (v) => { state.enabled.textContent = String(!!v); });
    nt?.subscribe(TOP.face, (v) => { state.face.textContent = String(v || '-'); });
    nt?.subscribe(TOP.side, (v) => { state.side.textContent = String(v || '-'); });
    nt?.subscribe(TOP.level, (v) => { state.level.textContent = String(v || '-'); });
    nt?.subscribe(TOP.name, (v) => { state.name.textContent = String(v || '-'); });

    // Branch DONE flags
    state.branches.innerHTML = '';
    BRANCHES.forEach((b) => {
      const el = document.createElement('div');
      el.className = 'badge';
      el.innerHTML = `<span>${b}</span><span class="val">-</span>`;
      state.branches.appendChild(el);
      const valEl = el.querySelector('.val');
      nt?.subscribe(`${TOP.root}/Branch/${b}`, (v) => {
        const on = !!v;
        valEl.textContent = on ? 'DONE' : 'FREE/CLAIMED';
        valEl.className = `val ${on ? 'good' : 'bad'}`;
      });
    });
  }

  function setConnectedUI(on) {
    connectBtn.disabled = on;
    disconnectBtn.disabled = !on;
  }

  connectBtn.addEventListener('click', async () => {
    nt = ensureNTLib();
    if (!nt) return;
    try {
      const url = rioUrlFromHost(rioHostInput.value);
      const name = (clientNameInput.value || 'ipad-ui').trim();
      setStatus('Connecting…', false);
      await nt.connect(url, name);
      setConnectedUI(true);
      setStatus('Connected', true);
      log(`Connected to ${url} as ${name}`);
      bindControls();
      subscribeState();
    } catch (e) {
      setStatus('Connect failed', false);
      log(`Connect error: ${e}`);
    }
  });

  disconnectBtn.addEventListener('click', () => {
    try { nt?.close(); } catch {}
    setConnectedUI(false);
    setStatus('Not connected', false);
    log('Disconnected');
  });
})();
