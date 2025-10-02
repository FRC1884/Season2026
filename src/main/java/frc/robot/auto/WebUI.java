package frc.robot.auto;

import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpHandler;
import com.sun.net.httpserver.HttpServer;
import java.io.IOException;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.nio.charset.StandardCharsets;
import java.util.Map;
import java.util.stream.Collectors;

/** Minimal embedded HTTP server for an iPad-friendly UI. */
public class WebUI {
  private final HttpServer server;
  private final TaskRegistry registry;
  private final String bindAddress;
  private final int port;

  public WebUI(TaskRegistry registry, int port) throws IOException {
    this(registry, null, port);
  }

  public WebUI(TaskRegistry registry, String bindAddress, int port) throws IOException {
    this.registry = registry;
    String address = (bindAddress == null || bindAddress.isBlank()) ? "0.0.0.0" : bindAddress;
    this.bindAddress = address;
    this.port = port;
    this.server = HttpServer.create(new InetSocketAddress(this.bindAddress, this.port), 0);

    server.createContext("/", new RootHandler());
    server.createContext("/api/state", new StateHandler());
    server.createContext("/api/toggle", new ToggleHandler());
    server.createContext("/api/branch", new BranchHandler());
    server.createContext("/api/load", new LoadHandler());
    server.createContext("/api/sourcepref", new SourcePrefHandler());
    server.createContext("/api/release", new ReleaseHandler());
    server.createContext("/api/target", new TargetHandler());
    server.createContext("/api/send", new SendHandler());
    server.createContext("/api/claimnext", new ClaimNextHandler());
    server.createContext("/api/freeall", new FreeAllHandler());
    server.createContext("/api/doneall", new DoneAllHandler());
    // Limit concurrency to avoid unbounded thread growth if a dashboard polls rapidly.
    // Use daemon threads so the JVM can exit cleanly.
    java.util.concurrent.ThreadFactory tf =
        r -> {
          Thread t = new Thread(r, "WebUI-HTTP");
          t.setDaemon(true);
          return t;
        };
    server.setExecutor(java.util.concurrent.Executors.newFixedThreadPool(4, tf));
  }

  public void start() {
    server.start();
  }

  public void stop() {
    server.stop(0);
  }

  class RootHandler implements HttpHandler {
    @Override
    public void handle(HttpExchange exchange) throws IOException {
      if (!exchange.getRequestMethod().equalsIgnoreCase("GET")) {
        exchange.sendResponseHeaders(405, -1);
        return;
      }
      String html = pageHtml();
      byte[] bytes = html.getBytes(StandardCharsets.UTF_8);
      exchange.getResponseHeaders().add("Content-Type", "text/html; charset=utf-8");
      exchange.sendResponseHeaders(200, bytes.length);
      try (OutputStream os = exchange.getResponseBody()) {
        os.write(bytes);
      }
    }
  }

  class StateHandler implements HttpHandler {
    @Override
    public void handle(HttpExchange exchange) throws IOException {
      if (!exchange.getRequestMethod().equalsIgnoreCase("GET")) {
        exchange.sendResponseHeaders(405, -1);
        return;
      }
      byte[] bytes = registry.toJson().getBytes(StandardCharsets.UTF_8);
      exchange.getResponseHeaders().add("Content-Type", "application/json");
      exchange.sendResponseHeaders(200, bytes.length);
      try (OutputStream os = exchange.getResponseBody()) {
        os.write(bytes);
      }
    }
  }

  class ToggleHandler implements HttpHandler {
    @Override
    public void handle(HttpExchange exchange) throws IOException {
      if (!exchange.getRequestMethod().equalsIgnoreCase("POST")) {
        exchange.sendResponseHeaders(405, -1);
        return;
      }
      String body = new String(exchange.getRequestBody().readAllBytes(), StandardCharsets.UTF_8);
      Map<String, String> form = parseForm(body);
      boolean enabled = Boolean.parseBoolean(form.getOrDefault("enabled", "false"));
      registry.setAutonomousEnabled(enabled);
      exchange.sendResponseHeaders(204, -1);
    }
  }

  class BranchHandler implements HttpHandler {
    @Override
    public void handle(HttpExchange exchange) throws IOException {
      if (!exchange.getRequestMethod().equalsIgnoreCase("POST")) {
        exchange.sendResponseHeaders(405, -1);
        return;
      }
      String body = new String(exchange.getRequestBody().readAllBytes(), StandardCharsets.UTF_8);
      Map<String, String> form = parseForm(body);
      String branch = form.get("branch");
      String state = form.get("state");
      if (branch == null || state == null) {
        exchange.sendResponseHeaders(400, -1);
        return;
      }
      TaskRegistry.ReefBranch b;
      try {
        b = TaskRegistry.ReefBranch.valueOf(branch);
      } catch (IllegalArgumentException e) {
        exchange.sendResponseHeaders(400, -1);
        return;
      }
      switch (state) {
        case "FREE" -> registry.markFree(b);
        case "CLAIMED" -> registry.markClaimed(b);
        case "DONE" -> registry.markDone(b);
        default -> {
          exchange.sendResponseHeaders(400, -1);
          return;
        }
      }
      exchange.sendResponseHeaders(204, -1);
    }
  }

  class LoadHandler implements HttpHandler {
    @Override
    public void handle(HttpExchange exchange) throws IOException {
      if (!exchange.getRequestMethod().equalsIgnoreCase("POST")) {
        exchange.sendResponseHeaders(405, -1);
        return;
      }
      registry.triggerLoad();
      exchange.sendResponseHeaders(204, -1);
    }
  }

  class ReleaseHandler implements HttpHandler {
    @Override
    public void handle(HttpExchange exchange) throws IOException {
      if (!exchange.getRequestMethod().equalsIgnoreCase("POST")) {
        exchange.sendResponseHeaders(405, -1);
        return;
      }
      registry.triggerRelease();
      exchange.sendResponseHeaders(204, -1);
    }
  }

  class ClaimNextHandler implements HttpHandler {
    @Override
    public void handle(HttpExchange exchange) throws IOException {
      if (!exchange.getRequestMethod().equalsIgnoreCase("POST")) {
        exchange.sendResponseHeaders(405, -1);
        return;
      }
      registry.claimNextFree();
      exchange.sendResponseHeaders(204, -1);
    }
  }

  class FreeAllHandler implements HttpHandler {
    @Override
    public void handle(HttpExchange exchange) throws IOException {
      if (!exchange.getRequestMethod().equalsIgnoreCase("POST")) {
        exchange.sendResponseHeaders(405, -1);
        return;
      }
      for (var b : TaskRegistry.ReefBranch.values()) {
        registry.markFree(b);
      }
      exchange.sendResponseHeaders(204, -1);
    }
  }

  class DoneAllHandler implements HttpHandler {
    @Override
    public void handle(HttpExchange exchange) throws IOException {
      if (!exchange.getRequestMethod().equalsIgnoreCase("POST")) {
        exchange.sendResponseHeaders(405, -1);
        return;
      }
      for (var b : TaskRegistry.ReefBranch.values()) {
        registry.markDone(b);
      }
      exchange.sendResponseHeaders(204, -1);
    }
  }

  class TargetHandler implements HttpHandler {
    @Override
    public void handle(HttpExchange exchange) throws IOException {
      if (!exchange.getRequestMethod().equalsIgnoreCase("POST")) {
        exchange.sendResponseHeaders(405, -1);
        return;
      }
      String body = new String(exchange.getRequestBody().readAllBytes(), StandardCharsets.UTF_8);
      Map<String, String> form = parseForm(body);
      boolean touched = false;
      String face = form.get("face");
      String side = form.get("side");
      String level = form.get("level");
      try {
        if (face != null) {
          registry.setSelectedFace(TaskRegistry.ReefFace.valueOf(face));
          touched = true;
        }
        if (side != null) {
          registry.setSelectedSide(TaskRegistry.ApproachSide.valueOf(side));
          touched = true;
        }
        if (level != null) {
          registry.setSelectedLevel(TaskRegistry.CoralLevel.valueOf(level));
          touched = true;
        }
      } catch (IllegalArgumentException e) {
        exchange.sendResponseHeaders(400, -1);
        return;
      }
      if (!touched) {
        exchange.sendResponseHeaders(400, -1);
        return;
      }
      exchange.sendResponseHeaders(204, -1);
    }
  }

  class SendHandler implements HttpHandler {
    @Override
    public void handle(HttpExchange exchange) throws IOException {
      if (!exchange.getRequestMethod().equalsIgnoreCase("POST")) {
        exchange.sendResponseHeaders(405, -1);
        return;
      }
      registry.requestSend();
      exchange.sendResponseHeaders(204, -1);
    }
  }

  class SourcePrefHandler implements HttpHandler {
    @Override
    public void handle(HttpExchange exchange) throws IOException {
      if (!exchange.getRequestMethod().equalsIgnoreCase("POST")) {
        exchange.sendResponseHeaders(405, -1);
        return;
      }
      String body = new String(exchange.getRequestBody().readAllBytes(), StandardCharsets.UTF_8);
      Map<String, String> form = parseForm(body);
      String pref = form.get("pref");
      if (pref == null) {
        exchange.sendResponseHeaders(400, -1);
        return;
      }
      try {
        registry.setSourcePreference(TaskRegistry.CoralSourcePreference.valueOf(pref));
        exchange.sendResponseHeaders(204, -1);
      } catch (IllegalArgumentException e) {
        exchange.sendResponseHeaders(400, -1);
      }
    }
  }

  private static Map<String, String> parseForm(String body) {
    return body == null || body.isEmpty()
        ? Map.of()
        : java.util.Arrays.stream(body.split("&"))
            .map(s -> s.split("=", 2))
            .collect(
                Collectors.toMap(a -> urlDecode(a[0]), a -> a.length > 1 ? urlDecode(a[1]) : ""));
  }

  private static String urlDecode(String s) {
    try {
      return java.net.URLDecoder.decode(s, StandardCharsets.UTF_8);
    } catch (Exception e) {
      return s;
    }
  }

  // Drop-in replacement for your pageHtml() using Java 17 text blocks.
  // Visually represents the REEF with interactive branches, bays, and a status list.
  private String pageHtml() {
    return """
<!doctype html>
<html lang=\"en\">
<head>
  <meta charset=\"utf-8\" />
  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\" />
  <title>Reefscape – Coral Control</title>
  <style>
    :root{
      --bg:#0b1020; --card:#0f172a; --muted:#9aa5b1; --text:#e5e7eb;
      --good:#16a34a; --warn:#eab308; --bad:#ef4444; --accent:#38bdf8; --accent2:#22c55e;
      --ring:#334155; --ring-strong:#475569;
    }
    html,body{height:100%}
    body{margin:0;font-family:system-ui,-apple-system,Segoe UI,Roboto,Helvetica,Arial,sans-serif;background:var(--bg);color:var(--text)}
    .wrap{display:grid;grid-template-columns:1fr;gap:12px;padding:14px}
    @media (min-width:1000px){.wrap{grid-template-columns:1.2fr 0.8fr;align-items:start}}

    .card{background:var(--card);border:1px solid var(--ring);border-radius:14px;padding:12px}
    .row{display:flex;gap:10px;align-items:center;flex-wrap:wrap}
    .btn{padding:10px 12px;border:1px solid var(--ring);border-radius:10px;cursor:pointer;background:#0b1220;color:var(--text);user-select:none}
    .btn:hover{border-color:var(--ring-strong)}
    .btn.primary{background:linear-gradient(135deg,#0ea5e9,#22c55e);border-color:transparent}
    .switch{display:inline-flex;gap:8px;align-items:center;border:1px solid var(--ring);border-radius:999px;padding:6px 10px}
    .switch input{accent-color:var(--accent2)}
    select{padding:8px;border-radius:8px;border:1px solid var(--ring);background:#0b1220;color:var(--text)}
    h1{margin:0 0 6px;font-size:20px}
    h2{margin:8px 0;font-size:16px;color:var(--muted)}
    .legend{display:flex;gap:8px;flex-wrap:wrap}
    .pill{border:1px solid var(--ring);border-radius:999px;padding:4px 10px;font-size:12px}
    .FREE{background:rgba(59,130,246,.12)}
    .CLAIMED{background:rgba(234,179,8,.15)}
    .DONE{background:rgba(22,163,74,.15)}

    /* Field layout */
    .field{position:relative;aspect-ratio:1/1;border-radius:16px;border:1px solid var(--ring);background:radial-gradient(120% 120% at 50% 50%, #0e162b 0,#0b1020 55%, #09101e 100%);overflow:hidden}
    .reef{position:absolute;inset:6%;border-radius:50%;border:2px dashed #1f2a44}
    .reef-center{position:absolute;inset:40%;border-radius:50%;border:2px solid #1f2a44;background:#0b1326;display:grid;place-items:center}
    .reef-center span{font-size:12px;color:var(--muted)}
    .branch{position:absolute;width:52px;height:52px;transform:translate(-50%,-50%);border-radius:50%;border:2px solid var(--ring);display:grid;place-items:center;background:#0b1220;transition:transform .12s ease, box-shadow .12s ease, border-color .12s}
    .branch:hover{box-shadow:0 0 0 3px rgba(56,189,248,.15)}
    .branch.FREE{border-color:#60a5fa}
    .branch.CLAIMED{border-color:#facc15}
    .branch.DONE{border-color:#22c55e}
    .branch .slot{width:24px;height:24px;border-radius:6px;display:grid;place-items:center}
    .coral{width:18px;height:18px;border-radius:4px;box-shadow:inset 0 0 0 2px rgba(0,0,0,.25)}
    .coral.free{background:#60a5fa}
    .coral.claimed{background:#facc15}
    .coral.done{background:#22c55e}
    .label{position:absolute;top:56px;font-size:11px;color:var(--muted);white-space:nowrap;left:50%;transform:translateX(-50%)}
    .branch.selected{outline:3px solid #38bdf8; outline-offset:2px}

    /* Face left/right selectors overlay */
    .face-sel{position:absolute;transform:translate(-50%,-50%);display:flex;gap:6px}
    .mini{border:1px solid var(--ring);background:#0b1220;color:var(--text);border-radius:999px;padding:4px 8px;font-size:11px;display:inline-flex;align-items:center;gap:4px;cursor:pointer}
    .mini:hover{border-color:var(--ring-strong)}
    .mini.selected{outline:2px solid #38bdf8; outline-offset:2px}

    .bay{position:absolute;width:140px;height:120px;border:1px solid var(--ring);border-radius:12px;background:#0a152c;display:flex;flex-direction:column;justify-content:space-between;padding:8px}
    .bay h3{margin:0;font-size:12px;color:var(--muted)}

    .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(250px,1fr));gap:10px}

    .list-item{display:flex;justify-content:space-between;align-items:center;border:1px solid var(--ring);border-radius:10px;padding:8px}
    .kbd{font-family:ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace;font-size:12px;padding:2px 6px;border-radius:6px;border:1px solid var(--ring);color:var(--muted)}

    .toast{position:fixed;bottom:14px;left:50%;transform:translateX(-50%);background:#0d1b34;border:1px solid var(--ring);padding:10px 14px;border-radius:10px;opacity:0;pointer-events:none;transition:opacity .2s}
    .toast.show{opacity:1}

    /* Right-side selection panel */
    .select-card .title{display:flex;align-items:center;gap:8px;margin-bottom:6px}
    .choices{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}
    .choice{border:1px solid var(--ring);border-radius:12px;padding:8px;background:#0b1220;display:grid;place-items:center;cursor:pointer;transition:border-color .12s, box-shadow .12s}
    .choice:hover{border-color:var(--ring-strong);box-shadow:0 0 0 3px rgba(56,189,248,.12)}
    .choice.selected{outline:2px solid #38bdf8; outline-offset:2px}
    .choice svg{width:42px;height:42px}
    .row.wrap{flex-wrap:wrap}
  </style>
</head>
<body>
  <div class=\"wrap\">
    <div class=\"card\" style=\"grid-column:1/-1\">
      <h1>Manual Control · REEFSCAPE</h1>
      <div class=\"row\" style=\"margin-top:6px\">
        <label class=\"switch\"><input type=\"checkbox\" id=\"auto\"> Controls Enabled</label>
        <button id=\"load\" class=\"btn\">Got Coral</button>
        <button id=\"release\" class=\"btn\">Released Coral</button>
        <div style=\"margin-left:16px;color:var(--muted)\">Target: <span id=\"targetName\">None</span></div>
        <div class=\"legend\" style=\"margin-left:auto\">
          <span class=\"pill FREE\">Free</span>
          <span class=\"pill CLAIMED\">Occupied</span>
          <span class=\"pill DONE\">Done</span>
        </div>
      </div>
    </div>

    <div class=\"card field\" id=\"field\" style=\"min-height:420px\">
      <div class=\"reef\"></div>
      <div class=\"reef-center\"><span>REEF</span></div>
      <!-- Loading bays (stylized) -->
      <div class=\"bay\" id=\"bay-left\" style=\"left:8px;top:8px\">
        <h3>LEFT BAY</h3>
        <div class=\"row\"><button class=\"btn\" id=\"left-free\">Free All Left</button><button class=\"btn\" id=\"left-done\">Done Left</button></div>
      </div>
      <div class=\"bay\" id=\"bay-right\" style=\"right:8px;bottom:8px\">
        <h3>RIGHT BAY</h3>
        <div class=\"row\"><button class=\"btn\" id=\"right-free\">Free All Right</button><button class=\"btn\" id=\"right-done\">Done Right</button></div>
      </div>
    </div>

    <!-- Right column: reef face selection and branch states -->
    <div class=\"card select-card\" id=\"actionsCard\">
      <div class=\"title\"><h2 style=\"margin:0\">Reef Status</h2></div>
      <div class=\"row\" style=\"gap:8px;align-items:center\">
        <label for=\"faceSel\">Face:</label>
        <select id=\"faceSel\">
          <option value=\"REEF_1\">1</option>
          <option value=\"REEF_2\">2</option>
          <option value=\"REEF_3\">3</option>
          <option value=\"REEF_4\">4</option>
          <option value=\"REEF_5\">5</option>
          <option value=\"REEF_6\">6</option>
        </select>
      </div>
      <div id=\"branchesPanel\" style=\"margin-top:10px\"></div>
    </div>

    <div class=\"card\" style=\"grid-column:1/-1\">
      <h2>Branches</h2>
      <div id=\"grid\" class=\"grid\"></div>
      <div class=\"row\" style=\"margin-top:8px\">
        <button id=\"freeAll\" class=\"btn\">Reset All (Free)</button>
        <button id=\"doneAll\" class=\"btn\">Mark All Done</button>
        <span class=\"kbd\">Tip: Tap a node on the reef to cycle states</span>
      </div>
    </div>
  </div>
  <div class=\"toast\" id=\"toast\"></div>

  <script>
    const toast = (msg)=>{const t=document.getElementById('toast');t.textContent=msg;t.classList.add('show');setTimeout(()=>t.classList.remove('show'),1200)};
    const vibrate = (ms=15)=>{if(window.navigator && 'vibrate' in navigator) navigator.vibrate(ms)};

    // Utility
    const postForm = (url, data)=> fetch(url,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:new URLSearchParams(data)});

    // Build the circular reef from the branch keys. Keeps positions stable by sorting.
    function layoutReef(state){
      const field=document.getElementById('field');
      // Clear existing branches
      field.querySelectorAll('.branch, .face-sel, .height-picker').forEach(n=>n.remove());
      const keys=Object.keys(state.branches).sort();
      const N=keys.length; // Typically 12 for REEFSCAPE
      const radiusPct=40; // circle radius inside .field
      keys.forEach((k,i)=>{
        const angle=((i/N)*Math.PI*2) - Math.PI/2; // start at top, clockwise
        const cx=50 + radiusPct * Math.cos(angle);
        const cy=50 + radiusPct * Math.sin(angle);
        const node=document.createElement('button');
        const selected = isSelectedKey(k, state.selection);
        node.className='branch '+state.branches[k]+(selected?' selected':'');
        node.style.left=cx+'%';
        node.style.top=cy+'%';
        node.title=k;
        node.dataset.branch=k;
        node.innerHTML=`<div class=\"slot\"><div class=\"coral ${state.branches[k].toLowerCase()}\"></div></div><div class=\"label\">${prettyName(k)}</div>`;
        node.addEventListener('click',()=>selectBranch(k));
        node.addEventListener('touchend',()=>selectBranch(k));
        field.appendChild(node);
      });

      // Height picker on field (bottom-left)
      renderHeightPicker(state.selection);
    }

    async function selectBranch(branchKey){
      const sel = keyToSelection(branchKey);
      await postForm('/api/target', sel);
      vibrate();
      load();
    }

    async function load(){
      const r = await fetch('/api/state');
      const s = await r.json();
      // Controls
      document.getElementById('auto').checked = s.autonomousEnabled;
      const sel = document.getElementById('pref');
      if (sel) sel.value = s.sourcePreference || 'NEAREST';

      // Reef
      layoutReef(s);

      // Face selector reflects current selection
      const faceSel = document.getElementById('faceSel');
      if (faceSel) faceSel.value = (s.selection && s.selection.face) ? s.selection.face : 'REEF_1';
      renderBranchesPanel(s);

      // Current named target (for Choreo/PathPlanner target names)
      const tgt = document.getElementById('targetName');
      if (tgt) tgt.textContent = s.targetName || 'None';

      // List/grid view
      const g = document.getElementById('grid');
      if (g) g.innerHTML = '';
    }

    function mkBtn(t, fn){ const b=document.createElement('button'); b.className='btn'; b.textContent=t; b.onclick=fn; return b; }
    async function setState(branch, ns){ await postForm('/api/branch',{branch, state: ns}); vibrate(); toast(`${branch}: ${ns}`); load(); }

    // Global controls
    document.addEventListener('DOMContentLoaded',()=>{
      document.getElementById('auto').addEventListener('change', async (e)=>{
        await postForm('/api/toggle',{enabled: e.target.checked});
        toast('Autonomous '+(e.target.checked?'enabled':'disabled'));
      });
      document.getElementById('load').addEventListener('click', async ()=>{ await fetch('/api/load',{method:'POST'}); toast('Has Coral'); });
      document.getElementById('release').addEventListener('click', async ()=>{ await fetch('/api/release',{method:'POST'}); toast('Release Requested'); });
      const freeAllBtn = document.getElementById('freeAll');
      const doneAllBtn = document.getElementById('doneAll');
      if (freeAllBtn) freeAllBtn.addEventListener('click', async ()=>{ await fetch('/api/freeall',{method:'POST'}); toast('All set to FREE'); load(); });
      if (doneAllBtn) doneAllBtn.addEventListener('click', async ()=>{ await fetch('/api/doneall',{method:'POST'}); toast('All set to DONE'); load(); });

      // Face selector changes selection and updates right panel
      const faceSelCtl = document.getElementById('faceSel');
      if (faceSelCtl) faceSelCtl.addEventListener('change', async (e)=>{
        await postForm('/api/target',{face:e.target.value});
        renderBranchesPanel(await (await fetch('/api/state')).json());
      });

      // Optional bay bulk actions (no server-side distinction; demo helpers). Customize if you name branches with prefixes.
      const mass = async (predicate, state)=>{
        const js = await (await fetch('/api/state')).json();
        const pairs = Object.entries(js.branches).filter(([k])=>predicate(k));
        for(const [k] of pairs){ await postForm('/api/branch',{branch:k,state}); }
        load();
      }
      document.getElementById('left-free').addEventListener('click',()=>mass(k=>/L|A|N/.test(k), 'FREE'));
      document.getElementById('left-done').addEventListener('click',()=>mass(k=>/L|A|N/.test(k), 'DONE'));
      document.getElementById('right-free').addEventListener('click',()=>mass(k=>/R|B|S/.test(k), 'FREE'));
      document.getElementById('right-done').addEventListener('click',()=>mass(k=>/R|B|S/.test(k), 'DONE'));

      load(); setInterval(load,1500);
    });

    function renderHeightPicker(selection){
      const field=document.getElementById('field');
      const wrap=document.createElement('div');
      wrap.className='height-picker';
      wrap.style.position='absolute';wrap.style.left='8px';wrap.style.bottom='8px';
      wrap.style.display='flex';wrap.style.flexDirection='column';wrap.style.gap='6px';
      ['L1','L2','L3','L4'].forEach((lvl,i)=>{
        const b=document.createElement('button');
        var selLevel = (selection && selection.level) ? selection.level : 'L2';
        b.className='mini'+(selLevel===lvl?' selected':'');
        b.innerHTML = svgStack(i+1);
        b.title = lvl;
        b.addEventListener('click', async ()=>{ await postForm('/api/target',{level:lvl}); vibrate(); load(); });
        wrap.appendChild(b);
      });
      field.appendChild(wrap);
    }

    function selectionToKey(sel){
      // Map face+side to our branch key naming (F#_A or F#_B)
      const faceNum = parseInt((sel.face||'REEF_1').split('_')[1]);
      const side = (sel.side||'LEFT')==='LEFT' ? 'A' : 'B';
      return `F${faceNum}_${side}`;
    }

    function keyToSelection(key){
      const parts = String(key||'F1_A').split('_');
      const numStr = (parts[0]||'F1').slice(1);
      const num = parseInt(numStr) || 1;
      const side = (parts[1]==='B') ? 'RIGHT' : 'LEFT';
      return { face: `REEF_${num}`, side };
    }

    function renderBranchesPanel(state){
      const panel = document.getElementById('branchesPanel');
      if (!panel) return;
      panel.innerHTML='';
      const face = (state.selection && state.selection.face) ? state.selection.face : 'REEF_1';
      const num = parseInt(face.split('_')[1]) || 1;
      const items = [
        { key:`F${num}_A`, label:'Left (A)' },
        { key:`F${num}_B`, label:'Right (B)' }
      ];
      for(const it of items){
        const st = state.branches[it.key] || 'FREE';
        const row = document.createElement('div');
        row.className='row'; row.style.justifyContent='space-between'; row.style.marginTop='8px';
        const left=document.createElement('div'); left.textContent=it.label;
        const right=document.createElement('div'); right.className='row';
        const pill=document.createElement('span'); pill.className='pill '+st; pill.textContent=(st==='CLAIMED'?'OCCUPIED':st);
        const bFree=mkBtn('Free', async()=>{ await postForm('/api/branch',{branch:it.key,state:'FREE'}); toast(it.label+': Free'); load(); });
        const bOcc=mkBtn('Occupied', async()=>{ await postForm('/api/branch',{branch:it.key,state:'CLAIMED'}); toast(it.label+': Occupied'); load(); });
        const bDone=mkBtn('Done', async()=>{ await postForm('/api/branch',{branch:it.key,state:'DONE'}); toast(it.label+': Done'); load(); });
        right.append(pill,bFree,bOcc,bDone);
        row.append(left,right);
        panel.appendChild(row);
      }
    }

    function isSelectedKey(k, selection){
      if(!selection) return false;
      const selKey = selectionToKey(selection);
      return selKey===k;
    }

    function svgReefFace(num){
      return `<svg viewBox='0 0 64 64' fill='none'>
        <circle cx='32' cy='32' r='30' stroke='${cssVar('--ring')}' stroke-width='2' fill='#0a152c'/>
        <text x='32' y='38' text-anchor='middle' font-size='22' fill='${cssVar('--text')}' font-family='system-ui, -apple-system, Segoe UI, Roboto, Helvetica, Arial, sans-serif'>${num}</text>
      </svg>`;
    }

    function svgStack(level){
      const bar = (y)=>`<rect x='18' y='${y}' width='28' height='6' rx='3' fill='${cssVar('--accent')}'/>`;
      return `<svg viewBox='0 0 64 64' fill='none'>
        <rect x='4' y='4' width='56' height='56' rx='12' stroke='${cssVar('--ring')}' fill='#0a152c'/>
        ${Array.from({length:level}).map((_,i)=>bar(44 - i*10)).join('')}
      </svg>`;
    }

    function prettyName(k){
      // Convert F#_A/B to 1..12 index for display
      const parts = String(k||'F1_wA').split('_');
      const num = parseInt((parts[0]||'F1').slice(1)) || 1;
      const isA = (parts[1]||'A')==='A';
      const base = (num-1)*2;
      const idx = base + (isA?1:2);
      return `Node ${idx}`;
    }

    function cssVar(name){ return getComputedStyle(document.documentElement).getPropertyValue(name).trim() || '#e5e7eb'; }
  </script>
</body>
</html>
""";
  }
}
