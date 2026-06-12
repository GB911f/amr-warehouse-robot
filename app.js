/* ============ NAV on scroll ============ */
const nav = document.getElementById('nav');
const onScroll = () => nav.classList.toggle('scrolled', window.scrollY > 40);
onScroll();
window.addEventListener('scroll', onScroll, { passive: true });

/* ============ Reveal on view ============ */
const io = new IntersectionObserver((entries) => {
  entries.forEach((e) => {
    if (e.isIntersecting) { e.target.classList.add('in'); io.unobserve(e.target); }
  });
}, { threshold: 0.12 });
document.querySelectorAll('.reveal').forEach((el) => io.observe(el));

/* ============ Stat counters ============ */
const animateCount = (el) => {
  const target = parseFloat(el.dataset.count);
  const dec = parseInt(el.dataset.decimals || '0', 10);
  const pre = el.dataset.prefix || '';
  const suf = el.dataset.suffix || '';
  const dur = 1400, t0 = performance.now();
  const ease = (t) => 1 - Math.pow(1 - t, 3);
  const tick = (now) => {
    const p = Math.min((now - t0) / dur, 1);
    const v = target * ease(p);
    el.textContent = pre + v.toFixed(dec) + suf;
    if (p < 1) requestAnimationFrame(tick);
    else el.textContent = pre + target.toFixed(dec) + suf;
  };
  requestAnimationFrame(tick);
};
const statIO = new IntersectionObserver((entries) => {
  entries.forEach((e) => { if (e.isIntersecting) { animateCount(e.target); statIO.unobserve(e.target); } });
}, { threshold: 0.6 });
document.querySelectorAll('.stat__val').forEach((el) => statIO.observe(el));

/* ============ Hi-DPI helper ============ */
function fitCanvas(canvas, h) {
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const w = canvas.clientWidth || canvas.parentElement.clientWidth;
  const height = h || Math.round(w * 0.62);
  canvas.width = w * dpr;
  canvas.height = height * dpr;
  canvas.style.height = height + 'px';
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { ctx, w, h: height };
}

/* ============ PLANNING — six selectable algorithms ============ */
(function planner() {
  const cv = document.getElementById('planCanvas');
  if (!cv) return;
  const chips = [...document.querySelectorAll('#navigation .chip')];
  const algoLabel = document.getElementById('demoAlgo');
  const capEl = document.getElementById('demoCaption');
  const btn = document.getElementById('replayBtn');

  let ctx, W, H, cell, COLS, ROWS, grid, start, goal;
  let algo = 'A*';
  let S = null, raf = null, started = false;

  const CFG = {
    'A*':       { kind: 'astar', w: 1.2, cap: 'Live A*: a heuristic pulls the frontier toward the goal, so it settles on the shortest route while exploring little.' },
    'Dijkstra': { kind: 'astar', w: 0,   cap: 'Dijkstra: no heuristic — the frontier floods outward evenly in every direction until it reaches the goal.' },
    'Theta*':   { kind: 'theta', w: 1.0, cap: 'Theta*: any-angle search — line-of-sight checks let the route cut corners into long straight diagonals.' },
    'JPS':      { kind: 'jps',   w: 1.0, cap: 'Jump Point Search: A* that leaps along straight lines, expanding far fewer nodes on an open grid.' },
    'RRT':      { kind: 'rrt',           cap: 'RRT: a tree grows from random samples until a branch reaches the goal — quick to find a path, but not the shortest one.' },
    'D* Lite':  { kind: 'astar', w: 1.0, backward: true, cap: 'D* Lite: searches backward from the goal, keeping a route it can quickly repair when the map changes.' }
  };

  /* ---- grid ---- */
  function buildGrid() {
    COLS = 30; ROWS = Math.max(16, Math.round(COLS * (H / W)));
    cell = W / COLS;
    grid = Array.from({ length: ROWS }, () => Array(COLS).fill(0));
    const blocks = [
      [4, 2, 1, 9], [4, 2, 6, 1], [9, 2, 1, 9], [9, 9, 6, 1],
      [14, 4, 1, 10], [19, 1, 1, 8], [19, 8, 6, 1], [24, 3, 1, 9],
      [6, 13, 12, 1], [12, 5, 4, 1]
    ];
    blocks.forEach(([cx, cy, w, h]) => {
      for (let y = cy; y < cy + h; y++) for (let x = cx; x < cx + w; x++)
        if (y >= 0 && y < ROWS && x >= 0 && x < COLS) grid[y][x] = 1;
    });
    start = { x: 1, y: ROWS - 2 };
    goal = { x: COLS - 2, y: 1 };
    grid[start.y][start.x] = 0; grid[goal.y][goal.x] = 0;
  }
  const walkable = (x, y) => x >= 0 && y >= 0 && x < COLS && y < ROWS && grid[y][x] === 0;
  const key = (n) => n.x + ',' + n.y;
  const octile = (a, b) => { const dx = Math.abs(a.x - b.x), dy = Math.abs(a.y - b.y); return (dx + dy) + (1.41421 - 2) * Math.min(dx, dy); };
  const dist = (a, b) => Math.hypot(a.x - b.x, a.y - b.y);

  /* ---- (re)start the current algorithm ---- */
  function reset() {
    const cfg = CFG[algo];
    S = { kind: cfg.kind, cfg, done: false, hold: 0, closed: new Set(), open: [], came: {}, g: {}, f: {}, path: null, jumps: [], tree: [], samples: 0 };
    S.src = cfg.backward ? goal : start;
    S.dst = cfg.backward ? start : goal;
    if (cfg.kind === 'rrt') {
      S.tree = [{ x: start.x, y: start.y, p: -1 }];
    } else {
      S.open = [S.src];
      S.g[key(S.src)] = 0;
      S.f[key(S.src)] = octile(S.src, S.dst);
      if (cfg.kind === 'theta') S.came[key(S.src)] = S.src;
    }
  }

  function popMin() {
    let bi = 0;
    for (let i = 1; i < S.open.length; i++)
      if ((S.f[key(S.open[i])] ?? 1e9) < (S.f[key(S.open[bi])] ?? 1e9)) bi = i;
    return S.open.splice(bi, 1)[0];
  }
  function reconstruct(cur) {
    const p = []; let n = cur, guard = 0;
    while (n && guard++ < 4000) { p.unshift(n); const par = S.came[key(n)]; if (!par || (par.x === n.x && par.y === n.y)) break; n = par; }
    S.path = p;
  }
  const DIRS = [[1, 0], [-1, 0], [0, 1], [0, -1], [1, 1], [1, -1], [-1, 1], [-1, -1]];

  /* ---- A* / Dijkstra / D* Lite ---- */
  function stepAstar() {
    if (!S.open.length) { S.done = true; return; }
    const cur = popMin();
    if (cur.x === S.dst.x && cur.y === S.dst.y) { reconstruct(cur); S.done = true; return; }
    S.closed.add(key(cur));
    for (const [dx, dy] of DIRS) {
      const nx = cur.x + dx, ny = cur.y + dy;
      if (!walkable(nx, ny)) continue;
      if (dx && dy && (!walkable(cur.x + dx, cur.y) || !walkable(cur.x, cur.y + dy))) continue;
      const n = { x: nx, y: ny }; if (S.closed.has(key(n))) continue;
      const ng = (S.g[key(cur)] ?? 1e9) + ((dx && dy) ? 1.41421 : 1);
      if (ng < (S.g[key(n)] ?? 1e9)) {
        S.came[key(n)] = cur; S.g[key(n)] = ng;
        S.f[key(n)] = ng + S.cfg.w * octile(n, S.dst);
        if (!S.open.some((o) => o.x === nx && o.y === ny)) S.open.push(n);
      }
    }
  }

  /* ---- Theta* (lazy, any-angle) ---- */
  function lineOfSight(a, b) {
    let x0 = a.x, y0 = a.y; const x1 = b.x, y1 = b.y;
    const dx = Math.abs(x1 - x0), dy = Math.abs(y1 - y0);
    const sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
    let err = dx - dy, guard = 0;
    while (guard++ < 200) {
      if (!walkable(x0, y0)) return false;
      if (x0 === x1 && y0 === y1) return true;
      const e2 = 2 * err;
      if (e2 > -dy) { err -= dy; x0 += sx; }
      if (e2 < dx) { err += dx; y0 += sy; }
    }
    return true;
  }
  function stepTheta() {
    if (!S.open.length) { S.done = true; return; }
    const cur = popMin();
    if (cur.x === S.dst.x && cur.y === S.dst.y) { reconstruct(cur); S.done = true; return; }
    S.closed.add(key(cur));
    const par = S.came[key(cur)] || cur;
    for (const [dx, dy] of DIRS) {
      const nx = cur.x + dx, ny = cur.y + dy;
      if (!walkable(nx, ny)) continue;
      if (dx && dy && (!walkable(cur.x + dx, cur.y) || !walkable(cur.x, cur.y + dy))) continue;
      const n = { x: nx, y: ny }; if (S.closed.has(key(n))) continue;
      let parent, ng;
      if (lineOfSight(par, n)) { parent = par; ng = (S.g[key(par)] ?? 1e9) + dist(par, n); }
      else { parent = cur; ng = (S.g[key(cur)] ?? 1e9) + ((dx && dy) ? 1.41421 : 1); }
      if (ng < (S.g[key(n)] ?? 1e9)) {
        S.came[key(n)] = parent; S.g[key(n)] = ng;
        S.f[key(n)] = ng + S.cfg.w * octile(n, S.dst);
        if (!S.open.some((o) => o.x === nx && o.y === ny)) S.open.push(n);
      }
    }
  }

  /* ---- Jump Point Search ---- */
  function jump(cx, cy, dx, dy, depth) {
    const nx = cx + dx, ny = cy + dy;
    if (!walkable(nx, ny)) return null;
    if (depth < 400) S.jumps.push({ x: nx, y: ny });
    if (nx === S.dst.x && ny === S.dst.y) return { x: nx, y: ny };
    if (dx && dy) {
      if ((walkable(nx - dx, ny + dy) && !walkable(nx - dx, ny)) || (walkable(nx + dx, ny - dy) && !walkable(nx, ny - dy))) return { x: nx, y: ny };
      if (jump(nx, ny, dx, 0, depth + 1) || jump(nx, ny, 0, dy, depth + 1)) return { x: nx, y: ny };
    } else if (dx) {
      if ((walkable(nx + dx, ny + 1) && !walkable(nx, ny + 1)) || (walkable(nx + dx, ny - 1) && !walkable(nx, ny - 1))) return { x: nx, y: ny };
    } else {
      if ((walkable(nx + 1, ny + dy) && !walkable(nx + 1, ny)) || (walkable(nx - 1, ny + dy) && !walkable(nx - 1, ny))) return { x: nx, y: ny };
    }
    return jump(nx, ny, dx, dy, depth + 1);
  }
  function prunedDirs(node) {
    const par = S.came[key(node)];
    if (!par || (par.x === node.x && par.y === node.y)) return DIRS;
    const dx = Math.sign(node.x - par.x), dy = Math.sign(node.y - par.y);
    const out = [];
    if (dx && dy) {
      if (walkable(node.x, node.y + dy)) out.push([0, dy]);
      if (walkable(node.x + dx, node.y)) out.push([dx, 0]);
      if (walkable(node.x + dx, node.y + dy)) out.push([dx, dy]);
      if (!walkable(node.x - dx, node.y) && walkable(node.x - dx, node.y + dy)) out.push([-dx, dy]);
      if (!walkable(node.x, node.y - dy) && walkable(node.x + dx, node.y - dy)) out.push([dx, -dy]);
    } else if (dx) {
      if (walkable(node.x + dx, node.y)) out.push([dx, 0]);
      if (!walkable(node.x, node.y + 1) && walkable(node.x + dx, node.y + 1)) out.push([dx, 1]);
      if (!walkable(node.x, node.y - 1) && walkable(node.x + dx, node.y - 1)) out.push([dx, -1]);
    } else {
      if (walkable(node.x, node.y + dy)) out.push([0, dy]);
      if (!walkable(node.x + 1, node.y) && walkable(node.x + 1, node.y + dy)) out.push([1, dy]);
      if (!walkable(node.x - 1, node.y) && walkable(node.x - 1, node.y + dy)) out.push([-1, dy]);
    }
    return out.length ? out : DIRS;
  }
  function stepJPS() {
    if (!S.open.length) { S.done = true; return; }
    const cur = popMin();
    if (cur.x === S.dst.x && cur.y === S.dst.y) { reconstruct(cur); S.done = true; return; }
    S.closed.add(key(cur));
    for (const [dx, dy] of prunedDirs(cur)) {
      const jp = jump(cur.x, cur.y, dx, dy, 0);
      if (!jp || S.closed.has(key(jp))) continue;
      const ng = (S.g[key(cur)] ?? 1e9) + dist(cur, jp);
      if (ng < (S.g[key(jp)] ?? 1e9)) {
        S.came[key(jp)] = cur; S.g[key(jp)] = ng;
        S.f[key(jp)] = ng + S.cfg.w * octile(jp, S.dst);
        if (!S.open.some((o) => o.x === jp.x && o.y === jp.y)) S.open.push(jp);
      }
    }
  }

  /* ---- RRT ---- */
  function segFree(x0, y0, x1, y1) {
    const steps = Math.max(1, Math.ceil(Math.hypot(x1 - x0, y1 - y0) * 3));
    for (let i = 0; i <= steps; i++) {
      const t = i / steps;
      if (!walkable(Math.floor(x0 + (x1 - x0) * t), Math.floor(y0 + (y1 - y0) * t))) return false;
    }
    return true;
  }
  function stepRRT() {
    if (S.path) { S.done = true; return; }
    if (S.samples++ > 6000) { S.done = true; return; }
    const toGoal = Math.random() < 0.12;
    const samp = toGoal ? { x: goal.x, y: goal.y } : { x: Math.random() * COLS, y: Math.random() * ROWS };
    let bi = 0, bd = 1e9;
    for (let i = 0; i < S.tree.length; i++) { const d = dist(S.tree[i], samp); if (d < bd) { bd = d; bi = i; } }
    const near = S.tree[bi];
    const ang = Math.atan2(samp.y - near.y, samp.x - near.x);
    const nx = near.x + Math.cos(ang) * 2.2, ny = near.y + Math.sin(ang) * 2.2;
    if (nx < 0 || ny < 0 || nx >= COLS || ny >= ROWS) return;
    if (!segFree(near.x, near.y, nx, ny)) return;
    const node = { x: nx, y: ny, p: bi };
    S.tree.push(node);
    if (dist(node, goal) < 1.7 && segFree(nx, ny, goal.x, goal.y)) {
      S.tree.push({ x: goal.x, y: goal.y, p: S.tree.length - 1 });
      const p = []; let idx = S.tree.length - 1, guard = 0;
      while (idx !== -1 && guard++ < 6000) { p.unshift(S.tree[idx]); idx = S.tree[idx].p; }
      S.path = p;
    }
  }

  function stepOnce() {
    switch (S.kind) {
      case 'theta': return stepTheta();
      case 'jps': return stepJPS();
      case 'rrt': return stepRRT();
      default: return stepAstar();
    }
  }

  /* ---- draw ---- */
  const rect = (x, y) => [x * cell, y * cell, cell, cell];
  function drawWalls() {
    for (let y = 0; y < ROWS; y++) for (let x = 0; x < COLS; x++)
      if (grid[y][x] === 1) { ctx.fillStyle = '#161b29'; const r = rect(x, y); ctx.fillRect(r[0], r[1], r[2], r[3]); }
  }
  function marker(n, c) {
    const x = n.x * cell + cell / 2, y = n.y * cell + cell / 2;
    ctx.fillStyle = c; ctx.shadowColor = c; ctx.shadowBlur = 14;
    ctx.beginPath(); ctx.arc(x, y, Math.max(4, cell * .34), 0, 7); ctx.fill(); ctx.shadowBlur = 0;
  }
  function draw() {
    ctx.fillStyle = '#05070d'; ctx.fillRect(0, 0, W, H);
    ctx.strokeStyle = 'rgba(255,255,255,.03)'; ctx.lineWidth = 1;
    for (let x = 0; x <= COLS; x++) { ctx.beginPath(); ctx.moveTo(x * cell, 0); ctx.lineTo(x * cell, H); ctx.stroke(); }
    for (let y = 0; y <= ROWS; y++) { ctx.beginPath(); ctx.moveTo(0, y * cell); ctx.lineTo(W, y * cell); ctx.stroke(); }

    if (S.kind === 'rrt') {
      drawWalls();
      ctx.strokeStyle = 'rgba(41,151,255,.5)'; ctx.lineWidth = 1.4;
      for (const n of S.tree) {
        if (n.p < 0) continue; const a = S.tree[n.p];
        ctx.beginPath(); ctx.moveTo(n.x * cell + cell / 2, n.y * cell + cell / 2); ctx.lineTo(a.x * cell + cell / 2, a.y * cell + cell / 2); ctx.stroke();
      }
      ctx.fillStyle = 'rgba(150,200,255,.75)';
      for (const n of S.tree) { ctx.beginPath(); ctx.arc(n.x * cell + cell / 2, n.y * cell + cell / 2, 1.6, 0, 7); ctx.fill(); }
    } else {
      S.closed.forEach((k) => { const [x, y] = k.split(',').map(Number); ctx.fillStyle = '#22304e'; const r = rect(x, y); ctx.fillRect(r[0] + 1, r[1] + 1, r[2] - 2, r[3] - 2); });
      if (S.kind === 'jps') { ctx.fillStyle = 'rgba(41,151,255,.16)'; for (const j of S.jumps) { const r = rect(j.x, j.y); ctx.fillRect(r[0] + 1, r[1] + 1, r[2] - 2, r[3] - 2); } }
      S.open.forEach((n) => { ctx.fillStyle = 'rgba(41,151,255,.9)'; const r = rect(n.x, n.y); ctx.fillRect(r[0] + 1, r[1] + 1, r[2] - 2, r[3] - 2); });
      drawWalls();
    }

    if (S.path) {
      ctx.strokeStyle = '#9fd0ff'; ctx.lineWidth = Math.max(3, cell * .32); ctx.lineJoin = 'round'; ctx.lineCap = 'round';
      ctx.shadowColor = 'rgba(143,196,255,.85)'; ctx.shadowBlur = 12; ctx.beginPath();
      S.path.forEach((n, i) => { const x = n.x * cell + cell / 2, y = n.y * cell + cell / 2; i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); });
      ctx.stroke(); ctx.shadowBlur = 0;
    }
    marker(start, '#8fc4ff'); marker(goal, '#ff5f57');
  }

  function loop() {
    const per = S.kind === 'rrt' ? 6 : S.kind === 'jps' ? 1 : S.cfg.w === 0 ? 3 : 2;
    if (!S.done) for (let i = 0; i < per; i++) if (!S.done) stepOnce();
    draw();
    if (S.done) { S.hold++; if (S.hold > 180) reset(); }
    raf = requestAnimationFrame(loop);
  }

  function init() { const r = fitCanvas(cv, undefined); ctx = r.ctx; W = r.w; H = r.h; buildGrid(); reset(); }

  function setAlgo(a) {
    if (!CFG[a]) return;
    algo = a;
    if (algoLabel) algoLabel.textContent = a;
    if (capEl) capEl.textContent = CFG[a].cap;
    chips.forEach((c) => c.classList.toggle('chip--on', c.dataset.algo === a));
    if (started) reset();
  }
  chips.forEach((c) => c.addEventListener('click', () => setAlgo(c.dataset.algo)));
  btn && btn.addEventListener('click', () => { if (started) reset(); });

  const vIO = new IntersectionObserver((es) => {
    es.forEach((e) => { if (e.isIntersecting && !started) { started = true; init(); loop(); } });
  }, { threshold: 0.25 });
  vIO.observe(cv);

  window.addEventListener('resize', () => { if (started) { cancelAnimationFrame(raf); init(); loop(); } });
})();
