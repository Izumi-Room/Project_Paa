/**
 * main.js — Smart Courier Pathfinding Simulator (Web Version)
 * ============================================================
 * Implementasi visual lengkap algoritma A*, Dijkstra, dan BFS
 * pada grid 2D dengan obstacle acak. Versi web dari smart_courier.py.
 *
 * Fitur:
 *   - Grid acak dengan obstacle, start, dan goal yang selalu menjamin jalur valid
 *   - Visualisasi langkah-demi-langkah (open set, closed set, path)
 *   - Animasi kurir bergerak di jalur terpendek
 *   - Perbandingan performa 3 algoritma pada peta yang sama
 *   - Kontrol kecepatan visualisasi (Lambat, Sedang, Cepat, Instan)
 */

// ╔══════════════════════════════════════════════════════════════════════╗
// ║                     KONFIGURASI & KONSTANTA                        ║
// ╚══════════════════════════════════════════════════════════════════════╝

const GRID_ROWS = 30;
const GRID_COLS = 40;
const CELL_SIZE = 18;
const OBSTACLE_DENSITY = 0.25;
const MIN_START_GOAL_DIST = 14; // Manhattan distance minimum

// Warna (cocok dengan CSS variables)
const COLORS = {
  empty:     '#141422',
  wall:      '#2D2D3C',
  gridLine:  '#1C1C30',
  start:     '#FFB028',
  goal:      '#FF5050',
  open:      '#28A0DC',
  closed:    '#6E32B4',
  path:      '#2ECC71',
  current:   '#FFD700',
  courierBg: '#FFB028',
};

// Delay per langkah visualisasi (ms)
const SPEED_MAP = {
  slow:    60,
  medium:  15,
  fast:    2,
  instant: 0,
};

// ╔══════════════════════════════════════════════════════════════════════╗
// ║                         STATE APLIKASI                             ║
// ╚══════════════════════════════════════════════════════════════════════╝

let canvas, ctx;
let obstacles = new Set();
let startNode = null;
let goalNode  = null;

// State simulasi: 'idle' | 'searching' | 'path_found' | 'animating' | 'done'
let simState = 'idle';
let selectedAlgo = 'astar';
let selectedSpeed = 'medium';

// Data visualisasi
let openSetVis   = new Set();
let closedSetVis = new Set();
let pathVis      = [];
let currentNode  = null;

// Statistik
let statPathLen = 0;
let statVisited = 0;
let statTimeMs  = 0;

// Animasi kurir
let courierIdx = 0;
let courierX = 0, courierY = 0;
let courierTargetX = 0, courierTargetY = 0;
let courierLerp = 0;

// Gambar kurir
let courierImg = new Image();
courierImg.src = 'Kurir.png';
let courierImgLoaded = false;
courierImg.onload = () => { courierImgLoaded = true; };

// Abort controller untuk membatalkan animasi
let animAbort = null;

// ╔══════════════════════════════════════════════════════════════════════╗
// ║                     UTILITAS GRID & KEY                            ║
// ╚══════════════════════════════════════════════════════════════════════╝

/** Konversi (row, col) menjadi string key unik untuk Set/Map */
function key(r, c) { return `${r},${c}`; }

/** Parse key kembali ke [row, col] */
function parseKey(k) {
  const parts = k.split(',');
  return [parseInt(parts[0]), parseInt(parts[1])];
}

/** Cek apakah sel (r,c) valid dan bukan obstacle */
function isWalkable(r, c) {
  return r >= 0 && r < GRID_ROWS && c >= 0 && c < GRID_COLS && !obstacles.has(key(r, c));
}

/** Tetangga 4-arah */
function getNeighbors(r, c) {
  const dirs = [[-1,0],[1,0],[0,-1],[0,1]];
  const result = [];
  for (const [dr, dc] of dirs) {
    const nr = r + dr, nc = c + dc;
    if (isWalkable(nr, nc)) result.push([nr, nc]);
  }
  return result;
}

// ╔══════════════════════════════════════════════════════════════════════╗
// ║                     GENERATOR PETA ACAK                            ║
// ╚══════════════════════════════════════════════════════════════════════╝

/**
 * Pemeriksaan cepat (BFS) apakah ada jalur dari start ke goal.
 */
function hasValidPath(start, goal, obs) {
  const queue = [start];
  const visited = new Set([key(start[0], start[1])]);
  let head = 0;

  while (head < queue.length) {
    const [r, c] = queue[head++];
    if (r === goal[0] && c === goal[1]) return true;

    for (const [dr, dc] of [[-1,0],[1,0],[0,-1],[0,1]]) {
      const nr = r + dr, nc = c + dc;
      const k = key(nr, nc);
      if (nr >= 0 && nr < GRID_ROWS && nc >= 0 && nc < GRID_COLS && !obs.has(k) && !visited.has(k)) {
        visited.add(k);
        queue.push([nr, nc]);
      }
    }
  }
  return false;
}

/**
 * Menghasilkan peta acak: obstacle, start, goal.
 * Menjamin ada jalur valid dan jarak manhattan start-goal >= MIN_START_GOAL_DIST.
 */
let selectedMapSource = 'procedural'; // 'procedural' | 'map_game' | 'map_update' | 'map_peta'

const MAP_FILES = {
  map_game: 'Peta/MapGame.png',
  map_update: 'Peta/MAP UPDATE 3.1.png',
  map_peta: 'Peta/Gambar peta.png'
};

const MAP_THRESHOLDS = {
  map_game: 150,
  map_update: 150,
  map_peta: 200
};

function loadMapImage(src) {
  return new Promise((resolve, reject) => {
    const img = new Image();
    img.src = src;
    img.onload = () => resolve(img);
    img.onerror = (err) => reject(err);
  });
}

async function generateImageMap(src, threshold) {
  try {
    const img = await loadMapImage(src);
    const tempCanvas = document.createElement('canvas');
    tempCanvas.width = GRID_COLS;
    tempCanvas.height = GRID_ROWS;
    const tempCtx = tempCanvas.getContext('2d');
    tempCtx.drawImage(img, 0, 0, GRID_COLS, GRID_ROWS);

    const imgData = tempCtx.getImageData(0, 0, GRID_COLS, GRID_ROWS);
    const data = imgData.data;

    const obs = new Set();
    for (let r = 0; r < GRID_ROWS; r++) {
      for (let c = 0; c < GRID_COLS; c++) {
        const idx = (r * GRID_COLS + c) * 4;
        const red = data[idx];
        const green = data[idx + 1];
        const blue = data[idx + 2];
        const gray = 0.299 * red + 0.587 * green + 0.114 * blue;
        if (gray < threshold) {
          obs.add(key(r, c));
        }
      }
    }
    return obs;
  } catch (e) {
    console.error('Failed to load map image:', e);
    return new Set();
  }
}

async function generateMap() {
  if (selectedMapSource === 'procedural') {
    generateRandomMap();
    return;
  }

  const src = MAP_FILES[selectedMapSource];
  const threshold = MAP_THRESHOLDS[selectedMapSource];
  const obs = await generateImageMap(src, threshold);

  // Kumpulkan sel kosong
  const emptyCells = [];
  for (let r = 0; r < GRID_ROWS; r++) {
    for (let c = 0; c < GRID_COLS; c++) {
      if (!obs.has(key(r, c))) {
        emptyCells.push([r, c]);
      }
    }
  }

  if (emptyCells.length < 2) {
    obstacles = new Set();
    startNode = [2, 2];
    goalNode = [GRID_ROWS - 3, GRID_COLS - 3];
    return;
  }

  // Cari start & goal yang valid
  let s = null, g = null;
  let found = false;

  // Coba dengan jarak minimal
  for (let attempt = 0; attempt < 1000; attempt++) {
    s = emptyCells[Math.floor(Math.random() * emptyCells.length)];
    g = emptyCells[Math.floor(Math.random() * emptyCells.length)];
    const dist = Math.abs(s[0] - g[0]) + Math.abs(s[1] - g[1]);
    if (dist >= MIN_START_GOAL_DIST && hasValidPath(s, g, obs)) {
      found = true;
      break;
    }
  }

  // Jika gagal, coba tanpa batasan jarak
  if (!found) {
    for (let attempt = 0; attempt < 1000; attempt++) {
      s = emptyCells[Math.floor(Math.random() * emptyCells.length)];
      g = emptyCells[Math.floor(Math.random() * emptyCells.length)];
      if (s[0] !== g[0] || s[1] !== g[1]) {
        if (hasValidPath(s, g, obs)) {
          found = true;
          break;
        }
      }
    }
  }

  if (found) {
    obstacles = obs;
    startNode = s;
    goalNode = g;
  } else {
    // Fallback
    obstacles = new Set();
    startNode = [2, 2];
    goalNode = [GRID_ROWS - 3, GRID_COLS - 3];
  }
}

/**
 * Menghasilkan peta acak: obstacle, start, goal.
 * Menjamin ada jalur valid dan jarak manhattan start-goal >= MIN_START_GOAL_DIST.
 */
function generateRandomMap() {
  for (let attempt = 0; attempt < 200; attempt++) {
    const obs = new Set();
    const numObs = Math.floor(GRID_ROWS * GRID_COLS * OBSTACLE_DENSITY);

    while (obs.size < numObs) {
      const r = Math.floor(Math.random() * GRID_ROWS);
      const c = Math.floor(Math.random() * GRID_COLS);
      obs.add(key(r, c));
    }

    // Kumpulkan sel kosong
    const emptyCells = [];
    for (let r = 0; r < GRID_ROWS; r++) {
      for (let c = 0; c < GRID_COLS; c++) {
        if (!obs.has(key(r, c))) emptyCells.push([r, c]);
      }
    }
    if (emptyCells.length < 2) continue;

    // Pilih start & goal dengan jarak cukup jauh
    let s, g, dist;
    let found = false;
    for (let i = 0; i < 100; i++) {
      s = emptyCells[Math.floor(Math.random() * emptyCells.length)];
      g = emptyCells[Math.floor(Math.random() * emptyCells.length)];
      dist = Math.abs(s[0] - g[0]) + Math.abs(s[1] - g[1]);
      if (dist >= MIN_START_GOAL_DIST) { found = true; break; }
    }
    if (!found) continue;

    obs.delete(key(s[0], s[1]));
    obs.delete(key(g[0], g[1]));

    if (hasValidPath(s, g, obs)) {
      obstacles = obs;
      startNode = s;
      goalNode = g;
      return;
    }
  }
  // Fallback: empty map
  obstacles = new Set();
  startNode = [2, 2];
  goalNode = [GRID_ROWS - 3, GRID_COLS - 3];
}

// ╔══════════════════════════════════════════════════════════════════════╗
// ║               IMPLEMENTASI ALGORITMA PATHFINDING                   ║
// ╚══════════════════════════════════════════════════════════════════════╝

// ── MinHeap sederhana ──
class MinHeap {
  constructor() { this.data = []; }
  push(item) {
    this.data.push(item);
    this._bubbleUp(this.data.length - 1);
  }
  pop() {
    const top = this.data[0];
    const last = this.data.pop();
    if (this.data.length > 0) { this.data[0] = last; this._sinkDown(0); }
    return top;
  }
  get length() { return this.data.length; }
  _bubbleUp(i) {
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (this.data[i][0] < this.data[p][0]) { [this.data[i], this.data[p]] = [this.data[p], this.data[i]]; i = p; }
      else break;
    }
  }
  _sinkDown(i) {
    const n = this.data.length;
    while (true) {
      let s = i, l = 2*i+1, r = 2*i+2;
      if (l < n && this.data[l][0] < this.data[s][0]) s = l;
      if (r < n && this.data[r][0] < this.data[s][0]) s = r;
      if (s !== i) { [this.data[i], this.data[s]] = [this.data[s], this.data[i]]; i = s; }
      else break;
    }
  }
}

/** Heuristik Manhattan Distance */
function heuristic(r1, c1, r2, c2) {
  return Math.abs(r1 - r2) + Math.abs(c1 - c2);
}

/**
 * Generator A* — menghasilkan state per langkah untuk visualisasi.
 * Setiap yield: { current, openSet, closedSet, cameFrom }
 */
function* astarGen(start, goal) {
  const heap = new MinHeap();
  const h0 = heuristic(start[0], start[1], goal[0], goal[1]);
  heap.push([h0, 0, start]);
  const openNodes = new Set([key(start[0], start[1])]);
  const closed = new Set();
  const cameFrom = {};
  const gScore = { [key(start[0], start[1])]: 0 };

  while (heap.length) {
    const [, , curr] = heap.pop();
    const ck = key(curr[0], curr[1]);
    openNodes.delete(ck);
    if (closed.has(ck)) continue;
    closed.add(ck);

    yield { current: curr, openSet: new Set(openNodes), closedSet: new Set(closed), cameFrom: {...cameFrom} };
    if (curr[0] === goal[0] && curr[1] === goal[1]) return;

    for (const [nr, nc] of getNeighbors(curr[0], curr[1])) {
      const nk = key(nr, nc);
      if (closed.has(nk)) continue;
      const tg = gScore[ck] + 1;
      if (tg < (gScore[nk] ?? Infinity)) {
        cameFrom[nk] = ck;
        gScore[nk] = tg;
        const f = tg + heuristic(nr, nc, goal[0], goal[1]);
        heap.push([f, tg, [nr, nc]]);
        openNodes.add(nk);
      }
    }
  }
}

/** Generator Dijkstra */
function* dijkstraGen(start, goal) {
  const heap = new MinHeap();
  heap.push([0, 0, start]);
  const openNodes = new Set([key(start[0], start[1])]);
  const closed = new Set();
  const cameFrom = {};
  const gScore = { [key(start[0], start[1])]: 0 };

  while (heap.length) {
    const [, , curr] = heap.pop();
    const ck = key(curr[0], curr[1]);
    openNodes.delete(ck);
    if (closed.has(ck)) continue;
    closed.add(ck);

    yield { current: curr, openSet: new Set(openNodes), closedSet: new Set(closed), cameFrom: {...cameFrom} };
    if (curr[0] === goal[0] && curr[1] === goal[1]) return;

    for (const [nr, nc] of getNeighbors(curr[0], curr[1])) {
      const nk = key(nr, nc);
      if (closed.has(nk)) continue;
      const tg = gScore[ck] + 1;
      if (tg < (gScore[nk] ?? Infinity)) {
        cameFrom[nk] = ck;
        gScore[nk] = tg;
        heap.push([tg, 0, [nr, nc]]);
        openNodes.add(nk);
      }
    }
  }
}

/** Generator BFS */
function* bfsGen(start, goal) {
  const queue = [start];
  const openNodes = new Set([key(start[0], start[1])]);
  const closed = new Set();
  const cameFrom = {};
  let head = 0;

  while (head < queue.length) {
    const curr = queue[head++];
    const ck = key(curr[0], curr[1]);
    openNodes.delete(ck);
    closed.add(ck);

    yield { current: curr, openSet: new Set(openNodes), closedSet: new Set(closed), cameFrom: {...cameFrom} };
    if (curr[0] === goal[0] && curr[1] === goal[1]) return;

    for (const [nr, nc] of getNeighbors(curr[0], curr[1])) {
      const nk = key(nr, nc);
      if (closed.has(nk) || openNodes.has(nk)) continue;
      cameFrom[nk] = ck;
      queue.push([nr, nc]);
      openNodes.add(nk);
    }
  }
}

/** Rekonstruksi jalur dari cameFrom map */
function reconstructPath(cameFrom, start, goal) {
  const sk = key(start[0], start[1]);
  const gk = key(goal[0], goal[1]);
  if (!(gk in cameFrom) && sk !== gk) return [];
  const path = [];
  let curr = gk;
  while (curr !== sk) {
    path.push(parseKey(curr));
    curr = cameFrom[curr];
    if (!curr) return []; // safety
  }
  path.push(start);
  path.reverse();
  return path;
}

/** Pencarian instan (non-visual) untuk perbandingan performa */
function runInstantSearch(algoName) {
  const start = startNode, goal = goalNode;
  const t0 = performance.now();

  let gen;
  if (algoName === 'A*') gen = astarGen(start, goal);
  else if (algoName === 'Dijkstra') gen = dijkstraGen(start, goal);
  else gen = bfsGen(start, goal);

  let lastState = null;
  for (const state of gen) { lastState = state; }
  const elapsed = performance.now() - t0;

  let path = [];
  if (lastState) {
    path = reconstructPath(lastState.cameFrom, start, goal);
  }

  return {
    found: path.length > 0,
    path,
    pathLength: path.length > 0 ? path.length - 1 : 0,
    visitedCount: lastState ? lastState.closedSet.size : 0,
    timeMs: elapsed,
  };
}

// ╔══════════════════════════════════════════════════════════════════════╗
// ║                        RENDERING CANVAS                            ║
// ╚══════════════════════════════════════════════════════════════════════╝

function drawGrid() {
  const w = GRID_COLS * CELL_SIZE;
  const h = GRID_ROWS * CELL_SIZE;
  ctx.clearRect(0, 0, w, h);

  for (let r = 0; r < GRID_ROWS; r++) {
    for (let c = 0; c < GRID_COLS; c++) {
      const x = c * CELL_SIZE;
      const y = r * CELL_SIZE;
      const k = key(r, c);

      // Base color
      let color = obstacles.has(k) ? COLORS.wall : COLORS.empty;

      // Visualization overlay
      if (simState !== 'idle') {
        if (pathVis.some(p => p[0] === r && p[1] === c)) {
          color = COLORS.path;
        } else if (closedSetVis.has(k)) {
          color = COLORS.closed;
        } else if (openSetVis.has(k)) {
          color = COLORS.open;
        }
      }

      // Start & Goal override
      if (startNode && r === startNode[0] && c === startNode[1]) color = COLORS.start;
      if (goalNode && r === goalNode[0] && c === goalNode[1]) color = COLORS.goal;

      ctx.fillStyle = color;
      ctx.fillRect(x, y, CELL_SIZE, CELL_SIZE);

      // Grid line
      ctx.strokeStyle = COLORS.gridLine;
      ctx.lineWidth = 0.5;
      ctx.strokeRect(x, y, CELL_SIZE, CELL_SIZE);
    }
  }

  // Current node highlight
  if (currentNode && simState === 'searching') {
    const cx = currentNode[1] * CELL_SIZE;
    const cy = currentNode[0] * CELL_SIZE;
    ctx.fillStyle = COLORS.current;
    ctx.fillRect(cx, cy, CELL_SIZE, CELL_SIZE);
  }

  // Draw courier
  if (simState === 'animating' || simState === 'done') {
    drawCourier();
  }

  // Start label "S" & Goal label "G"
  if (startNode) {
    drawCellLabel(startNode, 'S', '#0E0E1A');
  }
  if (goalNode) {
    drawCellLabel(goalNode, 'G', '#FFF');
  }
}

function drawCellLabel(node, label, color) {
  const x = node[1] * CELL_SIZE + CELL_SIZE / 2;
  const y = node[0] * CELL_SIZE + CELL_SIZE / 2;
  ctx.fillStyle = color;
  ctx.font = `bold ${Math.round(CELL_SIZE * 0.6)}px 'Inter', sans-serif`;
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText(label, x, y);
}

function drawCourier() {
  const px = courierX + 1;
  const py = courierY + 1;
  const sz = CELL_SIZE - 2;

  if (courierImgLoaded) {
    ctx.drawImage(courierImg, px, py, sz, sz);
  } else {
    // Fallback: lingkaran emas
    ctx.fillStyle = COLORS.courierBg;
    ctx.beginPath();
    ctx.arc(px + sz / 2, py + sz / 2, sz / 2, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = '#0E0E1A';
    ctx.font = `bold ${Math.round(sz * 0.55)}px 'Inter', sans-serif`;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText('K', px + sz / 2, py + sz / 2);
  }
}

// ╔══════════════════════════════════════════════════════════════════════╗
// ║                     SIMULASI & ANIMASI                             ║
// ╚══════════════════════════════════════════════════════════════════════╝

/**
 * Fungsi sleep yang bisa dibatalkan (abort-aware).
 */
function sleep(ms, signal) {
  if (ms <= 0) return Promise.resolve();
  return new Promise((resolve, reject) => {
    const timer = setTimeout(resolve, ms);
    if (signal) {
      signal.addEventListener('abort', () => {
        clearTimeout(timer);
        reject(new DOMException('Aborted', 'AbortError'));
      });
    }
  });
}

/**
 * Menjalankan simulasi pencarian jalur dengan visualisasi langkah-demi-langkah.
 */
async function runSimulation() {
  // Abort simulasi sebelumnya jika ada
  if (animAbort) animAbort.abort();
  animAbort = new AbortController();
  const signal = animAbort.signal;

  // Reset visual state
  openSetVis = new Set();
  closedSetVis = new Set();
  pathVis = [];
  currentNode = null;
  statPathLen = 0;
  statVisited = 0;
  statTimeMs = 0;
  courierIdx = 0;

  simState = 'searching';
  updateUI();

  const delayMs = SPEED_MAP[selectedSpeed];
  const start = startNode, goal = goalNode;

  let gen;
  if (selectedAlgo === 'astar') gen = astarGen(start, goal);
  else if (selectedAlgo === 'dijkstra') gen = dijkstraGen(start, goal);
  else gen = bfsGen(start, goal);

  const t0 = performance.now();
  let lastCameFrom = {};

  try {
    if (delayMs === 0) {
      // Instan — jalankan semua langkah sekaligus
      let lastState = null;
      for (const state of gen) { lastState = state; }
      statTimeMs = performance.now() - t0;

      if (lastState) {
        openSetVis = lastState.openSet;
        closedSetVis = lastState.closedSet;
        lastCameFrom = lastState.cameFrom;
        currentNode = lastState.current;
        statVisited = lastState.closedSet.size;
      }
    } else {
      // Visualisasi langkah per langkah
      let stepsPerFrame = delayMs <= 5 ? 4 : 1;

      for (const state of gen) {
        if (signal.aborted) throw new DOMException('Aborted', 'AbortError');

        openSetVis = state.openSet;
        closedSetVis = state.closedSet;
        currentNode = state.current;
        lastCameFrom = state.cameFrom;
        statVisited = state.closedSet.size;

        drawGrid();
        updateStats();

        await sleep(delayMs, signal);
      }
      statTimeMs = performance.now() - t0;
    }
  } catch (e) {
    if (e.name === 'AbortError') return; // Dibatalkan, keluar
    throw e;
  }

  // ── Path Found ──
  simState = 'path_found';
  const path = reconstructPath(lastCameFrom, start, goal);
  pathVis = path;
  statPathLen = path.length > 0 ? path.length - 1 : 0;
  updateUI();
  drawGrid();

  if (path.length === 0) {
    simState = 'done';
    updateUI();
    return;
  }

  // ── Animasi kurir ──
  try {
    await sleep(300, signal);
  } catch { return; }

  simState = 'animating';
  updateUI();

  courierIdx = 0;
  courierX = path[0][1] * CELL_SIZE;
  courierY = path[0][0] * CELL_SIZE;

  try {
    for (let i = 0; i < path.length - 1; i++) {
      if (signal.aborted) throw new DOMException('Aborted', 'AbortError');

      const from = path[i];
      const to = path[i + 1];
      const fromX = from[1] * CELL_SIZE, fromY = from[0] * CELL_SIZE;
      const toX = to[1] * CELL_SIZE, toY = to[0] * CELL_SIZE;

      // Interpolasi halus dalam beberapa frame
      const LERP_STEPS = 6;
      for (let s = 1; s <= LERP_STEPS; s++) {
        if (signal.aborted) throw new DOMException('Aborted', 'AbortError');
        const t = s / LERP_STEPS;
        courierX = fromX + (toX - fromX) * t;
        courierY = fromY + (toY - fromY) * t;
        drawGrid();
        await sleep(18, signal);
      }
      courierIdx = i + 1;
    }
  } catch (e) {
    if (e.name === 'AbortError') return;
    throw e;
  }

  simState = 'done';
  updateUI();
  drawGrid();
}

// ╔══════════════════════════════════════════════════════════════════════╗
// ║                         UI UPDATES                                 ║
// ╚══════════════════════════════════════════════════════════════════════╝

function updateUI() {
  const dot = document.getElementById('statusIndicator');
  const label = document.getElementById('statusText');
  const statEl = document.getElementById('statStatus');

  dot.className = 'status-dot';
  switch (simState) {
    case 'idle':
      dot.classList.add('idle');
      label.textContent = 'Siap';
      statEl.textContent = 'Siap';
      statEl.className = 'stat-value status-idle';
      break;
    case 'searching':
      dot.classList.add('searching');
      label.textContent = 'Mencari rute...';
      statEl.textContent = 'Mencari...';
      statEl.className = 'stat-value status-search';
      break;
    case 'path_found':
      dot.classList.add('found');
      label.textContent = 'Rute ditemukan!';
      statEl.textContent = 'Rute ditemukan!';
      statEl.className = 'stat-value status-found';
      break;
    case 'animating':
      dot.classList.add('courier');
      label.textContent = 'Kurir mengantar...';
      statEl.textContent = 'Mengantar...';
      statEl.className = 'stat-value status-courier';
      break;
    case 'done':
      dot.classList.add('found');
      label.textContent = statPathLen > 0 ? 'Selesai' : 'Jalur terblokir!';
      statEl.textContent = statPathLen > 0 ? 'Selesai' : 'Terblokir!';
      statEl.className = `stat-value ${statPathLen > 0 ? 'status-done' : 'status-idle'}`;
      break;
  }

  updateStats();
  updateButtonStates();
}

function updateStats() {
  document.getElementById('statPathLen').textContent = statPathLen > 0 ? `${statPathLen} langkah` : '—';
  document.getElementById('statVisited').textContent = statVisited > 0 ? `${statVisited} node` : '—';
  document.getElementById('statTime').textContent = statTimeMs > 0 ? `${statTimeMs.toFixed(3)} ms` : '—';
}

function updateButtonStates() {
  const isRunning = simState === 'searching' || simState === 'animating' || simState === 'path_found';
  document.getElementById('btnStart').disabled = isRunning || simState === 'done';
  document.getElementById('btnRandomize').disabled = isRunning;
  document.getElementById('btnCompare').disabled = isRunning;
}

// ╔══════════════════════════════════════════════════════════════════════╗
// ║                     PERBANDINGAN MODAL                             ║
// ╚══════════════════════════════════════════════════════════════════════╝

function showComparison() {
  const results = {
    'BFS':      runInstantSearch('BFS'),
    'Dijkstra': runInstantSearch('Dijkstra'),
    'A*':       runInstantSearch('A*'),
  };

  const tbody = document.getElementById('compareTableBody');
  tbody.innerHTML = '';

  const classMap = { 'BFS': 'algo-name-bfs', 'Dijkstra': 'algo-name-dijkstra', 'A*': 'algo-name-astar' };

  let bestAlgo = '', bestVisited = Infinity;

  for (const [name, res] of Object.entries(results)) {
    if (res.found && res.visitedCount < bestVisited) {
      bestVisited = res.visitedCount;
      bestAlgo = name;
    }
  }

  for (const [name, res] of Object.entries(results)) {
    const tr = document.createElement('tr');
    const isBest = name === bestAlgo;

    tr.innerHTML = `
      <td class="${classMap[name]}">${name}</td>
      <td class="${res.found ? 'badge-found' : 'badge-blocked'}">${res.found ? 'Ditemukan' : 'Terblokir'}</td>
      <td>${res.pathLength} langkah</td>
      <td>${res.visitedCount} node</td>
      <td>${res.timeMs.toFixed(4)} ms</td>
      <td>${isBest ? '<span class="badge-best">BEST</span>' : ''}</td>
    `;
    tbody.appendChild(tr);
  }

  // Kesimpulan dinamis
  const concl = document.getElementById('compareConclusion');
  const rA = results['A*'], rD = results['Dijkstra'], rB = results['BFS'];
  let html = '<strong>📊 Analisis Kesimpulan:</strong><br>';

  if (rA.found) {
    html += `• A* dan Dijkstra menemukan rute optimal <strong>${rA.pathLength} langkah</strong>.<br>`;
    const diff = rD.visitedCount - rA.visitedCount;
    html += `• A* mengeksplorasi <strong>${rA.visitedCount} node</strong> (hemat <strong>${diff} node</strong> vs Dijkstra) berkat heuristik Manhattan.<br>`;

    if (rB.pathLength > rA.pathLength) {
      html += `• BFS menghasilkan rute <strong>${rB.pathLength} langkah</strong> — tidak dijamin optimal pada graph berbobot.<br>`;
    } else {
      html += `• BFS menemukan panjang rute sama, namun menjelajahi <strong>${rB.visitedCount} node</strong> (lebih banyak).<br>`;
    }
    html += `<br><strong>Kesimpulan: ${bestAlgo} adalah algoritma paling efisien</strong> karena meminimalkan jumlah node yang dieksplorasi.`;
  } else {
    html += 'Jalur terblokir. Tidak ada jalur dari kurir ke pelanggan.';
  }
  concl.innerHTML = html;

  document.getElementById('comparisonOverlay').classList.remove('hidden');
}

// ╔══════════════════════════════════════════════════════════════════════╗
// ║                      EVENT LISTENERS                               ║
// ╚══════════════════════════════════════════════════════════════════════╝

async function resetSimulation(keepMap = true) {
  if (animAbort) animAbort.abort();
  simState = 'idle';
  openSetVis = new Set();
  closedSetVis = new Set();
  pathVis = [];
  currentNode = null;
  statPathLen = 0;
  statVisited = 0;
  statTimeMs = 0;
  courierIdx = 0;

  if (!keepMap) {
    await generateMap();
  }

  updateUI();
  drawGrid();
}

async function initApp() {
  canvas = document.getElementById('gridCanvas');
  canvas.width = GRID_COLS * CELL_SIZE;
  canvas.height = GRID_ROWS * CELL_SIZE;
  ctx = canvas.getContext('2d');

  // Generate peta awal
  await generateMap();
  drawGrid();
  updateUI();

  // ── Tombol Algoritma ──
  document.querySelectorAll('.algo-btn').forEach(btn => {
    btn.addEventListener('click', () => {
      if (simState === 'searching' || simState === 'animating') return;
      document.querySelectorAll('.algo-btn').forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      selectedAlgo = btn.dataset.algo;
    });
  });

  // ── Tombol Kecepatan ──
  document.querySelectorAll('.speed-btn').forEach(btn => {
    btn.addEventListener('click', () => {
      document.querySelectorAll('.speed-btn').forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      selectedSpeed = btn.dataset.speed;
    });
  });

  // ── Tombol Peta ──
  document.querySelectorAll('.map-btn').forEach(btn => {
    btn.addEventListener('click', async () => {
      if (simState === 'searching' || simState === 'animating') return;
      document.querySelectorAll('.map-btn').forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      selectedMapSource = btn.dataset.map;
      await resetSimulation(false);
    });
  });

  // ── Tombol Kontrol ──
  document.getElementById('btnStart').addEventListener('click', () => {
    if (simState === 'idle') {
      runSimulation();
    }
  });

  document.getElementById('btnReset').addEventListener('click', () => {
    resetSimulation(true);
  });

  document.getElementById('btnRandomize').addEventListener('click', () => {
    resetSimulation(false);
  });

  document.getElementById('btnCompare').addEventListener('click', () => {
    showComparison();
  });

  // ── Modal ──
  document.getElementById('btnCloseModal').addEventListener('click', () => {
    document.getElementById('comparisonOverlay').classList.add('hidden');
  });
  document.getElementById('comparisonOverlay').addEventListener('click', (e) => {
    if (e.target === e.currentTarget) {
      e.currentTarget.classList.add('hidden');
    }
  });

  // ── Klik pada Canvas untuk memindahkan Start / Goal ──
  canvas.addEventListener('click', (e) => {
    if (simState !== 'idle') return;

    const rect = canvas.getBoundingClientRect();
    const scaleX = canvas.width / rect.width;
    const scaleY = canvas.height / rect.height;
    const mx = (e.clientX - rect.left) * scaleX;
    const my = (e.clientY - rect.top) * scaleY;
    const col = Math.floor(mx / CELL_SIZE);
    const row = Math.floor(my / CELL_SIZE);

    if (row < 0 || row >= GRID_ROWS || col < 0 || col >= GRID_COLS) return;
    if (obstacles.has(key(row, col))) return;

    if (e.shiftKey) {
      // Shift + Click → Pindahkan Goal
      if (!(row === startNode[0] && col === startNode[1])) {
        goalNode = [row, col];
      }
    } else {
      // Click → Pindahkan Start
      if (!(row === goalNode[0] && col === goalNode[1])) {
        startNode = [row, col];
      }
    }
    drawGrid();
  });

  // Update label grid size
  document.getElementById('gridSizeLabel').textContent = `Grid: ${GRID_ROWS}×${GRID_COLS}`;
}

// ── Mulai Aplikasi ──
document.addEventListener('DOMContentLoaded', initApp);