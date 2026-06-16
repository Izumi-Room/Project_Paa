"""
smart_courier.py
================
Smart Courier Pathfinding Simulator — Proyek Mata Kuliah PAA
(Perancangan dan Analisis Algoritma)

Fitur:
  - Peta berbasis Grid dengan rintangan (Obstacles) yang diacak otomatis.
  - Menjamin selalu ada jalur yang valid pada setiap acakan peta (Path Validation).
  - Implementasi tiga algoritma pencarian jalur:
    1. Breadth First Search (BFS)
    2. Dijkstra (Uniform Cost Search pada grid)
    3. A* (A-Star) dengan Heuristik Manhattan Distance
  - Visualisasi pencarian langkah-demi-langkah dengan kecepatan yang bisa diatur.
  - Animasi kurir bergerak menyusuri jalur yang ditemukan menggunakan gambar Kurir.png.
  - Panel perbandingan performa langsung (Panjang Jalur, Node Dikunjungi, Waktu Eksekusi).
  - Tema UI gelap (Dark Mode) premium khas antarmuka modern.

Cara Menjalankan:
  pip install pygame
  python smart_courier.py
"""

import pygame
import sys
import math
import heapq
import time
import random
from collections import deque

# ╔══════════════════════════════════════════════════════════════════════╗
# ║                     KONFIGURASI & KONSTANTA                        ║
# ╚══════════════════════════════════════════════════════════════════════╝

WINDOW_WIDTH = 1000
WINDOW_HEIGHT = 650
FPS = 60

# Layout Grid & Panel
GRID_ROWS = 26
GRID_COLS = 28
CELL_SIZE = 25  # 28 * 25 = 700 px lebar peta

MAP_W = GRID_COLS * CELL_SIZE
MAP_H = GRID_ROWS * CELL_SIZE

PANEL_X = MAP_W
PANEL_Y = 0
PANEL_W = WINDOW_WIDTH - MAP_W
PANEL_H = WINDOW_HEIGHT

# Palet Warna (Premium Dark Cyberpunk Theme)
COLOR_BG          = (15, 15, 25)        # Latar belakang utama
COLOR_BG_PANEL    = (22, 22, 37)        # Sidebar panel
COLOR_GRID_LINE   = (28, 28, 48)        # Garis grid samar
COLOR_CELL_EMPTY  = (20, 20, 32)        # Sel jalan kosong
COLOR_CELL_WALL   = (45, 45, 60)        # Sel obstacle (tembok)

COLOR_SOURCE      = (255, 190, 40)      # Kuning emas (Source)
COLOR_DEST        = (255, 80, 80)       # Merah neon (Destination)
COLOR_OPEN        = (40, 160, 220)      # Biru neon (Open set / Frontier)
COLOR_CLOSED      = (110, 50, 180)      # Ungu magenta (Closed set / Visited)
COLOR_PATH        = (46, 204, 113)      # Hijau emerald (Jalur akhir)

COLOR_TEXT_WHITE  = (240, 240, 250)
COLOR_TEXT_GRAY   = (155, 160, 180)
COLOR_TEXT_DIM    = (95, 100, 120)
COLOR_TEXT_GOLD   = (255, 190, 40)
COLOR_TEXT_GREEN  = (46, 204, 113)
COLOR_TEXT_RED    = (255, 80, 80)

COLOR_BTN_NORMAL  = (38, 38, 58)
COLOR_BTN_HOVER   = (60, 60, 85)
COLOR_BTN_RUN     = (46, 204, 113)
COLOR_BTN_RESET   = (231, 76, 60)
COLOR_BTN_COMPARE = (155, 89, 182)

# Kecepatan Visualisasi (Delay milidetik per langkah)
SPEED_SLOW = 80
SPEED_MEDIUM = 20
SPEED_FAST = 2
SPEED_INSTANT = 0

# ╔══════════════════════════════════════════════════════════════════════╗
# ║                     FUNGSI GENERATOR PETA                          ║
# ╚══════════════════════════════════════════════════════════════════════╝

def has_valid_path(rows, cols, obstacles, start, goal):
    """
    Melakukan pemeriksaan cepat dengan BFS untuk memastikan ada jalur 
    yang menghubungkan titik start dan goal di dalam grid.
    """
    queue = deque([start])
    visited = {start}
    
    while queue:
        curr = queue.popleft()
        if curr == goal:
            return True
            
        r, c = curr
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols:
                if (nr, nc) not in obstacles and (nr, nc) not in visited:
                    visited.add((nr, nc))
                    queue.append((nr, nc))
    return False

def load_map_from_image(image_path, rows, cols, threshold=150):
    """
    Memuat peta dari gambar PNG. Mengubah ke grayscale, downsample ke grid rows x cols,
    dan mengembalikan set berisi koordinat rintangan (r, c).
    """
    try:
        from PIL import Image
        im = Image.open(image_path).convert('L')
        im_small = im.resize((cols, rows), Image.NEAREST)
        obstacles = set()
        for r in range(rows):
            for c in range(cols):
                val = im_small.getpixel((c, r))
                if val < threshold:
                    obstacles.add((r, c))
        return obstacles
    except Exception as e:
        print(f"Error loading map image {image_path}: {e}")
        return set()

def generate_random_map(rows, cols, obstacle_density=0.25):
    """
    Membuat grid secara acak: rintangan, titik asal (start), dan titik tujuan (goal).
    Menjamin ada jalur yang menghubungkan asal dan tujuan menggunakan BFS check.
    """
    while True:
        obstacles = set()
        # Isi obstacle secara acak
        num_obstacles = int(rows * cols * obstacle_density)
        while len(obstacles) < num_obstacles:
            r = random.randint(0, rows - 1)
            c = random.randint(0, cols - 1)
            obstacles.add((r, c))
            
        # Tentukan start dan goal di area kosong
        empty_cells = [(r, c) for r in range(rows) for c in range(cols) if (r, c) not in obstacles]
        if len(empty_cells) < 2:
            continue
            
        # Pilih start & goal dengan jarak minimal Manhattan > 12 agar visualisasinya menarik
        valid_pair = False
        attempts = 0
        while not valid_pair and attempts < 100:
            start = random.choice(empty_cells)
            goal = random.choice(empty_cells)
            dist = abs(start[0] - goal[0]) + abs(start[1] - goal[1])
            if start != goal and dist >= 12:
                valid_pair = True
            attempts += 1
            
        if not valid_pair:
            continue
            
        # Hapus start dan goal dari obstacle (safety net)
        obstacles.discard(start)
        obstacles.discard(goal)
        
        # Validasi konektivitas jalur
        if has_valid_path(rows, cols, obstacles, start, goal):
            return obstacles, start, goal

def generate_selected_map(rows, cols, map_source="procedural"):
    """
    Menghasilkan konfigurasi peta (obstacles, start, goal) berdasarkan
    sumber peta yang dipilih (procedural, map_game, map_update, map_peta).
    """
    if map_source == "procedural":
        return generate_random_map(rows, cols)
    
    if map_source == "map_game":
        obstacles = load_map_from_image("Peta/MapGame.png", rows, cols, threshold=150)
    elif map_source == "map_update":
        obstacles = load_map_from_image("Peta/MAP UPDATE 3.1.png", rows, cols, threshold=150)
    elif map_source == "map_peta":
        obstacles = load_map_from_image("Peta/Gambar peta.png", rows, cols, threshold=200)
    else:
        obstacles = set()

    empty_cells = [(r, c) for r in range(rows) for c in range(cols) if (r, c) not in obstacles]
    if len(empty_cells) < 2:
        return set(), (0, 0), (rows - 1, cols - 1)

    # Cari start & goal yang valid dengan jarak minimal Manhattan >= 10
    found = False
    start, goal = None, None
    for attempt in range(1000):
        start = random.choice(empty_cells)
        goal = random.choice(empty_cells)
        dist = abs(start[0] - goal[0]) + abs(start[1] - goal[1])
        if start != goal and dist >= 10:
            if has_valid_path(rows, cols, obstacles, start, goal):
                found = True
                break

    # Jika tidak ketemu, cari start & goal apa saja yang valid
    if not found:
        for attempt in range(1000):
            start = random.choice(empty_cells)
            goal = random.choice(empty_cells)
            if start != goal:
                if has_valid_path(rows, cols, obstacles, start, goal):
                    found = True
                    break

    if found:
        obstacles.discard(start)
        obstacles.discard(goal)
        return obstacles, start, goal
    else:
        return set(), (0, 0), (rows - 1, cols - 1)

# ╔══════════════════════════════════════════════════════════════════════╗
# ║                  IMPLEMENTASI ALGORITMA PATHFINDING                ║
# ╚══════════════════════════════════════════════════════════════════════╝

def heuristic_manhattan(p1, p2):
    """Heuristik Manhattan Distance (sangat cocok untuk pergerakan grid 4 arah)."""
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

# --- 1. A* Generator (untuk Visualisasi Langkah-demi-Langkah) ---
def astar_generator(rows, cols, obstacles, start, goal):
    """
    Generator untuk algoritma A*. Setiap iterasi menghasilkan:
    (current_node, open_set_dict, closed_set_set, came_from)
    """
    # Open set menyimpan (f_score, h_score, counter, node)
    counter = 0
    open_heap = [(heuristic_manhattan(start, goal), heuristic_manhattan(start, goal), counter, start)]
    open_set_nodes = {start}
    
    closed_set = set()
    came_from = {}
    
    g_score = {start: 0}
    
    while open_heap:
        _, _, _, current = heapq.heappop(open_heap)
        if current in open_set_nodes:
            open_set_nodes.remove(current)
            
        if current in closed_set:
            continue
            
        closed_set.add(current)
        yield current, open_set_nodes, closed_set, came_from
        
        if current == goal:
            break
            
        r, c = current
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            neighbor = (r + dr, c + dc)
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
                if neighbor in obstacles or neighbor in closed_set:
                    continue
                    
                # Bobot langkah grid selalu = 1
                tentative_g = g_score[current] + 1
                
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    h = heuristic_manhattan(neighbor, goal)
                    f = tentative_g + h
                    counter += 1
                    heapq.heappush(open_heap, (f, h, counter, neighbor))
                    open_set_nodes.add(neighbor)

# --- 2. Dijkstra Generator ---
def dijkstra_generator(rows, cols, obstacles, start, goal):
    """
    Generator untuk Dijkstra. Sama seperti A* tapi f(n) = g(n) saja (tanpa heuristik).
    """
    counter = 0
    open_heap = [(0, counter, start)]
    open_set_nodes = {start}
    
    closed_set = set()
    came_from = {}
    g_score = {start: 0}
    
    while open_heap:
        curr_g, _, current = heapq.heappop(open_heap)
        if current in open_set_nodes:
            open_set_nodes.remove(current)
            
        if current in closed_set:
            continue
            
        closed_set.add(current)
        yield current, open_set_nodes, closed_set, came_from
        
        if current == goal:
            break
            
        r, c = current
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            neighbor = (r + dr, c + dc)
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
                if neighbor in obstacles or neighbor in closed_set:
                    continue
                    
                tentative_g = g_score[current] + 1
                
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    counter += 1
                    heapq.heappush(open_heap, (tentative_g, counter, neighbor))
                    open_set_nodes.add(neighbor)

# --- 3. BFS Generator ---
def bfs_generator(rows, cols, obstacles, start, goal):
    """
    Generator untuk BFS. Menggunakan antrian FIFO (deque) untuk eksplorasi bertingkat.
    """
    queue = deque([start])
    open_set_nodes = {start}
    closed_set = set()
    came_from = {}
    
    while queue:
        current = queue.popleft()
        if current in open_set_nodes:
            open_set_nodes.remove(current)
            
        closed_set.add(current)
        yield current, open_set_nodes, closed_set, came_from
        
        if current == goal:
            break
            
        r, c = current
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            neighbor = (r + dr, c + dc)
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
                if neighbor in obstacles or neighbor in closed_set or neighbor in open_set_nodes:
                    continue
                    
                came_from[neighbor] = current
                queue.append(neighbor)
                open_set_nodes.add(neighbor)

# --- Pencarian Instan (Untuk Perbandingan Performa) ---
def run_instant_search(algo_name, rows, cols, obstacles, start, goal):
    """
    Menjalankan algoritma secara sinkron dan instan untuk mengukur waktu 
    dan statistik secara akurat.
    """
    start_time = time.perf_counter()
    
    if algo_name == "A*":
        gen = astar_generator(rows, cols, obstacles, start, goal)
    elif algo_name == "Dijkstra":
        gen = dijkstra_generator(rows, cols, obstacles, start, goal)
    else: # BFS
        gen = bfs_generator(rows, cols, obstacles, start, goal)
        
    last_state = None
    for state in gen:
        last_state = state
        
    exec_time = (time.perf_counter() - start_time) * 1000.0  # Konversi ke ms
    
    path = []
    if last_state:
        _, _, closed_set, came_from = last_state
        if goal in came_from or start == goal:
            curr = goal
            while curr != start:
                path.append(curr)
                curr = came_from[curr]
            path.append(start)
            path.reverse()
            
    return {
        'found': len(path) > 0,
        'path': path,
        'path_length': len(path) - 1 if path else 0,
        'visited_count': len(closed_set) if last_state else 0,
        'time_ms': exec_time
    }

# ╔══════════════════════════════════════════════════════════════════════╗
# ║                          KOMPONEN UI                                 ║
# ╚══════════════════════════════════════════════════════════════════════╝

class Button:
    """Tombol interaktif UI Pygame dengan efek hover."""
    def __init__(self, x, y, w, h, text, color, hover_color=None, active_color=None):
        self.rect = pygame.Rect(x, y, w, h)
        self.text = text
        self.color = color
        self.hover_color = hover_color or COLOR_BTN_HOVER
        self.active_color = active_color or COLOR_TEXT_GOLD
        self.hovered = False
        self.active = False
        self.disabled = False

    def update(self, mouse_pos):
        if self.disabled:
            self.hovered = False
        else:
            self.hovered = self.rect.collidepoint(mouse_pos)

    def draw(self, surface, font):
        if self.disabled:
            bg = (30, 30, 42)
            txt_color = COLOR_TEXT_DIM
        elif self.active:
            bg = self.active_color
            txt_color = COLOR_BG
        elif self.hovered:
            bg = self.hover_color
            txt_color = COLOR_TEXT_WHITE
        else:
            bg = self.color
            txt_color = COLOR_TEXT_WHITE

        pygame.draw.rect(surface, bg, self.rect, border_radius=6)
        if not self.disabled:
            # Border halus
            pygame.draw.rect(surface, (70, 70, 100), self.rect, width=1, border_radius=6)

        text_surf = font.render(self.text, True, txt_color)
        text_rect = text_surf.get_rect(center=self.rect.center)
        surface.blit(text_surf, text_rect)

    def is_clicked(self, event) -> bool:
        if self.disabled:
            return False
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            return self.rect.collidepoint(event.pos)
        return False

# ╔══════════════════════════════════════════════════════════════════════╗
# ║                         MAIN APPLICATION                           ║
# ╚══════════════════════════════════════════════════════════════════════╝

def main():
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption("Smart Courier Pathfinding Simulator (A*, Dijkstra, BFS)")
    clock = pygame.time.Clock()

    # Font Setup
    try:
        font_title = pygame.font.SysFont("segoeui", 22, bold=True)
        font_large = pygame.font.SysFont("segoeui", 18, bold=True)
        font_medium = pygame.font.SysFont("segoeui", 14, bold=True)
        font_small = pygame.font.SysFont("segoeui", 12)
        font_mono = pygame.font.SysFont("consolas", 12)
    except Exception:
        font_title = pygame.font.SysFont(None, 24, bold=True)
        font_large = pygame.font.SysFont(None, 20, bold=True)
        font_medium = pygame.font.SysFont(None, 16)
        font_small = pygame.font.SysFont(None, 14)
        font_mono = pygame.font.SysFont(None, 14)

    # Memuat Ikon Kurir (Kurir.png)
    courier_img = None
    try:
        courier_img = pygame.image.load("Kurir.png")
        # Ganti dengan smooth scale agar tidak pecah
        courier_img = pygame.transform.smoothscale(courier_img, (CELL_SIZE - 4, CELL_SIZE - 4))
    except Exception:
        # Jika gambar Kurir.png tidak ada, gambar alternatif akan digunakan
        pass

    # Inisialisasi peta awal
    selected_map_source = "procedural"
    obstacles, start_node, goal_node = generate_selected_map(GRID_ROWS, GRID_COLS, selected_map_source)

    # State aplikasi
    # State list: "idle", "searching", "path_found", "animating_courier", "done"
    state = "idle"
    selected_algo = "A*"
    visualization_speed = SPEED_MEDIUM

    # Hasil pencarian saat ini
    generator = None
    current_search_node = None
    open_set_nodes = set()
    closed_set_nodes = set()
    came_from = {}

    # Untuk menyimpan hasil final
    final_path = []
    final_path_len = 0
    final_visited_count = 0
    final_time_ms = 0.0

    # Animasi kurir bergerak
    courier_path_idx = 0
    courier_pos_x, courier_pos_y = 0.0, 0.0
    courier_move_speed = 0.15  # Persentase transisi sel (interpolasi linear)
    courier_t = 0.0

    # State overlay perbandingan
    show_comparison = False
    comparison_results = {}

    # Tombol UI
    bx = PANEL_X + 16
    bw = PANEL_W - 32

    # Tombol kontrol utama
    btn_start = Button(bx, 210, bw, 35, "Mulai Simulasi", COLOR_BTN_RUN)
    btn_reset = Button(bx, 252, bw, 35, "Reset", COLOR_BTN_RESET)
    
    # Tombol pemilihan Peta
    btn_map_procedural = Button(bx, 314, 58, 28, "Acak", COLOR_BTN_NORMAL)
    btn_map_game = Button(bx + 62, 314, 58, 28, "Map 1", COLOR_BTN_NORMAL)
    btn_map_update = Button(bx + 124, 314, 58, 28, "Map 2", COLOR_BTN_NORMAL)
    btn_map_peta = Button(bx + 186, 314, 68, 28, "Map 3", COLOR_BTN_NORMAL)
    btn_map_procedural.active = True

    btn_compare = Button(bx, 352, bw, 35, "Bandingkan 3 Algoritma", COLOR_BTN_COMPARE)

    # Tombol pemilihan Algoritma
    btn_algo_astar = Button(bx, 90, 80, 28, "A*", COLOR_BTN_NORMAL)
    btn_algo_dij = Button(bx + 86, 90, 80, 28, "Dijkstra", COLOR_BTN_NORMAL)
    btn_algo_bfs = Button(bx + 172, 90, 80, 28, "BFS", COLOR_BTN_NORMAL)
    btn_algo_astar.active = True

    # Tombol pemilihan Kecepatan
    btn_sp_slow = Button(bx, 150, 58, 28, "Lambat", COLOR_BTN_NORMAL)
    btn_sp_med = Button(bx + 62, 150, 58, 28, "Sedang", COLOR_BTN_NORMAL)
    btn_sp_fast = Button(bx + 124, 150, 58, 28, "Cepat", COLOR_BTN_NORMAL)
    btn_sp_inst = Button(bx + 186, 150, 68, 28, "Instan", COLOR_BTN_NORMAL)
    btn_sp_med.active = True

    all_buttons = [
        btn_start, btn_reset, btn_compare,
        btn_algo_astar, btn_algo_dij, btn_algo_bfs,
        btn_sp_slow, btn_sp_med, btn_sp_fast, btn_sp_inst,
        btn_map_procedural, btn_map_game, btn_map_update, btn_map_peta
    ]

    def reset_simulation(keep_map=True):
        nonlocal state, generator, current_search_node, open_set_nodes, closed_set_nodes, came_from
        nonlocal final_path, final_path_len, final_visited_count, final_time_ms
        nonlocal courier_path_idx, courier_t, show_comparison, obstacles, start_node, goal_node
        
        state = "idle"
        generator = None
        current_search_node = None
        open_set_nodes = set()
        closed_set_nodes = set()
        came_from = {}
        final_path = []
        final_path_len = 0
        final_visited_count = 0
        final_time_ms = 0.0
        courier_path_idx = 0
        courier_t = 0.0
        show_comparison = False
        
        if not keep_map:
            obstacles, start_node, goal_node = generate_selected_map(GRID_ROWS, GRID_COLS, selected_map_source)

    # ── Main Loop ──
    running = True
    while running:
        dt = clock.tick(FPS)
        mouse_pos = pygame.mouse.get_pos()

        # Update hover status
        for btn in all_buttons:
            btn.update(mouse_pos)

        # Disable tombol tertentu berdasarkan state
        is_running = (state in ("searching", "animating_courier"))
        btn_start.disabled = is_running or state == "done"
        
        btn_map_procedural.disabled = is_running
        btn_map_game.disabled = is_running
        btn_map_update.disabled = is_running
        btn_map_peta.disabled = is_running
        
        btn_compare.disabled = is_running
        
        # Highlight tombol Peta terpilih
        btn_map_procedural.active = (selected_map_source == "procedural")
        btn_map_game.active = (selected_map_source == "map_game")
        btn_map_update.active = (selected_map_source == "map_update")
        btn_map_peta.active = (selected_map_source == "map_peta")

        # Highlight tombol Algoritma terpilih
        btn_algo_astar.active = (selected_algo == "A*")
        btn_algo_dij.active = (selected_algo == "Dijkstra")
        btn_algo_bfs.active = (selected_algo == "BFS")

        # Highlight tombol Kecepatan terpilih
        btn_sp_slow.active = (visualization_speed == SPEED_SLOW)
        btn_sp_med.active = (visualization_speed == SPEED_MEDIUM)
        btn_sp_fast.active = (visualization_speed == SPEED_FAST)
        btn_sp_inst.active = (visualization_speed == SPEED_INSTANT)

        # ── Event Handling ──
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                break

            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                # Menutup overlay comparison dengan mengklik area manapun saat aktif
                if show_comparison:
                    show_comparison = False
                    continue

                # Klik pada Peta untuk memindahkan start/goal secara manual
                mx, my = event.pos
                if mx < MAP_W and my < MAP_H and state == "idle":
                    c = mx // CELL_SIZE
                    r = my // CELL_SIZE
                    if (r, c) not in obstacles:
                        # Klik kiri + Shift = ganti Goal, biasa = ganti Start
                        keys = pygame.key.get_pressed()
                        if keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]:
                            if (r, c) != start_node:
                                goal_node = (r, c)
                        else:
                            if (r, c) != goal_node:
                                start_node = (r, c)

                # Klik tombol
                if btn_start.is_clicked(event):
                    reset_simulation(keep_map=True)
                    state = "searching"
                    start_time = time.perf_counter()
                    
                    if selected_algo == "A*":
                        generator = astar_generator(GRID_ROWS, GRID_COLS, obstacles, start_node, goal_node)
                    elif selected_algo == "Dijkstra":
                        generator = dijkstra_generator(GRID_ROWS, GRID_COLS, obstacles, start_node, goal_node)
                    else:
                        generator = bfs_generator(GRID_ROWS, GRID_COLS, obstacles, start_node, goal_node)
                        
                elif btn_reset.is_clicked(event):
                    reset_simulation(keep_map=True)
                    
                elif btn_map_procedural.is_clicked(event) and not is_running:
                    selected_map_source = "procedural"
                    reset_simulation(keep_map=False)
                elif btn_map_game.is_clicked(event) and not is_running:
                    selected_map_source = "map_game"
                    reset_simulation(keep_map=False)
                elif btn_map_update.is_clicked(event) and not is_running:
                    selected_map_source = "map_update"
                    reset_simulation(keep_map=False)
                elif btn_map_peta.is_clicked(event) and not is_running:
                    selected_map_source = "map_peta"
                    reset_simulation(keep_map=False)
                    
                elif btn_compare.is_clicked(event):
                    # Menjalankan ketiga algoritma secara sinkron dan instan
                    res_astar = run_instant_search("A*", GRID_ROWS, GRID_COLS, obstacles, start_node, goal_node)
                    res_dij = run_instant_search("Dijkstra", GRID_ROWS, GRID_COLS, obstacles, start_node, goal_node)
                    res_bfs = run_instant_search("BFS", GRID_ROWS, GRID_COLS, obstacles, start_node, goal_node)
                    
                    comparison_results = {
                        'A*': res_astar,
                        'Dijkstra': res_dij,
                        'BFS': res_bfs
                    }
                    show_comparison = True

                # Pilih algoritma via UI
                elif btn_algo_astar.is_clicked(event) and not is_running:
                    selected_algo = "A*"
                elif btn_algo_dij.is_clicked(event) and not is_running:
                    selected_algo = "Dijkstra"
                elif btn_algo_bfs.is_clicked(event) and not is_running:
                    selected_algo = "BFS"

                # Pilih kecepatan via UI
                elif btn_sp_slow.is_clicked(event):
                    visualization_speed = SPEED_SLOW
                elif btn_sp_med.is_clicked(event):
                    visualization_speed = SPEED_MEDIUM
                elif btn_sp_fast.is_clicked(event):
                    visualization_speed = SPEED_FAST
                elif btn_sp_inst.is_clicked(event):
                    visualization_speed = SPEED_INSTANT

        # ── Proses Visualisasi Pathfinding ──
        if state == "searching" and generator:
            if visualization_speed == SPEED_INSTANT:
                # Selesaikan langsung tanpa delay
                try:
                    for val in generator:
                        current_search_node, open_set_nodes, closed_set_nodes, came_from = val
                except StopIteration:
                    pass
                generator = None
                state = "path_found"
                final_time_ms = (time.perf_counter() - start_time) * 1000.0
            else:
                # Jalankan langkah-langkah visualisasi sesuai kecepatan
                steps_per_frame = 1
                if visualization_speed == SPEED_FAST:
                    steps_per_frame = 5  # Percepat pencarian untuk Fast Speed
                    
                for _ in range(steps_per_frame):
                    try:
                        val = next(generator)
                        current_search_node, open_set_nodes, closed_set_nodes, came_from = val
                        # Tambahkan delay buatan untuk visualisasi yang rapi
                        if visualization_speed > 0:
                            pygame.time.delay(visualization_speed)
                    except StopIteration:
                        generator = None
                        state = "path_found"
                        final_time_ms = (time.perf_counter() - start_time) * 1000.0
                        break

        # ── Jalur Ditemukan, Siapkan Animasi Kurir ──
        if state == "path_found":
            final_visited_count = len(closed_set_nodes)
            
            # Rekonstruksi jalur hijau
            if goal_node in came_from:
                curr = goal_node
                while curr != start_node:
                    final_path.append(curr)
                    curr = came_from[curr]
                final_path.append(start_node)
                final_path.reverse()
                final_path_len = len(final_path) - 1
                
                # Masuk ke mode animasi kurir
                state = "animating_courier"
                courier_path_idx = 0
                courier_t = 0.0
                curr_node = final_path[0]
                courier_pos_x, courier_pos_y = curr_node[1] * CELL_SIZE, curr_node[0] * CELL_SIZE
            else:
                final_path = []
                final_path_len = 0
                state = "done"  # Tidak ditemukan rute

        # ── Animasi Gerak Kurir ──
        if state == "animating_courier":
            if courier_path_idx < len(final_path) - 1:
                courier_t += courier_move_speed
                if courier_t >= 1.0:
                    courier_t = 0.0
                    courier_path_idx += 1
                    
                if courier_path_idx < len(final_path) - 1:
                    p1 = final_path[courier_path_idx]
                    p2 = final_path[courier_path_idx + 1]
                    
                    # Interpolasi linear posisi kurir
                    x1, y1 = p1[1] * CELL_SIZE, p1[0] * CELL_SIZE
                    x2, y2 = p2[1] * CELL_SIZE, p2[0] * CELL_SIZE
                    courier_pos_x = x1 + (x2 - x1) * courier_t
                    courier_pos_y = y1 + (y2 - y1) * courier_t
            else:
                state = "done"

        # ╔══════════════════════════════════════════════════════════════╗
        # ║                     RENDERING TAMPILAN                       ║
        # ╚══════════════════════════════════════════════════════════════╝
        screen.fill(COLOR_BG)

        # ── 1. Render Grid Peta ──
        for r in range(GRID_ROWS):
            for c in range(GRID_COLS):
                cell_rect = pygame.Rect(c * CELL_SIZE, r * CELL_SIZE, CELL_SIZE, CELL_SIZE)
                
                # Warna sel dasar
                if (r, c) in obstacles:
                    color = COLOR_CELL_WALL
                else:
                    color = COLOR_CELL_EMPTY
                    
                # Warna overlay visualisasi
                if state in ("searching", "path_found", "animating_courier", "done"):
                    if (r, c) in final_path:
                        color = COLOR_PATH
                    elif (r, c) in closed_set_nodes:
                        color = COLOR_CLOSED
                    elif (r, c) in open_set_nodes:
                        color = COLOR_OPEN
                        
                # Warnai khusus Start & Goal
                if (r, c) == start_node:
                    color = COLOR_SOURCE
                elif (r, c) == goal_node:
                    color = COLOR_DEST

                # Gambar sel grid
                pygame.draw.rect(screen, color, cell_rect)
                
                # Gambar garis grid pembatas sel
                pygame.draw.rect(screen, COLOR_GRID_LINE, cell_rect, width=1)

        # Draw current node yang sedang dieksplorasi
        if state == "searching" and current_search_node:
            cr, cc = current_search_node
            curr_rect = pygame.Rect(cc * CELL_SIZE, cr * CELL_SIZE, CELL_SIZE, CELL_SIZE)
            pygame.draw.rect(screen, COLOR_TEXT_GOLD, curr_rect)

        # ── 2. Render Icon Kurir ──
        if state == "animating_courier":
            if courier_img:
                screen.blit(courier_img, (int(courier_pos_x) + 2, int(courier_pos_y) + 2))
            else:
                # Gambar lingkaran kurir premium jika file logo tidak ditemukan
                pygame.draw.circle(screen, COLOR_TEXT_GOLD, (int(courier_pos_x) + CELL_SIZE//2, int(courier_pos_y) + CELL_SIZE//2), CELL_SIZE//2 - 2)
                pygame.draw.circle(screen, (0, 0, 0), (int(courier_pos_x) + CELL_SIZE//2, int(courier_pos_y) + CELL_SIZE//2), CELL_SIZE//2 - 2, width=1)
                k_text = font_small.render("K", True, (0, 0, 0))
                screen.blit(k_text, k_text.get_rect(center=(int(courier_pos_x) + CELL_SIZE//2, int(courier_pos_y) + CELL_SIZE//2)))
        elif state == "done" and len(final_path) > 0:
            # Tetap gambar kurir di goal node jika simulasi selesai
            gx, gy = goal_node[1] * CELL_SIZE, goal_node[0] * CELL_SIZE
            if courier_img:
                screen.blit(courier_img, (gx + 2, gy + 2))
            else:
                pygame.draw.circle(screen, COLOR_TEXT_GOLD, (gx + CELL_SIZE//2, gy + CELL_SIZE//2), CELL_SIZE//2 - 2)

        # ── 3. Render Panel Control & Status (Sidebar) ──
        panel_rect = pygame.Rect(PANEL_X, PANEL_Y, PANEL_W, PANEL_H)
        pygame.draw.rect(screen, COLOR_BG_PANEL, panel_rect)
        pygame.draw.line(screen, (60, 60, 85), (PANEL_X, 0), (PANEL_X, PANEL_H), width=2)

        # Judul Sidebar
        title_surf = font_title.render("Smart Courier PAA", True, COLOR_TEXT_GOLD)
        screen.blit(title_surf, (PANEL_X + 16, 16))
        
        sub_title = font_small.render("Pathfinding Visualizer & Analyzer", True, COLOR_TEXT_DIM)
        screen.blit(sub_title, (PANEL_X + 18, 42))

        pygame.draw.line(screen, (45, 45, 65), (PANEL_X + 16, 60), (WINDOW_WIDTH - 16, 60), 1)

        # Teks Algoritma
        algo_lbl = font_medium.render("Pilih Algoritma Pencarian:", True, COLOR_TEXT_GRAY)
        screen.blit(algo_lbl, (PANEL_X + 16, 70))

        # Teks Kecepatan
        speed_lbl = font_medium.render("Kecepatan Visualisasi:", True, COLOR_TEXT_GRAY)
        screen.blit(speed_lbl, (PANEL_X + 16, 130))

        # Teks Petunjuk Kontrol Peta
        map_hint = font_small.render("Tips: Klik kiri grid kosong = Pindahkan Start", True, COLOR_TEXT_DIM)
        screen.blit(map_hint, (PANEL_X + 16, 185))
        map_hint2 = font_small.render("      Shift + Klik kiri = Pindahkan Goal", True, COLOR_TEXT_DIM)
        screen.blit(map_hint2, (PANEL_X + 16, 198))

        # Teks Pilih Peta
        map_lbl = font_medium.render("Pilih Peta:", True, COLOR_TEXT_GRAY)
        screen.blit(map_lbl, (PANEL_X + 16, 294))

        # Draw Semua Tombol
        for btn in all_buttons:
            btn.draw(screen, font_medium)

        pygame.draw.line(screen, (45, 45, 65), (PANEL_X + 16, 398), (WINDOW_WIDTH - 16, 398), 1)

        # Statistik Panel
        stat_lbl = font_large.render("Statistik & Hasil:", True, COLOR_TEXT_WHITE)
        screen.blit(stat_lbl, (PANEL_X + 16, 406))

        # Info Status Realtime
        state_map = {
            "idle": ("Siap dijalankan", COLOR_TEXT_WHITE),
            "searching": ("Sedang mencari rute...", COLOR_OPEN),
            "path_found": ("Rute ditemukan!", COLOR_TEXT_GREEN),
            "animating_courier": ("Kurir sedang mengantar...", COLOR_TEXT_GOLD),
            "done": ("Selesai", COLOR_TEXT_GREEN)
        }
        curr_status, status_color = state_map.get(state, ("Standby", COLOR_TEXT_WHITE))
        
        status_line = font_medium.render(f"Status: {curr_status}", True, status_color)
        screen.blit(status_line, (PANEL_X + 18, 428))

        # Tampilkan detail statistik rute
        path_stat_txt = "-" if state == "idle" else f"{final_path_len} langkah"
        if state == "done" and final_path_len == 0:
            path_stat_txt = "Jalur terblokir! (Tidak ketemu)"
            
        len_surf = font_medium.render(f"Panjang Jalur: {path_stat_txt}", True, COLOR_TEXT_WHITE)
        screen.blit(len_surf, (PANEL_X + 18, 448))

        visited_stat_txt = "-" if state == "idle" else f"{final_visited_count} node"
        vis_surf = font_medium.render(f"Eksplorasi Node: {visited_stat_txt}", True, COLOR_TEXT_WHITE)
        screen.blit(vis_surf, (PANEL_X + 18, 468))

        time_stat_txt = "-" if state == "idle" else f"{final_time_ms:.3f} ms"
        time_surf = font_medium.render(f"Waktu Eksekusi: {time_stat_txt}", True, COLOR_TEXT_WHITE)
        screen.blit(time_surf, (PANEL_X + 18, 488))

        # Petunjuk Legenda Warna
        leg_lbl = font_large.render("Legenda:", True, COLOR_TEXT_WHITE)
        screen.blit(leg_lbl, (PANEL_X + 16, 516))
        
        legend_data = [
            (COLOR_SOURCE, "Kurir (Asal)"),
            (COLOR_DEST, "Customer (Tujuan)"),
            (COLOR_OPEN, "Frontier (Open)"),
            (COLOR_CLOSED, "Visited (Closed)"),
            (COLOR_PATH, "Jalur Pengantaran")
        ]
        
        ly = 538
        for col, txt in legend_data:
            pygame.draw.rect(screen, col, (PANEL_X + 18, ly + 2, 12, 12), border_radius=2)
            pygame.draw.rect(screen, (80, 80, 100), (PANEL_X + 18, ly + 2, 12, 12), width=1, border_radius=2)
            lbl = font_small.render(txt, True, COLOR_TEXT_GRAY)
            screen.blit(lbl, (PANEL_X + 38, ly))
            ly += 18

        # ── 4. Render Overlay Perbandingan (Comparison Popup Table) ──
        if show_comparison and comparison_results:
            # Gelapkan background utama
            overlay = pygame.Surface((WINDOW_WIDTH, WINDOW_HEIGHT), pygame.SRCALPHA)
            overlay.fill((10, 10, 20, 210))
            screen.blit(overlay, (0, 0))

            # Kotak popup perbandingan
            card_w, card_h = 750, 420
            cx = (WINDOW_WIDTH - card_w) // 2
            cy = (WINDOW_HEIGHT - card_h) // 2
            card_rect = pygame.Rect(cx, cy, card_w, card_h)
            
            pygame.draw.rect(screen, (22, 22, 38), card_rect, border_radius=12)
            pygame.draw.rect(screen, COLOR_TEXT_GOLD, card_rect, width=2, border_radius=12)

            # Header Popup
            pop_title = font_title.render("Tabel Perbandingan Performa Algoritma", True, COLOR_TEXT_GOLD)
            screen.blit(pop_title, (cx + 24, cy + 20))
            
            pop_sub = font_small.render("Menjalankan BFS, Dijkstra, dan A* pada peta acak yang sama secara instan.", True, COLOR_TEXT_GRAY)
            screen.blit(pop_sub, (cx + 24, cy + 48))

            # Header Tabel
            col_x = [cx + 30, cx + 180, cx + 330, cx + 470, cx + 610]
            headers = ["Algoritma", "Status Rute", "Panjang Rute", "Node Dikunjungi", "Waktu Eksekusi"]
            table_y = cy + 90
            
            # Garis pembatas header tabel
            pygame.draw.rect(screen, (35, 35, 55), (cx + 20, table_y - 6, card_w - 40, 28), border_radius=4)
            for i, h in enumerate(headers):
                h_surf = font_medium.render(h, True, COLOR_TEXT_GOLD)
                screen.blit(h_surf, (col_x[i], table_y - 2))

            # Isi Baris Tabel
            row_y = table_y + 35
            algos = ["BFS", "Dijkstra", "A*"]
            colors = [COLOR_SOURCE, COLOR_OPEN, COLOR_TEXT_GREEN]
            
            best_node_visited = float('inf')
            best_algo = ""
            
            for idx, name in enumerate(algos):
                res = comparison_results.get(name, {})
                status_txt = "Ditemukan" if res['found'] else "Terblokir"
                status_col = COLOR_TEXT_GREEN if res['found'] else COLOR_TEXT_RED
                
                # Row background alternating
                bg_row_col = (28, 28, 48) if idx % 2 == 0 else (22, 22, 38)
                pygame.draw.rect(screen, bg_row_col, (cx + 20, row_y - 4, card_w - 40, 32), border_radius=4)

                # Kolom 1: Nama Algoritma
                name_surf = font_large.render(name, True, colors[idx])
                screen.blit(name_surf, (col_x[0], row_y))

                # Kolom 2: Status
                stat_surf = font_medium.render(status_txt, True, status_col)
                screen.blit(stat_surf, (col_x[1], row_y + 2))

                # Kolom 3: Panjang Rute
                len_surf = font_medium.render(f"{res['path_length']} langkah", True, COLOR_TEXT_WHITE)
                screen.blit(len_surf, (col_x[2], row_y + 2))

                # Kolom 4: Node Visited
                vis_surf = font_medium.render(f"{res['visited_count']} node", True, COLOR_TEXT_WHITE)
                screen.blit(vis_surf, (col_x[3], row_y + 2))

                # Kolom 5: Waktu Eksekusi
                time_surf = font_medium.render(f"{res['time_ms']:.4f} ms", True, COLOR_TEXT_WHITE)
                screen.blit(time_surf, (col_x[4], row_y + 2))

                # Evaluasi algoritma terbaik (A* biasanya paling efisien mengabaikan node tak perlu)
                if res['found'] and res['visited_count'] < best_node_visited:
                    best_node_visited = res['visited_count']
                    best_algo = name

                row_y += 38

            # Pembatas Kesimpulan
            pygame.draw.line(screen, (50, 50, 75), (cx + 20, row_y + 10), (cx + card_w - 20, row_y + 10), 1)

            # Kesimpulan Analisis
            concl_y = row_y + 25
            concl_title = font_large.render("Analisis Kesimpulan:", True, COLOR_TEXT_GOLD)
            screen.blit(concl_title, (cx + 24, concl_y))
            
            res_astar = comparison_results.get("A*", {})
            res_dij = comparison_results.get("Dijkstra", {})
            res_bfs = comparison_results.get("BFS", {})

            # Analisis teks dinamis
            c_lines = []
            if res_astar.get('found'):
                c_lines.append(f"1. A* dan Dijkstra berhasil menemukan rute terpendek yang optimal ({res_astar['path_length']} langkah).")
                diff = res_dij['visited_count'] - res_astar['visited_count']
                c_lines.append(f"2. A* mengeksplorasi {res_astar['visited_count']} node (menghemat {diff} node dibanding Dijkstra berbobot).")
                
                # Bandingkan BFS dengan A* jika BFS menemukan rute terpanjang
                if res_bfs['path_length'] > res_astar['path_length']:
                    c_lines.append(f"3. BFS menghasilkan rute {res_bfs['path_length']} langkah (BFS tidak dijamin optimal pada cost kumulatif).")
                else:
                    c_lines.append(f"3. BFS menemukan panjang rute yang sama, namun menjelajah lebih melebar ({res_bfs['visited_count']} node).")
                
                c_lines.append(f"Kesimpulan: Algoritma {best_algo} adalah yang paling efisien karena meminimalkan eksplorasi node.")
            else:
                c_lines.append("Jalur terblokir. Tidak ada jalur dari kurir ke pelanggan.")

            ly = concl_y + 25
            for line in c_lines:
                ln_surf = font_small.render(line, True, COLOR_TEXT_WHITE)
                screen.blit(ln_surf, (cx + 24, ly))
                ly += 20

            # Instruksi penutupan popup
            close_hint = font_small.render("[Klik di mana saja untuk menutup tabel]", True, COLOR_TEXT_DIM)
            screen.blit(close_hint, close_hint.get_rect(centerx=cx + card_w//2, bottom=cy + card_h - 10))

        # Tampilkan semua ke layar
        pygame.display.flip()

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
