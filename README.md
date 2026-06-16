# Smart Courier Pathfinding Simulator

Proyek akhir mata kuliah Perancangan dan Analisis Algoritma (PAA) yang mensimulasikan pencarian rute kurir pada peta/grid menggunakan beberapa algoritma pathfinding.

## Identitas

Nama: **Arysubakti**  
NIM: **2301020065**

## Deskripsi

Smart Courier Pathfinding Simulator dibuat untuk membandingkan cara kerja algoritma pencarian jalur dalam menentukan rute dari titik awal menuju titik tujuan dengan adanya hambatan pada peta.

Algoritma yang digunakan:

- Breadth First Search (BFS)
- Dijkstra's Algorithm
- A* Search

## Fitur

- Simulasi pencarian rute kurir pada peta.
- Perbandingan algoritma BFS, Dijkstra, dan A*.
- Visualisasi jalur, titik awal, titik tujuan, dan obstacle.
- Tersedia implementasi Python/Pygame dan tampilan web.

## Struktur File

```text
.
|-- smart_courier.py      # Program utama versi Python/Pygame
|-- index.html            # Tampilan web
|-- main.js               # Logika simulator versi web
|-- styles.css            # Styling tampilan web
|-- requirements.txt      # Dependensi Python
|-- LAPORAN_PAA.md        # Laporan proyek
|-- Kurir.png             # Aset kurir
`-- Peta/                 # Aset gambar peta
```

## Cara Menjalankan Versi Python

1. Pastikan Python sudah terpasang.
2. Install dependensi:

```bash
pip install -r requirements.txt
```

3. Jalankan program:

```bash
python smart_courier.py
```

## Cara Menjalankan Versi Web

Buka file `index.html` di browser.

## Teknologi

- Python
- Pygame
- HTML
- CSS
- JavaScript

## Tujuan Proyek

Proyek ini bertujuan untuk memahami penerapan dan perbandingan algoritma pathfinding, khususnya dalam kasus pencarian rute pengiriman yang efisien pada lingkungan berbasis grid.
