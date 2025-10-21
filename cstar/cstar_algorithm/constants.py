# --- Các hằng số thể hiện trạng thái của mỗi ô trong bản đồ ---
FREE_UNCOVERED = 0   # Ô trống, chưa được bao phủ
OBSTACLE = 1         # Ô là chướng ngại vật
COVERED = 2          # Ô đã được bao phủ
FRONTIER = 3         # Ô thuộc vùng biên (chưa được bao phủ nhưng gần vùng đã bao phủ)

# --- Hướng di chuyển cho robot (8 hướng: 4 chính + 4 chéo) ---
DIR8 = [
    (-1, -1), (-1, 0), (-1, 1),
    (0, -1),           (0, 1),
    (1, -1),  (1, 0),  (1, 1)
]

DIR4 = [
    (-1, 0),
    (0, -1),           (0, 1),
    (1, 0),
]
