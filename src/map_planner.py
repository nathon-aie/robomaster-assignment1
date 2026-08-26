import pygame
import heapq
import sys
import json

# ตั้งค่า Grid และขนาดหน้าจอ
WIDTH, HEIGHT = 600, 600
ROWS, COLS = 4, 6
GRID_SIZE = WIDTH // COLS
WALL_THICKNESS = 4  # ความหนาของกำแพง

# สี (RGB)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)       # กำแพง
GREEN = (0, 255, 0)     # Start
RED = (255, 0, 0)       # Goal
BLUE = (0, 120, 255)    # เส้นทาง
GREY = (220, 220, 220)  # เส้น Grid บางๆ

def create_outer_boundary(grid):
    """ ตีกรอบกำแพงรอบนอกสุดของ Grid ให้อัตโนมัติ """
    for r in range(ROWS):
        for c in range(COLS):
            cell = grid[r][c]
            if r == 0: cell.walls['top'] = True
            if r == ROWS - 1: cell.walls['bottom'] = True
            if c == 0: cell.walls['left'] = True
            if c == COLS - 1: cell.walls['right'] = True

def inspect_robot_view_walls(path):
    if not path or len(path) < 2: return []

    current_direction = 0  # เริ่มต้นหุ่นหันไปทาง North (ขึ้น)
    dir_map = {(0, -1): 0, (1, 0): 1, (0, 1): 2, (-1, 0): 3}
    world_wall_keys = ['top', 'right', 'bottom', 'left']

    robot_wall_history = []

    print("\n--- Robot Relative View Walls ---")
    for i in range(len(path)):
        curr_cell = path[i]

        if i < len(path) - 1:
            next_cell = path[i+1]
            dx = next_cell.col - curr_cell.col
            dy = next_cell.row - curr_cell.row
            current_direction = dir_map[(dx, dy)]

        robot_walls = {
            'Front': curr_cell.walls[world_wall_keys[current_direction]],
            'Right': curr_cell.walls[world_wall_keys[(current_direction + 1) % 4]],
            'Back':  curr_cell.walls[world_wall_keys[(current_direction + 2) % 4]],
            'Left':  curr_cell.walls[world_wall_keys[(current_direction + 3) % 4]]
        }

        detected = [direction for direction, has_wall in robot_walls.items() if has_wall]
        wall_str = ", ".join(detected) if detected else "None"

        dir_names = {0: "North (ขึ้น)", 1: "East (ขวา)", 2: "South (ลง)", 3: "West (ซ้าย)"}
        heading_str = dir_names[current_direction]

        tag = "(Start)" if i == 0 else f"(Step {i})" if i < len(path)-1 else "(Goal)"
        print(f"{tag} Pos ({curr_cell.col}, {curr_cell.row}) | Facing: {heading_str} -> Walls: [{wall_str}]")

        robot_wall_history.append({
            "step": i,
            "pos": [curr_cell.col, curr_cell.row],
            "facing": heading_str,
            "detected_walls": detected
        })

    return robot_wall_history

def generate_commands(path):
    if not path or len(path) < 2: return []

    current_direction = 0 
    dir_map = {(0, -1): 0, (1, 0): 1, (0, 1): 2, (-1, 0): 3}

    commands = []
    forward_count = 0

    for i in range(len(path) - 1):
        curr_x, curr_y = path[i].col, path[i].row
        next_x, next_y = path[i+1].col, path[i+1].row

        dx = next_x - curr_x
        dy = next_y - curr_y

        target_direction = dir_map[(dx, dy)]

        if target_direction != current_direction:
            if forward_count > 0:
                commands.append(f"Move Forward: {forward_count} cells")
                forward_count = 0

            diff = (target_direction - current_direction) % 4
            if diff == 1: commands.append("Turn Right (90 deg)")
            elif diff == 2: commands.append("Turn Around (180 deg)")
            elif diff == 3: commands.append("Turn Left (90 deg)")

            current_direction = target_direction

        forward_count += 1

    if forward_count > 0:
        commands.append(f"Move Forward: {forward_count} cells")

    return commands

class Cell:
    def __init__(self, row, col):
        self.row = row
        self.col = col
        self.walls = {'top': False, 'bottom': False, 'left': False, 'right': False}

    def draw(self, win, is_start=False, is_goal=False, is_path=False):
        x, y = self.col * GRID_SIZE, self.row * GRID_SIZE
        color = WHITE
        if is_start: color = GREEN
        elif is_goal: color = RED
        elif is_path: color = BLUE
        
        pygame.draw.rect(win, color, (x, y, GRID_SIZE, GRID_SIZE))
        pygame.draw.rect(win, GREY, (x, y, GRID_SIZE, GRID_SIZE), 1)

    def draw_walls(self, win):
        x, y = self.col * GRID_SIZE, self.row * GRID_SIZE
        if self.walls['top']:
            pygame.draw.line(win, BLACK, (x, y), (x + GRID_SIZE, y), WALL_THICKNESS)
        if self.walls['bottom']:
            pygame.draw.line(win, BLACK, (x, y + GRID_SIZE), (x + GRID_SIZE, y + GRID_SIZE), WALL_THICKNESS)
        if self.walls['left']:
            pygame.draw.line(win, BLACK, (x, y), (x, y + GRID_SIZE), WALL_THICKNESS)
        if self.walls['right']:
            pygame.draw.line(win, BLACK, (x + GRID_SIZE, y), (x + GRID_SIZE, y + GRID_SIZE), WALL_THICKNESS)

# 1. หาทางที่สั้นที่สุดปกติ (Spacebar)
def astar(grid, start, goal):
    count = 0
    open_set = []
    heapq.heappush(open_set, (0, count, start))
    came_from = {}
    g_score = {(cell.row, cell.col): float('inf') for row in grid for cell in row}
    g_score[(start.row, start.col)] = 0

    while open_set:
        current = heapq.heappop(open_set)[2]

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        directions = [
            (-1, 0, 'top', 'bottom'), (1, 0, 'bottom', 'top'),
            (0, -1, 'left', 'right'), (0, 1, 'right', 'left')
        ]

        for dr, dc, wall_curr, wall_next in directions:
            r, c = current.row + dr, current.col + dc
            if 0 <= r < ROWS and 0 <= c < COLS:
                neighbor = grid[r][c]
                if not current.walls[wall_curr] and not neighbor.walls[wall_next]:
                    temp_g = g_score[(current.row, current.col)] + 1
                    if temp_g < g_score[(neighbor.row, neighbor.col)]:
                        came_from[neighbor] = current
                        g_score[(neighbor.row, neighbor.col)] = temp_g
                        f_score = temp_g + abs(neighbor.row - goal.row) + abs(neighbor.col - goal.col)
                        count += 1
                        heapq.heappush(open_set, (f_score, count, neighbor))
    return None

# 2. หาทางที่เลี้ยวน้อยที่สุด (ปุ่ม T)
def astar_min_turns(grid, start, goal, turn_penalty=2.0):
    count = 0
    open_set = []
    heapq.heappush(open_set, (0, count, start, None))
    came_from = {}
    g_score = {}
    g_score[(start.row, start.col, None)] = 0

    directions = [
        (-1, 0, 'top', 'bottom', 0), (0, 1, 'right', 'left', 1),
        (1, 0, 'bottom', 'top', 2), (0, -1, 'left', 'right', 3)
    ]

    while open_set:
        _, _, current, current_dir = heapq.heappop(open_set)

        if current == goal:
            path = []
            curr_state = (current, current_dir)
            while curr_state in came_from:
                path.append(curr_state[0])
                curr_state = came_from[curr_state]
            path.append(start)
            return path[::-1]

        for dr, dc, wall_curr, wall_next, move_dir in directions:
            r, c = current.row + dr, current.col + dc
            if 0 <= r < ROWS and 0 <= c < COLS:
                neighbor = grid[r][c]
                if not current.walls[wall_curr] and not neighbor.walls[wall_next]:
                    step_cost = 1.0
                    if current_dir is not None and current_dir != move_dir:
                        turn_diff = abs(move_dir - current_dir)
                        if turn_diff == 3: turn_diff = 1
                        step_cost += turn_penalty * turn_diff

                    curr_g = g_score.get((current.row, current.col, current_dir), float('inf'))
                    tentative_g = curr_g + step_cost
                    neighbor_state = (neighbor.row, neighbor.col, move_dir)
                    
                    if tentative_g < g_score.get(neighbor_state, float('inf')):
                        came_from[(neighbor, move_dir)] = (current, current_dir)
                        g_score[neighbor_state] = tentative_g
                        f_score = tentative_g + (abs(neighbor.row - goal.row) + abs(neighbor.col - goal.col))
                        count += 1
                        heapq.heappush(open_set, (f_score, count, neighbor, move_dir))
    return None

def toggle_edge_wall(grid, mx, my):
    c = mx // GRID_SIZE
    r = my // GRID_SIZE
    if not (0 <= r < ROWS and 0 <= c < COLS): return

    rel_x, rel_y = mx % GRID_SIZE, my % GRID_SIZE
    dist_top, dist_bottom = rel_y, GRID_SIZE - rel_y
    dist_left, dist_right = rel_x, GRID_SIZE - rel_x

    min_dist = min(dist_top, dist_bottom, dist_left, dist_right)
    cell = grid[r][c]

    if min_dist == dist_top:
        cell.walls['top'] = not cell.walls['top']
        if r > 0: grid[r-1][c].walls['bottom'] = cell.walls['top']
    elif min_dist == dist_bottom:
        cell.walls['bottom'] = not cell.walls['bottom']
        if r < ROWS - 1: grid[r+1][c].walls['top'] = cell.walls['bottom']
    elif min_dist == dist_left:
        cell.walls['left'] = not cell.walls['left']
        if c > 0: grid[r][c-1].walls['right'] = cell.walls['left']
    elif min_dist == dist_right:
        cell.walls['right'] = not cell.walls['right']
        if c < COLS - 1: grid[r][c+1].walls['left'] = cell.walls['right']

# --- ฟังก์ชันบันทึกข้อมูล Map และ Execution Plan ---
def save_map_and_plan(grid, start, goal, path, commands, wall_history, filename="data/robot_map_plan.json"):
    from pathlib import Path
    data = {
        "grid_info": {
            "rows": ROWS,
            "cols": COLS,
            "grid_size_px": GRID_SIZE
        },
        "start": [start.col, start.row],
        "goal": [goal.col, goal.row],
        "walls": [],
        "path": [[c.col, c.row] for c in path] if path else [],
        "commands": commands,
        "step_wall_history": wall_history
    }

    # บันทึกสถานะกำแพงเฉพาะช่องที่มีกำแพง
    for r in range(ROWS):
        for c in range(COLS):
            cell = grid[r][c]
            if any(cell.walls.values()):
                data["walls"].append({
                    "pos": [c, r],
                    "walls": cell.walls
                })

    target_path = Path(filename)
    target_path.parent.mkdir(parents=True, exist_ok=True)
    with open(target_path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=4, ensure_ascii=False)
    print(f"\n[SUCCESS] บันทึกไฟล์แผนที่และคำสั่งลงใน '{target_path}' เรียบร้อยแล้ว!")

# --- ฟังก์ชันโหลดข้อมูล Map ---
def load_map_and_plan(grid, filename="data/robot_map_plan.json"):
    from pathlib import Path
    target_path = Path(filename)
    if not target_path.exists() and Path("robot_map_plan.json").exists():
        target_path = Path("robot_map_plan.json")

    try:
        with open(target_path, "r", encoding="utf-8") as f:
            data = json.load(f)

        # ล้างกำแพงเดิม
        for row in grid:
            for cell in row:
                cell.walls = {'top': False, 'bottom': False, 'left': False, 'right': False}

        # อ่านกำแพงจากไฟล์
        for wall_info in data.get("walls", []):
            c, r = wall_info["pos"]
            if 0 <= r < ROWS and 0 <= c < COLS:
                grid[r][c].walls = wall_info["walls"]

        start_pos = data.get("start", [0, 0])
        goal_pos = data.get("goal", [COLS-1, ROWS-1])
        
        start = grid[start_pos[1]][start_pos[0]]
        goal = grid[goal_pos[1]][goal_pos[0]]

        # โหลด Path
        path = []
        for pos in data.get("path", []):
            path.append(grid[pos[1]][pos[0]])

        print(f"\n[SUCCESS] โหลดไฟล์ '{target_path}' สำเร็จ!")
        return start, goal, path
    except Exception as e:
        print(f"\n[ERROR] ไม่สามารถโหลดไฟล์ได้: {e}")
        return grid[0][0], grid[ROWS-1][COLS-1], []

def main():
    pygame.init()
    win = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Grid Planner (Space=Shortest, T=Min Turns, K=Save, L=Load, C=Clear)")

    grid = [[Cell(r, c) for c in range(COLS)] for r in range(ROWS)]
    create_outer_boundary(grid)

    start = grid[0][0]
    goal = grid[ROWS-1][COLS-1]
    path = []
    commands = []
    wall_history = []

    running = True

    while running:
        win.fill(WHITE)
        keys = pygame.key.get_pressed()

        for row in grid:
            for cell in row:
                is_start = (cell == start)
                is_goal = (cell == goal)
                is_path = (cell in path and not is_start and not is_goal)
                cell.draw(win, is_start, is_goal, is_path)

        for row in grid:
            for cell in row: cell.draw_walls(win)

        pygame.display.update()

        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False

            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    mx, my = pygame.mouse.get_pos()
                    r, c = my // GRID_SIZE, mx // GRID_SIZE
                    
                    if 0 <= r < ROWS and 0 <= c < COLS:
                        selected_cell = grid[r][c]
                        if keys[pygame.K_s]: start, path = selected_cell, []
                        elif keys[pygame.K_g]: goal, path = selected_cell, []
                        else: toggle_edge_wall(grid, mx, my); path = []

            elif event.type == pygame.KEYDOWN:
                # Spacebar = Shortest Path
                if event.key == pygame.K_SPACE:
                    found = astar(grid, start, goal)
                    if found:
                        path = found
                        print("\n=== [Mode: Shortest Path] ===")
                        wall_history = inspect_robot_view_walls(path)
                        commands = generate_commands(path)
                        print("\n--- Robot Execution Plan ---")
                        for cmd in commands: print(cmd)
                    else:
                        path = []
                        print("ไม่พบเส้นทาง!")

                # T = Min Turns Path
                elif event.key == pygame.K_t:
                    found = astar_min_turns(grid, start, goal, turn_penalty=2.0)
                    if found:
                        path = found
                        print("\n=== [Mode: Min Turns Path] ===")
                        wall_history = inspect_robot_view_walls(path)
                        commands = generate_commands(path)
                        print("\n--- Robot Execution Plan ---")
                        for cmd in commands: print(cmd)
                    else:
                        path = []
                        print("ไม่พบเส้นทาง!")

                # K = Save Map & Plan ลง JSON
                elif event.key == pygame.K_k:
                    save_map_and_plan(grid, start, goal, path, commands, wall_history)

                # L = Load Map & Plan จาก JSON
                elif event.key == pygame.K_l:
                    start, goal, path = load_map_and_plan(grid)
                    if path:
                        wall_history = inspect_robot_view_walls(path)
                        commands = generate_commands(path)

                # C = Clear Map
                elif event.key == pygame.K_c:
                    path = []
                    commands = []
                    wall_history = []
                    for row in grid:
                        for cell in row:
                            cell.walls = {'top': False, 'bottom': False, 'left': False, 'right': False}
                    create_outer_boundary(grid)

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()