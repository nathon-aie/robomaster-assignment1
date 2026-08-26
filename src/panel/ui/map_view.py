#!/usr/bin/env python3
"""Interactive grid map: editor, live robot tracking and sensor visualisation.

Behaves like a small robotics simulator view - clickable cells, drag-to-draw
walls, a rotating robot sprite, sensor rays, planned vs actual path and a live
occupancy overlay that updates while the robot maps.
"""

import math

import pygame

from ..geometry import DIR_NAMES, wrap180
from ..occupancy import FREE, OBSTACLE, UNKNOWN, WALL
from ..sensors import SENSOR_SPECS
from . import theme
from .widgets import draw_text

TOOL_SELECT = "Select"
TOOL_WALL = "Wall"
TOOL_ERASE = "Eraser"
TOOL_START = "Start"
TOOL_GOAL = "Goal"
TOOL_CHECKPOINT = "Checkpoint"
TOOL_ROBOT = "Robot"
TOOL_OBSTACLE = "Obstacle"
TOOL_OBJECT = "Object"
TOOL_DELIVERY = "Delivery"

#: Kept so saved layouts / older code naming the single Place tool still work.
TOOL_PLACE = TOOL_DELIVERY

TOOLS = (TOOL_SELECT, TOOL_WALL, TOOL_ERASE, TOOL_START, TOOL_GOAL,
         TOOL_CHECKPOINT, TOOL_ROBOT, TOOL_OBJECT, TOOL_DELIVERY, TOOL_OBSTACLE)


class MapView(object):
    def __init__(self, rect):
        self.rect = pygame.Rect(rect)
        self.tool = TOOL_SELECT
        self.cell_px = 32
        self.origin = (0, 0)
        self.hover_cell = None
        self.selected_cell = None
        self.show_trail = True
        self.show_sensors = True
        self.show_planned = True
        self.show_frontiers = True
        self.show_grid_coords = True
        self.reveal_truth = False
        self.on_edit = None            # callback(kind, payload) -> app reacts
        self._dragging = None          # 'wall' | 'erase' | None
        self._painted = set()
        self._draw_col = None
        self._draw_row = None
        self._draw_heading = None

    # ------------------------------------------------------------------ layout
    def layout(self, grid):
        pad = 14
        avail_w = self.rect.width - pad * 2
        avail_h = self.rect.height - pad * 2
        self.cell_px = max(8, int(min(avail_w / float(grid.width), avail_h / float(grid.height))))
        board_w = self.cell_px * grid.width
        board_h = self.cell_px * grid.height
        self.origin = (
            self.rect.x + (self.rect.width - board_w) // 2,
            self.rect.y + (self.rect.height - board_h) // 2,
        )

    def board_rect(self, grid):
        return pygame.Rect(self.origin[0], self.origin[1],
                           self.cell_px * grid.width, self.cell_px * grid.height)

    def cell_to_px(self, col, row):
        return (self.origin[0] + col * self.cell_px, self.origin[1] + row * self.cell_px)

    def cell_center_px(self, col, row):
        return (self.origin[0] + (col + 0.5) * self.cell_px,
                self.origin[1] + (row + 0.5) * self.cell_px)

    def px_to_cell(self, pos, grid):
        col = int((pos[0] - self.origin[0]) // self.cell_px)
        row = int((pos[1] - self.origin[1]) // self.cell_px)
        if grid.in_bounds(col, row):
            return (col, row)
        return None

    def px_to_edge(self, pos, grid):
        """Nearest cell edge to a pixel position -> (col, row, direction)."""
        cell = self.px_to_cell(pos, grid)
        if cell is None:
            return None
        col, row = cell
        rel_x = pos[0] - (self.origin[0] + col * self.cell_px)
        rel_y = pos[1] - (self.origin[1] + row * self.cell_px)
        distances = (
            (rel_y, 0),                       # top
            (self.cell_px - rel_x, 1),        # right
            (self.cell_px - rel_y, 2),        # bottom
            (rel_x, 3),                       # left
        )
        _, direction = min(distances)
        return (col, row, direction)

    # ------------------------------------------------------------------ events
    def handle_event(self, event, grid, editable=True):
        if event.type == pygame.MOUSEMOTION:
            self.hover_cell = self.px_to_cell(event.pos, grid) if self.rect.collidepoint(event.pos) else None
            if self._dragging and editable:
                self._paint(event.pos, grid)
                return True
            return False

        if event.type == pygame.MOUSEBUTTONUP:
            if self._dragging:
                self._dragging = None
                self._painted = set()
                return True
            return False

        if event.type != pygame.MOUSEBUTTONDOWN or not self.rect.collidepoint(event.pos):
            return False

        cell = self.px_to_cell(event.pos, grid)
        if cell is None:
            return False

        if event.button == 3:
            # Right click rotates whichever directional marker was clicked.
            if grid.delivery_cell == cell:
                grid.delivery_dir = (grid.delivery_dir + 1) % 4
                self._emit("delivery_dir", grid.delivery_dir)
            elif grid.robot_cell == cell:
                grid.robot_dir = (grid.robot_dir + 1) % 4
                self._emit("robot_dir", grid.robot_dir)
            return True

        if event.button != 1:
            return False

        self.selected_cell = cell
        if not editable:
            return True

        if self.tool == TOOL_WALL:
            self._dragging = "wall"
            self._painted = set()
            self._paint(event.pos, grid, toggle_first=True)
        elif self.tool == TOOL_ERASE:
            self._dragging = "erase"
            self._painted = set()
            self._paint(event.pos, grid)
        elif self.tool == TOOL_START:
            grid.start = cell
            if grid.get(*cell) in (WALL, OBSTACLE):
                grid.set(cell[0], cell[1], FREE)
            self._emit("start", cell)
        elif self.tool == TOOL_GOAL:
            grid.goal = cell
            if grid.get(*cell) in (WALL, OBSTACLE):
                grid.set(cell[0], cell[1], FREE)
            self._emit("goal", cell)
        elif self.tool == TOOL_CHECKPOINT:
            if cell in grid.checkpoints:
                grid.checkpoints.remove(cell)
            else:
                grid.checkpoints.append(cell)
            self._emit("checkpoint", cell)
        elif self.tool == TOOL_ROBOT:
            grid.robot_cell = cell
            self._emit("robot", cell)
        elif self.tool == TOOL_DELIVERY:
            if grid.delivery_cell == cell:
                # Clicking it again turns it, so one tool sets both cell and facing.
                grid.delivery_dir = (grid.delivery_dir + 1) % 4
            else:
                grid.delivery_cell = cell
                if grid.get(*cell) in (WALL, OBSTACLE):
                    grid.set(cell[0], cell[1], FREE)
            self._emit("delivery", cell)
        elif self.tool == TOOL_OBJECT:
            if grid.object_cell == cell:
                grid.object_cell = None      # click again to clear it
            else:
                grid.object_cell = cell
                if grid.get(*cell) in (WALL, OBSTACLE):
                    grid.set(cell[0], cell[1], FREE)
            self._emit("object", cell)
        elif self.tool == TOOL_OBSTACLE:
            grid.set(cell[0], cell[1], FREE if grid.get(*cell) == OBSTACLE else OBSTACLE)
            self._emit("obstacle", cell)
        return True

    def _paint(self, pos, grid, toggle_first=False):
        """Paints one edge.  The first click toggles (so clicking a wall removes
        it); dragging afterwards only adds, so a stroke never erases itself."""
        edge = self.px_to_edge(pos, grid)
        if edge is None:
            return
        col, row, direction = edge
        key = (col, row, direction)
        if key in self._painted:
            return
        self._painted.add(key)
        if self._dragging == "erase" or self.tool == TOOL_ERASE:
            grid.set_wall(col, row, direction, False, known=True)
            if grid.get(col, row) in (WALL, OBSTACLE):
                grid.set(col, row, FREE)
        elif toggle_first:
            grid.toggle_wall(col, row, direction)
        else:
            grid.set_wall(col, row, direction, True, known=True)
        self._emit("wall", key)

    def _emit(self, kind, payload):
        if self.on_edit:
            self.on_edit(kind, payload)

    # ---------------------------------------------------------------- drawing
    def draw(self, surface, fonts, ctx):
        grid = ctx["grid"]
        self.layout(grid)
        board = self.board_rect(grid)
        pygame.draw.rect(surface, theme.BG, self.rect)
        pygame.draw.rect(surface, theme.PANEL_EDGE, board.inflate(6, 6), 1, border_radius=4)

        self._draw_cells(surface, grid, ctx)
        if self.show_frontiers and ctx.get("mapping"):
            self._draw_frontiers(surface, grid)
        self._draw_grid_lines(surface, grid)
        self._draw_markers(surface, fonts, grid)
        # Trail first, planned/completed path on top: the operator needs to see
        # both, and the actual trajectory is the wider stroke underneath.
        if self.show_trail:
            self._draw_trail(surface, ctx.get("trail") or [])
        if self.show_planned:
            self._draw_paths(surface, ctx)
        self._draw_walls(surface, grid)

        state = ctx.get("state")
        live = state is not None and state.valid
        # Always show the placed start pose; the live marker draws over it once
        # the robot actually reports where it is.
        self._draw_placed_robot(surface, fonts, grid)
        if live:
            self._update_smooth(state)
            if self.show_sensors:
                self._draw_sensor_rays(surface, ctx)
            self._draw_robot(surface, state, ctx)
        self._draw_hover(surface, fonts, grid, ctx)

    def _draw_cells(self, surface, grid, ctx):
        truth = ctx.get("ground_truth") if self.reveal_truth else None
        for row in range(grid.height):
            for col in range(grid.width):
                state = grid.get(col, row)
                if state == UNKNOWN:
                    color = CELL_LOOKUP[UNKNOWN]
                    if truth is not None:
                        color = theme.GHOST_TRUTH if truth.get(col, row) != WALL else theme.CELL_WALL
                else:
                    color = CELL_LOOKUP.get(state, theme.CELL_FREE)
                x, y = self.cell_to_px(col, row)
                pygame.draw.rect(surface, color, (x, y, self.cell_px, self.cell_px))

    def _draw_frontiers(self, surface, grid):
        """Outlines cells that still have something unexplored next to them."""
        for col, row in grid.frontier_cells():
            x, y = self.cell_to_px(col, row)
            pygame.draw.rect(surface, theme.CELL_FRONTIER,
                             (x + 2, y + 2, self.cell_px - 4, self.cell_px - 4),
                             1, border_radius=3)

    def _draw_grid_lines(self, surface, grid):
        board = self.board_rect(grid)
        for col in range(grid.width + 1):
            x = self.origin[0] + col * self.cell_px
            pygame.draw.line(surface, theme.GRID_LINE, (x, board.y), (x, board.bottom))
        for row in range(grid.height + 1):
            y = self.origin[1] + row * self.cell_px
            pygame.draw.line(surface, theme.GRID_LINE, (board.x, y), (board.right, y))

    def _draw_unknown_edges(self, surface, grid):
        """Marks edges that have never been looked at, on the frontier of what
        is known.

        Without this an unobserved edge is drawn exactly like a confirmed-open
        one, so a half-explored map reads as a wide-open room and the operator
        cannot tell "there is no wall here" from "I never looked here".
        Only edges next to a known cell are marked - deep inside an unexplored
        region the dark cell colour already says everything.
        """
        from ..occupancy import EDGE_UNKNOWN

        dash = max(3, self.cell_px // 8)
        for row in range(grid.height):
            for col in range(grid.width):
                known_here = grid.get(col, row) != UNKNOWN
                for direction in (0, 3, 1, 2):
                    if direction in (1, 2) and not (
                        (direction == 1 and col == grid.width - 1)
                        or (direction == 2 and row == grid.height - 1)
                    ):
                        continue
                    if grid.edge_state(col, row, direction) != EDGE_UNKNOWN:
                        continue
                    d_col, d_row = ((0, -1), (1, 0), (0, 1), (-1, 0))[direction]
                    neighbour = (col + d_col, row + d_row)
                    known_there = (grid.in_bounds(*neighbour)
                                   and grid.get(*neighbour) != UNKNOWN)
                    if not (known_here or known_there):
                        continue
                    x, y = self.cell_to_px(col, row)
                    if direction == 0:
                        start, end = (x, y), (x + self.cell_px, y)
                    elif direction == 3:
                        start, end = (x, y), (x, y + self.cell_px)
                    elif direction == 1:
                        start, end = (x + self.cell_px, y), (x + self.cell_px, y + self.cell_px)
                    else:
                        start, end = (x, y + self.cell_px), (x + self.cell_px, y + self.cell_px)
                    self._dashed_lines(surface, theme.WALL_EDGE_UNKNOWN, [start, end],
                                       2, dash=dash, gap=dash)

    def _draw_walls(self, surface, grid):
        self._draw_unknown_edges(surface, grid)
        thickness = max(3, self.cell_px // 10)
        for row in range(grid.height):
            for col in range(grid.width):
                x, y = self.cell_to_px(col, row)
                if grid.has_wall(col, row, 0):
                    pygame.draw.line(surface, theme.WALL_EDGE, (x, y), (x + self.cell_px, y), thickness)
                if grid.has_wall(col, row, 3):
                    pygame.draw.line(surface, theme.WALL_EDGE, (x, y), (x, y + self.cell_px), thickness)
                if col == grid.width - 1 and grid.has_wall(col, row, 1):
                    pygame.draw.line(surface, theme.WALL_EDGE, (x + self.cell_px, y),
                                     (x + self.cell_px, y + self.cell_px), thickness)
                if row == grid.height - 1 and grid.has_wall(col, row, 2):
                    pygame.draw.line(surface, theme.WALL_EDGE, (x, y + self.cell_px),
                                     (x + self.cell_px, y + self.cell_px), thickness)

    def _draw_markers(self, surface, fonts, grid):
        def badge(cell, color, letter):
            if cell is None:
                return
            x, y = self.cell_to_px(cell[0], cell[1])
            pygame.draw.rect(surface, color, (x + 2, y + 2, self.cell_px - 4, self.cell_px - 4),
                             border_radius=4)
            img = fonts.small.render(letter, True, theme.TEXT)
            surface.blit(img, img.get_rect(center=(x + self.cell_px / 2, y + self.cell_px / 2)))

        badge(grid.start, theme.CELL_START, "S")
        badge(grid.goal, theme.CELL_GOAL, "G")
        for i, cp in enumerate(grid.checkpoints):
            badge(cp, theme.CELL_CHECKPOINT, str(i + 1))
        self._draw_object_marker(surface, fonts, grid)
        self._draw_place_marker(surface, fonts, grid)

    def _draw_object_marker(self, surface, fonts, grid):
        """The square the object stands on, plus the aim point inside it."""
        cell = grid.object_cell
        if cell is None:
            return
        x, y = self.cell_to_px(cell[0], cell[1])
        rect = pygame.Rect(x + 2, y + 2, self.cell_px - 4, self.cell_px - 4)
        pygame.draw.rect(surface, theme.CELL_OBJECT, rect, border_radius=4)
        pygame.draw.rect(surface, theme.OBJECT_AIM, rect, 2, border_radius=4)
        img = fonts.small.render("O", True, theme.TEXT)
        surface.blit(img, img.get_rect(
            center=(rect.centerx, int(rect.centery - self.cell_px * 0.12))))
        self._draw_aim_dot(surface, cell, grid.object_offset, theme.OBJECT_AIM)

    def _draw_aim_dot(self, surface, cell, offset, colour):
        """Marks the chosen sub-cell aim point inside a marker square."""
        off_x, off_y = offset
        cx, cy = self.cell_center_px(cell[0], cell[1])
        px = cx + off_x * self.cell_px
        py = cy + off_y * self.cell_px
        radius = max(2, int(self.cell_px * 0.07))
        pygame.draw.circle(surface, colour, (int(px), int(py)), radius)
        pygame.draw.circle(surface, theme.TEXT, (int(px), int(py)), radius, 1)

    def _draw_place_marker(self, surface, fonts, grid):
        """Where a carried object is put down, plus the facing it is placed at."""
        cell = grid.delivery_cell
        if cell is None:
            return
        x, y = self.cell_to_px(cell[0], cell[1])
        rect = pygame.Rect(x + 2, y + 2, self.cell_px - 4, self.cell_px - 4)
        pygame.draw.rect(surface, theme.CELL_PLACE, rect, border_radius=4)
        pygame.draw.rect(surface, theme.PLACE_ARROW, rect, 2, border_radius=4)

        cx, cy = rect.center
        img = fonts.small.render("P", True, theme.TEXT)
        surface.blit(img, img.get_rect(center=(cx, cy - self.cell_px * 0.10)))

        # Arrow showing the heading the robot faces while releasing the object.
        angle = math.radians(grid.delivery_dir * 90.0)
        dx, dy = math.sin(angle), -math.cos(angle)
        px, py = -dy, dx
        reach = self.cell_px * 0.30
        base = (cx + dx * reach * 0.2, cy + dy * reach * 0.2)
        tip = (cx + dx * reach, cy + dy * reach)
        left = (tip[0] - dx * reach * 0.45 + px * reach * 0.3,
                tip[1] - dy * reach * 0.45 + py * reach * 0.3)
        right = (tip[0] - dx * reach * 0.45 - px * reach * 0.3,
                 tip[1] - dy * reach * 0.45 - py * reach * 0.3)
        pygame.draw.line(surface, theme.PLACE_ARROW, base, tip, max(2, self.cell_px // 20))
        pygame.draw.polygon(surface, theme.PLACE_ARROW, (tip, left, right))
        self._draw_aim_dot(surface, cell, grid.delivery_offset, theme.PLACE_ARROW)

    def _draw_paths(self, surface, ctx):
        result = ctx.get("path_result")
        if result is None or not result.ok or len(result.cells) < 2:
            return
        executed = min(ctx.get("executed_index", 0), len(result.cells) - 1)
        points = [self.cell_center_px(c, r) for c, r in result.cells]
        width = max(2, self.cell_px // 12)
        if executed >= 1:
            pygame.draw.lines(surface, theme.PATH_DONE, False, points[: executed + 1], width + 1)
        if executed < len(points) - 1:
            self._dashed_lines(surface, theme.PATH_REMAINING, points[executed:], width)
        for point in points:
            pygame.draw.circle(surface, theme.PATH_PLANNED, (int(point[0]), int(point[1])),
                               max(2, self.cell_px // 16))

    @staticmethod
    def _dashed_lines(surface, color, points, width, dash=9, gap=6):
        for i in range(len(points) - 1):
            x1, y1 = points[i]
            x2, y2 = points[i + 1]
            length = math.hypot(x2 - x1, y2 - y1)
            if length < 1:
                continue
            dx, dy = (x2 - x1) / length, (y2 - y1) / length
            pos = 0.0
            while pos < length:
                end = min(pos + dash, length)
                pygame.draw.line(surface, color,
                                 (x1 + dx * pos, y1 + dy * pos),
                                 (x1 + dx * end, y1 + dy * end), width)
                pos = end + gap

    def _draw_trail(self, surface, trail):
        if len(trail) < 2:
            return
        points = [(self.origin[0] + (c + 0.5) * self.cell_px,
                   self.origin[1] + (r + 0.5) * self.cell_px) for c, r in trail]
        pygame.draw.lines(surface, theme.TRAIL, False, points, max(2, self.cell_px // 14))

    def _draw_sensor_rays(self, surface, ctx):
        reading = ctx.get("reading")
        state = ctx.get("state")
        if reading is None or state is None:
            return
        cell_m = ctx.get("cell_size_m", 0.60) or 0.60
        cx = self.origin[0] + (self._draw_col + 0.5) * self.cell_px
        cy = self.origin[1] + (self._draw_row + 0.5) * self.cell_px
        heading = self._draw_heading if self._draw_heading is not None else state.map_heading

        for spec in SENSOR_SPECS:
            value = reading.distance(spec.name)
            if value is None:
                continue
            valid = reading.is_valid(spec.name)
            dist_cells = (value / 1000.0) / cell_m
            dist_cells = min(dist_cells, spec.max_range_m / cell_m)
            length = dist_cells * self.cell_px
            angle = math.radians(heading + spec.angle_deg)
            dx, dy = math.sin(angle), -math.cos(angle)
            end = (cx + dx * length, cy + dy * length)
            color = theme.SENSOR_RAY if valid else theme.TEXT_FAINT
            pygame.draw.line(surface, color, (cx, cy), end, 2)
            # Field of view fan.
            for sign in (-1, 1):
                fan = math.radians(heading + spec.angle_deg + sign * spec.fov_deg / 2.0)
                fx, fy = math.sin(fan), -math.cos(fan)
                pygame.draw.line(surface, theme.blend(color, theme.BG, 0.6),
                                 (cx, cy), (cx + fx * length, cy + fy * length), 1)
            hit = valid and (value / 1000.0) < spec.max_range_m * 0.95
            if hit:
                pygame.draw.circle(surface, theme.SENSOR_HIT, (int(end[0]), int(end[1])), 4)

    def _update_smooth(self, state):
        """Fast exponential follow: smooth to look at, still tracks the real pose."""
        if self._draw_col is None:
            self._draw_col, self._draw_row = state.map_col, state.map_row
            self._draw_heading = state.map_heading
            return
        alpha = 0.45
        self._draw_col += (state.map_col - self._draw_col) * alpha
        self._draw_row += (state.map_row - self._draw_row) * alpha
        delta = wrap180(state.map_heading - self._draw_heading)
        self._draw_heading = wrap180(self._draw_heading + delta * alpha)

    def reset_smooth(self):
        self._draw_col = self._draw_row = self._draw_heading = None

    def _robot_glyph(self, surface, cx, cy, heading_deg, radius, body, outline, filled=True):
        """Draws the robot marker: body plus a heading arrow sticking out past it."""
        angle = math.radians(heading_deg)
        dx, dy = math.sin(angle), -math.cos(angle)
        px, py = -dy, dx

        tip = (cx + dx * radius * 1.9, cy + dy * radius * 1.9)
        base = (cx + dx * radius * 0.7, cy + dy * radius * 0.7)
        left = (base[0] + px * radius * 0.55, base[1] + py * radius * 0.55)
        right = (base[0] - px * radius * 0.55, base[1] - py * radius * 0.55)

        if filled:
            pygame.draw.circle(surface, body, (int(cx), int(cy)), int(radius))
        pygame.draw.circle(surface, outline, (int(cx), int(cy)), int(radius), 2)
        if filled:
            pygame.draw.polygon(surface, outline, (tip, left, right))
        else:
            pygame.draw.polygon(surface, outline, (tip, left, right), 2)
        pygame.draw.line(surface, outline, (cx, cy), base, max(2, int(radius * 0.25)))

    def _draw_robot(self, surface, state, ctx):
        cx = self.origin[0] + (self._draw_col + 0.5) * self.cell_px
        cy = self.origin[1] + (self._draw_row + 0.5) * self.cell_px
        stale = not ctx.get("tracking_ok", True)
        body = theme.ORANGE if stale else theme.ROBOT_BODY
        self._robot_glyph(surface, cx, cy, self._draw_heading, self.cell_px * 0.34,
                          body, theme.ROBOT)

    def _draw_placed_robot(self, surface, fonts, grid):
        """Shows where the robot is *placed* on the map before it is connected.

        Without this the map looks empty until a live pose arrives, and there is
        no way to see which cell and heading the mission will start from.
        """
        cell = grid.robot_cell or grid.start
        if cell is None or not grid.in_bounds(cell[0], cell[1]):
            return
        cx, cy = self.cell_center_px(cell[0], cell[1])
        radius = self.cell_px * 0.34
        self._robot_glyph(surface, cx, cy, grid.robot_dir * 90.0, radius,
                          theme.ROBOT_BODY, theme.ROBOT_PLACED, filled=False)
        label = DIR_NAMES[grid.robot_dir % 4]
        draw_text(surface, fonts.tiny, label,
                  (cx, cy + radius + 2), theme.ROBOT_PLACED, align="center")

    def _draw_hover(self, surface, fonts, grid, ctx):
        if self.hover_cell is None:
            return
        col, row = self.hover_cell
        x, y = self.cell_to_px(col, row)
        pygame.draw.rect(surface, theme.ACCENT, (x, y, self.cell_px, self.cell_px), 2)
        if not self.show_grid_coords:
            return
        transform = ctx.get("transform")
        text = "cell ({}, {})".format(col, row)
        if transform is not None:
            pose = transform.map_to_robot(col, row)
            text += "   robot ({:+.2f} m, {:+.2f} m)".format(pose.x_m, pose.y_m)
        draw_text(surface, fonts.tiny, text, (self.rect.x + 10, self.rect.bottom - 18), theme.TEXT_DIM)


CELL_LOOKUP = {
    UNKNOWN: theme.CELL_UNKNOWN,
    FREE: theme.CELL_FREE,
    WALL: theme.CELL_WALL,
    OBSTACLE: theme.CELL_OBSTACLE,
}
