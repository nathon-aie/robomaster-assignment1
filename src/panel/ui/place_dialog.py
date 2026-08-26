#!/usr/bin/env python3
"""Sub-cell aiming windows for the object square and the delivery square.

A grid cell is 60 cm across but a bottle is a few centimetres wide, so "it is
in that square" is not precise enough for either end of the job.  This opens a
zoomed view of one marker cell, divided into sub-squares, and clicking one sets
the exact spot inside the cell:

    TARGET: OBJECT     where the object actually stands, so the robot knows
                       which side to come at it from and how far to reach
    TARGET: DELIVERY   where the object should be released, plus the heading
                       the robot faces while letting go

Aim points are stored on the map in screen axes (+x East, +y South) as
fractions of a cell, so they rotate with the map and are saved with it.
"""

import pygame

from ..geometry import DIR_LONG, DIR_VECTORS
from . import theme
from .widgets import Button, draw_text

#: What each kind of target reads and writes on the map.
OBJECT_TARGET = "object"
DELIVERY_TARGET = "delivery"


class SubCellTargetDialog(object):
    """Zoomed sub-cell picker for one marker square."""

    #: Sub-squares per side.  Odd, so there is a true centre square.
    #: 9 across a 60 cm cell is ~6.7 cm per square - finer than the object.
    DIVISIONS = 9

    def __init__(self, grid, kind=DELIVERY_TARGET, on_change=None, cell_size_m=0.60):
        self.grid = grid
        self.kind = kind
        self.on_change = on_change
        self.cell_size_m = cell_size_m
        self.done = False
        self.rect = pygame.Rect(0, 0, 500, 600)
        self.board = pygame.Rect(0, 0, 360, 360)
        self.close_btn = Button((0, 0, 130, 34), "DONE", self._close, "accent",
                                font_key="body")
        self.centre_btn = Button((0, 0, 130, 34), "CENTRE", self._recentre, "ghost",
                                 font_key="body")
        # Only the delivery target has a facing: the robot has to end up
        # pointing somewhere to let go.  An object just sits where it sits.
        self.turn_btn = Button((0, 0, 130, 34), "TURN", self._turn, "warn",
                               font_key="body")
        self.turn_btn.visible = self.has_facing

    # ------------------------------------------------------------------ fields
    @property
    def has_facing(self):
        return self.kind == DELIVERY_TARGET

    @property
    def title(self):
        return "TARGET: {}".format(self.kind.upper())

    @property
    def accent(self):
        return theme.PLACE_ARROW if self.has_facing else theme.OBJECT_AIM

    @property
    def cell(self):
        return self.grid.delivery_cell if self.has_facing else self.grid.object_cell

    def _get_offset(self):
        return (self.grid.delivery_offset if self.has_facing
                else self.grid.object_offset)

    def _set_offset(self, offset):
        if self.has_facing:
            self.grid.delivery_offset = offset
        else:
            self.grid.object_offset = offset

    # ----------------------------------------------------------------- actions
    def _close(self):
        self.done = True

    def _recentre(self):
        self._set_offset((0.0, 0.0))
        self._changed()

    def _turn(self):
        self.grid.delivery_dir = (self.grid.delivery_dir + 1) % 4
        self._changed()

    def _changed(self):
        if self.on_change:
            self.on_change()

    # ------------------------------------------------------------------ layout
    def layout(self, screen_rect):
        self.rect.center = screen_rect.center
        self.board.centerx = self.rect.centerx
        self.board.y = self.rect.y + 92
        row_y = self.rect.bottom - 50
        self.centre_btn.rect.topleft = (self.rect.x + 20, row_y)
        self.turn_btn.rect.topleft = (self.centre_btn.rect.right + 10, row_y)
        self.close_btn.rect.topright = (self.rect.right - 20, row_y)

    def _sub_rect(self, col, row):
        size = self.board.width / float(self.DIVISIONS)
        return pygame.Rect(int(self.board.x + col * size), int(self.board.y + row * size),
                           int(size) + 1, int(size) + 1)

    def _offset_for(self, col, row):
        """Sub-square indices -> offset in cell fractions from the centre."""
        step = 1.0 / self.DIVISIONS
        return ((col + 0.5) * step - 0.5, (row + 0.5) * step - 0.5)

    def _selected_indices(self):
        off_x, off_y = self._get_offset()
        step = 1.0 / self.DIVISIONS
        col = int(min(self.DIVISIONS - 1, max(0, (off_x + 0.5) / step)))
        row = int(min(self.DIVISIONS - 1, max(0, (off_y + 0.5) / step)))
        return col, row

    # ------------------------------------------------------------------ events
    def handle_event(self, event):
        if event.type == pygame.KEYDOWN and event.key in (pygame.K_ESCAPE,
                                                          pygame.K_RETURN,
                                                          pygame.K_KP_ENTER):
            self._close()
            return True
        for button in (self.close_btn, self.centre_btn, self.turn_btn):
            if button.visible:
                button.handle_event(event)
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if self.board.collidepoint(event.pos):
                size = self.board.width / float(self.DIVISIONS)
                col = int((event.pos[0] - self.board.x) / size)
                row = int((event.pos[1] - self.board.y) / size)
                col = max(0, min(self.DIVISIONS - 1, col))
                row = max(0, min(self.DIVISIONS - 1, row))
                self._set_offset(self._offset_for(col, row))
                self._changed()
        return True   # modal: swallow everything else

    # ----------------------------------------------------------------- drawing
    def draw(self, surface, fonts):
        overlay = pygame.Surface(surface.get_size(), pygame.SRCALPHA)
        overlay.fill((0, 0, 0, 180))
        surface.blit(overlay, (0, 0))
        pygame.draw.rect(surface, theme.PANEL, self.rect, border_radius=8)
        pygame.draw.rect(surface, self.accent, self.rect, 2, border_radius=8)

        draw_text(surface, fonts.h1, self.title, (self.rect.x + 20, self.rect.y + 16))
        subtitle = "cell {}".format(self.cell)
        if self.has_facing:
            subtitle += "   facing {}".format(DIR_LONG[self.grid.delivery_dir % 4])
        draw_text(surface, fonts.small, subtitle,
                  (self.rect.x + 20, self.rect.y + 44), theme.TEXT_DIM)
        what = ("release the object on" if self.has_facing
                else "the object actually stands on")
        draw_text(surface, fonts.tiny,
                  "Click the sub-square {} ({}x{} grid)".format(
                      what, self.DIVISIONS, self.DIVISIONS),
                  (self.rect.x + 20, self.rect.y + 66), theme.TEXT_FAINT)

        pygame.draw.rect(surface, theme.CELL_FREE, self.board)
        selected = self._selected_indices()
        fill = theme.CELL_PLACE if self.has_facing else theme.CELL_OBJECT
        for row in range(self.DIVISIONS):
            for col in range(self.DIVISIONS):
                rect = self._sub_rect(col, row)
                if (col, row) == selected:
                    pygame.draw.rect(surface, fill, rect)
                    pygame.draw.rect(surface, self.accent, rect, 2)
                else:
                    pygame.draw.rect(surface, theme.GRID_LINE, rect, 1)
        pygame.draw.rect(surface, theme.WALL_EDGE, self.board, 3)

        if self.has_facing:
            self._draw_facing(surface, fonts)

        off_x, off_y = self._get_offset()
        draw_text(surface, fonts.small,
                  "offset  {:+.0f} cm E   {:+.0f} cm S".format(
                      off_x * self.cell_size_m * 100, off_y * self.cell_size_m * 100),
                  (self.rect.x + 20, self.board.bottom + 40), theme.TEXT)
        draw_text(surface, fonts.tiny, "north is up, same as the map",
                  (self.rect.x + 20, self.board.bottom + 60), theme.TEXT_FAINT)

        for button in (self.centre_btn, self.turn_btn, self.close_btn):
            if button.visible:
                button.draw(surface, fonts)

    def _draw_facing(self, surface, fonts):
        d_col, d_row = DIR_VECTORS[self.grid.delivery_dir % 4]
        cx, cy = self.board.center
        reach = self.board.width * 0.5 + 18
        tip = (cx + d_col * reach, cy + d_row * reach)
        pygame.draw.circle(surface, theme.PLACE_ARROW, (int(tip[0]), int(tip[1])), 7)
        draw_text(surface, fonts.tiny, "facing", (int(tip[0]), int(tip[1]) + 10),
                  theme.PLACE_ARROW, align="center")


#: Older name, kept so existing imports keep working.
PlaceTargetDialog = SubCellTargetDialog
