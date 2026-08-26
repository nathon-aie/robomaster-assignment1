#!/usr/bin/env python3
"""Small pygame widget set: buttons, numeric fields, progress bars, modals."""

import pygame

from . import theme


def draw_text(surface, font, text, pos, color=theme.TEXT, align="left"):
    img = font.render(str(text), True, color)
    rect = img.get_rect()
    if align == "left":
        rect.topleft = pos
    elif align == "right":
        rect.topright = pos
    else:
        rect.midtop = pos
    surface.blit(img, rect)
    return rect


def draw_panel(surface, rect, title=None, fonts=None, fill=theme.PANEL):
    pygame.draw.rect(surface, fill, rect, border_radius=6)
    pygame.draw.rect(surface, theme.PANEL_EDGE, rect, 1, border_radius=6)
    if title and fonts:
        draw_text(surface, fonts.small, title.upper(), (rect.x + 10, rect.y + 7), theme.TEXT_DIM)
        pygame.draw.line(
            surface, theme.PANEL_EDGE,
            (rect.x + 8, rect.y + 24), (rect.right - 8, rect.y + 24), 1,
        )
    return rect


def draw_kv(surface, fonts, x, y, key, value, width, value_color=theme.TEXT):
    draw_text(surface, fonts.small, key, (x, y), theme.TEXT_DIM)
    draw_text(surface, fonts.small, value, (x + width, y), value_color, align="right")
    return y + 17


class Widget(object):
    def __init__(self, rect):
        self.rect = pygame.Rect(rect)
        self.visible = True

    def handle_event(self, event):
        return False

    def draw(self, surface, fonts):
        pass


class Button(Widget):
    STYLES = {
        "normal": (theme.PANEL_ALT, theme.TEXT, theme.PANEL_EDGE),
        "accent": (theme.ACCENT_DARK, theme.TEXT, theme.ACCENT),
        "danger": (theme.DANGER_DARK, (255, 235, 235), theme.DANGER),
        "ok": ((20, 83, 45), (220, 252, 231), theme.OK),
        "warn": ((113, 63, 18), (254, 249, 195), theme.WARN),
        "ghost": (theme.PANEL, theme.TEXT_DIM, theme.PANEL_EDGE),
    }

    def __init__(self, rect, label, on_click=None, style="normal", enabled=None,
                 active=None, font_key="small", tooltip="", dynamic_label=None):
        Widget.__init__(self, rect)
        self.label = label
        self.on_click = on_click
        self.style = style
        self._enabled = enabled
        self._active = active
        #: Callable returning the caption to draw, for buttons that show state.
        self._dynamic_label = dynamic_label
        self.font_key = font_key
        self.tooltip = tooltip
        self.hover = False
        self._pressed = False

    def text(self):
        if self._dynamic_label is not None:
            try:
                return self._dynamic_label()
            except Exception:
                pass
        return self.label

    def enabled(self):
        if self._enabled is None:
            return True
        return bool(self._enabled())

    def active(self):
        if self._active is None:
            return False
        return bool(self._active())

    def handle_event(self, event):
        if not self.visible:
            return False
        if event.type == pygame.MOUSEMOTION:
            self.hover = self.rect.collidepoint(event.pos)
        elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if self.rect.collidepoint(event.pos) and self.enabled():
                self._pressed = True
                return True
        elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            if self._pressed:
                self._pressed = False
                if self.rect.collidepoint(event.pos) and self.enabled() and self.on_click:
                    self.on_click()
                    return True
        return False

    def draw(self, surface, fonts):
        if not self.visible:
            return
        bg, fg, edge = self.STYLES.get(self.style, self.STYLES["normal"])
        enabled = self.enabled()
        active = self.active()
        if not enabled:
            bg = theme.blend(bg, theme.BG, 0.55)
            fg = theme.TEXT_FAINT
            edge = theme.blend(edge, theme.BG, 0.6)
        elif active:
            bg = theme.blend(bg, edge, 0.55)
        elif self.hover:
            bg = theme.blend(bg, (255, 255, 255), 0.10)
        if self._pressed and enabled:
            bg = theme.blend(bg, (0, 0, 0), 0.25)

        pygame.draw.rect(surface, bg, self.rect, border_radius=5)
        pygame.draw.rect(surface, edge, self.rect, 2 if active else 1, border_radius=5)
        font = getattr(fonts, self.font_key, fonts.small)
        img = font.render(self.text(), True, fg)
        surface.blit(img, img.get_rect(center=self.rect.center))


class NumberInput(Widget):
    """Small integer field used for map width/height."""

    def __init__(self, rect, label, value, on_commit=None, min_value=1, max_value=200):
        Widget.__init__(self, rect)
        self.label = label
        self.value = int(value)
        self.text = str(int(value))
        self.on_commit = on_commit
        self.min_value = min_value
        self.max_value = max_value
        self.focused = False

    def set_value(self, value):
        self.value = int(value)
        self.text = str(self.value)

    def _commit(self):
        try:
            value = int(self.text)
        except ValueError:
            value = self.value
        value = max(self.min_value, min(self.max_value, value))
        self.value = value
        self.text = str(value)
        if self.on_commit:
            self.on_commit(value)

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            inside = self.rect.collidepoint(event.pos)
            if inside != self.focused:
                if self.focused:
                    self._commit()
                self.focused = inside
            return inside
        if not self.focused:
            return False
        if event.type == pygame.KEYDOWN:
            if event.key in (pygame.K_RETURN, pygame.K_KP_ENTER, pygame.K_TAB):
                self._commit()
                self.focused = False
            elif event.key == pygame.K_BACKSPACE:
                self.text = self.text[:-1]
            elif event.key == pygame.K_ESCAPE:
                self.text = str(self.value)
                self.focused = False
            elif event.unicode and event.unicode.isdigit() and len(self.text) < 4:
                self.text += event.unicode
            return True
        return False

    def draw(self, surface, fonts):
        draw_text(surface, fonts.tiny, self.label, (self.rect.x, self.rect.y - 13), theme.TEXT_DIM)
        edge = theme.ACCENT if self.focused else theme.PANEL_EDGE
        pygame.draw.rect(surface, theme.BG, self.rect, border_radius=4)
        pygame.draw.rect(surface, edge, self.rect, 1, border_radius=4)
        shown = self.text + ("_" if self.focused else "")
        draw_text(surface, fonts.small, shown, (self.rect.x + 7, self.rect.y + 5))


class ProgressBar(Widget):
    def __init__(self, rect, color=theme.ACCENT):
        Widget.__init__(self, rect)
        self.value = 0.0
        self.color = color

    def draw(self, surface, fonts):
        pygame.draw.rect(surface, theme.BG, self.rect, border_radius=4)
        pygame.draw.rect(surface, theme.PANEL_EDGE, self.rect, 1, border_radius=4)
        width = int((self.rect.width - 4) * max(0.0, min(1.0, self.value)))
        if width > 0:
            inner = pygame.Rect(self.rect.x + 2, self.rect.y + 2, width, self.rect.height - 4)
            pygame.draw.rect(surface, self.color, inner, border_radius=3)


class Modal(object):
    """Blocking confirmation dialog - used for anything that can move hardware."""

    def __init__(self, title, lines, on_confirm, confirm_label="CONFIRM",
                 confirm_style="danger", on_cancel=None):
        self.title = title
        self.lines = lines
        self.on_confirm = on_confirm
        self.on_cancel = on_cancel
        self.confirm_label = confirm_label
        self.confirm_style = confirm_style
        self.rect = pygame.Rect(0, 0, 480, 40 + 22 * len(lines) + 80)
        self.confirm_btn = Button((0, 0, 160, 38), confirm_label, self._confirm, confirm_style,
                                  font_key="body")
        self.cancel_btn = Button((0, 0, 160, 38), "CANCEL", self._cancel, "ghost", font_key="body")
        self.done = False

    def _confirm(self):
        self.done = True
        if self.on_confirm:
            self.on_confirm()

    def _cancel(self):
        self.done = True
        if self.on_cancel:
            self.on_cancel()

    def layout(self, screen_rect):
        self.rect.center = screen_rect.center
        self.confirm_btn.rect.bottomright = (self.rect.right - 16, self.rect.bottom - 16)
        self.cancel_btn.rect.bottomright = (self.confirm_btn.rect.left - 10, self.rect.bottom - 16)

    def handle_event(self, event):
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                self._cancel()
                return True
            if event.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
                self._confirm()
                return True
        self.confirm_btn.handle_event(event)
        self.cancel_btn.handle_event(event)
        return True  # modal swallows everything

    def draw(self, surface, fonts):
        overlay = pygame.Surface(surface.get_size(), pygame.SRCALPHA)
        overlay.fill((0, 0, 0, 170))
        surface.blit(overlay, (0, 0))
        pygame.draw.rect(surface, theme.PANEL, self.rect, border_radius=8)
        pygame.draw.rect(surface, theme.DANGER if self.confirm_style == "danger" else theme.ACCENT,
                         self.rect, 2, border_radius=8)
        draw_text(surface, fonts.h1, self.title, (self.rect.x + 20, self.rect.y + 16))
        y = self.rect.y + 46
        for line in self.lines:
            draw_text(surface, fonts.small, line, (self.rect.x + 20, y), theme.TEXT_DIM)
            y += 22
        self.confirm_btn.draw(surface, fonts)
        self.cancel_btn.draw(surface, fonts)
