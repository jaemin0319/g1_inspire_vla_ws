"""pygame status window + keyboard. Keys are read ONLY while this window has focus (unlike the official
send_commands_keyboard.py, whose pynput listener is global and reacts to typing in any window).
Key-up is a real release event, so walking stops when a key is released."""
from __future__ import annotations

from g1iv_common.loco import KeyState

ONE_SHOT = ("engage_toggle", "recenter", "hand_left_toggle", "hand_right_toggle", "hand_both_toggle", "arm_select",
            "record_start", "record_stop_success", "record_stop_failure", "record_abort", "reset_env", "quit")


class Console:
    def __init__(self, keys: dict, title: str = "G1IV teleop hub", size=(760, 500)):
        import os

        os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
        import pygame

        self.pg = pygame
        pygame.init()
        self.screen = pygame.display.set_mode(size)
        pygame.display.set_caption(title)
        self.font = pygame.font.SysFont("monospace", 15)
        self.big = pygame.font.SysFont("monospace", 20, bold=True)
        kc = pygame.key.key_code
        self.code = {}
        for action, name in keys.items():
            if action == "arm_nudge":
                continue
            self.code[kc(str(name))] = action
        self.nudge_codes = [kc(k) for k in keys.get("arm_nudge", [])]
        self.walk = {a: kc(keys[a]) for a in ("walk_forward", "walk_back", "walk_left", "walk_right", "turn_left", "turn_right", "stop")}

    def poll(self):
        pg = self.pg
        events, quit_ = [], False
        nudge = [0, 0, 0]
        for e in pg.event.get():
            if e.type == pg.QUIT:
                quit_ = True
            elif e.type == pg.KEYDOWN:
                a = self.code.get(e.key)
                if a in ONE_SHOT:
                    events.append(a)
                if e.key in self.nudge_codes:
                    i = self.nudge_codes.index(e.key)
                    nudge[i // 2] += 1 if i % 2 == 0 else -1
        p = pg.key.get_pressed()
        focused = bool(pg.key.get_focused())
        ks = KeyState(forward=p[self.walk["walk_forward"]] or p[pg.K_UP], back=p[self.walk["walk_back"]] or p[pg.K_DOWN],
                      left=p[self.walk["walk_left"]] or p[pg.K_LEFT], right=p[self.walk["walk_right"]] or p[pg.K_RIGHT],
                      turn_left=p[self.walk["turn_left"]], turn_right=p[self.walk["turn_right"]], stop=p[self.walk["stop"]],
                      focused=focused)
        if "quit" in events:
            quit_ = True
        return ks, events, nudge, quit_

    def draw(self, lines):
        pg = self.pg
        self.screen.fill((18, 18, 22))
        y = 8
        for item in lines:
            text, color = (item, (220, 220, 220)) if isinstance(item, str) else item
            f = self.big if text.startswith("#") else self.font
            self.screen.blit(f.render(text.lstrip("#"), True, color), (10, y))
            y += 24 if text.startswith("#") else 19
        pg.display.flip()

    def close(self):
        self.pg.quit()
