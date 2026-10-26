"""ForceHand: simulated Inspire hand with a force model (shared by mock_p3_test.py and mock_p3b_test.py; no robot access)."""
import sys
import time

sys.path.insert(0, "/home/unitree/h2_20261007/tools")
import test_h2_v2_offline as T2  # noqa: E402


class ForceHand(T2.Hand2):
    def __init__(self, pos, contact=None, base=None):
        super().__init__(pos, [0, 0, 0, 0, 0, 1000], [1000] * 6)
        self.contact = contact or [None] * 6
        self.base = base or [60, 110, 90, 20, -100, 90]
        self.force_set = [2000] * 6
        self.err = [0] * 6
        self.on_force_write = None                 # test hook: fault injection

    def force(self, k):
        c = self.contact[k]
        return self.base[k] + (int(4 * (c - self.pos[k])) if c is not None and self.pos[k] < c else 0)

    def kin(self):
        while True:
            time.sleep(0.003)
            with self.lock:
                for k in range(6):
                    t = self.target[k]
                    if t is None:
                        continue
                    eff = t - (15 if k == 5 else 0)
                    d = eff - self.pos[k]
                    if d < -0.5 and self.force(k) >= self.force_set[k]:
                        self.status[k], self.cur[k] = 3, 150
                        continue
                    if abs(d) > 0.5 and self.speed[k] > 0:
                        self.pos[k] += max(-4.0 * self.speed[k] * 0.003, min(4.0 * self.speed[k] * 0.003, d))
                        self.status[k], self.cur[k] = (0 if d > 0 else 1), (100 if k < 4 else 20)
                    else:
                        self.status[k], self.cur[k] = 2, 0

    def read(self, a, n):
        if a == 1582:
            with self.lock:
                return [self.force(k) & 0xFFFF for k in range(6)]
        if a == 1498:
            with self.lock:
                return list(self.force_set)
        if a == 1606:
            with self.lock:
                return self.words(self.err)
        return super().read(a, n)

    def w06(self, a, v):
        with self.lock:
            if 1498 <= a < 1510:
                self.force_set[(a - 1498) // 2] = v
                if self.on_force_write:
                    self.on_force_write((a - 1498) // 2, v)
                return
        super().w06(a, v)
