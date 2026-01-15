# showrunner.py
import time
from typing import Callable, List, Tuple

Event = Tuple[float, Callable[[], None]]  # (offset_seconds, callback)

class ShowRunner:
    """
    Tiny scheduler:
      - add(t, fn): run fn at absolute offset t (sec) from start
      - run(): blocking until all queued events fire
    """
    def __init__(self):
        self.events: List[Event] = []

    def add(self, offset_s: float, fn: Callable[[], None]):
        self.events.append((float(offset_s), fn))

    def run(self, start_delay_s: float = 0.0):
        base = time.time() + float(start_delay_s)
        for t, fn in sorted(self.events, key=lambda e: e[0]):
            wait = base + t - time.time()
            if wait > 0: time.sleep(wait)
            try:
                fn()
            except Exception as e:
                print("[ShowRunner] event error:", e)

if __name__ == "__main__":
    print("showrunner ready — ShowRunner().add(offset, lambda: action()); .run()")
