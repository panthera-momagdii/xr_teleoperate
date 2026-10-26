"""Headset video knobs and a closed-socket guard for televuer's ZMQ image task. RT5b.

televuer is an upstream submodule, so its image task is replaced from here instead of
patched in place (a `git submodule update` would silently revert a submodule edit).
`install()` swaps `TeleVuer.main_image_monocular_zmq` for the version below. It must
run BEFORE TeleVuerWrapper is constructed: TeleVuer.__init__ binds the method and
hands it to vuer, and the vuer server is a forked child that keeps what it was given.

Why
---
1. Bandwidth. Upstream re-encodes the head image as JPEG quality 80 at display_fps
   (30) and sends it whether or not a new camera frame arrived. RT5 measured about
   20 Mbit/s on the 8012 websocket through the laptop forward, with 267..325 ms TCP RTT
   and three websocket drops while the VR session started. The knobs:

       XR_DISPLAY_FPS     frames per second sent to the headset (default: televuer's)
       XR_JPEG_QUALITY    1..95                                  (default: 80)
       XR_DISPLAY_SCALE   0 < s <= 1, frame size factor          (default: 1.0)

   At the defaults the output is what upstream sends.

2. "Websocket session is missing". Upstream's loop never checks that its websocket
   is still open; after a drop the next upsert hits vuer's assertion and the task
   dies with a traceback in the operator's pane ("Task exception was never
   retrieved", 9 times in RT5). Here the loop ends quietly when its session is gone.
   A reconnect gets a new task from vuer, exactly as before.

Only the monocular ZMQ path is replaced: it is the one the PC2 head camera uses.
"""

import asyncio
import os


class TelevuerStreamError(ValueError):
    pass


def settings_from_env():
    def _num(name, default, cast, lo, hi, lo_open=False):
        raw = os.environ.get(name)
        if raw is None or raw.strip() == "":
            return default
        try:
            v = cast(raw)
        except ValueError:
            raise TelevuerStreamError(f"{name}={raw!r} is not a number") from None
        if v != v or v > hi or v < lo or (lo_open and v == lo):
            raise TelevuerStreamError(f"{name}={raw!r} must be in "
                                      f"{'(' if lo_open else '['}{lo}, {hi}]")
        return v
    return {"fps": _num("XR_DISPLAY_FPS", None, float, 0.0, 120.0, lo_open=True),
            "quality": _num("XR_JPEG_QUALITY", 80, int, 1, 95),
            "scale": _num("XR_DISPLAY_SCALE", 1.0, float, 0.0, 1.0, lo_open=True)}


def install(settings=None):
    """Replace TeleVuer.main_image_monocular_zmq. Returns a one-line description."""
    s = settings_from_env() if settings is None else settings
    import cv2
    from televuer import televuer as tvm
    from vuer.schemas import ImageBackground, Hands, MotionControllers

    fps, quality, scale = s["fps"], int(s["quality"]), float(s["scale"])

    async def main_image_monocular_zmq(self, session):
        def alive():
            try:
                return session.CURRENT_WS_ID in session.vuer.ws
            except Exception:
                return False

        try:
            if self.use_hand_tracking:
                session.upsert(Hands(stream=True, key="hands", hideLeft=True, hideRight=True),
                               to="bgChildren")
            else:
                session.upsert(MotionControllers(stream=True, key="motionControllers",
                                                 left=True, right=True),
                               to="bgChildren")
        except AssertionError:
            return                      # the websocket closed before we started

        period = 1.0 / (fps if fps else self.display_fps)
        while alive():
            img = self.img2display
            if scale < 1.0:
                h, w = img.shape[:2]
                img = cv2.resize(img, (max(1, int(w * scale)), max(1, int(h * scale))),
                                 interpolation=cv2.INTER_AREA)
            try:
                session.upsert([ImageBackground(img, aspect=self.aspect_ratio, height=1,
                                                distanceToCamera=1, format="jpeg",
                                                quality=quality, key="background-mono",
                                                interpolate=True)],
                               to="bgChildren")
            except AssertionError:
                return                  # closed between the check and the send
            await asyncio.sleep(period)

    tvm.TeleVuer.main_image_monocular_zmq = main_image_monocular_zmq
    return (f"[video] headset stream: {'televuer default' if not fps else f'{fps:g}'} fps, "
            f"JPEG quality {quality}, scale {scale:g}; stops quietly when its websocket closes")
