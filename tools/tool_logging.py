"""Shared logging setup for the operator tools.

Timestamps are monotonic seconds since the tool started, not wall clock: these
logs get read next to DDS traces where only elapsed time is meaningful, and wall
clock can step under NTP mid-session.
"""

import logging
import os
import time

_T0 = time.monotonic()

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOG_DIR = os.path.join(REPO_ROOT, "logs")


class MonotonicFormatter(logging.Formatter):
    def format(self, record):
        record.mono = f"{time.monotonic() - _T0:9.3f}"
        return super().format(record)


def timestamp():
    """Filename-safe wall-clock stamp, for naming artefacts only."""
    return time.strftime("%Y%m%d_%H%M%S")


def setup_tool_logger(name, filename=None):
    os.makedirs(LOG_DIR, exist_ok=True)
    path = os.path.join(LOG_DIR, filename or f"{name}_{timestamp()}.log")

    logger = logging.getLogger(name)
    logger.setLevel(logging.INFO)
    logger.handlers.clear()
    logger.propagate = False

    fmt = MonotonicFormatter("[%(mono)ss] %(levelname)-7s %(message)s")
    for handler in (logging.StreamHandler(), logging.FileHandler(path)):
        handler.setFormatter(fmt)
        logger.addHandler(handler)

    logger.info(f"log file: {path}")
    return logger
