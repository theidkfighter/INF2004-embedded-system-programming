"""Finds the Laptop_Codec for the laptop-track tests (MQTT/ is not a package)."""

import sys
from pathlib import Path

REPO_DIR = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO_DIR / "MQTT"))

import is02_codec  # noqa: E402  (needs the path above)
