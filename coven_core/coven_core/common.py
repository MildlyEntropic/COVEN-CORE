# SPDX-License-Identifier: MIT
"""
common.py — COVEN backward-compatibility re-export hub.

All symbols formerly defined here now live in focused modules:
  enums.py    — COLOR_*
  naming.py   — WITCH_NAMES, COVEN_NAMES, get_witch_name, etc.
  messages.py — message dataclasses
  codecs.py   — encode/decode wrapper functions

This file re-exports everything so that existing
``from coven_core.common import X`` and
``import coven_core.common as common`` statements continue to work.

Author: Alexander Shultis
Date: September 2025
"""

from coven_core.enums import *      # noqa: F401,F403
from coven_core.naming import *     # noqa: F401,F403
from coven_core.messages import *   # noqa: F401,F403
from coven_core.codecs import *     # noqa: F401,F403
