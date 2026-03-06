# SPDX-License-Identifier: MIT
"""
naming.py — COVEN witch naming system.

Modules are named after famous witches from mythology, literature, and pop culture.
The dock (coven leader) is named after famous covens or witch gatherings.

Author: Alexander Shultis
Date: September 2025
"""

import random
import threading
import time
from typing import Optional, Dict


# Witches for module naming (rovers/modules)
WITCH_NAMES = [
    # Arthurian
    "Morgan_Le_Fay",
    # Greek
    "Hecate",
    "Circe",
    # Celtic
    "Scathach",
    "Morrigan",
    # Germanic
    "Lorelei",
    "Frau_Holle",
    # Finnish
    "Louhi",
    # Slavic
    "Baba_Yaga",
    # West African
    "Mami_Wata",
    # Japanese
    "Princess_Kaguya",
    # Wizard of Oz
    "Elphaba",
    "Glinda",
    # Marvel
    "Wanda_Maximoff",
    "Agatha_Harkness",
    # DC Comics
    "Zatanna_Zatara",
    # Harry Potter
    "Hermione_Granger",
    "Minerva_McGonagall",
    # Sabrina
    "Sabrina_Spellman",
    # Buffy
    "Willow_Rosenberg",
    # Disney
    "Maleficent",
    # Bewitched
    "Endora",
    "Samantha_Stephens",
    # Studio Ghibli
    "Kiki",
    "Yubaba",
    # Little Witch Academia
    "Akko",
    # Witch Watch
    "Nico_Wakatsuki",
    # Star Wars (Dathomir)
    "Mother_Talzin",
    "Old_Daka",
    "Axkva_Min",
]

# Coven names for dock/hub naming (famous witch groups/sisterhoods)
COVEN_NAMES = [
    # Greek
    "The_Graeae",
    "The_Erinyes",
    # Norse
    "The_Norns",
    # Shakespeare
    "The_Weird_Sisters",
    # Hocus Pocus
    "The_Sanderson_Sisters",
    # Scooby-Doo
    "The_Hex_Girls",
    # The Witcher
    "The_Crones",
    # Brave
    "The_Hags_of_Dun_Broch",
    # Dune
    "The_Bene_Gesserit",
    # Stardust
    "The_Lilim",
]

# Track used names with lifecycle management
# Structure: {name: {"status": "active"|"missing"|"available", "last_seen": timestamp, "timeout": seconds}}
_witch_name_registry: Dict[str, dict] = {}
_used_coven_names: set = set()
_registry_lock = threading.Lock()


def get_witch_name(returning_name: Optional[str] = None) -> str:
    """
    Get a witch name for a module, supporting reconnection scenarios.

    Lifecycle states:
    - "active": Currently connected rover
    - "missing": Rover disconnected but within timeout window (held for reabsorption)
    - "available": Can be assigned to new rovers

    Args:
        returning_name: If rover claims to be a known witch, check if it can reclaim

    Returns:
        A witch name string (e.g., "Baba_Yaga", "Hermione_Granger")
    """
    global _witch_name_registry
    now = time.time()

    with _registry_lock:
        # Check if this is a returning rover trying to reclaim their name
        if returning_name and returning_name in _witch_name_registry:
            entry = _witch_name_registry[returning_name]
            if entry["status"] in ("missing", "active"):
                # Welcome back! Reabsorb the rover
                entry["status"] = "active"
                entry["last_seen"] = now
                return returning_name
            # Name was released and possibly reassigned - fall through to assign new name

        # First, check for any "missing" rovers that have exceeded timeout
        # and mark them as "available" for reassignment
        _cleanup_expired_names_unlocked()

        # Get available names (not in registry, or marked available)
        used_names = {name for name, entry in _witch_name_registry.items()
                      if entry["status"] in ("active", "missing")}
        available = [n for n in WITCH_NAMES if n not in used_names]

        # If all names exhausted, look for "missing" names past their timeout
        if not available:
            # Force cleanup and try again
            available = [n for n in WITCH_NAMES if n not in
                        {name for name, entry in _witch_name_registry.items()
                         if entry["status"] == "active"}]

        # If STILL no names (all active), reset entirely (shouldn't happen with reasonable fleet)
        if not available:
            _witch_name_registry.clear()
            available = WITCH_NAMES.copy()

        # Pick randomly from available
        name = random.choice(available)
        _witch_name_registry[name] = {
            "status": "active",
            "last_seen": now,
            "timeout": 0.0,  # Set when rover disconnects
        }
        return name


def mark_witch_missing(name: str, timeout_secs: float):
    """
    Mark a witch as missing (disconnected) but hold their name for potential return.

    Args:
        name: The witch name to mark as missing
        timeout_secs: How long to hold the name before allowing reassignment
    """
    global _witch_name_registry
    with _registry_lock:
        if name in _witch_name_registry:
            _witch_name_registry[name]["status"] = "missing"
            _witch_name_registry[name]["last_seen"] = time.time()
            _witch_name_registry[name]["timeout"] = timeout_secs


def mark_witch_active(name: str):
    """Mark a witch as active (connected)."""
    global _witch_name_registry
    with _registry_lock:
        if name in _witch_name_registry:
            _witch_name_registry[name]["status"] = "active"
            _witch_name_registry[name]["last_seen"] = time.time()


def release_witch_name(name: str):
    """Immediately release a witch name for reassignment."""
    global _witch_name_registry
    with _registry_lock:
        if name in _witch_name_registry:
            _witch_name_registry[name]["status"] = "available"


def is_witch_known(name: str) -> bool:
    """Check if a witch name is known to the system (active or missing)."""
    with _registry_lock:
        return name in _witch_name_registry and _witch_name_registry[name]["status"] in ("active", "missing")


def get_witch_status(name: str) -> Optional[str]:
    """Get the status of a witch name, or None if not in registry."""
    with _registry_lock:
        if name in _witch_name_registry:
            return _witch_name_registry[name]["status"]
        return None


def _cleanup_expired_names_unlocked():
    """Mark any 'missing' names that have exceeded their timeout as 'available'.

    Must be called while holding _registry_lock.
    """
    global _witch_name_registry
    now = time.time()
    for name, entry in _witch_name_registry.items():
        if entry["status"] == "missing":
            elapsed = now - entry["last_seen"]
            if elapsed > entry["timeout"]:
                entry["status"] = "available"


def get_coven_name() -> str:
    """
    Get a random available coven name for a dock/hub.

    Returns unique names randomly selected from the pool.
    Once all names are used, the pool resets.

    Returns:
        A coven name string (e.g., "The_Graeae", "The_Norns")
    """
    global _used_coven_names

    with _registry_lock:
        # Get available names (not yet used)
        available = [n for n in COVEN_NAMES if n not in _used_coven_names]

        # If all names used, reset the pool
        if not available:
            _used_coven_names.clear()
            available = COVEN_NAMES.copy()

        # Pick randomly from available
        name = random.choice(available)
        _used_coven_names.add(name)
        return name


def reset_naming():
    """Reset naming system (useful for testing)."""
    global _witch_name_registry, _used_coven_names
    with _registry_lock:
        _witch_name_registry.clear()
        _used_coven_names.clear()
