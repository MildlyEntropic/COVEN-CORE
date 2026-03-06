# SPDX-License-Identifier: MIT
"""
serialization.py - Generic JSON serialization for COVEN message dataclasses

Provides two functions:
- encode(obj) -> str: Serialize any dataclass to JSON
- decode(data, cls) -> Optional[T]: Deserialize JSON to typed dataclass

Replaces 24 individual encode/decode functions with generic implementation.
Uses dataclass introspection for type-safe field handling.

Author: Alexander Shultis
Date: December 2025
"""

import json
import logging
from dataclasses import fields, is_dataclass, MISSING
from typing import TypeVar, Type, Optional, Any, Union, get_type_hints, get_origin, get_args

from std_msgs.msg import String

logger = logging.getLogger(__name__)

T = TypeVar('T')


def encode(obj: Any) -> str:
    """
    Encode a dataclass instance to JSON string.

    Args:
        obj: A dataclass instance

    Returns:
        JSON string representation

    Raises:
        TypeError: If obj is not a dataclass instance
    """
    if not is_dataclass(obj) or isinstance(obj, type):
        raise TypeError(f"encode() requires a dataclass instance, got {type(obj)}")

    return json.dumps(_dataclass_to_dict(obj))


def decode(data: Union[str, String], cls: Type[T]) -> Optional[T]:
    """
    Decode JSON string or ROS String message to a typed dataclass.

    Args:
        data: JSON string or ROS String message with .data attribute
        cls: Target dataclass type

    Returns:
        Decoded dataclass instance, or None if decoding fails
    """
    # Extract string from ROS message if needed
    if hasattr(data, 'data'):
        raw_str = data.data
    else:
        raw_str = data

    # Handle empty string
    if not raw_str:
        logger.debug(f"Empty string passed to decode({cls.__name__})")
        return None

    try:
        parsed = json.loads(raw_str)
    except json.JSONDecodeError as e:
        logger.error(f"Failed to decode {cls.__name__}: {e}")
        logger.debug(f"Malformed data: {raw_str[:100]}...")
        return None

    # JSON must be an object (dict), not array or primitive
    if not isinstance(parsed, dict):
        logger.error(f"Expected JSON object for {cls.__name__}, got {type(parsed).__name__}")
        return None

    try:
        return _dict_to_dataclass(parsed, cls)
    except Exception as e:
        logger.error(f"Failed to construct {cls.__name__}: {e}")
        return None


def _dataclass_to_dict(obj: Any) -> dict:
    """
    Convert a dataclass instance to a JSON-serializable dict.

    Handles nested dataclasses and lists of dataclasses.
    """
    result = {}
    for f in fields(obj):
        value = getattr(obj, f.name)
        result[f.name] = _serialize_value(value)

    return result


def _serialize_value(value: Any) -> Any:
    """
    Recursively serialize a value to JSON-compatible types.

    Handles:
    - None
    - Primitives (str, int, float, bool)
    - Dataclasses
    - Lists (with recursive handling)
    - Dicts (with recursive handling for values)
    - Objects with to_dict() method (e.g., WaypointResult)
    """
    if value is None:
        return None

    if isinstance(value, (str, int, float, bool)):
        return value

    if is_dataclass(value) and not isinstance(value, type):
        # Nested dataclass
        return _dataclass_to_dict(value)

    if isinstance(value, list):
        # List - recursively serialize each item
        return [_serialize_value(item) for item in value]

    if isinstance(value, dict):
        # Dict - recursively serialize values (keys should be strings)
        return {k: _serialize_value(v) for k, v in value.items()}

    if hasattr(value, 'to_dict'):
        # Objects with custom to_dict method (e.g., WaypointResult)
        return value.to_dict()

    # Fallback - try to return as-is (may fail at json.dumps if not serializable)
    return value


def _dict_to_dataclass(data: dict, cls: Type[T]) -> T:
    """
    Convert a dict to a dataclass instance with type coercion.

    Handles:
    - Missing fields with defaults
    - Type coercion (str→int, int→bool, str→float)
    - Nested dataclasses
    - Lists of dataclasses
    """
    kwargs = {}
    type_hints = get_type_hints(cls) if hasattr(cls, '__annotations__') else {}

    for f in fields(cls):
        field_name = f.name
        field_type = type_hints.get(field_name, f.type)

        if field_name in data:
            raw_value = data[field_name]
            kwargs[field_name] = _coerce_value(raw_value, field_type, field_name)
        else:
            # Use default if available
            if f.default is not MISSING:
                kwargs[field_name] = f.default
            elif f.default_factory is not MISSING:
                kwargs[field_name] = f.default_factory()
            else:
                # No default - use type-appropriate empty value
                kwargs[field_name] = _get_default_for_type(field_type)

    return cls(**kwargs)


def _coerce_value(value: Any, target_type: Any, field_name: str) -> Any:
    """
    Coerce a value to the target type.

    Handles:
    - bool (from int, str)
    - int (from str)
    - float (from str, int)
    - str (from anything)
    - Optional[T]
    - List[T] with nested dataclasses
    - Nested dataclasses
    """
    if value is None:
        return None

    # Handle Optional[T] - extract inner type
    origin = get_origin(target_type)
    if origin is Union:
        args = get_args(target_type)
        # Optional[T] is Union[T, None]
        non_none_args = [a for a in args if a is not type(None)]
        if len(non_none_args) == 1:
            target_type = non_none_args[0]
            origin = get_origin(target_type)

    # Handle List[T]
    if origin is list:
        if not isinstance(value, list):
            return value  # Can't coerce non-list to list
        args = get_args(target_type)
        if args:
            inner_type = args[0]
            return [_coerce_value(item, inner_type, field_name) for item in value]
        return value

    # Handle nested dataclass
    if is_dataclass(target_type) and isinstance(value, dict):
        return _dict_to_dataclass(value, target_type)

    # Handle primitive types
    if target_type is bool:
        return bool(value)
    if target_type is int:
        return int(value)
    if target_type is float:
        return float(value)
    if target_type is str:
        return str(value)

    # Handle dict type (for exploration_metrics, etc.)
    if target_type is dict or (origin is dict):
        if isinstance(value, dict):
            return value
        return {}

    return value


def _get_default_for_type(target_type: Any) -> Any:
    """Get a sensible default value for a type when no default is specified."""
    origin = get_origin(target_type)

    # Handle Optional[T]
    if origin is Union:
        return None

    # Handle List[T]
    if origin is list:
        return []

    # Handle dict
    if target_type is dict or origin is dict:
        return {}

    # Handle primitives
    if target_type is str:
        return ""
    if target_type is int:
        return 0
    if target_type is float:
        return 0.0
    if target_type is bool:
        return False

    # Unknown type - return None
    return None
