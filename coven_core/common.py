"""
common.py — COVEN Phase 1

Shared definitions, message dataclasses, and encode/decode helpers
for the COVEN docking + module FSM system.

Responsibilities:
- Define DockState and ModuleState enums.
- Provide dataclasses for Identify, Verify, Heartbeat, and Task messages.
- Provide JSON encode/decode helpers with error handling.

Author: Alexander Shultis
Date: September 2025
"""

# ------------------------
# --- Imports ---
# ------------------------
# --- Standard library ---
import json
import logging
from dataclasses import dataclass
from enum import Enum

# --- Third-party (ROS2) ---
from std_msgs.msg import String

# Module-level logger
logger = logging.getLogger(__name__)


# ------------------------
# --- Enums ---
# ------------------------
class DockState(Enum):
    """FSM states for the docking hub."""

    IDLE = 0
    DETECTED = 1
    IDENTIFY = 2
    VERIFY = 3
    ENABLED = 4
    NORMAL = 5
    REJECTED = 6


class ModuleState(Enum):
    """FSM states for a COVEN module."""

    BOOT = 0
    IDENTIFY = 1
    WAIT_VERIFY = 2
    NORMAL = 3
    REJECTED = 4
    DISCONNECTED = 5
    FIELD_OPS = 6


# ------------------------
# --- ANSI Color Codes ---
# ------------------------
COLOR_GREEN  = "\033[92m"
COLOR_YELLOW = "\033[93m"
COLOR_ORANGE = "\033[38;5;208m"
COLOR_RED    = "\033[91m"
COLOR_RESET  = "\033[0m"


# ------------------------
# --- Data Classes ---
# ------------------------
@dataclass
class IdentifyReq:
    """Request for module identification."""

    req_id: str


@dataclass
class IdentifyRep:
    """Response with module identification."""

    req_id: str
    module_id: str
    module_type: str
    fw: str


@dataclass
class VerifyReq:
    """Request for module verification."""

    module_id: str


@dataclass
class VerifyRep:
    """Response with module verification result."""

    module_id: str
    ok: bool
    reason: str


@dataclass
class Heartbeat:
    """Heartbeat message from module to dock."""

    module_id: str
    seq: int


@dataclass
class MissionRequest:
    """Top-level mission request from user to dock."""

    task: str


@dataclass
class TaskReq:
    """Request from dock to module to perform a task."""

    module_id: str
    task: str


@dataclass
class TaskAck:
    """Module acknowledgment: accepted/rejected task assignment."""

    module_id: str
    accepted: bool
    reason: str = ""


@dataclass
class TaskStart:
    """Module notification to dock that task has begun."""

    module_id: str
    task: str


@dataclass
class TaskComplete:
    """Module notification to dock that task is complete."""

    module_id: str
    task: str
    success: bool = True
    note: str = ""
    map_data: str = ""  # Base64-encoded gzipped map file (PGM)
    map_yaml: str = ""  # Base64-encoded map metadata (YAML)
    exploration_metrics: dict = None  # Area, duration, distance, etc.

    def __post_init__(self):
        if self.exploration_metrics is None:
            self.exploration_metrics = {}


# ------------------------
# --- Bidding System ---
# ------------------------
@dataclass
class BidNotice:
    """Dock broadcasts this to solicit bids from available modules."""

    task_id: str       # Unique ID for this task auction
    task: str          # Task description (e.g., "explore_zone_a")
    deadline: float    # Seconds to respond with bid


@dataclass
class BidProposal:
    """Module response to a BidNotice with cost estimate."""

    task_id: str       # Must match BidNotice.task_id
    module_id: str     # Bidding module
    cost: float        # Lower = better (battery, idle time, distance, etc.)
    can_execute: bool  # False if module cannot execute this task
    reason: str = ""   # Explanation if can_execute is False


# ------------------------
# --- Encode / Decode ---
# ------------------------

# IDENTIFY
def ident_req_encode(req: IdentifyReq) -> str:
    return json.dumps({"req_id": req.req_id})

def ident_req_decode(msg: String):
    try:
        d = json.loads(msg.data)
        return IdentifyReq(req_id=d.get("req_id", ""))
    except json.JSONDecodeError as e:
        logger.error(f"Failed to decode IdentifyReq: {e}")
        logger.debug(f"Malformed data: {msg.data[:100]}...")
        return None
    except Exception as e:
        logger.error(f"Unexpected error decoding IdentifyReq: {e}")
        return None

def ident_rep_encode(rep: IdentifyRep) -> str:
    return json.dumps({
        "req_id": rep.req_id,
        "module_id": rep.module_id,
        "module_type": rep.module_type,
        "fw": rep.fw,
    })

def ident_rep_decode(msg: String):
    try:
        d = json.loads(msg.data)
        return IdentifyRep(
            req_id=d.get("req_id", ""),
            module_id=d.get("module_id", ""),
            module_type=d.get("module_type", ""),
            fw=d.get("fw", ""),
        )
    except json.JSONDecodeError as e:
        logger.error(f"Failed to decode IdentifyRep: {e}")
        logger.debug(f"Malformed data: {msg.data[:100]}...")
        return None
    except Exception as e:
        logger.error(f"Unexpected error decoding IdentifyRep: {e}")
        return None

# VERIFY
def verify_req_encode(req: VerifyReq) -> str:
    return json.dumps({"module_id": req.module_id})

def verify_req_decode(msg: String):
    try:
        d = json.loads(msg.data)
        return VerifyReq(module_id=d.get("module_id", ""))
    except json.JSONDecodeError as e:
        logger.error(f"Failed to decode VerifyReq: {e}")
        logger.debug(f"Malformed data: {msg.data[:100]}...")
        return None
    except Exception as e:
        logger.error(f"Unexpected error decoding VerifyReq: {e}")
        return None

def verify_rep_encode(rep: VerifyRep) -> str:
    return json.dumps({
        "module_id": rep.module_id,
        "ok": rep.ok,
        "reason": rep.reason
    })

def verify_rep_decode(msg: String):
    try:
        d = json.loads(msg.data)
        return VerifyRep(
            module_id=d.get("module_id", ""),
            ok=bool(d.get("ok", False)),
            reason=d.get("reason", ""),
        )
    except json.JSONDecodeError as e:
        logger.error(f"Failed to decode VerifyRep: {e}")
        logger.debug(f"Malformed data: {msg.data[:100]}...")
        return None
    except Exception as e:
        logger.error(f"Unexpected error decoding VerifyRep: {e}")
        return None

# HEARTBEAT
def hb_encode(hb: Heartbeat) -> str:
    return json.dumps({"module_id": hb.module_id, "seq": hb.seq})

def hb_decode(msg: String):
    try:
        d = json.loads(msg.data)
        return Heartbeat(module_id=d.get("module_id", ""), seq=int(d.get("seq", 0)))
    except json.JSONDecodeError as e:
        logger.error(f"Failed to decode Heartbeat: {e}")
        logger.debug(f"Malformed data: {msg.data[:100]}...")
        return None
    except (ValueError, TypeError) as e:
        logger.error(f"Invalid seq value in Heartbeat: {e}")
        return None
    except Exception as e:
        logger.error(f"Unexpected error decoding Heartbeat: {e}")
        return None

# MISSION_REQ
def mission_req_encode(req: MissionRequest) -> str:
    return json.dumps({"task": req.task})

def mission_req_decode(msg: String):
    try:
        d = json.loads(msg.data)
        return MissionRequest(task=d.get("task", ""))
    except json.JSONDecodeError as e:
        logger.error(f"Failed to decode MissionRequest: {e}")
        logger.debug(f"Malformed data: {msg.data[:100]}...")
        return None
    except Exception as e:
        logger.error(f"Unexpected error decoding MissionRequest: {e}")
        return None

# TASK_REQ
def task_req_encode(req: TaskReq) -> str:
    return json.dumps({"module_id": req.module_id, "task": req.task})

def task_req_decode(msg: String):
    try:
        d = json.loads(msg.data)
        return TaskReq(module_id=d.get("module_id", ""), task=d.get("task", ""))
    except json.JSONDecodeError as e:
        logger.error(f"Failed to decode TaskReq: {e}")
        logger.debug(f"Malformed data: {msg.data[:100]}...")
        return None
    except Exception as e:
        logger.error(f"Unexpected error decoding TaskReq: {e}")
        return None

# TASK_ACK
def task_ack_encode(ack: TaskAck) -> str:
    return json.dumps({
        "module_id": ack.module_id,
        "accepted": ack.accepted,
        "reason": ack.reason
    })

def task_ack_decode(msg: String):
    try:
        d = json.loads(msg.data)
        return TaskAck(
            module_id=d.get("module_id", ""),
            accepted=bool(d.get("accepted", False)),
            reason=d.get("reason", "")
        )
    except json.JSONDecodeError as e:
        logger.error(f"Failed to decode TaskAck: {e}")
        logger.debug(f"Malformed data: {msg.data[:100]}...")
        return None
    except Exception as e:
        logger.error(f"Unexpected error decoding TaskAck: {e}")
        return None

# TASK_START
def task_start_encode(ts: TaskStart) -> str:
    return json.dumps({
        "module_id": ts.module_id,
        "task": ts.task
    })

def task_start_decode(msg: String):
    try:
        d = json.loads(msg.data)
        return TaskStart(
            module_id=d.get("module_id", ""),
            task=d.get("task", "")
        )
    except json.JSONDecodeError as e:
        logger.error(f"Failed to decode TaskStart: {e}")
        logger.debug(f"Malformed data: {msg.data[:100]}...")
        return None
    except Exception as e:
        logger.error(f"Unexpected error decoding TaskStart: {e}")
        return None

# TASK_COMPLETE
def task_complete_encode(tc: TaskComplete) -> str:
    return json.dumps({
        "module_id": tc.module_id,
        "task": tc.task,
        "success": tc.success,
        "note": tc.note,
        "map_data": tc.map_data,
        "map_yaml": tc.map_yaml,
        "exploration_metrics": tc.exploration_metrics
    })

def task_complete_decode(msg: String):
    try:
        d = json.loads(msg.data)
        return TaskComplete(
            module_id=d.get("module_id", ""),
            task=d.get("task", ""),
            success=bool(d.get("success", True)),
            note=d.get("note", ""),
            map_data=d.get("map_data", ""),
            map_yaml=d.get("map_yaml", ""),
            exploration_metrics=d.get("exploration_metrics", {})
        )
    except json.JSONDecodeError as e:
        logger.error(f"Failed to decode TaskComplete: {e}")
        logger.debug(f"Malformed data: {msg.data[:100]}...")
        return None
    except Exception as e:
        logger.error(f"Unexpected error decoding TaskComplete: {e}")
        return None


# BID_NOTICE
def bid_notice_encode(bn: BidNotice) -> str:
    return json.dumps({
        "task_id": bn.task_id,
        "task": bn.task,
        "deadline": bn.deadline
    })

def bid_notice_decode(msg: String):
    try:
        d = json.loads(msg.data)
        return BidNotice(
            task_id=d.get("task_id", ""),
            task=d.get("task", ""),
            deadline=float(d.get("deadline", 2.0))
        )
    except json.JSONDecodeError as e:
        logger.error(f"Failed to decode BidNotice: {e}")
        logger.debug(f"Malformed data: {msg.data[:100]}...")
        return None
    except Exception as e:
        logger.error(f"Unexpected error decoding BidNotice: {e}")
        return None


# BID_PROPOSAL
def bid_proposal_encode(bp: BidProposal) -> str:
    return json.dumps({
        "task_id": bp.task_id,
        "module_id": bp.module_id,
        "cost": bp.cost,
        "can_execute": bp.can_execute,
        "reason": bp.reason
    })

def bid_proposal_decode(msg: String):
    try:
        d = json.loads(msg.data)
        return BidProposal(
            task_id=d.get("task_id", ""),
            module_id=d.get("module_id", ""),
            cost=float(d.get("cost", 999.0)),
            can_execute=bool(d.get("can_execute", False)),
            reason=d.get("reason", "")
        )
    except json.JSONDecodeError as e:
        logger.error(f"Failed to decode BidProposal: {e}")
        logger.debug(f"Malformed data: {msg.data[:100]}...")
        return None
    except Exception as e:
        logger.error(f"Unexpected error decoding BidProposal: {e}")
        return None