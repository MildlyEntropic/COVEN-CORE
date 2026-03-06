# SPDX-License-Identifier: MIT
"""
codecs.py — COVEN message encode/decode wrappers.

Typed wrappers around the generic serializer for each message type.
Each pair (encode/decode) provides type safety and a consistent API.

Author: Alexander Shultis
Date: September 2025
"""

from typing import Optional

from std_msgs.msg import String

from coven_core.serialization import encode as _generic_encode, decode as _generic_decode
from coven_core.messages import (
    IdentifyReq, IdentifyRep, VerifyReq, VerifyRep, Heartbeat,
    MissionRequest, TaskReq, TaskAck, TaskStart, TaskComplete,
    BidNotice, BidProposal,
    CoverageGoal, CoverageStatus, CoverageMissionComplete,
    RoverRegistration, RoverRegistrationAck,
    SensorData, VelocityCommand, RoverStatusMsgMsg, DockCommand,
)


# IDENTIFY
def ident_req_encode(req: IdentifyReq) -> str:
    """Encode IdentifyReq to JSON string."""
    return _generic_encode(req)


def ident_req_decode(msg: String) -> Optional[IdentifyReq]:
    """Decode IdentifyReq from ROS String message."""
    return _generic_decode(msg, IdentifyReq)


def ident_rep_encode(rep: IdentifyRep) -> str:
    """Encode IdentifyRep to JSON string."""
    return _generic_encode(rep)


def ident_rep_decode(msg: String) -> Optional[IdentifyRep]:
    """Decode IdentifyRep from ROS String message."""
    return _generic_decode(msg, IdentifyRep)


# VERIFY
def verify_req_encode(req: VerifyReq) -> str:
    """Encode VerifyReq to JSON string."""
    return _generic_encode(req)


def verify_req_decode(msg: String) -> Optional[VerifyReq]:
    """Decode VerifyReq from ROS String message."""
    return _generic_decode(msg, VerifyReq)


def verify_rep_encode(rep: VerifyRep) -> str:
    """Encode VerifyRep to JSON string."""
    return _generic_encode(rep)


def verify_rep_decode(msg: String) -> Optional[VerifyRep]:
    """Decode VerifyRep from ROS String message."""
    return _generic_decode(msg, VerifyRep)


# HEARTBEAT
def hb_encode(hb: Heartbeat) -> str:
    """Encode Heartbeat to JSON string."""
    return _generic_encode(hb)


def hb_decode(msg: String) -> Optional[Heartbeat]:
    """Decode Heartbeat from ROS String message."""
    return _generic_decode(msg, Heartbeat)


# MISSION_REQ
def mission_req_encode(req: MissionRequest) -> str:
    """Encode MissionRequest to JSON string."""
    return _generic_encode(req)


def mission_req_decode(msg: String) -> Optional[MissionRequest]:
    """Decode MissionRequest from ROS String message."""
    return _generic_decode(msg, MissionRequest)


# TASK_REQ
def task_req_encode(req: TaskReq) -> str:
    """Encode TaskReq to JSON string."""
    return _generic_encode(req)


def task_req_decode(msg: String) -> Optional[TaskReq]:
    """Decode TaskReq from ROS String message."""
    return _generic_decode(msg, TaskReq)


# TASK_ACK
def task_ack_encode(ack: TaskAck) -> str:
    """Encode TaskAck to JSON string."""
    return _generic_encode(ack)


def task_ack_decode(msg: String) -> Optional[TaskAck]:
    """Decode TaskAck from ROS String message."""
    return _generic_decode(msg, TaskAck)


# TASK_START
def task_start_encode(ts: TaskStart) -> str:
    """Encode TaskStart to JSON string."""
    return _generic_encode(ts)


def task_start_decode(msg: String) -> Optional[TaskStart]:
    """Decode TaskStart from ROS String message."""
    return _generic_decode(msg, TaskStart)


# TASK_COMPLETE
def task_complete_encode(tc: TaskComplete) -> str:
    """Encode TaskComplete to JSON string."""
    return _generic_encode(tc)


def task_complete_decode(msg: String) -> Optional[TaskComplete]:
    """Decode TaskComplete from ROS String message."""
    return _generic_decode(msg, TaskComplete)


# BID_NOTICE
def bid_notice_encode(bn: BidNotice) -> str:
    """Encode BidNotice to JSON string."""
    return _generic_encode(bn)


def bid_notice_decode(msg: String) -> Optional[BidNotice]:
    """Decode BidNotice from ROS String message."""
    return _generic_decode(msg, BidNotice)


# BID_PROPOSAL
def bid_proposal_encode(bp: BidProposal) -> str:
    """Encode BidProposal to JSON string."""
    return _generic_encode(bp)


def bid_proposal_decode(msg: String) -> Optional[BidProposal]:
    """Decode BidProposal from ROS String message."""
    return _generic_decode(msg, BidProposal)


# COVERAGE_GOAL
def coverage_goal_encode(cg: CoverageGoal) -> str:
    """Encode CoverageGoal to JSON string."""
    return _generic_encode(cg)


def coverage_goal_decode(msg: String) -> Optional[CoverageGoal]:
    """Decode CoverageGoal from ROS String message."""
    return _generic_decode(msg, CoverageGoal)


# COVERAGE_STATUS
def coverage_status_encode(cs: CoverageStatus) -> str:
    """Encode CoverageStatus to JSON string."""
    return _generic_encode(cs)


def coverage_status_decode(msg: String) -> Optional[CoverageStatus]:
    """Decode CoverageStatus from ROS String message."""
    return _generic_decode(msg, CoverageStatus)


# COVERAGE_MISSION_COMPLETE
def coverage_mission_complete_encode(cmc: CoverageMissionComplete) -> str:
    """Encode CoverageMissionComplete to JSON string."""
    return _generic_encode(cmc)


def coverage_mission_complete_decode(msg: String) -> Optional[CoverageMissionComplete]:
    """Decode CoverageMissionComplete from ROS String message."""
    return _generic_decode(msg, CoverageMissionComplete)


# ROVER_REGISTRATION
def rover_registration_encode(reg: RoverRegistration) -> str:
    """Encode RoverRegistration to JSON string."""
    return _generic_encode(reg)


def rover_registration_decode(msg: String) -> Optional[RoverRegistration]:
    """Decode RoverRegistration from ROS String message."""
    return _generic_decode(msg, RoverRegistration)


# ROVER_REGISTRATION_ACK
def rover_registration_ack_encode(ack: RoverRegistrationAck) -> str:
    """Encode RoverRegistrationAck to JSON string."""
    return _generic_encode(ack)


def rover_registration_ack_decode(msg: String) -> Optional[RoverRegistrationAck]:
    """Decode RoverRegistrationAck from ROS String message."""
    return _generic_decode(msg, RoverRegistrationAck)


# SENSOR_DATA
def sensor_data_encode(sd: SensorData) -> str:
    """Encode SensorData to JSON string."""
    return _generic_encode(sd)


def sensor_data_decode(msg: String) -> Optional[SensorData]:
    """Decode SensorData from ROS String message."""
    return _generic_decode(msg, SensorData)


# VELOCITY_COMMAND
def velocity_command_encode(vc: VelocityCommand) -> str:
    """Encode VelocityCommand to JSON string."""
    return _generic_encode(vc)


def velocity_command_decode(msg: String) -> Optional[VelocityCommand]:
    """Decode VelocityCommand from ROS String message."""
    return _generic_decode(msg, VelocityCommand)


# ROVER_STATUS
def rover_status_encode(rs: RoverStatusMsg) -> str:
    """Encode RoverStatusMsg to JSON string."""
    return _generic_encode(rs)


def rover_status_decode(msg: String) -> Optional[RoverStatusMsg]:
    """Decode RoverStatusMsg from ROS String message."""
    return _generic_decode(msg, RoverStatusMsg)


# DOCK_COMMAND
def dock_command_encode(dc: DockCommand) -> str:
    """Encode DockCommand to JSON string."""
    return _generic_encode(dc)


def dock_command_decode(msg: String) -> Optional[DockCommand]:
    """Decode DockCommand from ROS String message."""
    return _generic_decode(msg, DockCommand)
