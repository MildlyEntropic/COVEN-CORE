from enum import Enum, auto
from dataclasses import dataclass, asdict
import json
from std_msgs.msg import String, Bool

START=0x7E; END=0x7F  # symbolic framing from v0.2

class DockState(Enum):
    IDLE=auto()
    DETECTED=auto()
    IDENTIFY=auto()
    VERIFY=auto()
    ENABLED=auto()
    NORMAL=auto()
    REJECTED=auto()

class ModuleState(Enum):
    BOOT=auto()
    IDENTIFY=auto()
    WAIT_VERIFY=auto()
    NORMAL=auto()
    REJECTED=auto()
    DISCONNECTED=auto()

@dataclass
class IdentifyReq:  # [TYPE=0x01]
    req_id: str
def ident_req_encode(x: IdentifyReq)->String:
    return String(data=json.dumps({"t":1,"req_id":x.req_id}))
@dataclass
class IdentifyRep:  # [TYPE=0x02]
    req_id: str
    module_id: str
    module_type: str
    fw: str
def ident_rep_encode(x: IdentifyRep)->String:
    return String(data=json.dumps({"t":2, **asdict(x)}))
def ident_rep_decode(msg:String)->IdentifyRep|None:
    try:
        d=json.loads(msg.data); 
        if d.get("t")!=2: return None
        return IdentifyRep(req_id=d["req_id"], module_id=d["module_id"], module_type=d["module_type"], fw=d["fw"])
    except Exception:
        return None

@dataclass
class VerifyReq:  # [TYPE=0x03]
    module_id: str
def verify_req_encode(x: VerifyReq)->String:
    return String(data=json.dumps({"t":3,"module_id":x.module_id}))
@dataclass
class VerifyRep:  # [TYPE=0x04]
    module_id: str
    ok: bool
    reason: str
def verify_rep_encode(x: VerifyRep)->String:
    return String(data=json.dumps({"t":4, **asdict(x)}))
def verify_rep_decode(msg:String)->VerifyRep|None:
    try:
        d=json.loads(msg.data)
        if d.get("t")!=4: return None
        return VerifyRep(module_id=d["module_id"], ok=bool(d["ok"]), reason=d.get("reason",""))
    except Exception:
        return None

@dataclass
class Heartbeat:  # [TYPE=0x20]
    module_id: str
    seq: int
def hb_encode(x:Heartbeat)->String:
    return String(data=json.dumps({"t":32, **asdict(x)}))