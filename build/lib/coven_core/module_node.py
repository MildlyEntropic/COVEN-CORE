import rclpy, random
from rclpy.node import Node
from std_msgs.msg import String, Bool
import coven_core.common as common  # <--- This is the fix!

HB_PERIOD = 0.8

class Module(Node):
    def __init__(self, module_id='RR-01', module_type='ReconRover', fw='0.0.1'):
        super().__init__('coven_module')
        self.module_id = module_id
        self.module_type = module_type
        self.fw = fw
        self.state = common.ModuleState.BOOT
        # “5V always on” → boot, then raise ID1 (simulated by listening/responding)
        self.sub_ident_req = self.create_subscription(String, 'coven/identify_req', self.on_ident_req, 10)
        self.pub_ident_rep = self.create_publisher(String, 'coven/identify_rep', 10)
        self.sub_verify_req = self.create_subscription(String, 'coven/verify_req', self.on_verify_req, 10)
        self.pub_verify_rep = self.create_publisher(String, 'coven/verify_rep', 10)
        self.sub_enable_12v = self.create_subscription(Bool, 'coven/enable_12v', self.on_power, 10)
        self.pub_hb = self.create_publisher(String, 'coven/heartbeat', 10)

        self.hb_timer = None
        self.seq = 0
        self.get_logger().info('Module booted on 5V; waiting for IDENTIFY.')

    def on_ident_req(self, msg: String):
        if self.state in (common.ModuleState.BOOT, common.ModuleState.IDENTIFY):
            self.state = common.ModuleState.IDENTIFY
            from json import loads
            d = loads(msg.data); req_id = d.get("req_id","?")
            self.get_logger().info(f'IDENTIFY_REQ {req_id} → sending reply')
            rep = common.IdentifyRep(req_id=req_id, module_id=self.module_id, module_type=self.module_type, fw=self.fw)
            self.pub_ident_rep.publish(common.ident_rep_encode(rep))
            self.state = common.ModuleState.WAIT_VERIFY

    def on_verify_req(self, msg: String):
        if self.state != common.ModuleState.WAIT_VERIFY: return
        from json import loads
        d = loads(msg.data); mid = d.get("module_id")
        ok = (mid == self.module_id)
        reason = "" if ok else "Module ID mismatch"
        self.get_logger().info(f'VERIFY_REQ for {mid} → {"OK" if ok else "FAIL"}')
        self.pub_verify_rep.publish(common.verify_rep_encode(common.VerifyRep(module_id=self.module_id, ok=ok, reason=reason)))

    def on_power(self, msg: Bool):
        if msg.data:
            if self.state in (common.ModuleState.WAIT_VERIFY, common.ModuleState.REJECTED):
                self.get_logger().info('+12V enabled → entering NORMAL')
                self.state = common.ModuleState.NORMAL
                self.start_heartbeat()
        else:
            if self.state != common.ModuleState.DISCONNECTED:
                self.get_logger().warn('+12V disabled → DISCONNECTED')
                self.state = common.ModuleState.DISCONNECTED
                self.stop_heartbeat()

    def start_heartbeat(self):
        if self.hb_timer: return
        self.hb_timer = self.create_timer(HB_PERIOD, self.send_hb)

    def stop_heartbeat(self):
        if self.hb_timer:
            self.hb_timer.cancel()
            self.hb_timer = None

    def send_hb(self):
        self.seq += 1
        self.pub_hb.publish(common.hb_encode(common.Heartbeat(module_id=self.module_id, seq=self.seq)))
        # optional: simulate occasional fault drop
        if random.random() < 0.02:
            self.get_logger().warn('Simulated transient fault (no-op for stubs)')

def main():
    rclpy.init()
    node = Module()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()