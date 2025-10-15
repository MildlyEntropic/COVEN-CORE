import rclpy, time, uuid
from rclpy.node import Node
from std_msgs.msg import String, Bool
import coven_core.common as common

HB_TIMEOUT_SEC = 3.0
VERIFY_DELAY_SEC = 0.5

class Dock(Node):
    def __init__(self):
        super().__init__('coven_dock')
        # Topics (stringified frames to avoid custom .msg for stubs)
        self.pub_ident_req = self.create_publisher(String, 'coven/identify_req', 10)
        self.sub_ident_rep = self.create_subscription(String, 'coven/identify_rep', self.on_ident_rep, 10)
        self.pub_verify_req = self.create_publisher(String, 'coven/verify_req', 10)
        self.sub_verify_rep = self.create_subscription(String, 'coven/verify_rep', self.on_verify_rep, 10)
        self.pub_enable_12v = self.create_publisher(Bool, 'coven/enable_12v', 10)
        self.sub_hb = self.create_subscription(String, 'coven/heartbeat', self.on_heartbeat, 10)

        # Simulated “ID1 sensed” detection pulse
        self.timer_detect = self.create_timer(2.0, self.fake_detection)

        self.state = common.DockState.IDLE
        self.last_hb = None
        self.req_id = None
        self.current_module = None
        self.get_logger().info('Dock ready: waiting for module detection.')

        # Liveness monitor
        self.create_timer(0.5, self.watchdog)

    def fake_detection(self):
        if self.state == common.DockState.IDLE:
            self.get_logger().info('ID1 sensed: DETECTED')
            self.state = common.DockState.DETECTED
            self.req_id = str(uuid.uuid4())[:8]
            # Send IDENTIFY after a tick
            self.state = common.DockState.IDENTIFY
            self.get_logger().info(f'Sending IDENTIFY_REQ {self.req_id}')
            self.pub_ident_req.publish(common.ident_req_encode(common.IdentifyReq(req_id=self.req_id)))

    def on_ident_rep(self, msg: String):
        if self.state != common.DockState.IDENTIFY: return
        rep = common.ident_rep_decode(msg)
        if not rep or rep.req_id != self.req_id:
            return
        self.get_logger().info(f'IDENTIFY_REP from {rep.module_id} ({rep.module_type} fw {rep.fw})')
        self.current_module = rep.module_id
        self.state = common.DockState.VERIFY
        # Simulate DB/policy check then verify
        self.create_timer(VERIFY_DELAY_SEC, self.send_verify)

    def send_verify(self):
        if self.state != common.DockState.VERIFY: return
        self.get_logger().info(f'Sending VERIFY_REQ for {self.current_module}')
        self.pub_verify_req.publish(common.verify_req_encode(common.VerifyReq(module_id=self.current_module)))

    def on_verify_rep(self, msg: String):
        if self.state != common.DockState.VERIFY: return
        rep = common.verify_rep_decode(msg)
        if not rep or rep.module_id != self.current_module:
            return
        if rep.ok:
            self.get_logger().info('VERIFY_OK → enabling +12V')
            self.pub_enable_12v.publish(Bool(data=True))
            self.state = common.DockState.ENABLED
            self.last_hb = self.get_clock().now()
        else:
            self.get_logger().warn(f'VERIFY_FAIL: {rep.reason}')
            self.state = common.DockState.REJECTED
            self.pub_enable_12v.publish(Bool(data=False))

    def on_heartbeat(self, msg: String):
        if self.state in (common.DockState.ENABLED, common.DockState.NORMAL):
            self.last_hb = self.get_clock().now()
            if self.state == common.DockState.ENABLED:
                self.get_logger().info('First heartbeat → NORMAL')
                self.state = common.DockState.NORMAL

    def watchdog(self):
        # heartbeat timeout → drop +12V
        if self.state in (common.DockState.ENABLED, common.DockState.NORMAL):
            if self.last_hb is None: return
            dt = (self.get_clock().now() - self.last_hb).nanoseconds * 1e-9
            if dt > HB_TIMEOUT_SEC:
                self.get_logger().error(f'Heartbeat timeout ({dt:.1f}s) → power OFF')
                self.pub_enable_12v.publish(Bool(data=False))
                self.state = common.DockState.IDLE
                self.current_module = None
                self.req_id = None

def main():
    rclpy.init()
    node = Dock()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()