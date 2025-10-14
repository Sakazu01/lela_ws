#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandLong
from std_msgs.msg import Bool

MAV_CMD_DO_SET_SERVO = 183  # param1=servo number, param2=pwm (µs)

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')

        # === Parameters (unchanged) ===
        self.declare_parameter('servo_channel', 2)
        self.declare_parameter('pwm_drop', 1900)
        self.declare_parameter('pwm_hold', 1100)
        self.declare_parameter('reset_delay', 2.0)

        self.servo_ch    = int(self.get_parameter('servo_channel').value)
        self.pwm_drop    = int(self.get_parameter('pwm_drop').value)
        self.pwm_hold    = int(self.get_parameter('pwm_hold').value)
        self.reset_delay = float(self.get_parameter('reset_delay').value)

        # === Service client (same name) ===
        self.cmd_client = self.create_client(CommandLong, '/mavros/cmd/command')

        self.get_logger().info('⏳ Waiting for MAVROS...')
        if not self.cmd_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('❌ MAVROS not available!')
        else:
            self.get_logger().info('✅ MAVROS connected')

        # === IO (unchanged topics) ===
        self.drop_sub = self.create_subscription(Bool, '/drop/execute', self.drop_callback, 10)
        self.done_pub = self.create_publisher(Bool, '/drop/completed', 10)

        self.busy = False
        self.reset_timer = None

        self.get_logger().info(f'✅ Servo ready (CH{self.servo_ch})')

    # ---------- Helpers ----------
    def _make_request(self, pwm: int) -> CommandLong.Request:
        req = CommandLong.Request()
        req.command = MAV_CMD_DO_SET_SERVO
        req.confirmation = 0
        req.param1 = float(self.servo_ch)  # servo index (ArduPilot: SERVOx)
        req.param2 = float(pwm)            # microseconds
        req.param3 = 0.0
        req.param4 = 0.0
        req.param5 = 0.0
        req.param6 = 0.0
        req.param7 = 0.0
        req.broadcast = False              # let MAVROS target the connected FCU
        return req

    def _call_command_async(self, pwm: int, on_done):
        # ensure service is ready for each call (fast check)
        if not self.cmd_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('⏳ COMMAND_LONG not ready')
            return None
        future = self.cmd_client.call_async(self._make_request(pwm))
        future.add_done_callback(on_done)
        return future

    # ---------- Drop flow (unchanged semantics) ----------
    def drop_callback(self, msg: Bool):
        if not msg.data or self.busy:
            return
        self.busy = True
        self.get_logger().info('💣 Executing drop...')
        self._call_command_async(self.pwm_drop, lambda f: self.handle_response(f, self.pwm_drop))

    def handle_response(self, future, pwm):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f'❌ COMMAND_LONG error: {e}')
            self.busy = False
            return

        if resp and resp.success:
            self.get_logger().info(f'✅ Servo → {pwm} µs (MAV_RESULT={resp.result})')
            if pwm == self.pwm_drop:
                if self.reset_timer:
                    self.reset_timer.cancel()
                self.reset_timer = self.create_timer(self.reset_delay, self.reset_servo)
        else:
            result = getattr(resp, "result", "n/a")
            self.get_logger().error(f'❌ Servo command failed (result={result})')
            self.busy = False

    def reset_servo(self):
        if self.reset_timer:
            self.reset_timer.cancel()
            self.reset_timer = None

        self.get_logger().info('🔒 Resetting servo...')
        self._call_command_async(self.pwm_hold, self.handle_reset_response)

    def handle_reset_response(self, future):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f'❌ Reset error: {e}')
            self.busy = False
            return

        if resp and resp.success:
            self.get_logger().info(f'✅ Reset to {self.pwm_hold} µs (MAV_RESULT={resp.result})')
            self.done_pub.publish(Bool(data=True))
        else:
            result = getattr(resp, "result", "n/a")
            self.get_logger().error(f'❌ Reset failed (result={result})')
            self.done_pub.publish(Bool(data=False))
        self.busy = False

def main(args=None):
    rclpy.init(args=args)
    node = ServoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
