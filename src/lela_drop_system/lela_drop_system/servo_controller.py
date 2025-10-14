#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandLong
from std_msgs.msg import Bool

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        
        self.declare_parameter('servo_channel', 9)
        self.declare_parameter('pwm_drop', 1900)
        self.declare_parameter('pwm_hold', 1100)
        self.declare_parameter('reset_delay', 2.0)
        
        self.servo_ch = self.get_parameter('servo_channel').value
        self.pwm_drop = self.get_parameter('pwm_drop').value
        self.pwm_hold = self.get_parameter('pwm_hold').value
        self.reset_delay = self.get_parameter('reset_delay').value
        
        self.cmd_client = self.create_client(CommandLong, '/mavros/cmd/command')
        
        self.get_logger().info('⏳ Waiting for MAVROS...')
        if not self.cmd_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('❌ MAVROS not available!')
        else:
            self.get_logger().info('✅ MAVROS connected')
        
        self.drop_sub = self.create_subscription(
            Bool, '/drop/execute', self.drop_callback, 10
        )
        
        self.done_pub = self.create_publisher(Bool, '/drop/completed', 10)
        
        self.busy = False
        self.reset_timer = None
        
        self.get_logger().info(f'✅ Servo ready (CH{self.servo_ch})')
    
    def drop_callback(self, msg):
        if not msg.data or self.busy:
            return
        
        self.busy = True
        self.get_logger().info('💣 Executing drop...')
        self.send_servo(self.pwm_drop)
    
    def send_servo(self, pwm):
        req = CommandLong.Request()
        req.command = 183
        req.param1 = float(self.servo_ch)
        req.param2 = float(pwm)
        
        future = self.cmd_client.call_async(req)
        future.add_done_callback(lambda f: self.handle_response(f, pwm))
    
    def handle_response(self, future, pwm):
        try:
            response = future.result()
            
            if response.success:
                self.get_logger().info(f'✅ Servo → {pwm} PWM')
                
                if pwm == self.pwm_drop:
                    if self.reset_timer:
                        self.reset_timer.cancel()
                    self.reset_timer = self.create_timer(self.reset_delay, self.reset_servo)
            else:
                self.get_logger().error('❌ Servo failed')
                self.busy = False
        
        except Exception as e:
            self.get_logger().error(f'❌ Error: {e}')
            self.busy = False
    
    def reset_servo(self):
        if self.reset_timer:
            self.reset_timer.cancel()
            self.reset_timer = None
        
        self.get_logger().info('🔒 Resetting servo...')
        
        req = CommandLong.Request()
        req.command = 183
        req.param1 = float(self.servo_ch)
        req.param2 = float(self.pwm_hold)
        
        future = self.cmd_client.call_async(req)
        future.add_done_callback(self.handle_reset_response)
    
    def handle_reset_response(self, future):
        try:
            response = future.result()
            
            if response.success:
                self.get_logger().info(f'✅ Reset to {self.pwm_hold} PWM')
                
                done = Bool()
                done.data = True
                self.done_pub.publish(done)
                
                self.busy = False
            else:
                self.get_logger().error('❌ Reset failed')
                self.busy = False
        
        except Exception as e:
            self.get_logger().error(f'❌ Error: {e}')
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
