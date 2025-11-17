import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import struct
import threading
import time


class HubMotorController(Node):
    def __init__(self):
        super().__init__('hub_motor_controller')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_base', 0.2)  #TODO: measurement
        self.declare_parameter('wheel_radius', 0.05) #TODO: Radius measurement
        self.declare_parameter('max_rpm', 100)
        
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_rpm = self.get_parameter('max_rpm').value
        
        # Initialize serial connection
        try:
            self.serial_conn = serial.Serial(serial_port, baud_rate, timeout=0.1)
            self.get_logger().info(f'Connected to motor controller on {serial_port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to motor controller: {e}')
            self.serial_conn = None
        
        # Subscribe to cmd_vel topic
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel_wheels',
            self.cmd_vel_callback,
            10
        )
        
        # Motor states
        self.left_motor_rpm = 0.0
        self.right_motor_rpm = 0.0
        self.enabled = False
        
        # Watchdog timer
        self.last_cmd_time = time.time()
        self.watchdog_timeout = 0.5  # seconds
        self.create_timer(0.1, self.watchdog_callback)
        
        self.get_logger().info('Hub motor controller initialized')
    
    def cmd_vel_callback(self, msg):
        """Convert Twist message to wheel RPMs and send to motors"""
        if not self.enabled:
            return
            
        linear_vel = msg.linear.x  # m/s
        angular_vel = msg.angular.z  # rad/s
        
        # Differential drive kinematics
        left_vel = linear_vel - (angular_vel * self.wheel_base / 2.0)
        right_vel = linear_vel + (angular_vel * self.wheel_base / 2.0)
        
        # Convert to RPM
        self.left_motor_rpm = (left_vel / (2 * 3.14159 * self.wheel_radius)) * 60
        self.right_motor_rpm = (right_vel / (2 * 3.14159 * self.wheel_radius)) * 60
        
        # Clamp to max RPM
        self.left_motor_rpm = max(-self.max_rpm, min(self.max_rpm, self.left_motor_rpm))
        self.right_motor_rpm = max(-self.max_rpm, min(self.max_rpm, self.right_motor_rpm))
        
        # Send commands to motors
        self.send_motor_commands()
        
        # Update watchdog
        self.last_cmd_time = time.time()
    
    def send_motor_commands(self):
        """Send motor RPM commands via serial"""
        if not self.serial_conn:
            return
        
        try:
            # Convert RPM to 16-bit signed integer
            left_rpm_int = int(self.left_motor_rpm * 10)  # Scale for precision
            right_rpm_int = int(self.right_motor_rpm * 10)
            
            commands = []
            motor_ids = [0x01, 0x02]  # Front-Left, Front-Right only
            rpms = [left_rpm_int, right_rpm_int]
            
            for motor_id, rpm in zip(motor_ids, rpms):
                rpm_high = (rpm >> 8) & 0xFF
                rpm_low = rpm & 0xFF
                checksum = (motor_id + rpm_high + rpm_low) & 0xFF
                
                packet = bytes([0xAA, motor_id, rpm_high, rpm_low, checksum, 0x55])
                commands.append(packet)
            
            # Send commands to front motors only
            for cmd in commands:
                self.serial_conn.write(cmd)
            
        except Exception as e:
            self.get_logger().error(f'Error sending motor commands: {e}')
    
    def watchdog_callback(self):
        """Stop motors if no commands received recently"""
        if time.time() - self.last_cmd_time > self.watchdog_timeout and self.enabled:
            self.left_motor_rpm = 0.0
            self.right_motor_rpm = 0.0
            self.send_motor_commands()
    
    def enable_motors(self):
        """Enable motor control"""
        self.enabled = True
        self.get_logger().info('Hub motors enabled')
    
    def disable_motors(self):
        """Disable motor control and stop motors"""
        self.enabled = False
        self.left_motor_rpm = 0.0
        self.right_motor_rpm = 0.0
        self.send_motor_commands()
        self.get_logger().info('Hub motors disabled')
    
    def destroy_node(self):
        """Clean up on shutdown"""
        self.disable_motors()
        if self.serial_conn:
            self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HubMotorController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()