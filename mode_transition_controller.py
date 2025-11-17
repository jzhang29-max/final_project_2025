import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
import math
from enum import Enum


class RobotMode(Enum):
    WALKING = 1
    DRIVING = 2
    TRANSITIONING = 3


class ModeTransitionController(Node):
    def __init__(self):
        super().__init__('mode_transition_controller')
        
        # Parameters
        self.declare_parameter('transition_duration', 3.0)  # seconds
        self.declare_parameter('transition_button', 4)  # Button index on joystick
        
        self.transition_duration = self.get_parameter('transition_duration').value
        self.transition_button = self.get_parameter('transition_button').value
        
        # Robot state
        self.current_mode = RobotMode.WALKING
        self.previous_button_state = False
        
        # Joint positions for different modes
        # Walking mode: end effectors on ground (default position)
        self.walking_positions = {
            'leg_front_r_1': 0.26,
            'leg_front_r_2': 0.0,
            'leg_front_r_3': -0.52,
            'leg_front_l_1': -0.26,
            'leg_front_l_2': 0.0,
            'leg_front_l_3': 0.52,
            'leg_back_r_1': 0.26,
            'leg_back_r_2': 0.0,
            'leg_back_r_3': -0.52,
            'leg_back_l_1': -0.26,
            'leg_back_l_2': 0.0,
            'leg_back_l_3': 0.52,
        }
        
        # Driving mode: wheels on ground
        self.driving_positions = {
            # Front legs - powered wheels
            'leg_front_r_1': 0.26 + math.pi/2,  # TODO: Fine tuning
            'leg_front_r_2': -math.pi/4,  # Adjust to level the robot
            'leg_front_r_3': -0.52 + math.pi/4,
            'leg_front_l_1': -0.26 - math.pi/2,
            'leg_front_l_2': -math.pi/4,
            'leg_front_l_3': 0.52 - math.pi/4,
            # Back legs - omni wheels (passive, but must touch ground for stability)
            'leg_back_r_1': 0.26 + math.pi/2,  # Same rotation to bring omni wheel down
            'leg_back_r_2': -math.pi/4,  # Level with front
            'leg_back_r_3': -0.52 + math.pi/4,
            'leg_back_l_1': -0.26 - math.pi/2,
            'leg_back_l_2': -math.pi/4,
            'leg_back_l_3': 0.52 - math.pi/4,
        }
        
        # Publishers
        self.joint_trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )
        
        self.mode_pub = self.create_publisher(
            String,
            '/robot_mode',
            10
        )
        
        self.hub_motor_enable_pub = self.create_publisher(
            Bool,
            '/hub_motors/enable',
            10
        )
        
        # Subscribers
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        
        self.get_logger().info('Mode transition controller initialized')
        self.get_logger().info(f'Press button {self.transition_button} to switch modes')
    
    def joy_callback(self, msg):
        """Handle joystick input for mode transitions"""
        if len(msg.buttons) <= self.transition_button:
            return
        
        button_pressed = msg.buttons[self.transition_button] == 1
        
        # Detect button press (rising edge)
        if button_pressed and not self.previous_button_state:
            if self.current_mode == RobotMode.WALKING:
                self.transition_to_driving()
            elif self.current_mode == RobotMode.DRIVING:
                self.transition_to_walking()
        
        self.previous_button_state = button_pressed
    
    def transition_to_driving(self):
        """Transition from walking to driving mode"""
        if self.current_mode == RobotMode.TRANSITIONING:
            self.get_logger().warn('Already transitioning, please wait')
            return
        
        self.get_logger().info('Transitioning to DRIVING mode...')
        self.current_mode = RobotMode.TRANSITIONING
        
        # Disable neural controller
        # Send trajectory to move legs to driving position
        self.send_joint_trajectory(self.driving_positions)
        
        # Enable hub motors after transition
        self.create_timer(
            self.transition_duration + 0.5,
            self.enable_driving_mode,
            runs_once=True
        )
    
    def transition_to_walking(self):
        """Transition from driving to walking mode"""
        if self.current_mode == RobotMode.TRANSITIONING:
            self.get_logger().warn('Already transitioning, please wait')
            return
        
        self.get_logger().info('Transitioning to WALKING mode...')
        self.current_mode = RobotMode.TRANSITIONING
        
        # Disable hub motors
        self.hub_motor_enable_pub.publish(Bool(data=False))
        
        # Send trajectory to move legs to walking position
        self.send_joint_trajectory(self.walking_positions)
        
        # Enable neural controller after transition
        self.create_timer(
            self.transition_duration + 0.5,
            self.enable_walking_mode,
            runs_once=True
        )
    
    def send_joint_trajectory(self, target_positions):
        """Send trajectory command to move joints to target positions"""
        trajectory_msg = JointTrajectory()
        trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Set joint names
        trajectory_msg.joint_names = list(target_positions.keys())
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = list(target_positions.values())
        point.time_from_start.sec = int(self.transition_duration)
        point.time_from_start.nanosec = int((self.transition_duration % 1) * 1e9)
        
        trajectory_msg.points = [point]
        
        # Publish trajectory
        self.joint_trajectory_pub.publish(trajectory_msg)
        self.get_logger().info(f'Sent trajectory command for {len(target_positions)} joints')
    
    def enable_driving_mode(self):
        """Complete transition to driving mode"""
        self.current_mode = RobotMode.DRIVING
        self.mode_pub.publish(String(data='DRIVING'))
        self.hub_motor_enable_pub.publish(Bool(data=True))
        self.get_logger().info('DRIVING mode active - hub motors enabled')
    
    def enable_walking_mode(self):
        """Complete transition to walking mode"""
        self.current_mode = RobotMode.WALKING
        self.mode_pub.publish(String(data='WALKING'))
        self.get_logger().info('WALKING mode active - neural controller ready')


def main(args=None):
    rclpy.init(args=args)
    node = ModeTransitionController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()