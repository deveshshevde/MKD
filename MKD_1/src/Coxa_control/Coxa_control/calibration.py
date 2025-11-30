import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import sys
import termios
import tty
import select
import time
import math

# --- Configuration ---
SWEEP_RANGE = 3   # Move from -0.5 to 0.5 radians
SWEEP_TIME = 10.0    # Time for one full sweep cycle (seconds)
UPDATE_RATE = 50    # Hz (how often to publish commands)
TIME_STEP = 1.0 / UPDATE_RATE

# --- Joint Definitions ---
# Map joints to their controller topics and the index within that controller's command
CONTROLLER_MAP = {
    'Revolute_Coxa_1':   {'topic': '/leg1_controller/joint_trajectory', 'idx': 0},
    'Revolute_Femur_1':  {'topic': '/leg1_controller/joint_trajectory', 'idx': 1},
    'Revolute_Tibia_1':  {'topic': '/leg1_controller/joint_trajectory', 'idx': 2},
    
    'Revolute_Coxa_2':   {'topic': '/leg2_controller/joint_trajectory', 'idx': 0},
    'Revolute_Femur_2':  {'topic': '/leg2_controller/joint_trajectory', 'idx': 1},
    'Revolute_Tibia_2':  {'topic': '/leg2_controller/joint_trajectory', 'idx': 2},
    
    'Revolute_Coxa_3':   {'topic': '/leg3_controller/joint_trajectory', 'idx': 0},
    'Revolute_Femur_3':  {'topic': '/leg3_controller/joint_trajectory', 'idx': 1},
    'Revolute_Tibia_3':  {'topic': '/leg3_controller/joint_trajectory', 'idx': 2},
    
    'Revolute_Coxa_4':   {'topic': '/leg4_controller/joint_trajectory', 'idx': 0},
    'Revolute_Femur_4':  {'topic': '/leg4_controller/joint_trajectory', 'idx': 1},
    'Revolute_Tibia_4':  {'topic': '/leg4_controller/joint_trajectory', 'idx': 2},
}

# Define the default (non-active) positions for each joint type.
# Set to 0.0 initially, adjust these for stability if the robot collapses!
DEFAULT_POSITIONS = {
    'coxa': 0.0,
    'femur': 0.0, # Suggest: 0.785 (45 degrees) for stable stand
    'tibia': 0.0, # Suggest: -1.57 (-90 degrees) for stable stand
}

class PoseFinderNode(Node):
    def __init__(self):
        super().__init__('pose_finder_node')
        self.get_logger().info('Starting Pose Finder Node...')

        # State Variables
        self.joint_names = list(CONTROLLER_MAP.keys())
        self.current_joint_index = 0
        self.sweep_time_elapsed = 0.0
        self.start_time = self.get_clock().now().nanoseconds / 1e9

        self.final_pose = {}
        for name in self.joint_names:
            self.final_pose[name] = 0.0 # Start with 0.0 for initial command

        # Publisher Setup
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        # RENAMED: self.publishers was renamed to self.trajectory_publishers to avoid conflict
        self.trajectory_publishers = {
            '/leg1_controller/joint_trajectory': self.create_publisher(JointTrajectory, '/leg1_controller/joint_trajectory', qos_profile),
            '/leg2_controller/joint_trajectory': self.create_publisher(JointTrajectory, '/leg2_controller/joint_trajectory', qos_profile),
            '/leg3_controller/joint_trajectory': self.create_publisher(JointTrajectory, '/leg3_controller/joint_trajectory', qos_profile),
            '/leg4_controller/joint_trajectory': self.create_publisher(JointTrajectory, '/leg4_controller/joint_trajectory', qos_profile),
        }

        # Timer for continuous publishing
        self.timer = self.create_timer(TIME_STEP, self.timer_callback)

        # Terminal setup for non-blocking input
        self.settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        self.get_logger().info('--- READY ---')
        self.get_logger().info('Press "f" to select the current position as the default for the active joint.')
        self.get_logger().info(f'Starting with joint: {self.joint_names[self.current_joint_index]}')

    def shutdown(self):
        """Restores terminal settings and prints results."""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        self.get_logger().info('\n--- POSE FINDER RESULTS ---')
        self.get_logger().info('Final calculated stable pose (absolute radians):')
        self.get_logger().info('----------------------------------')
        
        # Log results in a clean format
        for name, pos in self.final_pose.items():
            print(f"  {name}: {pos:.4f}")
        
        self.get_logger().info('----------------------------------')
        self.get_logger().info('Node successfully shut down.')
        
    def get_joint_type(self, joint_name):
        """Helper to determine if joint is coxa, femur, or tibia."""
        if 'Coxa' in joint_name:
            return 'coxa'
        elif 'Femur' in joint_name:
            return 'femur'
        elif 'Tibia' in joint_name:
            return 'tibia'
        return None

    def get_current_sweep_position(self):
        """Calculates the sweeping angle using a sine wave."""
        # Normalize elapsed time to a 0-1 cycle
        normalized_time = (self.sweep_time_elapsed / SWEEP_TIME) % 1.0
        
        # Simple back-and-forth movement:
        if normalized_time <= 0.5:
            # -1 + 2*t*2 = -1 + 4*t (linear sweep up)
            sweep_value = -SWEEP_RANGE + 4 * SWEEP_RANGE * normalized_time
        else:
            # 1 - 2*(t-0.5)*2 = 1 - 4*(t-0.5) (linear sweep down)
            sweep_value = SWEEP_RANGE - 4 * SWEEP_RANGE * (normalized_time - 0.5)
            
        # Ensure it stays within bounds
        return max(-SWEEP_RANGE, min(SWEEP_RANGE, sweep_value))

    def timer_callback(self):
        """Main loop: check input, update joint state, publish commands."""
        
        # 1. Handle keyboard input
        self.check_keyboard_input()
        
        # If we have cycled through all joints, just return
        if self.current_joint_index >= len(self.joint_names):
            return

        # 2. Update sweep position and time
        current_joint_name = self.joint_names[self.current_joint_index]
        self.sweep_time_elapsed += TIME_STEP
        
        # Calculate the sweep angle (this is the value we are searching for)
        sweep_angle = self.get_current_sweep_position()
        
        # Log current position for user reference
        self.get_logger().info(
            f'Joint: {current_joint_name} | Sweeping Pos: {sweep_angle:.4f} rad. | Press "f" to fix.',
            throttle_duration_sec=0.2
        )

        # 3. Assemble and publish commands
        
        # Use the renamed publishers variable
        controller_commands = {topic: JointTrajectory() for topic in self.trajectory_publishers.keys()}

        for i, joint_name in enumerate(self.joint_names):
            info = CONTROLLER_MAP[joint_name]
            controller_topic = info['topic']
            joint_type = self.get_joint_type(joint_name)
            
            # Use a list to store the positions for the 3 joints in this leg
            if not controller_commands[controller_topic].points:
                # Initialize the joint names and the point structure for the first time
                controller_commands[controller_topic].joint_names = [j for j in self.joint_names if CONTROLLER_MAP[j]['topic'] == controller_topic]
                point = JointTrajectoryPoint()
                point.positions = [0.0] * 3 
                point.time_from_start.sec = 0 # Execute immediately
                controller_commands[controller_topic].points.append(point)

            # Determine the angle for this joint
            if joint_name == current_joint_name:
                # This is the active joint, use the sweep angle
                angle = sweep_angle
            elif joint_name in self.final_pose:
                # This joint is already fixed, use the stored final position
                angle = self.final_pose[joint_name]
            else:
                # Use the hardcoded default angle (e.g., 0.0)
                angle = DEFAULT_POSITIONS.get(joint_type, 0.0)
            
            # Place the angle in the correct position within the point list
            controller_commands[controller_topic].points[0].positions[info['idx']] = angle

        # 4. Publish all commands
        # Use the renamed publishers variable
        for topic, msg in controller_commands.items():
            if msg.joint_names:
                self.trajectory_publishers[topic].publish(msg)

    def check_keyboard_input(self):
        """Checks if the 'f' key has been pressed."""
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            if key.lower() == 'f':
                self.fix_joint_position()
                
    def fix_joint_position(self):
        """Stops sweep, records position, and moves to the next joint."""
        if self.current_joint_index >= len(self.joint_names):
            self.get_logger().warn('All joints already set. Press Ctrl+C to exit and see final results.')
            return

        current_joint_name = self.joint_names[self.current_joint_index]
        
        # 1. Calculate the final recorded position
        final_angle = self.get_current_sweep_position()
        self.final_pose[current_joint_name] = final_angle
        
        self.get_logger().info(f'>>> FIXED {current_joint_name}: {final_angle:.4f} radians <<<')
        
        # 2. Advance to the next joint
        self.current_joint_index += 1
        self.sweep_time_elapsed = 0.0 # Reset sweep timer

        if self.current_joint_index < len(self.joint_names):
            next_joint_name = self.joint_names[self.current_joint_index]
            self.get_logger().info(f'Switching to next joint: {next_joint_name}. Adjust your robot and press "f" when satisfied.')
        else:
            self.get_logger().info('!!! All joints have been set! Publishing the final fixed pose.')
            # After all joints are set, all joints in the next timer callback will use final_pose
            self.get_logger().info('Press Ctrl+C to exit and view the summarized YAML configuration.')

def main(args=None):
    rclpy.init(args=args)
    node = PoseFinderNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()