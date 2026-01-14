#!/usr/bin/env python3
"""
LCM to ROS 2 bridge for drake::lcmt_panda_status messages.
Subscribes to LCM PANDA_STATUS and republishes as ROS 2 sensor_msgs/JointState.

No modifications to existing drake-franka-driver code required.
"""

import lcm
import sys
import signal
from threading import Thread

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

# Add drake lcmtypes to path - adjust if drake is installed elsewhere
sys.path.insert(0, '/opt/drake/lib/python3.10/site-packages')

try:
    from drake import lcmt_panda_status
except ImportError:
    print("ERROR: Could not import drake.lcmt_panda_status")
    print("Make sure Drake is installed and PYTHONPATH includes drake lcmtypes")
    print("Typical location: /opt/drake/lib/python3.10/site-packages")
    sys.exit(1)


class LcmPandaBridge(Node):
    """Bridge node that converts LCM panda status to ROS JointState."""
    
    def __init__(self):
        super().__init__('lcm_panda_bridge')
        
        # ROS publisher
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/franka/joint_states',
            10
        )
        
        # LCM subscriber
        self.lcm = lcm.LCM()
        self.subscription = self.lcm.subscribe(
            "PANDA_STATUS",
            self._lcm_callback
        )
        
        # Joint names for Franka Panda
        self.joint_names = [
            'panda_joint1',
            'panda_joint2', 
            'panda_joint3',
            'panda_joint4',
            'panda_joint5',
            'panda_joint6',
            'panda_joint7'
        ]
        
        self.get_logger().info('LCM Panda Bridge started')
        self.get_logger().info('Subscribing to LCM channel: PANDA_STATUS')
        self.get_logger().info('Publishing to ROS topic: /franka/joint_states')
        
        # Start LCM handling thread
        self.lcm_thread = Thread(target=self._lcm_handle_loop, daemon=True)
        self.lcm_thread.start()
    
    def _lcm_callback(self, channel, data):
        """Handle incoming LCM message and republish to ROS."""
        try:
            msg = lcmt_panda_status.decode(data)
            
            # Create ROS JointState message
            joint_state = JointState()
            
            # Header with timestamp from LCM message
            joint_state.header = Header()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.header.frame_id = 'panda_link0'
            
            # Joint names
            joint_state.name = self.joint_names[:msg.num_joints]
            
            # Positions, velocities, efforts (torques)
            joint_state.position = list(msg.joint_position)
            joint_state.velocity = list(msg.joint_velocity)
            joint_state.effort = list(msg.joint_torque)
            
            # Publish to ROS
            self.joint_state_pub.publish(joint_state)
            
        except Exception as e:
            self.get_logger().error(f'Error processing LCM message: {e}')
    
    def _lcm_handle_loop(self):
        """Continuously handle LCM messages in separate thread."""
        self.get_logger().info('LCM handler thread started')
        try:
            while rclpy.ok():
                # Handle with timeout to allow checking rclpy.ok()
                self.lcm.handle_timeout(100)  # 100ms timeout
        except KeyboardInterrupt:
            pass
        except Exception as e:
            self.get_logger().error(f'LCM handler error: {e}')


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    bridge = LcmPandaBridge()
    
    def signal_handler(sig, frame):
        bridge.get_logger().info('Shutting down bridge...')
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

