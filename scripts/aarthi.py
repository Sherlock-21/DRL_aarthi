#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import JointState
import time

class JointSequencePublisher(Node):
    def __init__(self):
        super().__init__('joint_sequence_publisher')
        
        # Configure QoS profile with Best Effort
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create publisher with Best Effort QoS
        self.publisher = self.create_publisher(
            JointState,
            'joint_states',
            qos_profile
        )
        
        # Define your joint sequence here
        self.joint_sequence = [
            [95.0, 80.0, 0.0, 90.0, 90.0, 90.0],

            [85.0, 80.0, 0.0, 90.0, 90.0, 90.0],
           # [75.0, 80.0, 0.0, 90.0, 90.0, 90.0],
            [65.0, 80.0, 0.0, 90.0, 90.0, 90.0],
            # [55.0, 80.0, 0.0, 90.0, 90.0, 90.0],
            [45.0, 80.0, 0.0, 90.0, 90.0, 90.0],
            # [35.0, 80.0, 0.0, 90.0, 90.0, 90.0],
            [25.0, 80.0, 0.0, 90.0, 90.0, 90.0],

            [25.0, 80.0, 5.0, 90.0, 90.0, 90.0],
            [25.0, 77.0, 10.0, 90.0, 90.0, 90.0],
            [25.0, 75.0, 15.0, 90.0, 90.0, 90.0],

            [35.0, 75.0, 15.0, 90.0, 90.0, 90.0],
            [55.0, 75.0, 15.0, 90.0, 90.0, 90.0],
            [75.0, 75.0, 15.0, 90.0, 90.0, 90.0],
            [85.0, 75.0, 15.0, 90.0, 90.0, 90.0],


            [95.0, 75.0, 15.0, 90.0, 90.0, 90.0],
            [95.0, 77.0, 7.0, 90.0, 90.0, 90.0],
            [95.0, 79.0, 0.0, 90.0, 90.0, 90.0],


            [95.0, 80.0, 0.0, 90.0, 90.0, 90.0],
        ]
        
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        # Delay between publishing each position (seconds)
        self.publish_delay = 0.5
        
        self.get_logger().info('Joint Sequence Publisher Node Started')
        self.get_logger().info(f'Publishing {len(self.joint_sequence)} waypoints with {self.publish_delay}s delay')
        

    def publish_joint_states(self, joint_angles):
        """Publish joint states"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.name = self.joint_names
        msg.position = joint_angles.copy()
        msg.velocity = []
        msg.effort = []
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {joint_angles}')


    def publish_sequence(self):
        """Publish all joint positions in sequence continuously"""
        while rclpy.ok():
            for idx, joint_angles in enumerate(self.joint_sequence):
                self.get_logger().info(f'Publishing waypoint {idx + 1}/{len(self.joint_sequence)}')
                self.publish_joint_states(joint_angles)
                
                # Wait before publishing next position
                if idx < len(self.joint_sequence) - 1:
                    time.sleep(self.publish_delay)
            
            self.get_logger().info('Sequence complete! Restarting...')
            time.sleep(self.publish_delay)  # Optional delay before restarting loop


def main(args=None):
    rclpy.init(args=args)
    
    node = JointSequencePublisher()
    
    try:
        # Publish the sequence
        node.publish_sequence()
        
        # Keep node alive to maintain ROS2 connection
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
