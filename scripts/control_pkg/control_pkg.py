#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        
        # Create QoS profile with best effort reliability
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create publisher with best effort QoS
        self.publisher = self.create_publisher(
            JointState, 
            'joint_states', 
            qos_profile
        )
        
        # Timer to publish at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_joint_states)
        
        self.get_logger().info('Joint State Publisher started with BEST_EFFORT QoS')

    def publish_joint_states(self):
        msg = JointState()
        
        # Set timestamp
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        
        # Define joint names
        msg.name = ['joint_0', 'joint_1', 'joint_2', 'joint_3', 'joint_4']
        
        # Set joint positions in DEGREES (0-180)
        msg.position = [90.0, 45.0, 135.0, 60.0, 120.0]
        
        # Optional: Set velocities and efforts
        msg.velocity = []
        msg.effort = []
        
        # Publish the message
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.position}')

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
