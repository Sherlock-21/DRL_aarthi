#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import JointState
from pynput import keyboard
import threading
import sys

class JointTeleop(Node):
    def __init__(self):
        super().__init__('joint_teleop')
        
        # Configure QoS profile with Best Effortqqeeqwwwwddddeee
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
        
        # Initialize joint angles (in degrees)
        self.joint_angles = [90.0, 90.0, 90.0, 90.0, 90.0 ,0.0]
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5','joint6']
        
        # Increment/decrement step size (degrees)
        self.step_size = 5.0
        
        # Publish initial state
        self.publish_joint_states()
        
        self.get_logger().info('Joint Teleop Node Started')
        self.get_logger().info('Controls:')
        self.get_logger().info('  q/a: Increase/Decrease Joint 1')
        self.get_logger().info('  w/s: Increase/Decrease Joint 2')
        self.get_logger().info('  e/d: Increase/Decrease Joint 3')
        self.get_logger().info('  r/f: Increase/Decrease Joint 4')
        self.get_logger().info('  t/g: Increase/Decrease Joint 5')
        self.get_logger().info('  c/v: Open / Close gripper')
        self.get_logger().info('  ESC: Quit')
        self.get_logger().info(f'Current angles: {self.joint_angles}')

    def publish_joint_states(self):
        """Publish current joint states"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.name = self.joint_names
        msg.position = self.joint_angles.copy()
        msg.velocity = []
        msg.effort = []
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {self.joint_angles}')

    def on_press(self, key):
        """Handle key press events"""
        try:
            # Increase joint angles
            if key.char == 'q':
                self.joint_angles[0] += self.step_size
                self.get_logger().info(f'Joint 1: {self.joint_angles[0]:.1f}°')
                self.publish_joint_states()
            elif key.char == 'w':
                self.joint_angles[1] += self.step_size
                self.get_logger().info(f'Joint 2: {self.joint_angles[1]:.1f}°')
                self.publish_joint_states()
            elif key.char == 'e':
                self.joint_angles[2] += self.step_size
                self.get_logger().info(f'Joint 3: {self.joint_angles[2]:.1f}°')
                self.publish_joint_states()
            elif key.char == 'r':
                self.joint_angles[3] += self.step_size
                self.get_logger().info(f'Joint 4: {self.joint_angles[3]:.1f}°')
                self.publish_joint_states()
            elif key.char == 't':
                self.joint_angles[4] += self.step_size
                self.get_logger().info(f'Joint 5: {self.joint_angles[4]:.1f}°')
                self.publish_joint_states()
            elif key.char == 'c':
                self.joint_angles[5] = 90 
                self.get_logger().info(f'Joint 6: {self.joint_angles[5]:.1f}°')
                self.publish_joint_states()
            
            # Decrease joint angles
            elif key.char == 'a':
                self.joint_angles[0] -= self.step_size
                self.get_logger().info(f'Joint 1: {self.joint_angles[0]:.1f}°')
                self.publish_joint_states()
            elif key.char == 's':
                self.joint_angles[1] -= self.step_size
                self.get_logger().info(f'Joint 2: {self.joint_angles[1]:.1f}°')
                self.publish_joint_states()
            elif key.char == 'd':
                self.joint_angles[2] -= self.step_size
                self.get_logger().info(f'Joint 3: {self.joint_angles[2]:.1f}°')
                self.publish_joint_states()
            elif key.char == 'f':
                self.joint_angles[3] -= self.step_size
                self.get_logger().info(f'Joint 4: {self.joint_angles[3]:.1f}°')
                self.publish_joint_states()
            elif key.char == 'g':
                self.joint_angles[4] -= self.step_size
                self.get_logger().info(f'Joint 5: {self.joint_angles[4]:.1f}°')
                self.publish_joint_states()
            elif key.char == 'v':
                self.joint_angles[5] = 0 
                self.get_logger().info(f'Joint 6: {self.joint_angles[5]:.1f}°')
                self.publish_joint_states()
                
        except AttributeError:
            # Handle special keys
            if key == keyboard.Key.esc:
                self.get_logger().info('ESC pressed - shutting down')
                return False  # Stop listener

    def start_keyboard_listener(self):
        """Start the keyboard listener in a separate thread"""
        with keyboard.Listener(on_press=self.on_press) as listener:
            listener.join()


def main(args=None):
    rclpy.init(args=args)
    
    node = JointTeleop()
    
    # Start keyboard listener in a separate thread
    keyboard_thread = threading.Thread(target=node.start_keyboard_listener)
    keyboard_thread.daemon = True
    keyboard_thread.start()
    
    try:
        # Spin the node
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
