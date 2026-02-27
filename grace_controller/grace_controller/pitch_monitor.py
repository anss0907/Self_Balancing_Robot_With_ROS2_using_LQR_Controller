#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3Stamped
import math
from tf2_ros import TransformException, Buffer, TransformListener
from tf_transformations import euler_from_quaternion
import numpy as np


class PitchMonitor(Node):
    """
    Monitors IMU data and publishes the robot's pitch angle (tilt from vertical).
    
    For a two-wheeled balancing robot:
    - Pitch = rotation around the wheel axis (y-axis)
    - Positive pitch = leaning forward
    - Negative pitch = leaning backward
    """
    
    def __init__(self):
        super().__init__('pitch_monitor')
        
        # TF2 buffer and listener for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribe to IMU data
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/out',
            self.imu_callback,
            10
        )
        
        # Publish pitch angle in radians
        self.pitch_pub = self.create_publisher(
            Float64,
            '/robot_pitch',
            10
        )
        
        # Publish pitch in degrees (easier to read)
        self.pitch_deg_pub = self.create_publisher(
            Float64,
            '/robot_pitch_degrees',
            10
        )
        
        # Publish full orientation (roll, pitch, yaw) for debugging
        self.orientation_pub = self.create_publisher(
            Vector3Stamped,
            '/robot_orientation',
            10
        )
        
        # Parameters
        self.declare_parameter('use_tf_transform', False)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('imu_frame', 'imu_link')
        
        self.use_tf = self.get_parameter('use_tf_transform').value
        self.base_frame = self.get_parameter('base_frame').value
        self.imu_frame = self.get_parameter('imu_frame').value
        
        self.get_logger().info('Pitch Monitor started')
        self.get_logger().info(f'Publishing pitch to /robot_pitch and /robot_pitch_degrees')
        self.get_logger().info(f'Use TF transform: {self.use_tf}')
        
    def quaternion_to_euler(self, x, y, z, w):
        """
        Convert quaternion to Euler angles (roll, pitch, yaw).
        
        Returns:
            tuple: (roll, pitch, yaw) in radians
        """
        return euler_from_quaternion([x, y, z, w])
    
    def imu_callback(self, msg):
        """Process IMU message and extract pitch angle."""
        
        # Get quaternion from IMU
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w
        
        # Convert quaternion to Euler angles
        # For a standard IMU orientation:
        # - Roll: rotation around x-axis (side-to-side tilt)
        # - Pitch: rotation around y-axis (front-back tilt) - THIS IS WHAT WE WANT
        # - Yaw: rotation around z-axis (heading)
        
        roll, pitch, yaw = self.quaternion_to_euler(qx, qy, qz, qw)
        
        # Publish pitch in radians
        pitch_msg = Float64()
        pitch_msg.data = pitch
        self.pitch_pub.publish(pitch_msg)
        
        # Publish pitch in degrees
        pitch_deg_msg = Float64()
        pitch_deg_msg.data = math.degrees(pitch)
        self.pitch_deg_pub.publish(pitch_deg_msg)
        
        # Publish full orientation for debugging
        orientation_msg = Vector3Stamped()
        orientation_msg.header = msg.header
        orientation_msg.vector.x = roll
        orientation_msg.vector.y = pitch
        orientation_msg.vector.z = yaw
        self.orientation_pub.publish(orientation_msg)
        
        # Log periodically (every 50 messages = ~0.5s at 100Hz IMU rate)
        if not hasattr(self, 'msg_count'):
            self.msg_count = 0
        
        self.msg_count += 1
        if self.msg_count % 50 == 0:
            self.get_logger().info(
                f'Pitch: {math.degrees(pitch):.2f}° | '
                f'Roll: {math.degrees(roll):.2f}° | '
                f'Yaw: {math.degrees(yaw):.2f}°'
            )


def main(args=None):
    rclpy.init(args=args)
    node = PitchMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
