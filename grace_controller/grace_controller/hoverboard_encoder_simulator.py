#!/usr/bin/env python3
"""
Hoverboard Encoder Simulation Node

Simulates realistic encoder behavior for hoverboard motors:
- Hall sensor quantization (90 pulses/rev with FOC interpolation)
- Measurement noise from EMI and halls
- Velocity filtering/delays from firmware
- Communications latency

Subscribes to: /joint_states (perfect simulation)
Publishes to: /joint_states_noisy (realistic encoder data)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np


class HoverboardEncoderSimulator(Node):
    """
    Simulates realistic hoverboard motor encoder characteristics.
    
    Based on hoverboard-firmware-hack-FOC:
    - Hall sensors: 90 electrical cycles/rev (15 pole pairs * 6)
    - FOC provides interpolation: effective 360-1440 counts/rev
    - Velocity estimation with filtering in firmware
    - ~1ms communication delay (serial/CAN)
    """
    
    def __init__(self):
        super().__init__('hoverboard_encoder_simulator')
        
        # Declare parameters
        self.declare_parameter('encoder_resolution', 360)  # counts per revolution
        self.declare_parameter('position_noise_stddev', 0.001)  # radians
        self.declare_parameter('velocity_noise_stddev', 0.05)  # rad/s
        self.declare_parameter('velocity_filter_alpha', 0.7)  # low-pass filter
        self.declare_parameter('communication_delay_ms', 1.0)  # milliseconds
        self.declare_parameter('enable_quantization', True)
        self.declare_parameter('enable_noise', True)
        
        # Get parameters
        self.encoder_resolution = self.get_parameter('encoder_resolution').value
        self.pos_noise_std = self.get_parameter('position_noise_stddev').value
        self.vel_noise_std = self.get_parameter('velocity_noise_stddev').value
        self.vel_filter_alpha = self.get_parameter('velocity_filter_alpha').value
        self.enable_quantization = self.get_parameter('enable_quantization').value
        self.enable_noise = self.get_parameter('enable_noise').value
        
        # Calculated values
        self.quantization_step = (2 * np.pi) / self.encoder_resolution  # radians per count
        
        # State variables for velocity filtering
        self.filtered_velocities = {}
        self.last_positions = {}
        self.last_time = None
        
        # Subscribe to perfect joint states from Gazebo
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        
        # Publish noisy/realistic joint states
        self.joint_pub = self.create_publisher(
            JointState,
            '/joint_states_noisy',
            10
        )
        
        self.get_logger().info('Hoverboard Encoder Simulator started')
        self.get_logger().info(f'  Encoder resolution: {self.encoder_resolution} counts/rev')
        self.get_logger().info(f'  Quantization step: {np.degrees(self.quantization_step):.3f}°')
        self.get_logger().info(f'  Position noise: {self.pos_noise_std:.4f} rad')
        self.get_logger().info(f'  Velocity noise: {self.vel_noise_std:.4f} rad/s')
        self.get_logger().info(f'  Velocity filter alpha: {self.vel_filter_alpha}')
        
    def quantize_position(self, position):
        """
        Simulate encoder quantization based on hall sensor resolution.
        
        Args:
            position: True position in radians
            
        Returns:
            Quantized position in radians
        """
        if not self.enable_quantization:
            return position
            
        # Convert to counts, round, convert back
        counts = position / self.quantization_step
        quantized_counts = np.round(counts)
        quantized_position = quantized_counts * self.quantization_step
        
        return quantized_position
    
    def add_noise(self, value, noise_std):
        """Add Gaussian noise to measurement."""
        if not self.enable_noise:
            return value
        return value + np.random.normal(0, noise_std)
    
    def filter_velocity(self, joint_name, velocity):
        """
        Apply low-pass filter to velocity (simulates firmware filtering).
        
        Args:
            joint_name: Name of the joint
            velocity: Current velocity measurement
            
        Returns:
            Filtered velocity
        """
        if joint_name not in self.filtered_velocities:
            self.filtered_velocities[joint_name] = velocity
            return velocity
        
        # Exponential moving average (low-pass filter)
        filtered = (self.vel_filter_alpha * velocity + 
                   (1 - self.vel_filter_alpha) * self.filtered_velocities[joint_name])
        self.filtered_velocities[joint_name] = filtered
        
        return filtered
    
    def joint_callback(self, msg):
        """Process perfect joint states and add realistic encoder noise."""
        
        noisy_msg = JointState()
        noisy_msg.header = msg.header
        noisy_msg.name = msg.name
        
        noisy_positions = []
        noisy_velocities = []
        
        for i, joint_name in enumerate(msg.name):
            # Process position
            true_position = msg.position[i]
            
            # Apply quantization (hall sensor resolution)
            quantized_position = self.quantize_position(true_position)
            
            # Add measurement noise (EMI, hall sensor noise)
            noisy_position = self.add_noise(quantized_position, self.pos_noise_std)
            noisy_positions.append(noisy_position)
            
            # Process velocity
            true_velocity = msg.velocity[i]
            
            # Add velocity measurement noise
            noisy_velocity = self.add_noise(true_velocity, self.vel_noise_std)
            
            # Apply firmware-like filtering
            filtered_velocity = self.filter_velocity(joint_name, noisy_velocity)
            noisy_velocities.append(filtered_velocity)
        
        noisy_msg.position = noisy_positions
        noisy_msg.velocity = noisy_velocities
        noisy_msg.effort = msg.effort  # Effort not affected by encoder noise
        
        self.joint_pub.publish(noisy_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HoverboardEncoderSimulator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
