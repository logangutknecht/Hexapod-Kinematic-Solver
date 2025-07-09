"""
Hexapod Kinematics Engine

This module provides forward and inverse kinematics calculations for a 6-legged hexapod robot.
The hexapod has 3 degrees of freedom per leg (hip, thigh, shin) and 6 legs arranged in a 3+3 configuration.
"""

import numpy as np
import math
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass
from enum import Enum

class GaitPattern(Enum):
    """Available gait patterns for the hexapod"""
    TRIPOD = "tripod"      # 3 legs up, 3 legs down
    WAVE = "wave"          # Sequential leg lifting
    RIPPLE = "ripple"      # Alternating leg groups

@dataclass
class JointAngles:
    """Joint angles for a single leg (hip, thigh, shin)"""
    hip: float    # Rotation around Z-axis (yaw)
    thigh: float  # Rotation around Y-axis (pitch)
    shin: float   # Rotation around Y-axis (pitch)
    
    def to_array(self) -> np.ndarray:
        return np.array([self.hip, self.thigh, self.shin])

@dataclass
class LegPosition:
    """3D position of a leg's end-effector"""
    x: float
    y: float
    z: float
    
    def to_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])

@dataclass
class HexapodConfig:
    """Physical configuration of the hexapod"""
    body_radius: float = 0.15      # Distance from center to leg attachment
    hip_length: float = 0.05       # Length of hip segment
    thigh_length: float = 0.12     # Length of thigh segment  
    shin_length: float = 0.12      # Length of shin segment
    leg_spacing: float = 60.0      # Degrees between legs
    min_z: float = -0.05           # Minimum Z position (ground clearance)
    max_z: float = 0.10            # Maximum Z position

class HexapodKinematics:
    """
    Main kinematics engine for the hexapod robot.
    
    The hexapod has 6 legs arranged in a 3+3 configuration:
    - Legs 0, 2, 4 on the left side
    - Legs 1, 3, 5 on the right side
    - Each leg has 3 DOF: hip (yaw), thigh (pitch), shin (pitch)
    """
    
    def __init__(self, config: Optional[HexapodConfig] = None):
        self.config = config or HexapodConfig()
        self.leg_positions = self._calculate_leg_attachments()
        
    def _calculate_leg_attachments(self) -> List[np.ndarray]:
        """Calculate the attachment points of legs to the body"""
        positions = []
        for i in range(6):
            angle = math.radians(i * self.config.leg_spacing)
            x = self.config.body_radius * math.cos(angle)
            y = self.config.body_radius * math.sin(angle)
            positions.append(np.array([x, y, 0]))
        return positions
    
    def forward_kinematics(self, leg_id: int, joint_angles: JointAngles) -> LegPosition:
        """
        Calculate end-effector position for given joint angles using forward kinematics.
        
        Args:
            leg_id: Leg index (0-5)
            joint_angles: Joint angles for the leg
            
        Returns:
            End-effector position in 3D space
        """
        # Get leg attachment point
        base = self.leg_positions[leg_id]
        
        # Calculate position step by step
        # Start at leg attachment point
        x, y, z = base[0], base[1], base[2]
        
        # Apply hip rotation (around Z-axis)
        hip_x = x + self.config.hip_length * math.cos(joint_angles.hip)
        hip_y = y + self.config.hip_length * math.sin(joint_angles.hip)
        hip_z = z
        
        # Apply thigh rotation (around Y-axis)
        thigh_x = hip_x + self.config.thigh_length * math.cos(joint_angles.thigh) * math.cos(joint_angles.hip)
        thigh_y = hip_y + self.config.thigh_length * math.cos(joint_angles.thigh) * math.sin(joint_angles.hip)
        thigh_z = hip_z + self.config.thigh_length * math.sin(joint_angles.thigh)
        
        # Apply shin rotation (around Y-axis)
        final_x = thigh_x + self.config.shin_length * math.cos(joint_angles.thigh + joint_angles.shin) * math.cos(joint_angles.hip)
        final_y = thigh_y + self.config.shin_length * math.cos(joint_angles.thigh + joint_angles.shin) * math.sin(joint_angles.hip)
        final_z = thigh_z + self.config.shin_length * math.sin(joint_angles.thigh + joint_angles.shin)
        
        return LegPosition(final_x, final_y, final_z)
    
    def inverse_kinematics(self, leg_id: int, target_position: LegPosition) -> Optional[JointAngles]:
        """
        Calculate joint angles for desired end-effector position using inverse kinematics.
        
        Args:
            leg_id: Leg index (0-5)
            target_position: Desired end-effector position
            
        Returns:
            Joint angles if solution exists, None otherwise
        """
        # Get leg attachment point
        base = self.leg_positions[leg_id]
        
        # Transform target to leg coordinate system
        target_local = target_position.to_array() - base
        
        # Extract components
        x, y, z = target_local
        
        # Calculate hip angle (yaw)
        hip_angle = math.atan2(y, x)
        
        # Project to XZ plane for thigh and shin calculations
        x_proj = math.sqrt(x**2 + y**2) - self.config.hip_length
        z_proj = z
        
        # Use geometric approach for thigh and shin angles
        # Distance from hip to target
        distance = math.sqrt(x_proj**2 + z_proj**2)
        
        # Check if target is reachable
        max_reach = self.config.thigh_length + self.config.shin_length
        min_reach = abs(self.config.thigh_length - self.config.shin_length)
        
        if distance > max_reach or distance < min_reach:
            return None
        
        # Calculate angles using cosine law
        cos_thigh = (distance**2 + self.config.thigh_length**2 - self.config.shin_length**2) / \
                   (2 * distance * self.config.thigh_length)
        cos_shin = (self.config.thigh_length**2 + self.config.shin_length**2 - distance**2) / \
                  (2 * self.config.thigh_length * self.config.shin_length)
        
        # Clamp to valid range
        cos_thigh = max(-1, min(1, cos_thigh))
        cos_shin = max(-1, min(1, cos_shin))
        
        thigh_angle = math.acos(cos_thigh)
        shin_angle = math.acos(cos_shin)
        
        # Adjust angles based on target position
        if z_proj < 0:
            thigh_angle = -thigh_angle
            shin_angle = -shin_angle
        
        # Ensure angles are within limits
        hip_angle = max(-math.pi/2, min(math.pi/2, hip_angle))
        thigh_angle = max(-math.pi/3, min(math.pi/3, thigh_angle))
        shin_angle = max(-math.pi/2, min(math.pi/2, shin_angle))
        
        return JointAngles(hip_angle, thigh_angle, shin_angle)
    
    def _rotation_matrix_z(self, angle: float) -> np.ndarray:
        """Create rotation matrix around Z-axis"""
        c, s = math.cos(angle), math.sin(angle)
        return np.array([
            [c, -s, 0, 0],
            [s, c, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
    
    def _rotation_matrix_y(self, angle: float) -> np.ndarray:
        """Create rotation matrix around Y-axis"""
        c, s = math.cos(angle), math.sin(angle)
        return np.array([
            [c, 0, s, 0],
            [0, 1, 0, 0],
            [-s, 0, c, 0],
            [0, 0, 0, 1]
        ])
    
    def _translation_matrix(self, translation: List[float]) -> np.ndarray:
        """Create translation matrix"""
        return np.array([
            [1, 0, 0, translation[0]],
            [0, 1, 0, translation[1]],
            [0, 0, 1, translation[2]],
            [0, 0, 0, 1]
        ])
    
    def get_all_leg_positions(self, joint_angles: List[JointAngles]) -> List[LegPosition]:
        """Calculate positions for all legs"""
        positions = []
        for i, angles in enumerate(joint_angles):
            pos = self.forward_kinematics(i, angles)
            positions.append(pos)
        return positions
    
    def validate_joint_limits(self, joint_angles: JointAngles) -> bool:
        """Check if joint angles are within physical limits"""
        # Define joint limits (in radians)
        limits = {
            'hip': (-math.pi/2, math.pi/2),    # ±90 degrees
            'thigh': (-math.pi/3, math.pi/3),  # ±60 degrees
            'shin': (-math.pi/2, math.pi/2)    # ±90 degrees
        }
        
        return (limits['hip'][0] <= joint_angles.hip <= limits['hip'][1] and
                limits['thigh'][0] <= joint_angles.thigh <= limits['thigh'][1] and
                limits['shin'][0] <= joint_angles.shin <= limits['shin'][1]) 