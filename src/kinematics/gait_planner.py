"""
Gait Planning System for Hexapod Robot

This module provides different gait patterns and trajectory planning for smooth hexapod movement.
"""

import numpy as np
import math
from typing import List, Tuple, Dict
from dataclasses import dataclass
from enum import Enum
from .hexapod_kinematics import JointAngles, LegPosition, GaitPattern, HexapodKinematics

@dataclass
class GaitState:
    """Current state of the gait cycle"""
    phase: float  # 0.0 to 1.0, represents position in gait cycle
    leg_phases: List[float]  # Individual phase for each leg
    support_legs: List[bool]  # Which legs are in support phase

@dataclass
class TrajectoryPoint:
    """A point in a leg trajectory"""
    position: LegPosition
    velocity: Tuple[float, float, float]
    time: float

class GaitPlanner:
    """
    Generates gait patterns and trajectories for hexapod movement.
    
    Supports multiple gait patterns:
    - Tripod: 3 legs up, 3 legs down (fastest, less stable)
    - Wave: Sequential leg lifting (slowest, most stable)
    - Ripple: Alternating leg groups (balanced)
    """
    
    def __init__(self, kinematics: HexapodKinematics):
        self.kinematics = kinematics
        self.current_gait = GaitPattern.TRIPOD
        self.step_height = 0.03  # Height of leg lift during swing phase
        self.step_length = 0.08  # Length of step forward
        self.cycle_time = 1.0    # Time for complete gait cycle
        
    def set_gait_pattern(self, pattern: GaitPattern):
        """Change the current gait pattern"""
        self.current_gait = pattern
        
    def get_leg_phases(self) -> List[float]:
        """Get the phase offset for each leg based on current gait"""
        if self.current_gait == GaitPattern.TRIPOD:
            # Tripod gait: legs 0,2,4 and 1,3,5 are in opposite phases
            return [0.0, 0.5, 0.0, 0.5, 0.0, 0.5]
        
        elif self.current_gait == GaitPattern.WAVE:
            # Wave gait: sequential leg lifting
            return [i * 1.0/6.0 for i in range(6)]
        
        elif self.current_gait == GaitPattern.RIPPLE:
            # Ripple gait: alternating groups
            return [0.0, 0.33, 0.66, 0.0, 0.33, 0.66]
        
        else:
            return [0.0] * 6
    
    def is_support_phase(self, leg_id: int, phase: float) -> bool:
        """Determine if a leg is in support phase (on ground) or swing phase (lifting)"""
        leg_phases = self.get_leg_phases()
        leg_phase = (phase + leg_phases[leg_id]) % 1.0
        
        if self.current_gait == GaitPattern.TRIPOD:
            # Tripod: support phase is 0.5 of cycle
            return leg_phase < 0.5
        
        elif self.current_gait == GaitPattern.WAVE:
            # Wave: support phase is 0.83 of cycle (5/6)
            return leg_phase < 0.83
        
        elif self.current_gait == GaitPattern.RIPPLE:
            # Ripple: support phase is 0.67 of cycle (2/3)
            return leg_phase < 0.67
        
        return True
    
    def generate_leg_trajectory(self, leg_id: int, phase: float, 
                              direction: Tuple[float, float], 
                              body_rotation: float = 0.0) -> LegPosition:
        """
        Generate target position for a leg based on current phase and movement direction.
        
        Args:
            leg_id: Leg index (0-5)
            phase: Current phase in gait cycle (0.0 to 1.0)
            direction: Movement direction (x, y) as unit vector
            body_rotation: Body rotation around Z-axis
            
        Returns:
            Target leg position
        """
        leg_phases = self.get_leg_phases()
        leg_phase = (phase + leg_phases[leg_id]) % 1.0
        
        # Get leg attachment point
        base = self.kinematics.leg_positions[leg_id]
        
        # Calculate movement offset
        move_x = direction[0] * self.step_length * leg_phase
        move_y = direction[1] * self.step_length * leg_phase
        
        # Apply body rotation to movement
        cos_rot = math.cos(body_rotation)
        sin_rot = math.sin(body_rotation)
        rotated_x = move_x * cos_rot - move_y * sin_rot
        rotated_y = move_x * sin_rot + move_y * cos_rot
        
        # Calculate Z position based on phase
        if self.is_support_phase(leg_id, phase):
            # Support phase: leg on ground
            z = self.kinematics.config.min_z
        else:
            # Swing phase: leg lifted with parabolic trajectory
            swing_phase = leg_phase
            if self.current_gait == GaitPattern.TRIPOD:
                swing_phase = (leg_phase - 0.5) * 2  # Normalize to 0-1
            elif self.current_gait == GaitPattern.WAVE:
                swing_phase = (leg_phase - 0.83) * 6  # Normalize to 0-1
            elif self.current_gait == GaitPattern.RIPPLE:
                swing_phase = (leg_phase - 0.67) * 3  # Normalize to 0-1
            
            # Parabolic lift trajectory
            z = self.kinematics.config.min_z + self.step_height * 4 * swing_phase * (1 - swing_phase)
        
        # Calculate final position
        target_x = base[0] + rotated_x
        target_y = base[1] + rotated_y
        target_z = z
        
        return LegPosition(target_x, target_y, target_z)
    
    def get_gait_state(self, phase: float) -> GaitState:
        """Get current state of all legs in the gait cycle"""
        leg_phases = self.get_leg_phases()
        support_legs = []
        
        for i in range(6):
            leg_phase = (phase + leg_phases[i]) % 1.0
            support_legs.append(self.is_support_phase(i, leg_phase))
        
        return GaitState(phase, leg_phases, support_legs)
    
    def calculate_stability(self, leg_positions: List[LegPosition], 
                          support_legs: List[bool]) -> float:
        """
        Calculate stability margin based on support polygon.
        
        Args:
            leg_positions: Current positions of all legs
            support_legs: Which legs are in support phase
            
        Returns:
            Stability margin (distance from COM to support polygon edge)
        """
        if sum(support_legs) < 3:
            return 0.0  # Not stable with less than 3 support legs
        
        # Get support leg positions
        support_positions = []
        for i, is_support in enumerate(support_legs):
            if is_support:
                support_positions.append(leg_positions[i])
        
        # Calculate center of mass (assumed at body center)
        com = np.array([0.0, 0.0, 0.0])
        
        # Calculate support polygon (convex hull of support points)
        if len(support_positions) >= 3:
            # Simple approach: calculate minimum distance to any support leg
            min_distance = float('inf')
            for pos in support_positions:
                distance = np.linalg.norm(pos.to_array() - com)
                min_distance = min(min_distance, distance)
            return min_distance
        
        return 0.0
    
    def optimize_step_parameters(self, direction: Tuple[float, float], 
                               speed: float) -> Tuple[float, float]:
        """
        Optimize step length and cycle time based on desired speed and direction.
        
        Args:
            direction: Movement direction
            speed: Desired movement speed (m/s)
            
        Returns:
            Optimized (step_length, cycle_time)
        """
        # Calculate effective step length in movement direction
        effective_step_length = abs(direction[0]) + abs(direction[1])
        
        if effective_step_length > 0:
            # Adjust step length based on speed
            target_step_length = speed * self.cycle_time / effective_step_length
            target_step_length = max(0.02, min(0.15, target_step_length))  # Clamp to reasonable range
            
            # Adjust cycle time for stability
            if self.current_gait == GaitPattern.TRIPOD:
                # Tripod is faster but less stable
                cycle_time = max(0.5, min(2.0, self.cycle_time))
            elif self.current_gait == GaitPattern.WAVE:
                # Wave is slower but more stable
                cycle_time = max(1.0, min(3.0, self.cycle_time))
            else:  # Ripple
                cycle_time = max(0.8, min(2.5, self.cycle_time))
            
            return target_step_length, cycle_time
        
        return self.step_length, self.cycle_time
    
    def generate_smooth_trajectory(self, leg_id: int, start_pos: LegPosition, 
                                 end_pos: LegPosition, duration: float, 
                                 num_points: int = 50) -> List[TrajectoryPoint]:
        """
        Generate a smooth trajectory between two leg positions.
        
        Args:
            leg_id: Leg index
            start_pos: Starting position
            end_pos: Ending position
            duration: Time duration
            num_points: Number of trajectory points
            
        Returns:
            List of trajectory points
        """
        trajectory = []
        
        for i in range(num_points):
            t = i / (num_points - 1)
            
            # Smooth interpolation using cubic spline
            # Use smoothstep function for natural acceleration/deceleration
            smooth_t = 3 * t**2 - 2 * t**3
            
            # Interpolate position
            x = start_pos.x + (end_pos.x - start_pos.x) * smooth_t
            y = start_pos.y + (end_pos.y - start_pos.y) * smooth_t
            z = start_pos.z + (end_pos.z - start_pos.z) * smooth_t
            
            # Calculate velocity (derivative of position)
            vel_t = 6 * t * (1 - t)  # Derivative of smoothstep
            vx = (end_pos.x - start_pos.x) * vel_t / duration
            vy = (end_pos.y - start_pos.y) * vel_t / duration
            vz = (end_pos.z - start_pos.z) * vel_t / duration
            
            position = LegPosition(x, y, z)
            velocity = (vx, vy, vz)
            time = t * duration
            
            trajectory.append(TrajectoryPoint(position, velocity, time))
        
        return trajectory 