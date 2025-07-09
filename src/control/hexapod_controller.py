"""
Hexapod Controller

Main control system that handles keyboard input and coordinates hexapod movement.
Integrates kinematics engine with gait planning for smooth robot operation.
"""

import time
import math
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass
from enum import Enum

from kinematics.hexapod_kinematics import (
    HexapodKinematics, JointAngles, LegPosition, GaitPattern, HexapodConfig
)
from kinematics.gait_planner import GaitPlanner, GaitState

class MovementCommand(Enum):
    """Available movement commands"""
    FORWARD = "forward"
    BACKWARD = "backward"
    LEFT = "left"
    RIGHT = "right"
    ROTATE_LEFT = "rotate_left"
    ROTATE_RIGHT = "rotate_right"
    STOP = "stop"

@dataclass
class HexapodState:
    """Current state of the hexapod robot"""
    position: Tuple[float, float, float]  # (x, y, z) in world coordinates
    rotation: float  # Rotation around Z-axis (yaw)
    joint_angles: List[JointAngles]  # Current joint angles for all legs
    leg_positions: List[LegPosition]  # Current leg end-effector positions
    gait_phase: float  # Current phase in gait cycle (0.0 to 1.0)
    gait_state: GaitState  # Current gait state
    stability: float  # Current stability margin
    is_moving: bool  # Whether the hexapod is currently moving

class HexapodController:
    """
    Main controller for the hexapod robot.
    
    Handles:
    - Keyboard input processing
    - Movement coordination
    - Gait pattern selection
    - Real-time control loop
    """
    
    def __init__(self, config: Optional[HexapodConfig] = None):
        self.kinematics = HexapodKinematics(config)
        self.gait_planner = GaitPlanner(self.kinematics)
        
        # Control parameters
        self.movement_speed = 0.1  # m/s
        self.rotation_speed = 0.5  # rad/s
        self.control_frequency = 60  # Hz
        self.dt = 1.0 / self.control_frequency
        
        # Current state
        self.state = HexapodState(
            position=(0.0, 0.0, 0.0),
            rotation=0.0,
            joint_angles=self._get_initial_joint_angles(),
            leg_positions=[],
            gait_phase=0.0,
            gait_state=GaitState(0.0, [0.0] * 6, [True] * 6),
            stability=0.0,
            is_moving=False
        )
        
        # Movement state
        self.current_command = MovementCommand.STOP
        self.target_direction = (0.0, 0.0)
        self.target_rotation = 0.0
        
        # Update initial leg positions
        self._update_leg_positions()
        
    def _get_initial_joint_angles(self) -> List[JointAngles]:
        """Get initial joint angles for all legs (standing position)"""
        angles = []
        for i in range(6):
            # Calculate initial angles for standing position
            # Each leg should be positioned to support the body
            angles.append(JointAngles(
                hip=0.0,      # No hip rotation
                thigh=0.2,    # Slight forward tilt
                shin=-0.4     # Bend to reach ground
            ))
        return angles
    
    def _update_leg_positions(self):
        """Update current leg positions based on joint angles"""
        self.state.leg_positions = self.kinematics.get_all_leg_positions(self.state.joint_angles)
    
    def _update_stability(self):
        """Calculate and update current stability margin"""
        self.state.stability = self.gait_planner.calculate_stability(
            self.state.leg_positions, 
            self.state.gait_state.support_legs
        )
    
    def process_keyboard_input(self, key: str, pressed: bool):
        """
        Process keyboard input and update movement commands.
        
        Args:
            key: Key that was pressed/released
            pressed: True if key was pressed, False if released
        """
        if not pressed:
            # Key released - stop movement if no other keys are held
            if key in ['w', 's', 'a', 'd', 'arrow_up', 'arrow_down', 'arrow_left', 'arrow_right']:
                self.current_command = MovementCommand.STOP
                self.target_direction = (0.0, 0.0)
                self.target_rotation = 0.0
            return
        
        # Key pressed - update movement command
        if key == 'arrow_up' or key == 'w':
            self.current_command = MovementCommand.FORWARD
            self.target_direction = (1.0, 0.0)
        elif key == 'arrow_down' or key == 's':
            self.current_command = MovementCommand.BACKWARD
            self.target_direction = (-1.0, 0.0)
        elif key == 'arrow_left' or key == 'a':
            self.current_command = MovementCommand.LEFT
            self.target_direction = (0.0, 1.0)
        elif key == 'arrow_right' or key == 'd':
            self.current_command = MovementCommand.RIGHT
            self.target_direction = (0.0, -1.0)
        elif key == 'q':
            self.current_command = MovementCommand.ROTATE_LEFT
            self.target_rotation = self.rotation_speed
        elif key == 'e':
            self.current_command = MovementCommand.ROTATE_RIGHT
            self.target_rotation = -self.rotation_speed
        elif key == 'space':
            # Toggle gait pattern
            current_pattern = self.gait_planner.current_gait
            patterns = list(GaitPattern)
            current_index = patterns.index(current_pattern)
            next_index = (current_index + 1) % len(patterns)
            self.gait_planner.set_gait_pattern(patterns[next_index])
            print(f"Switched to {patterns[next_index].value} gait")
        elif key == 'r':
            # Reset to initial position
            self.reset_position()
    
    def reset_position(self):
        """Reset hexapod to initial position"""
        self.state.position = (0.0, 0.0, 0.0)
        self.state.rotation = 0.0
        self.state.joint_angles = self._get_initial_joint_angles()
        self.state.gait_phase = 0.0
        self.current_command = MovementCommand.STOP
        self.target_direction = (0.0, 0.0)
        self.target_rotation = 0.0
        self._update_leg_positions()
        print("Hexapod reset to initial position")
    
    def update(self, dt: Optional[float] = None):
        """
        Update hexapod state for one control cycle.
        
        Args:
            dt: Time delta (uses self.dt if None)
        """
        if dt is None:
            dt = self.dt
        
        # Update gait phase
        if self.current_command != MovementCommand.STOP:
            self.state.is_moving = True
            # Advance gait phase based on movement speed
            phase_increment = dt / self.gait_planner.cycle_time
            self.state.gait_phase = (self.state.gait_phase + phase_increment) % 1.0
        else:
            self.state.is_moving = False
        
        # Update gait state
        self.state.gait_state = self.gait_planner.get_gait_state(self.state.gait_phase)
        
        # Update body position and rotation
        self._update_body_movement(dt)
        
        # Update leg positions based on gait
        self._update_leg_movement()
        
        # Update stability
        self._update_stability()
    
    def _update_body_movement(self, dt: float):
        """Update body position and rotation based on current commands"""
        # Update rotation
        if self.target_rotation != 0.0:
            self.state.rotation += self.target_rotation * dt
            self.state.rotation = self.state.rotation % (2 * math.pi)
        
        # Update position
        if self.target_direction != (0.0, 0.0):
            # Calculate movement in body frame
            dx = self.target_direction[0] * self.movement_speed * dt
            dy = self.target_direction[1] * self.movement_speed * dt
            
            # Transform to world frame
            cos_rot = math.cos(self.state.rotation)
            sin_rot = math.sin(self.state.rotation)
            world_dx = dx * cos_rot - dy * sin_rot
            world_dy = dx * sin_rot + dy * cos_rot
            
            # Update position
            self.state.position = (
                self.state.position[0] + world_dx,
                self.state.position[1] + world_dy,
                self.state.position[2]
            )
    
    def _update_leg_movement(self):
        """Update leg positions and joint angles based on gait"""
        # Calculate target positions for all legs
        target_positions = []
        for leg_id in range(6):
            target_pos = self.gait_planner.generate_leg_trajectory(
                leg_id, 
                self.state.gait_phase,
                self.target_direction,
                self.state.rotation
            )
            target_positions.append(target_pos)
        
        # Calculate joint angles for each leg
        new_joint_angles = []
        for leg_id in range(6):
            target_pos = target_positions[leg_id]
            current_angles = self.state.joint_angles[leg_id]
            
            # Use inverse kinematics to get target joint angles
            ik_solution = self.kinematics.inverse_kinematics(leg_id, target_pos)
            
            if ik_solution is not None and self.kinematics.validate_joint_limits(ik_solution):
                # Smooth interpolation between current and target angles
                alpha = 0.1  # Interpolation factor
                new_hip = current_angles.hip + alpha * (ik_solution.hip - current_angles.hip)
                new_thigh = current_angles.thigh + alpha * (ik_solution.thigh - current_angles.thigh)
                new_shin = current_angles.shin + alpha * (ik_solution.shin - current_angles.shin)
                
                new_joint_angles.append(JointAngles(new_hip, new_thigh, new_shin))
            else:
                # Keep current angles if IK fails
                new_joint_angles.append(current_angles)
        
        self.state.joint_angles = new_joint_angles
        self._update_leg_positions()
    
    def get_state(self) -> HexapodState:
        """Get current hexapod state"""
        return self.state
    
    def get_debug_info(self) -> Dict:
        """Get debug information for monitoring"""
        return {
            'position': self.state.position,
            'rotation': math.degrees(self.state.rotation),
            'gait_pattern': self.gait_planner.current_gait.value,
            'gait_phase': self.state.gait_phase,
            'stability': self.state.stability,
            'is_moving': self.state.is_moving,
            'command': self.current_command.value,
            'support_legs': self.state.gait_state.support_legs,
            'joint_angles': [
                {
                    'hip': math.degrees(angles.hip),
                    'thigh': math.degrees(angles.thigh),
                    'shin': math.degrees(angles.shin)
                }
                for angles in self.state.joint_angles
            ]
        }
    
    def set_movement_speed(self, speed: float):
        """Set movement speed in m/s"""
        self.movement_speed = max(0.01, min(0.5, speed))
    
    def set_rotation_speed(self, speed: float):
        """Set rotation speed in rad/s"""
        self.rotation_speed = max(0.1, min(2.0, speed))
    
    def set_gait_pattern(self, pattern: GaitPattern):
        """Set gait pattern"""
        self.gait_planner.set_gait_pattern(pattern) 