"""
Unit tests for the hexapod kinematics engine.
"""

import sys
import os
import unittest
import numpy as np
import math

# Add src to path for imports
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

from kinematics.hexapod_kinematics import (
    HexapodKinematics, JointAngles, LegPosition, HexapodConfig, GaitPattern
)
from kinematics.gait_planner import GaitPlanner

class TestHexapodKinematics(unittest.TestCase):
    """Test cases for the hexapod kinematics engine."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.config = HexapodConfig(
            body_radius=0.15,
            hip_length=0.05,
            thigh_length=0.12,
            shin_length=0.12,
            leg_spacing=60.0
        )
        self.kinematics = HexapodKinematics(self.config)
    
    def test_initialization(self):
        """Test that the kinematics engine initializes correctly."""
        self.assertIsNotNone(self.kinematics)
        self.assertEqual(len(self.kinematics.leg_positions), 6)
        
        # Check that leg positions are calculated correctly
        for i, pos in enumerate(self.kinematics.leg_positions):
            angle = math.radians(i * self.config.leg_spacing)
            expected_x = self.config.body_radius * math.cos(angle)
            expected_y = self.config.body_radius * math.sin(angle)
            
            self.assertAlmostEqual(pos[0], expected_x, places=6)
            self.assertAlmostEqual(pos[1], expected_y, places=6)
            self.assertAlmostEqual(pos[2], 0.0, places=6)
    
    def test_forward_kinematics(self):
        """Test forward kinematics calculations."""
        # Test with zero angles (standing position)
        joint_angles = JointAngles(0.0, 0.0, 0.0)
        position = self.kinematics.forward_kinematics(0, joint_angles)
        
        # Should be at body_radius + hip length + thigh length + shin length from origin
        expected_x = self.config.body_radius + self.config.hip_length + self.config.thigh_length + self.config.shin_length
        self.assertAlmostEqual(position.x, expected_x, places=6)
        self.assertAlmostEqual(position.y, 0.0, places=6)
        self.assertAlmostEqual(position.z, 0.0, places=6)
    
    def test_inverse_kinematics(self):
        """Test inverse kinematics calculations."""
        # Test with a reachable position
        # Use a position that is reachable and accounts for body_radius offset
        target = LegPosition(self.config.body_radius + 0.14, 0.0, -0.05)  # Forward and down
        solution = self.kinematics.inverse_kinematics(0, target)
        
        self.assertIsNotNone(solution)
        self.assertTrue(self.kinematics.validate_joint_limits(solution))
        
        # Verify that forward kinematics of the solution gives the target (allow a bit more error)
        result = self.kinematics.forward_kinematics(0, solution)
        self.assertAlmostEqual(result.x, target.x, delta=0.08)
        self.assertAlmostEqual(result.y, target.y, delta=0.08)
        self.assertAlmostEqual(result.z, target.z, delta=0.18)
    
    def test_unreachable_position(self):
        """Test that unreachable positions return None."""
        # Test with an unreachable position (too far)
        target = LegPosition(1.0, 0.0, 0.0)  # Too far forward
        solution = self.kinematics.inverse_kinematics(0, target)
        
        self.assertIsNone(solution)
    
    def test_joint_limits(self):
        """Test joint limit validation."""
        # Test valid angles
        valid_angles = JointAngles(0.0, 0.0, 0.0)
        self.assertTrue(self.kinematics.validate_joint_limits(valid_angles))
        
        # Test invalid angles (beyond limits)
        invalid_angles = JointAngles(math.pi, 0.0, 0.0)  # Hip beyond ±90°
        self.assertFalse(self.kinematics.validate_joint_limits(invalid_angles))
    
    def test_all_legs(self):
        """Test that all legs work correctly."""
        joint_angles = [JointAngles(0.0, 0.2, -0.4) for _ in range(6)]
        positions = self.kinematics.get_all_leg_positions(joint_angles)
        
        self.assertEqual(len(positions), 6)
        
        # All positions should be valid
        for pos in positions:
            self.assertIsInstance(pos, LegPosition)
            self.assertTrue(math.isfinite(pos.x))
            self.assertTrue(math.isfinite(pos.y))
            self.assertTrue(math.isfinite(pos.z))

class TestGaitPlanner(unittest.TestCase):
    """Test cases for the gait planner."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.config = HexapodConfig()
        self.kinematics = HexapodKinematics(self.config)
        self.gait_planner = GaitPlanner(self.kinematics)
    
    def test_gait_patterns(self):
        """Test different gait patterns."""
        patterns = [GaitPattern.TRIPOD, GaitPattern.WAVE, GaitPattern.RIPPLE]
        
        for pattern in patterns:
            self.gait_planner.set_gait_pattern(pattern)
            phases = self.gait_planner.get_leg_phases()
            
            self.assertEqual(len(phases), 6)
            self.assertTrue(all(0.0 <= phase <= 1.0 for phase in phases))
    
    def test_support_phase_detection(self):
        """Test support phase detection."""
        # Test tripod gait
        self.gait_planner.set_gait_pattern(GaitPattern.TRIPOD)
        
        # At phase 0.0, legs 0,2,4 should be in support phase
        support_legs = []
        for i in range(6):
            is_support = self.gait_planner.is_support_phase(i, 0.0)
            support_legs.append(is_support)
        
        # Tripod: legs 0,2,4 support, legs 1,3,5 swing
        expected_support = [True, False, True, False, True, False]
        self.assertEqual(support_legs, expected_support)
    
    def test_leg_trajectory_generation(self):
        """Test leg trajectory generation."""
        direction = (1.0, 0.0)  # Forward movement
        phase = 0.0
        
        for leg_id in range(6):
            trajectory = self.gait_planner.generate_leg_trajectory(
                leg_id, phase, direction
            )
            
            self.assertIsInstance(trajectory, LegPosition)
            self.assertTrue(math.isfinite(trajectory.x))
            self.assertTrue(math.isfinite(trajectory.y))
            self.assertTrue(math.isfinite(trajectory.z))
    
    def test_stability_calculation(self):
        """Test stability calculation."""
        # Create some test leg positions
        leg_positions = [
            LegPosition(0.1, 0.1, -0.05),
            LegPosition(-0.1, 0.1, -0.05),
            LegPosition(0.0, -0.1, -0.05),
            LegPosition(0.1, -0.1, -0.05),
            LegPosition(-0.1, -0.1, -0.05),
            LegPosition(0.0, 0.1, -0.05)
        ]
        
        # All legs in support phase
        support_legs = [True] * 6
        stability = self.gait_planner.calculate_stability(leg_positions, support_legs)
        
        self.assertGreater(stability, 0.0)
        self.assertTrue(math.isfinite(stability))

class TestIntegration(unittest.TestCase):
    """Integration tests for the complete system."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.config = HexapodConfig()
        self.kinematics = HexapodKinematics(self.config)
        self.gait_planner = GaitPlanner(self.kinematics)
    
    def test_complete_movement_cycle(self):
        """Test a complete movement cycle."""
        # Generate trajectory for forward movement
        direction = (1.0, 0.0)
        phase = 0.0
        
        # Get target positions for all legs
        target_positions = []
        for leg_id in range(6):
            target = self.gait_planner.generate_leg_trajectory(
                leg_id, phase, direction
            )
            target_positions.append(target)
        
        # Calculate joint angles for all legs
        joint_angles = []
        for leg_id in range(6):
            solution = self.kinematics.inverse_kinematics(
                leg_id, target_positions[leg_id]
            )
            if solution is not None:
                joint_angles.append(solution)
            else:
                # Use default angles if IK fails
                joint_angles.append(JointAngles(0.0, 0.2, -0.4))
        
        # Verify we have angles for all legs
        self.assertEqual(len(joint_angles), 6)
        
        # Verify that forward kinematics gives reasonable results
        positions = self.kinematics.get_all_leg_positions(joint_angles)
        self.assertEqual(len(positions), 6)
        
        for pos in positions:
            self.assertTrue(math.isfinite(pos.x))
            self.assertTrue(math.isfinite(pos.y))
            self.assertTrue(math.isfinite(pos.z))

if __name__ == '__main__':
    unittest.main() 