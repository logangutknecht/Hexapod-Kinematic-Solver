#!/usr/bin/env python3
"""
Hexapod Kinematic Solver - Demo Script

This script demonstrates the hexapod kinematics engine with visual output.
It shows different gait patterns and movement capabilities.
"""

import sys
import time
import math
import numpy as np

# Add src to path for imports
sys.path.append('src')

from src.kinematics.hexapod_kinematics import (
    HexapodKinematics, JointAngles, LegPosition, HexapodConfig, GaitPattern
)
from src.kinematics.gait_planner import GaitPlanner
from src.control.hexapod_controller import HexapodController

def print_hexapod_state(controller, step):
    """Print current hexapod state in a formatted way."""
    state = controller.get_state()
    debug_info = controller.get_debug_info()
    
    print(f"\n=== Step {step} ===")
    print(f"Position: ({state.position[0]:.3f}, {state.position[1]:.3f}, {state.position[2]:.3f})")
    print(f"Rotation: {debug_info['rotation']:.1f}°")
    print(f"Gait Pattern: {debug_info['gait_pattern']}")
    print(f"Gait Phase: {state.gait_phase:.2f}")
    print(f"Stability: {state.stability:.3f}")
    print(f"Moving: {state.is_moving}")
    print(f"Support Legs: {sum(debug_info['support_legs'])}/6")
    
    # Print leg states
    print("\nLeg States:")
    for i, (angles, support) in enumerate(zip(debug_info['joint_angles'], debug_info['support_legs'])):
        status = "SUPPORT" if support else "SWING"
        print(f"  Leg {i}: {status} | Hip: {angles['hip']:6.1f}° | Thigh: {angles['thigh']:6.1f}° | Shin: {angles['shin']:6.1f}°")

def demo_forward_movement():
    """Demonstrate forward movement with different gait patterns."""
    print("=== Hexapod Forward Movement Demo ===")
    
    controller = HexapodController()
    
    # Test different gait patterns
    gait_patterns = [GaitPattern.TRIPOD, GaitPattern.WAVE, GaitPattern.RIPPLE]
    
    for pattern in gait_patterns:
        print(f"\n--- Testing {pattern.value.upper()} Gait ---")
        controller.set_gait_pattern(pattern)
        
        # Move forward for 10 steps
        for step in range(10):
            # Simulate forward movement command
            controller.process_keyboard_input('arrow_up', True)
            controller.update(0.1)  # 100ms time step
            
            if step % 3 == 0:  # Print every 3rd step
                print_hexapod_state(controller, step)
            
            time.sleep(0.1)  # Real-time delay
        
        # Stop movement
        controller.process_keyboard_input('arrow_up', False)
        controller.update(0.1)
        
        print(f"Final position: {controller.get_state().position}")
        time.sleep(1.0)  # Pause between gaits

def demo_rotation():
    """Demonstrate rotation movement."""
    print("\n=== Hexapod Rotation Demo ===")
    
    controller = HexapodController()
    controller.set_gait_pattern(GaitPattern.TRIPOD)
    
    # Rotate left
    print("Rotating left...")
    for step in range(10):
        controller.process_keyboard_input('q', True)
        controller.update(0.1)
        
        if step % 3 == 0:
            print_hexapod_state(controller, step)
        
        time.sleep(0.1)
    
    controller.process_keyboard_input('q', False)
    controller.update(0.1)
    
    # Rotate right
    print("Rotating right...")
    for step in range(10):
        controller.process_keyboard_input('e', True)
        controller.update(0.1)
        
        if step % 3 == 0:
            print_hexapod_state(controller, step)
        
        time.sleep(0.1)
    
    controller.process_keyboard_input('e', False)
    controller.update(0.1)

def demo_gait_comparison():
    """Compare different gait patterns side by side."""
    print("\n=== Gait Pattern Comparison ===")
    
    config = HexapodConfig()
    kinematics = HexapodKinematics(config)
    gait_planner = GaitPlanner(kinematics)
    
    gait_patterns = [GaitPattern.TRIPOD, GaitPattern.WAVE, GaitPattern.RIPPLE]
    
    print("Gait Pattern | Support Legs | Swing Legs | Stability")
    print("-" * 50)
    
    for pattern in gait_patterns:
        gait_planner.set_gait_pattern(pattern)
        
        # Test at different phases
        phases = [0.0, 0.25, 0.5, 0.75]
        
        for phase in phases:
            gait_state = gait_planner.get_gait_state(phase)
            support_count = sum(gait_state.support_legs)
            swing_count = 6 - support_count
            
            # Calculate stability
            leg_positions = []
            for i in range(6):
                target = gait_planner.generate_leg_trajectory(i, phase, (1.0, 0.0))
                leg_positions.append(target)
            
            stability = gait_planner.calculate_stability(leg_positions, gait_state.support_legs)
            
            print(f"{pattern.value:12} | {support_count:11} | {swing_count:9} | {stability:.3f}")

def demo_kinematics_validation():
    """Demonstrate kinematics calculations."""
    print("\n=== Kinematics Validation Demo ===")
    
    config = HexapodConfig()
    kinematics = HexapodKinematics(config)
    
    # Test forward kinematics
    print("Testing Forward Kinematics:")
    test_angles = JointAngles(0.2, 0.3, -0.4)
    position = kinematics.forward_kinematics(0, test_angles)
    print(f"  Joint angles: Hip={math.degrees(test_angles.hip):.1f}°, "
          f"Thigh={math.degrees(test_angles.thigh):.1f}°, "
          f"Shin={math.degrees(test_angles.shin):.1f}°")
    print(f"  End effector position: ({position.x:.3f}, {position.y:.3f}, {position.z:.3f})")
    
    # Test inverse kinematics
    print("\nTesting Inverse Kinematics:")
    ik_solution = kinematics.inverse_kinematics(0, position)
    if ik_solution:
        print(f"  IK solution: Hip={math.degrees(ik_solution.hip):.1f}°, "
              f"Thigh={math.degrees(ik_solution.thigh):.1f}°, "
              f"Shin={math.degrees(ik_solution.shin):.1f}°")
        
        # Verify solution
        verification = kinematics.forward_kinematics(0, ik_solution)
        error = math.sqrt((position.x - verification.x)**2 + 
                         (position.y - verification.y)**2 + 
                         (position.z - verification.z)**2)
        print(f"  Verification error: {error:.6f}")
    else:
        print("  No IK solution found")

def main():
    """Main demo function."""
    print("Hexapod Kinematic Solver - Demo")
    print("=" * 40)
    
    try:
        # Run demos
        demo_kinematics_validation()
        demo_gait_comparison()
        demo_forward_movement()
        demo_rotation()
        
        print("\n=== Demo Complete ===")
        print("The hexapod kinematics engine is working correctly!")
        
    except KeyboardInterrupt:
        print("\nDemo interrupted by user")
    except Exception as e:
        print(f"Error during demo: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main() 