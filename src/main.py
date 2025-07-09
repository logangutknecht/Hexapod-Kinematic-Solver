#!/usr/bin/env python3
"""
Hexapod Kinematic Solver - Main Application

This is the main entry point for the hexapod simulation.
It provides a real-time control interface with keyboard input and visualization.
"""

import sys
import time
import threading
import json
from typing import Dict, Any

# Add src to path for imports
sys.path.append('src')

from src.control.hexapod_controller import HexapodController
from src.kinematics.hexapod_kinematics import GaitPattern

class HexapodSimulation:
    """
    Main simulation class that runs the hexapod control loop.
    
    Features:
    - Real-time control loop
    - Keyboard input handling
    - State monitoring and logging
    - Unreal Engine communication (future)
    """
    
    def __init__(self):
        self.controller = HexapodController()
        self.running = False
        self.control_thread = None
        self.input_thread = None
        
        # Communication with Unreal Engine
        self.unreal_interface = None
        
        # Debug and monitoring
        self.debug_mode = True
        self.log_interval = 1.0  # seconds
        self.last_log_time = 0.0
        
    def start(self):
        """Start the hexapod simulation"""
        print("Starting Hexapod Kinematic Solver...")
        print("Controls:")
        print("  Arrow Keys / WASD: Move hexapod")
        print("  Q/E: Rotate left/right")
        print("  Space: Toggle gait pattern")
        print("  R: Reset position")
        print("  Ctrl+C: Exit")
        print()
        
        self.running = True
        
        # Start control thread
        self.control_thread = threading.Thread(target=self._control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        
        # Start input thread
        self.input_thread = threading.Thread(target=self._input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()
        
        try:
            # Main loop
            while self.running:
                time.sleep(0.1)
                
                # Log debug info periodically
                current_time = time.time()
                if self.debug_mode and current_time - self.last_log_time >= self.log_interval:
                    self._log_debug_info()
                    self.last_log_time = current_time
                    
        except KeyboardInterrupt:
            print("\nShutting down...")
            self.stop()
    
    def stop(self):
        """Stop the hexapod simulation"""
        self.running = False
        if self.control_thread:
            self.control_thread.join(timeout=1.0)
        if self.input_thread:
            self.input_thread.join(timeout=1.0)
        print("Simulation stopped.")
    
    def _control_loop(self):
        """Main control loop that updates hexapod state"""
        last_time = time.time()
        
        while self.running:
            current_time = time.time()
            dt = current_time - last_time
            
            # Update hexapod controller
            self.controller.update(dt)
            
            # Send state to Unreal Engine (if connected)
            if self.unreal_interface:
                self._send_to_unreal()
            
            # Maintain control frequency
            sleep_time = max(0, (1.0 / self.controller.control_frequency) - dt)
            if sleep_time > 0:
                time.sleep(sleep_time)
            
            last_time = time.time()
    
    def _input_loop(self):
        """Input handling loop for keyboard input"""
        try:
            import msvcrt  # Windows
            self._windows_input_loop()
        except ImportError:
            try:
                import tty
                import termios
                self._unix_input_loop()
            except ImportError:
                print("Warning: No keyboard input support available")
                self._dummy_input_loop()
    
    def _windows_input_loop(self):
        """Windows-specific input handling"""
        import msvcrt
        
        while self.running:
            if msvcrt.kbhit():
                key = msvcrt.getch()
                self._process_key(key.decode('utf-8', errors='ignore'), True)
            time.sleep(0.01)
    
    def _unix_input_loop(self):
        """Unix-specific input handling"""
        import tty
        import termios
        import sys
        
        # Save terminal settings
        old_settings = termios.tcgetattr(sys.stdin)
        
        try:
            tty.setraw(sys.stdin.fileno())
            
            while self.running:
                if sys.stdin.readable():
                    key = sys.stdin.read(1)
                    self._process_key(key, True)
                time.sleep(0.01)
                
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    
    def _dummy_input_loop(self):
        """Dummy input loop for systems without keyboard support"""
        while self.running:
            time.sleep(1.0)
    
    def _process_key(self, key: str, pressed: bool):
        """Process a single key press/release"""
        # Map common key codes
        key_map = {
            '\x1b[A': 'arrow_up',      # Up arrow
            '\x1b[B': 'arrow_down',    # Down arrow
            '\x1b[C': 'arrow_right',   # Right arrow
            '\x1b[D': 'arrow_left',    # Left arrow
            ' ': 'space',              # Space
            'q': 'q',                  # Q
            'e': 'e',                  # E
            'r': 'r',                  # R
            'w': 'w',                  # W
            'a': 'a',                  # A
            's': 's',                  # S
            'd': 'd',                  # D
        }
        
        mapped_key = key_map.get(key, key)
        self.controller.process_keyboard_input(mapped_key, pressed)
    
    def _send_to_unreal(self):
        """Send current state to Unreal Engine"""
        if not self.unreal_interface:
            return
        
        state = self.controller.get_state()
        data = {
            'position': state.position,
            'rotation': state.rotation,
            'joint_angles': [
                [angles.hip, angles.thigh, angles.shin]
                for angles in state.joint_angles
            ],
            'leg_positions': [
                [pos.x, pos.y, pos.z]
                for pos in state.leg_positions
            ],
            'gait_phase': state.gait_phase,
            'is_moving': state.is_moving
        }
        
        try:
            self.unreal_interface.send_state(data)
        except Exception as e:
            print(f"Error sending to Unreal: {e}")
    
    def _log_debug_info(self):
        """Log debug information"""
        debug_info = self.controller.get_debug_info()
        
        print(f"Position: ({debug_info['position'][0]:.3f}, {debug_info['position'][1]:.3f}, {debug_info['position'][2]:.3f})")
        print(f"Rotation: {debug_info['rotation']:.1f}°")
        print(f"Gait: {debug_info['gait_pattern']} (phase: {debug_info['gait_phase']:.2f})")
        print(f"Stability: {debug_info['stability']:.3f}")
        print(f"Moving: {debug_info['is_moving']}")
        print(f"Support legs: {sum(debug_info['support_legs'])}/6")
        print("-" * 50)

def main():
    """Main entry point"""
    print("Hexapod Kinematic Solver & Digital Twin")
    print("=======================================")
    
    # Create and start simulation
    simulation = HexapodSimulation()
    
    try:
        simulation.start()
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        simulation.stop()

if __name__ == "__main__":
    main() 