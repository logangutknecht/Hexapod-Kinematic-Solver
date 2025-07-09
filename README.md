# Hexapod Kinematic Solver & Digital Twin

A comprehensive kinematic solver and digital twin simulation for a hexapod robot with Unreal Engine 3D visualization.

## Features

- **6-DOF Hexapod Kinematics**: Forward and inverse kinematics for a 3+3 leg configuration
- **Real-time Simulation**: Live 3D visualization in Unreal Engine
- **Keyboard Control**: Arrow key movement with smooth gait patterns
- **Modular Architecture**: Separable kinematics engine and visualization
- **Digital Twin**: Real-time physics simulation with ground contact

## Project Structure

```
├── src/
│   ├── kinematics/          # Core mathematical engine
│   ├── unreal/             # Unreal Engine integration
│   └── control/            # Input handling and gait patterns
├── docs/                   # Documentation and tutorials
├── assets/                 # 3D models and textures
└── tests/                  # Unit tests and validation
```

## Quick Start

### Prerequisites
- Unreal Engine 5.3+ 
- Python 3.8+ (for kinematics engine)
- C++ development tools

### Setup
1. Clone this repository
2. Install Python dependencies: `pip install -r requirements.txt`
3. Open the Unreal project in `unreal/HexapodSimulation.uproject`
4. Run the kinematics engine: `python src/kinematics/main.py`

## Architecture

### Kinematics Engine
- **Forward Kinematics**: Calculate end-effector positions from joint angles
- **Inverse Kinematics**: Calculate joint angles for desired end-effector positions
- **Gait Planning**: Tripod, wave, and ripple gait patterns
- **Stability Analysis**: Center of mass and support polygon calculations

### Unreal Integration
- **Blueprint System**: Visual scripting for robot behavior
- **Physics Simulation**: Realistic ground contact and dynamics
- **Visual Feedback**: Real-time joint angle and trajectory visualization

## Controls

- **Arrow Keys**: Move hexapod forward/backward/left/right
- **WASD**: Rotate hexapod body
- **Space**: Toggle gait pattern
- **R**: Reset to initial position

## Development

See `docs/DEVELOPMENT.md` for detailed development guidelines and API documentation.
