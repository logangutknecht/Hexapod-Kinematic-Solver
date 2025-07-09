# Hexapod Kinematic Solver - Setup Guide

This guide will help you set up and run the hexapod kinematic solver with Unreal Engine integration.

## Prerequisites

### Required Software
- **Python 3.8+** - For the kinematics engine
- **Unreal Engine 5.3+** - For 3D visualization
- **C++ Development Tools** - For Unreal Engine compilation
- **Git** - For version control

### System Requirements
- **Windows 10/11** (recommended for Unreal Engine)
- **macOS 10.15+** (alternative)
- **Linux** (Python engine only)
- **8GB RAM minimum** (16GB recommended)
- **Graphics card** supporting DirectX 11 or OpenGL 4.3

## Installation Steps

### 1. Clone the Repository
```bash
git clone https://github.com/yourusername/Hexapod-Kinematic-Solver.git
cd Hexapod-Kinematic-Solver
```

### 2. Set Up Python Environment
```bash
# Create virtual environment
python -m venv venv

# Activate virtual environment
# Windows:
venv\Scripts\activate
# macOS/Linux:
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt
```

### 3. Set Up Unreal Engine

#### Option A: Using Epic Games Launcher
1. Download and install **Epic Games Launcher**
2. Install **Unreal Engine 5.3** from the launcher
3. Open the project: `unreal/HexapodSimulation.uproject`

#### Option B: Using Unreal Engine Source
1. Clone Unreal Engine source code
2. Compile Unreal Engine
3. Open the project: `unreal/HexapodSimulation.uproject`

### 4. Configure Unreal Engine Project

#### Enable Required Plugins
1. Open the project in Unreal Engine
2. Go to **Edit > Plugins**
3. Enable the following plugins:
   - **Python Script Plugin**
   - **SocketIO Client** (if available)

#### Build C++ Code
1. Right-click on `HexapodSimulation.uproject`
2. Select **Generate Visual Studio project files**
3. Open the generated `.sln` file
4. Build the solution (Release or Development)

### 5. Set Up Communication (Optional)

For real-time communication between Python and Unreal Engine:

#### Install Socket.IO (Python)
```bash
pip install python-socketio
```

#### Install Socket.IO (Unreal Engine)
1. Download Socket.IO plugin for Unreal Engine
2. Place in `unreal/Plugins/` directory
3. Rebuild the project

## Running the Application

### 1. Start Python Kinematics Engine
```bash
# From project root
python src/main.py
```

This will start the hexapod control system with keyboard input.

### 2. Start Unreal Engine Visualization
1. Open `unreal/HexapodSimulation.uproject`
2. Click **Play** to run the simulation
3. The hexapod should appear in the 3D environment

### 3. Controls

#### Python Engine Controls
- **Arrow Keys / WASD**: Move hexapod
- **Q/E**: Rotate left/right
- **Space**: Toggle gait pattern
- **R**: Reset position
- **Ctrl+C**: Exit

#### Unreal Engine Controls
- **Mouse**: Look around
- **WASD**: Move camera
- **F**: Focus on hexapod
- **G**: Toggle debug visualization

## Configuration

### Python Configuration
Edit `src/kinematics/hexapod_kinematics.py` to modify:
- Robot dimensions
- Joint limits
- Movement parameters

### Unreal Engine Configuration
Edit the `AHexapodRobot` class to modify:
- Visual appearance
- Animation speed
- Debug settings

## Troubleshooting

### Common Issues

#### Python Import Errors
```bash
# Ensure you're in the correct directory
cd /path/to/Hexapod-Kinematic-Solver
python src/main.py
```

#### Unreal Engine Build Errors
1. Clean and rebuild the solution
2. Ensure all plugins are enabled
3. Check Unreal Engine version compatibility

#### Communication Issues
1. Verify Socket.IO plugin is installed
2. Check firewall settings
3. Ensure both applications are running

### Performance Optimization

#### Python Engine
- Reduce control frequency in `HexapodController`
- Disable debug logging
- Use optimized NumPy operations

#### Unreal Engine
- Reduce polygon count on meshes
- Disable unnecessary debug visualization
- Use LOD (Level of Detail) for complex scenes

## Development

### Project Structure
```
├── src/                    # Python source code
│   ├── kinematics/         # Mathematical engine
│   ├── control/           # Control system
│   └── main.py            # Main application
├── unreal/                # Unreal Engine project
│   ├── Source/            # C++ source code
│   ├── Content/           # Assets and blueprints
│   └── HexapodSimulation.uproject
├── docs/                  # Documentation
├── tests/                 # Unit tests
└── requirements.txt       # Python dependencies
```

### Adding New Features

#### Python Engine
1. Add new classes in appropriate modules
2. Update imports in `main.py`
3. Add tests in `tests/` directory

#### Unreal Engine
1. Add new C++ classes in `unreal/Source/`
2. Create Blueprint assets in `unreal/Content/`
3. Update build configuration if needed

### Testing
```bash
# Run Python tests
pytest tests/

# Run with coverage
pytest --cov=src tests/
```

## Support

For issues and questions:
1. Check the troubleshooting section
2. Review the documentation
3. Create an issue on GitHub
4. Contact the development team

## License

This project is licensed under the MIT License - see the LICENSE file for details. 