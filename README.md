# Cubs2 - Fixed-Wing Aircraft Simulation and Control

[![CI](https://github.com/jgoppert/cubs2/actions/workflows/ci.yml/badge.svg)](https://github.com/jgoppert/cubs2/actions/workflows/ci.yml)
[![Coverage](https://img.shields.io/badge/coverage-62%25-orange)](https://github.com/jgoppert/cubs2)
[![Documentation](https://img.shields.io/badge/docs-latest-blue)](https://cognipilot.github.io/cubs2/)

![Figure-8 Racecourse](racecourse_description/images/figure8.png)

A ROS2 Jazzy monorepo for simulating and controlling fixed-wing aircraft using the SportCub aircraft model. This workspace provides complete simulation, visualization, planning, and control capabilities for autonomous fixed-wing aircraft.

📚 **[Full Documentation](https://cognipilot.github.io/cubs2/)** | [Installation Guide](docs/user_guide/installation.rst) | [Quick Start](docs/user_guide/quickstart.rst)

## Packages

### cubs2_msgs
Message definitions for aircraft control and telemetry.
- `AircraftControl.msg` - Normalized control inputs (aileron, elevator, rudder, throttle)

### cubs2_description  
Aircraft description files: URDF models and 3D meshes for the SportCub.

### racecourse_description
Racecourse configurations: gate and pylon definitions in YAML format, visualization markers.

### cubs2_data
Data files including rosbag recordings and reference flight data.

### cubs2_dynamics
Aircraft dynamics modeling and analysis tools using CasADi.
- Differentiable dynamics models
- Linearization and trim analysis
- Numerical integration

### cubs2_control
Control algorithms for aircraft (used in simulation and hardware).
- Autolevel attitude stabilization controller
- Closed-loop system composition (aircraft + controller)
- PID controller implementation
- Reusable across simulation and real hardware

### cubs2_simulation
Simulation nodes and runtime.
- SportCub simulation node
- Gamepad/keyboard control interfaces
- TF broadcasting

### cubs2_planning
Path planning algorithms for autonomous flight.
- Dubins path planner for racecourse navigation
- Waypoint gate management

### cubs2_rviz
Custom RViz panel plugins:
- Virtual joystick for manual control
- Simulation control (reset, pause, speed)
- HUD for telemetry display

### cubs2_bringup
Launch files for starting complete systems (simulation, visualization, planning).

## Quick Start

```bash
# Build workspace
cd /path/to/your/workspace
colcon build --symlink-install
source install/setup.bash

# Launch simulation with visualization (all-in-one)
ros2 launch cubs2_bringup sim.xml
```

This single command starts simulation, visualization, and replay by default.

### Launch Options

Customize the launch with optional arguments:

```bash
# Disable visualization
ros2 launch cubs2_bringup sim.xml viz:=false

# Disable replay
ros2 launch cubs2_bringup sim.xml replay:=false

# Enable gamepad control
ros2 launch cubs2_bringup sim.xml gamepad:=true

# Launch only specific components
ros2 launch cubs2_bringup viz.xml        # Visualization only
ros2 launch cubs2_bringup gamepad_control.xml  # Gamepad only
```

### Replay Mode

Replay is **enabled by default**, showing a "ghost" plane following recorded flight data.
The replay loops continuously for easy comparison with live simulation.

```bash
# Use different bag file
ros2 launch cubs2_bringup sim.xml bag_path:=/path/to/your/bag.mcap
```

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/control` | `cubs2_msgs/AircraftControl` | Control inputs to aircraft |
| `/pose` | `geometry_msgs/PoseStamped` | Aircraft pose |
| `/imu` | `sensor_msgs/Imu` | IMU data from simulation |
| `/racecourse/gates` | `visualization_msgs/MarkerArray` | Racecourse gate markers |

## Dependencies

- ROS2 Jazzy
- Python 3.12+
- NumPy
- CasADi (for dynamics modeling)
- Qt5 (for RViz panels)

## Development

### Building and Testing

```bash
# Build all packages
colcon build --symlink-install

# Run tests
colcon test
colcon test-result --verbose

# Or use the Makefile
cd src/cubs2
make build
make test
```

### Code Formatting and Linting

This project follows ROS2 coding standards with automated formatting:

**Python** (Black + isort + flake8):
```bash
cd src/cubs2
make format-python
```

**C++** (clang-format):
```bash
cd src/cubs2
make format-cpp
```

**All code**:
```bash
cd src/cubs2
make format
```

**Run linters**:
```bash
cd src/cubs2
make lint
```

Configuration files in `.devtools/`:
- Python: `pyproject.toml`, `.flake8`
- C++: `.clang-format`
- Scripts: `format_python.sh`, `format_cpp.sh`

## Features

- **Hierarchical Model Composition**: Compose aircraft dynamics and controllers into integrated systems
- **Type-Safe Modeling**: Structured state/input/output access with dataclasses
- **Single Integration Loop**: Numerically accurate unified integration of all subsystems
- **Dubins Path Planning**: Optimal path generation through waypoint gates
- **Physics-Based Simulation**: CasADi-based differentiable dynamics
- **RViz Integration**: 3D visualization with custom panels
- **Configurable Racecourses**: YAML-based gate definitions
- **Replay Support**: Compare simulation with recorded flights

## Package Documentation

Each package has detailed documentation in its own README:

- [cubs2_msgs](cubs2_msgs/README.md) - Message definitions
- [cubs2_description](cubs2_description/README.md) - Aircraft URDF and meshes
- [cubs2_data](cubs2_data/README.md) - Rosbag data files
- [cubs2_dynamics](cubs2_dynamics/README.md) - Dynamics modeling
- [cubs2_control](cubs2_control/README.md) - Control algorithms
- [cubs2_simulation](cubs2_simulation/README.md) - Simulation runtime
- [cubs2_planning](cubs2_planning/README.md) - Path planning
- [cubs2_rviz](cubs2_rviz/README.md) - RViz plugins
- [cubs2_bringup](cubs2_bringup/README.md) - Launch files
- [racecourse_description](racecourse_description/README.md) - Racecourse configuration

## License

Apache-2.0

