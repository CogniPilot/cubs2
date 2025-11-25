# cubs2_bringup

System-level launch files orchestrating simulation, visualization, and planning.

**📚 [Full Documentation](https://cognipilot.github.io/cubs2/packages/cubs2_bringup.html)**

## Launch Files

### sim.xml

Complete aircraft simulation with visualization and autonomous flight.

**Arguments:**
- `replay` (default: true) - Show recorded flight data
- `viz` (default: true) - Launch RViz visualization
- `gamepad` (default: false) - Enable gamepad control
- `bag_path` (default: cubs2_data rosbag) - Path to replay data

**Nodes Started:**
- Simulation node (`cubs2_simulation`)
- Dubins path planner (`cubs2_planning`)
- Robot state publisher (`cubs2_description`)
- RViz2 with custom panels (if `viz:=true`)
- Rosbag replay player (if `replay:=true`)

**Usage:**
```bash
# Standard simulation (visualization + replay enabled by default)
ros2 launch cubs2_bringup sim.xml

# Simulation only (no visualization)
ros2 launch cubs2_bringup sim.xml viz:=false

# Disable replay
ros2 launch cubs2_bringup sim.xml replay:=false

# Enable gamepad control
ros2 launch cubs2_bringup sim.xml gamepad:=true

# Custom bag file
ros2 launch cubs2_bringup sim.xml bag_path:=/path/to/bag.mcap
```

### viz.xml

RViz2 visualization with custom panels.

**Nodes Started:**
- RViz2 with pre-configured layout
- All visualization plugins loaded (Joy, Sim, HUD panels)

**Usage:**
```bash
ros2 launch cubs2_bringup viz.xml
```

### gamepad_control.xml

Manual control via gamepad/joystick.

**Arguments:**
- `device` (default: /dev/input/js0) - Joystick device path

**Nodes Started:**
- Joy node (sensor_msgs/Joy publisher)
- Gamepad control translator (`cubs2_simulation`)

**Usage:**
```bash
ros2 launch cubs2_bringup gamepad_control.xml
```

### keyboard_control.xml

Manual control via keyboard.

**Nodes Started:**
- Keyboard control node

**Usage:**
```bash
ros2 launch cubs2_bringup keyboard_control.xml
```

### planning.xml

Standalone path planner (no simulation).

**Arguments:**
- `gate_sequence` (default: [0,1,2,...]) - Waypoint order

**Usage:**
```bash
ros2 launch cubs2_bringup planning.xml gate_sequence:="[0,2,4,6]"
```

## Complete Workflow

### 1. Standard Simulation (Recommended)

```bash
# Single command - starts everything
ros2 launch cubs2_bringup sim.xml
```

This launches simulation, visualization, and replay in one terminal.

### 2. Simulation Without Visualization

```bash
# Run simulation in headless mode
ros2 launch cubs2_bringup sim.xml viz:=false
```

### 3. Manual Control

```bash
# With gamepad
ros2 launch cubs2_bringup sim.xml gamepad:=true

# Or launch gamepad separately
ros2 launch cubs2_bringup gamepad_control.xml
```

### 4. Visualization Only

```bash
# Connect to running simulation or real aircraft
ros2 launch cubs2_bringup viz.xml
```

## Configuration

### RViz Layout

Default RViz configuration: `config/cubs2.rviz` (not in this package, but loaded from cubs2_rviz)

### Racecourse Gates

Gate definitions: `racecourse_description/config/racecourse.yaml`

## Dependencies

**Execution Dependencies:**
- cubs2_simulation
- cubs2_planning
- cubs2_rviz
- racecourse_description

All runtime components are brought together in this package.

## Design Philosophy

This package follows the ROS2 "bringup" pattern:

1. **No code** - Only launch files and configuration
2. **System integration** - Combines components from other packages
3. **Parameterized** - Flexible launch arguments for different scenarios
4. **Documented** - Clear usage examples

## License

Apache-2.0
