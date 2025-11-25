# cubs2_bringup

System-level launch files orchestrating simulation, visualization, and planning.

**📚 [Full Documentation](https://cognipilot.github.io/cubs2/packages/cubs2_bringup.html)**

## Launch Files

### sim.xml

Complete aircraft simulation with autonomous flight.

**Arguments:**
- `enable_sim` (default: true) - Run live simulation
- `enable_replay` (default: false) - Show recorded flight data
- `show_forces` (default: true) - Visualize forces/moments
- `rate` (default: 1.0) - Simulation speed multiplier
- `gate_sequence` (default: [0,1,2,4,3,...]) - Waypoint order
- `bag_path` (default: cubs2_data rosbag) - Path to replay data

**Nodes Started:**
- Simulation node (`cubs2_simulation`)
- Dubins path planner (`cubs2_planning`)
- Robot state publisher (`cubs2_description`)
- Racecourse markers (`racecourse_description`)

**Usage:**
```bash
# Standard simulation
ros2 launch cubs2_bringup sim.xml

# 2x speed with replay comparison
ros2 launch cubs2_bringup sim.xml rate:=2.0 enable_replay:=true

# Custom gate sequence
ros2 launch cubs2_bringup sim.xml gate_sequence:="[0,1,2,3,4]"
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

### 1. Standard Simulation

```bash
# Terminal 1: Launch simulation
ros2 launch cubs2_bringup sim.xml

# Terminal 2: Launch visualization
ros2 launch cubs2_bringup viz.xml
```

### 2. Manual Control

```bash
# Terminal 1: Simulation only (no autonomous planner)
ros2 run cubs2_simulation sim

# Terminal 2: Gamepad control
ros2 launch cubs2_bringup gamepad_control.xml

# Terminal 3: Visualization
ros2 launch cubs2_bringup viz.xml
```

### 3. Replay Analysis

```bash
# Compare simulation with recorded flight
ros2 launch cubs2_bringup sim.xml enable_replay:=true show_forces:=true

# In separate terminal
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
