# racecourse_description

Racecourse environment configuration and visualization for Cubs2 aircraft racing.

## Features

- **YAML-based configuration** - Easy gate and pylon definitions
- **Python loader** - Programmatic access to racecourse data
- **RViz markers** - Real-time visualization of gates and pylons

## Contents

### Configuration

`config/racecourse.yaml` - Gate and pylon definitions

```yaml
gates:
  - name: "Start/Finish"
    position: [0.0, 0.0, 10.0]   # [x, y, z] meters
    yaw_deg: 0.0                 # Orientation
    width: 20.0                  # Gate width
    height: 15.0                 # Gate height

pylons:
  - name: "Pylon A"
    position: [50.0, -30.0, 0.0]
    radius: 10.0                 # Turn radius
```

### Python API

`racecourse_description/loader.py` - Load racecourse configuration

```python
from racecourse_description import RacecourseLoader

loader = RacecourseLoader()
gates = loader.get_gates()       # List of gate dicts
pylons = loader.get_pylons()     # List of pylon dicts
```

### Visualization Node

`nodes/racecourse_markers.py` - Publishes visualization markers

**Published Topics:**
- `/racecourse/gates` (visualization_msgs/MarkerArray)
- `/racecourse/pylons` (visualization_msgs/MarkerArray)

## Usage

### Launch Marker Node

```bash
ros2 run racecourse_description racecourse_markers
```

### Load Configuration in Python

```python
from racecourse_description import RacecourseLoader
import numpy as np

loader = RacecourseLoader()

# Get all gates
for gate in loader.get_gates():
    print(f"Gate {gate['name']} at {gate['position']}")

# Get specific gate
gate_5 = loader.get_gate(5)
position = np.array(gate_5['position'])
```

### Visualize in RViz

1. Launch visualization: `ros2 launch cubs2_bringup viz.xml`
2. Add MarkerArray display
3. Set topic to `/racecourse/gates`

## Coordinate System

- **Origin**: Start/finish gate location
- **Units**: Meters
- **Frame**: `map` (world frame)
- **Orientation**: Yaw angle in degrees (0° = north, 90° = east)

## Gate Structure

Each gate is defined by:
- **Position** - 3D coordinates of gate center
- **Orientation** - Yaw angle (heading direction to pass through)
- **Dimensions** - Width and height for collision detection
- **Name** - Human-readable identifier

## Pylon Structure

Each pylon is defined by:
- **Position** - 3D coordinates (typically ground level)
- **Radius** - Turn radius around pylon
- **Name** - Human-readable identifier

## Editing the Racecourse

1. Edit `config/racecourse.yaml`
2. Rebuild package: `colcon build --packages-select racecourse_description`
3. Restart visualization to see changes

## Integration

### With Planning

The `cubs2_planning` package reads this configuration to generate waypoint sequences:

```python
from racecourse_description import RacecourseLoader

loader = RacecourseLoader()
gates = loader.get_gates()

# Plan path through gates
for gate_id in gate_sequence:
    target = gates[gate_id]['position']
    # ... path planning ...
```

### With Simulation

Racecourse markers are displayed alongside simulation for visual reference.

## Testing

```bash
# Test marker node
ros2 run racecourse_description racecourse_markers

# Verify markers appear
ros2 topic echo /racecourse/gates
```

## Dependencies

**ROS2 Packages:**
- rclpy
- visualization_msgs
- geometry_msgs

**Python Libraries:**
- yaml (PyYAML)

## License

Apache-2.0
