# cubs2_planning

Path planning algorithms for autonomous flight of the Cubs2 SportCub aircraft.

## Features

- **Dubins path planning** - Optimal paths for fixed-wing aircraft
- **Racecourse navigation** - Waypoint sequencing through gates
- **RViz visualization** - Path and waypoint markers

## Nodes

### planner_dubins.py

Autonomous path planner using Dubins curves.

**Published Topics:**
- `/planned_path` (nav_msgs/Path) - Generated trajectory
- `/waypoint_markers` (visualization_msgs/MarkerArray) - Gate visualization

**Subscribed Topics:**
- `/sportcub/pose` (geometry_msgs/PoseStamped) - Current aircraft position

**Parameters:**
- `gate_sequence` (list[int]) - Order of gates to visit
- `turning_radius` (float, default: 50.0) - Minimum turn radius (meters)
- `cruise_altitude` (float, default: 15.0) - Flight altitude (meters)

## Dubins Path Planning

Dubins paths are the shortest curves connecting two poses with bounded curvature. Perfect for fixed-wing aircraft that cannot turn arbitrarily tight.

### Path Types

- **RSR** - Right-Straight-Right
- **RSL** - Right-Straight-Left
- **LSR** - Left-Straight-Right
- **LSL** - Left-Straight-Left
- **RLR** - Right-Left-Right
- **LRL** - Left-Right-Left

The planner automatically selects the shortest valid path.

## Usage

### Launch Planner

```bash
ros2 run cubs2_planning planner_dubins
```

### Custom Gate Sequence

```bash
ros2 launch cubs2_bringup planning.xml gate_sequence:="[0,1,2,3,4,5]"
```

### Python API

```python
from cubs2_planning.dubins import dubins_path

# Compute path between two poses
start = (0, 0, 0)  # (x, y, heading)
goal = (100, 50, np.pi/4)
turning_radius = 50.0

path_length, path_type, params = dubins_path(start, goal, turning_radius)
```

## Racecourse Integration

The planner reads gate positions from `racecourse_description`:

```python
from racecourse_description import RacecourseLoader

loader = RacecourseLoader()
gates = loader.get_gates()  # Returns list of gate positions
```

## Visualization

The planner publishes:
- **Path** - Green line showing planned trajectory
- **Waypoints** - Colored spheres at gate positions
- **Current Target** - Highlighted gate

View in RViz:
```bash
ros2 launch cubs2_bringup viz.xml
```

## Testing

```bash
colcon test --packages-select cubs2_planning
colcon test-result --verbose
```

## Dependencies

**ROS2 Packages:**
- rclpy
- geometry_msgs
- visualization_msgs
- nav_msgs

**Cubs2 Packages:**
- cubs2_msgs
- cubs2_description
- racecourse_description

**Python Libraries:**
- numpy

## Algorithm Details

Dubins path computation uses the classification from:
> Shkel, A. M. and Lumelsky, V. (2001). "Classification of the Dubins set"

The implementation handles:
- All 6 path types (CSC and CCC families)
- Singularities (e.g., when start and goal are collinear)
- Path length optimization
- Parameter extraction for trajectory generation

## License

Apache-2.0
