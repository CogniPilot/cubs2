# cubs2_description

URDF models and visual meshes for the Cubs2 SportCub aircraft.

## Contents

### URDF Models

- `urdf/sportcub.urdf.xacro` - Main aircraft description with parametric geometry

### Meshes

- `meshes/plane.glb` - High-fidelity 3D model of SportCub aircraft

## Usage

### Launch with Robot State Publisher

```bash
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(xacro $(ros2 pkg prefix cubs2_description)/share/cubs2_description/urdf/sportcub.urdf.xacro)"
```

### Load in Launch File

```xml
<node pkg="robot_state_publisher" exec="robot_state_publisher">
  <param name="robot_description" 
         value="$(command 'xacro $(find-pkg-share cubs2_description)/urdf/sportcub.urdf.xacro')"/>
</node>
```

### Python API

```python
from ament_index_python.packages import get_package_share_directory
import os

description_path = get_package_share_directory('cubs2_description')
urdf_file = os.path.join(description_path, 'urdf', 'sportcub.urdf.xacro')
mesh_file = os.path.join(description_path, 'meshes', 'plane.glb')
```

## Dependencies

- robot_state_publisher - Publishes robot transforms
- xacro - URDF preprocessing

## License

Apache-2.0
