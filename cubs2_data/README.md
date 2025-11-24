# cubs2_data

Data files for Cubs2 including rosbag recordings and reference flight data.

## Contents

### Rosbag Recordings

- `data/cub_stabilize_2025-10-21.mcap` - Stabilized flight test data

### Download Script

```bash
cd data/
./download_rosbags.sh
```

The script automatically downloads missing rosbag files from the repository.

## Usage

### Replay a Rosbag

```bash
ros2 bag play $(ros2 pkg prefix cubs2_data)/share/cubs2_data/data/cub_stabilize_2025-10-21.mcap
```

### Use in Launch File

```python
from ament_index_python.packages import get_package_share_directory
import os

data_path = get_package_share_directory('cubs2_data')
bag_file = os.path.join(data_path, 'data', 'cub_stabilize_2025-10-21.mcap')
```

### Replay with Simulation

See `cubs2_bringup` package for launching replay mode:

```bash
ros2 launch cubs2_bringup sim.xml enable_replay:=true
```

## File Format

All recordings use the MCAP format (.mcap), which is the ROS2 Jazzy default rosbag format.

## License

Apache-2.0
