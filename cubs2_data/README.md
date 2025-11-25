# cubs2_data

Data files for Cubs2 including rosbag recordings and configuration.

**📚 [Full Documentation](https://cognipilot.github.io/cubs2/packages/cubs2_data.html)**

## Quick Start

### List Available Rosbag Files

```bash
cd $(ros2 pkg prefix cubs2_data)/share/cubs2_data/data
./download_rosbags.py --list
```

### Download Specific Files

```bash
# Download a specific file
./download_rosbags.py cub_stabilize_2025-10-21.mcap

# Download multiple files
./download_rosbags.py cub_stabilize_2025-10-21.mcap cub_stabilize_2025_11_13.mcap

# Download all files matching pattern
./download_rosbags.py cub_stabilize_*.mcap

# Download all available files
./download_rosbags.py --all
```

### Replay a Rosbag

```bash
ros2 bag play cub_stabilize_2025-10-21.mcap
```


