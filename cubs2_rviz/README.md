# cubs2_rviz

Custom RViz2 panels including low-latency GStreamer video streaming.

**📚 [Full Documentation](https://cognipilot.github.io/cubs2/packages/cubs2_rviz.html)**

## Plugins

### Video Panel

Live GStreamer video streaming panel for low-latency drone camera visualization.

**Features:**
- Direct GStreamer rendering (no ROS 2 image topic overhead)
- Multiple input sources: USB cameras, RTSP, UDP (H.264/MJPEG), test patterns
- Predefined source templates for common use cases
- Custom GStreamer pipeline support
- Real-time video display with automatic aspect ratio

**Usage:**
1. In RViz: `Panels` → `Add New Panel` → `cubs2::VideoPanel`
2. Select a predefined source or enter custom GStreamer URI
3. Click "Connect"

**Predefined Sources:**
- **Test Pattern**: SMPTE color bars for testing
- **USB Camera**: V4L2 camera on `/dev/video0`
- **RTSP Stream**: Network camera stream
- **UDP Stream (H.264)**: H.264 encoded UDP on port 5600
- **UDP Stream (MJPEG)**: MJPEG encoded UDP on port 5600

**Custom Pipeline Examples:**
```bash
# Different USB camera device
v4l2src device=/dev/video1 ! video/x-raw,width=1280,height=720,framerate=30/1

# RTSP with authentication
rtspsrc location=rtsp://username:password@192.168.1.100:8554/video latency=0 ! decodebin

# Custom UDP port
udpsrc port=5000 ! application/x-rtp,encoding-name=H264 ! rtph264depay ! avdec_h264

# File playback
filesrc location=/path/to/video.mp4 ! decodebin
```

**Drone Camera Setup:**

On companion computer (Raspberry Pi):
```bash
# Stream USB camera via UDP H.264
gst-launch-1.0 v4l2src device=/dev/video0 ! \
  video/x-raw,width=640,height=480,framerate=30/1 ! \
  x264enc tune=zerolatency bitrate=2000 speed-preset=superfast ! \
  rtph264pay ! udpsink host=<ground-station-ip> port=5600
```

In RViz Video Panel, select "UDP Stream (H.264)" and click "Connect".

**Required Dependencies:**
```bash
sudo apt-get install \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav
```

**Troubleshooting:**
- Check camera: `ls /dev/video*`
- Test pipeline: `gst-launch-1.0 videotestsrc ! autovideosink`
- View RViz terminal for detailed GStreamer errors
- For low latency: use UDP over RTSP, reduce resolution/framerate for limited bandwidth

**Performance Tips:**
- Start with 640x480 @ 30fps
- Use H.264 for network streams
- Set `tune=zerolatency` on encoder
- Prefer wired Ethernet for reliability

### Joy Panel

Virtual joystick for manual aircraft control.

**Features:**
- 2D joystick widget (aileron/elevator)
- Throttle and rudder sliders
- Trim controls for aileron and elevator
- Enable/disable checkbox to prevent conflicts with gamepad
- Spring-back to center when released

**Topics:**
- Publishes: `/control` (cubs2_msgs/AircraftControl)
- Subscribes: `/control` (to display external control when disabled)

### Sim Panel

Simulation control and configuration.

**Features:**
- Reset simulation button
- Pause/resume checkbox
- Simulation speed selector (0.25x to 100x)
- Time step selector (0.001s to 0.1s)
- Camera position publishing for advanced visualization

**Topics:**
- Publishes: `/reset`, `/pause`, `/set_speed`, `/set_dt`
- Subscribes: `/sat/paused` (to sync pause state)

### HUD Panel

Heads-up display for flight telemetry.

**Features:**
- Artificial horizon (pitch/roll)
- Roll indicator (arc with angle markers)
- Altitude display
- Airspeed display
- Real-time attitude visualization

**Topics:**
- Subscribes: `/sportcub/pose`, `/sportcub/velocity`

## Usage

### Load Panels in RViz

1. Launch RViz: `ros2 launch cubs2_bringup viz.xml`
2. In RViz menu: `Panels` → `Add New Panel`
3. Select from:
   - `cubs2::VideoPanel`
   - `cubs2/JoyPanel`
   - `cubs2/SimPanel`
   - `cubs2/HUDPanel`

### Pre-configured RViz Config

Panels are automatically loaded with:
```bash
ros2 launch cubs2_bringup viz.xml
```

## Development

### Building

```bash
colcon build --packages-select cubs2_rviz
```

### Plugin Architecture

Plugins use the RViz2 `rviz_common::Panel` API with Qt5 widgets.

**Key Components:**
- `include/cubs2_rviz/*.hpp` - Header files
- `src/rviz/*.cpp` - Implementation
- `plugin_description.xml` - Plugin registration
- `config/cubs2.rviz` - Pre-configured layout

### Adding a New Panel

1. Create header in `include/cubs2_rviz/my_panel.hpp`:
```cpp
#include <rviz_common/panel.hpp>

namespace cubs2 {
class MyPanel : public rviz_common::Panel {
  Q_OBJECT
public:
  MyPanel(QWidget* parent = nullptr);
};
}
```

2. Implement in `src/rviz/my_panel.cpp`
3. Export with `PLUGINLIB_EXPORT_CLASS(cubs2::MyPanel, rviz_common::Panel)`
4. Register in `plugin_description.xml`

### Qt Integration

Panels use Qt5 for GUI:
- **Widgets**: QSlider, QPushButton, QCheckBox, QLabel
- **Layouts**: QVBoxLayout, QHBoxLayout
- **Custom painting**: QPainter for HUD graphics
- **Timers**: QTimer for periodic updates

## Dependencies

**ROS2 Packages:**
- rclcpp
- rviz_common
- pluginlib

**Cubs2 Packages:**
- cubs2_msgs
- cubs2_description
- racecourse_description

**System:**
- Qt5 (qtbase5-dev)
- GStreamer 1.0 (libgstreamer1.0-dev, libgstreamer-plugins-base1.0-dev)
- geometry_msgs
- std_msgs
- sensor_msgs

## Testing

```bash
# Lint and style checks
colcon test --packages-select cubs2_rviz

# Test video stream (for Video Panel testing)
./scripts/test_video_stream.sh --pattern ball
```

## License

Apache-2.0
