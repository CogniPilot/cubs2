# cubs2_rviz

Custom RViz2 panel plugins for Cubs2 aircraft simulation and control.

## Plugins

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
- geometry_msgs
- std_msgs
- sensor_msgs

## Testing

```bash
# Lint and style checks
colcon test --packages-select cubs2_rviz
```

## License

Apache-2.0
