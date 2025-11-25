# Gamepad Configuration Guide

This guide explains how to configure your gamepad for use with the cubs2 gamepad control system.

## Quick Start

The default configuration in `8bitdo_81hd.yaml` is set up for an **8BitDo Ultimate Wireless / 2.4g Controller (Xbox Mode)**. If you have this controller, you should be able to use it immediately without any changes.

For other controllers, follow the steps below to customize the configuration.

## Finding Your Gamepad's Button and Axis Mappings

1. **Connect your gamepad** via USB or Bluetooth

2. **Launch the joy node** to start reading gamepad input:
   ```bash
   ros2 run joy joy_node
   ```

3. **Echo the joy topic** to see the raw data:
   ```bash
   ros2 topic echo /joy
   ```

4. **Press buttons and move sticks** one at a time:
   - When you press a button, look at the `buttons` array - the index of the `1` is your button number
   - When you move a stick or trigger, look at the `axes` array - the index of the changing value is your axis number
   - Note the direction: does moving the stick left/up produce positive or negative values?

5. **Record your mappings** in a text file for reference

## Editing the Configuration File

Open `cubs2_bringup/config/8bitdo_81hd.yaml` and update the values based on your findings:

### Button Mappings

Update these to match your controller's button layout:

```yaml
button_reset_neutral: 0        # Button to reset sticks and trim to neutral
button_send_reset: 1           # Button to reset the simulation
button_trim_rudder_left: 2     # Button to trim rudder left
button_trim_rudder_right: 3    # Button to trim rudder right
button_exit: 8                 # Button to exit the node
button_pause_toggle: 7         # Button to pause/unpause simulation
```

**Note:** Set a button to `-1` to disable that function.

### Axis Mappings

Update these to match your controller's stick layout:

```yaml
axis_throttle: 1               # Left stick vertical
axis_rudder: 0                 # Left stick horizontal
axis_elevator: 4               # Right stick vertical
axis_aileron: 3                # Right stick horizontal
```

### D-pad Configuration

There are two types of D-pads:

#### Axis-based D-pad (most common)
If your D-pad shows up as axes when you echo `/joy`:

```yaml
dpad_is_buttons: false
axis_dpad_horizontal: 6        # D-pad left/right axis
axis_dpad_vertical: 7          # D-pad up/down axis
dpad_deadzone: 0.5             # Threshold for detecting D-pad press
```

#### Button-based D-pad
If your D-pad shows up as buttons when you echo `/joy`:

```yaml
dpad_is_buttons: true
button_dpad_up: 11
button_dpad_down: 12
button_dpad_left: 13
button_dpad_right: 14
```

### Axis Inversions

If a control surface moves the wrong direction, invert its axis:

```yaml
invert_aileron: false          # Set to true if aileron is backwards
invert_elevator: false         # Set to true if elevator is backwards
invert_rudder: false           # Set to true if rudder is backwards
invert_throttle: false         # Set to true if throttle is backwards
```

## Common Controller Profiles

### 8BitDo Ultimate Wireless / 2.4g Controller - Xbox Mode (Default)
```yaml
# Face buttons
button_reset_neutral: 0        # A
button_send_reset: 1           # B
button_trim_rudder_left: 2     # X
button_trim_rudder_right: 3    # Y

# Axes
axis_throttle: 1               # Left stick Y
axis_rudder: 0                 # Left stick X
axis_elevator: 4               # Right stick Y
axis_aileron: 3                # Right stick X
axis_dpad_horizontal: 6
axis_dpad_vertical: 7
dpad_is_buttons: false
```

### Xbox Controller
Similar to Switch Pro, but check your specific model's mappings.

### PlayStation Controller
Face buttons are typically in different positions:
- Cross (X) is usually button 0
- Circle (O) is usually button 1
- Square is usually button 2
- Triangle is usually button 3

## Control Mappings

Once configured, your gamepad controls the aircraft as follows:

### Sticks
- **Left Stick Up/Down**: Throttle increment/decrement (RC transmitter style)
  - Push up to increase throttle
  - Push down to decrease throttle
  - Spring-loaded stick returns to center automatically
  - Gradual acceleration: 1.5x after 0.5s, 2x after 1.5s, 4x after 3s
  - Separate up/down tracking for precise control
- **Left Stick Left/Right**: Rudder
- **Right Stick Up/Down**: Elevator (pitch)
- **Right Stick Left/Right**: Aileron (roll)

### Trim Controls
- **D-pad Up**: Trim elevator up
- **D-pad Down**: Trim elevator down
- **D-pad Left**: Trim aileron left
- **D-pad Right**: Trim aileron right
- **X Button**: Trim rudder left
- **Y Button**: Trim rudder right

**Trim Acceleration**: Hold a trim button to accelerate the trim rate (2x after 5 presses, 4x after 15, 8x after 30)

### Other Buttons
- **A Button**: Reset all controls and trim to neutral
- **B Button**: Send reset signal (resets simulation)
- **Start/Plus**: Toggle pause
- **Home**: Exit gamepad control node

## Testing Your Configuration

1. **Build the package** after editing the config:
   ```bash
   cd ~/ws_fixedwing
   colcon build --packages-select cubs2_bringup cubs2_control
   source install/setup.bash
   ```

2. **Launch the simulation with gamepad**:
   ```bash
   ros2 launch cubs2_bringup sim.xml gamepad:=true
   ```

3. **Verify the controls** work correctly:
   - Move sticks to verify control surfaces respond correctly
   - Test trim buttons to ensure they adjust properly
   - Check that all buttons perform their expected functions

## Troubleshooting

### My controller isn't detected
- Check that `/dev/input/js0` exists: `ls -l /dev/input/js*`
- Try a different device: edit `gamepad_control.xml` and change `js0` to `js1`

### Controls are reversed
- Set the appropriate `invert_*` flag to `true` in `8bitdo_81hd.yaml`

### Buttons don't work
- Verify button indices with `ros2 topic echo /joy`
- Make sure your button indices are within the range of buttons your controller has
- Check that you've rebuilt the package after editing the config

### D-pad doesn't work
- Determine if your D-pad uses axes or buttons (echo `/joy` and press D-pad)
- Set `dpad_is_buttons` accordingly in `8bitdo_81hd.yaml`
- Update either the axis or button mappings for the D-pad

### Trim changes too slowly/quickly
- Adjust `trim_step` in `8bitdo_81hd.yaml` (default is 0.01 radians = ~0.57 degrees)
- The trim automatically accelerates when held, so be patient for fine adjustments

### Throttle changes too slowly/quickly  
- Adjust `throttle_step` in `8bitdo_81hd.yaml` (default is 0.01 = 1% per stick deflection)
- The throttle has slower acceleration than trim controls (1.5x → 2x → 4x) for better precision
- Each up/down direction has its own acceleration counter for fine control
