# Aircraft control surface inputs
# All values are normalized

std_msgs/Header header

# Control surfaces [-1.0, 1.0]
float32 aileron   # Positive = roll right
float32 elevator  # Positive = pitch up
float32 rudder    # Positive = yaw right

# Throttle [0.0, 1.0]
float32 throttle  # 0 = idle, 1 = full power

# Control mode
uint8 mode

# Control mode constants
uint8 MODE_MANUAL = 0      # Direct manual control
uint8 MODE_STABILIZED = 1  # Stabilized flight mode
