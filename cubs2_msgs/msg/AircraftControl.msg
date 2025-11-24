# Aircraft control surface inputs
# All values are normalized

std_msgs/Header header

# Control surfaces [-1.0, 1.0]
float32 aileron   # Positive = roll right
float32 elevator  # Positive = pitch up
float32 rudder    # Positive = yaw right

# Throttle [0.0, 1.0]
float32 throttle  # 0 = idle, 1 = full power
