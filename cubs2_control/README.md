# cubs2_control

Control algorithms for Cubs2 aircraft. These controllers are designed to be reusable across both simulation and real hardware.

## Features

- **Platform-agnostic** - No ROS-specific code in controllers
- **Reusable** - Same algorithms work in simulation and on hardware
- **Well-tested** - Comprehensive unit tests

## Controllers

### PID Controller

`pid_controller.py` - Classic PID control with anti-windup.

**Features:**
- Proportional, Integral, Derivative terms
- Anti-windup (integral clamping)
- Derivative filtering
- Configurable gains

**Usage:**

```python
from cubs2_control.pid_controller import PIDController

# Create controller
pid = PIDController(kp=1.0, ki=0.1, kd=0.05, dt=0.01)

# Set limits
pid.set_output_limits(-1.0, 1.0)

# Compute control
setpoint = 10.0
measurement = 8.5
control_output = pid.update(setpoint, measurement)

# Reset (e.g., on mode switch)
pid.reset()
```

## Testing

```bash
# Run controller tests
colcon test --packages-select cubs2_control
colcon test-result --verbose
```

## Dependencies

**Package Dependencies:**
- cubs2_dynamics - For dynamics models used in testing

**Build Dependencies:**
- ament_cmake
- ament_cmake_python

## Design Philosophy

Control algorithms in this package:

1. **Have no ROS dependencies** - Can be used outside ROS
2. **Are pure Python** - Easy to test and verify
3. **Use composition** - Controllers can be combined and tested independently
4. **Have clear interfaces** - Simple input/output relationships

This allows the same control code to run in:
- Simulation (`cubs2_simulation`)
- Hardware platforms
- MATLAB/Simulink
- Standalone test scripts

## License

Apache-2.0
