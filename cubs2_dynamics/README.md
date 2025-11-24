# cubs2_dynamics

Aircraft dynamics modeling and analysis tools for Cubs2. This package provides pure analysis and modeling capabilities with **no ROS dependencies**, making it reusable in non-ROS contexts.

## Features

- **Differentiable dynamics models** - Using CasADi for automatic differentiation
- **Linearization** - Compute linear approximations around operating points
- **Trim analysis** - Find equilibrium flight conditions
- **Numerical integration** - RK4 and other integrators
- **SportCub model** - Complete 6-DOF aircraft dynamics

## Modules

### Model Framework

- `model.py` - Base class for dynamics models with symbolic differentiation
- `integrators.py` - Numerical integration schemes (RK4, Euler)
- `linearize.py` - Linearization utilities for nonlinear systems

### Aircraft Models

- `sportcub.py` - Complete SportCub aircraft dynamics model
  - Aerodynamics (lift, drag, moments)
  - Propulsion (thrust)
  - Ground contact dynamics
  - 6-DOF rigid body equations

### Analysis Tools

- `trim_fixed_wing.py` - Compute trim conditions for steady flight

## Usage

### Simulate Aircraft Dynamics

```python
from cubs2_dynamics.sportcub import SportCubModel
from cubs2_dynamics.integrators import rk4
import numpy as np

# Create model
model = SportCubModel()

# Initial state [x, y, z, vx, vy, vz, qw, qx, qy, qz, p, q, r]
x0 = model.get_initial_state()

# Control input [aileron, elevator, rudder, throttle]
u = np.array([0.0, 0.1, 0.0, 0.5])

# Time step
dt = 0.01

# Simulate one step
x_next = rk4(lambda x: model.dynamics(x, u), x0, dt)
```

### Compute Trim Condition

```python
from cubs2_dynamics.trim_fixed_wing import compute_trim

# Find trim for level flight at 20 m/s
trim_state, trim_control = compute_trim(
    velocity=20.0,
    altitude=100.0,
    gamma=0.0  # level flight
)
```

### Linearization

```python
from cubs2_dynamics.linearize import linearize_dynamics

# Linearize around trim condition
A, B = linearize_dynamics(model, trim_state, trim_control)
```

## Testing

```bash
# Run all dynamics tests
colcon test --packages-select cubs2_dynamics
colcon test-result --verbose
```

Tests cover:
- Model consistency and Jacobian correctness
- Integration accuracy
- Trim computation
- Linearization

## Dependencies

**Python Libraries:**
- numpy - Numerical arrays
- scipy - Scientific computing
- casadi - Automatic differentiation and optimization

**Build Dependencies:**
- ament_cmake
- ament_cmake_python

## No ROS Dependencies

This package intentionally has **no ROS runtime dependencies** to keep the dynamics models reusable in:
- Non-ROS simulation environments
- MATLAB/Simulink integration
- Standalone analysis scripts
- Hardware-in-the-loop testing

## License

Apache-2.0
