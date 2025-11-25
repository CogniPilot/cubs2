# cubs2_dynamics

Aircraft dynamics modeling and analysis tools for Cubs2.

**📚 [Full Documentation](https://cognipilot.github.io/cubs2/packages/cubs2_dynamics.html)**

## Features

- **Differentiable dynamics models** - Using CasADi for automatic differentiation
- **Hierarchical model composition** - Combine multiple models into integrated systems
- **Type-safe modeling** - Structured dataclass-based state, input, and output definitions
- **Linearization** - Compute linear approximations around operating points
- **Trim analysis** - Find equilibrium flight conditions
- **Numerical integration** - RK4 and other integrators with single-loop composition
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

### Hierarchical Model Composition

The framework supports composing multiple subsystem models into a single integrated model with a unified integration loop:

```python
from cubs2_dynamics.model import ModelSX
from cubs2_dynamics.sportcub import sportcub
from cubs2_control.autolevel_controller import autolevel_controller

# Create submodels
aircraft = sportcub()
controller = autolevel_controller()

# Compose into parent system
parent = ModelSX.compose({
    "plant": aircraft,
    "controller": controller
})

# Connect signals between subsystems
parent.connect("controller.u.q", "plant.x.r")  # Controller reads aircraft orientation
parent.connect("controller.u.omega", "plant.x.w")  # Controller reads angular velocity
parent.connect("plant.u.ail", "controller.y.ail")  # Aircraft gets aileron command

# Build integrated dynamics with single RK4 integrator
parent.build_composed(integrator="rk4")

# Simulate composed system (single integration step for all subsystems)
x_next = parent.f_step(x=x0_vec, u=u_vec, p=p_vec, dt=0.01)

# Access subsystem states with structured syntax
aircraft_pos = x0.plant.p  # Aircraft position
controller_integral = x0.controller.i_p  # Controller integral state
```

**Key Features:**
- **Single integration loop** - All subsystems integrated together for numerical accuracy
- **Structured state access** - `x.plant.p`, `x.controller.i_p` instead of index-based access
- **Type-safe connections** - String-based paths with compile-time validation
- **Automatic state composition** - Parent state created by merging subsystem states

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
