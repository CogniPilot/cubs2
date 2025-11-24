"""Type-safe CasADi modeling framework with full hybrid DAE support.

Provides declarative API for building hybrid dynamical systems without dynamic
class generation. Full autocomplete and type safety throughout.

Features:
- Continuous states (x): dx/dt = f_x(...)
- Algebraic variables (z_alg): 0 = g(x, z_alg, u, p) for DAE constraints
- Dependent variables (dep): dep = f_dep(x, u, p) for computed quantities
- Quadrature states (q): dq/dt = f_q(x, u, p) for path integrals
- Discrete states (z): z⁺ = f_z(...) updated at events
- Discrete variables (m): m⁺ = f_m(...) for integers/booleans
- Event indicators (c): event when c crosses zero
- Inputs (u): control signals
- Parameters (p): time-independent constants
- Outputs (y): y = f_y(x, u, p) observables/diagnostics

Example:
    @symbolic
    class States:
        h: ca.SX = state(1, 10.0, "height (m)")
        v: ca.SX = state(1, 0.0, "velocity (m/s)")

    @symbolic
    class Inputs:
        thrust: ca.SX = input_var(0.0, "thrust (N)")

    @symbolic
    class Params:
        m: ca.SX = param(1.0, "mass (kg)")
        g: ca.SX = param(9.81, "gravity (m/s^2)")

    model = ModelSX.create(States, Inputs, Params)

    x = model.x()
    u = model.u()
    p = model.p()

    f_x = ca.vertcat(x.v, u.thrust / p.m - p.g)
    model.build(f_x=f_x, integrator='rk4')
"""

from dataclasses import dataclass, field, fields
from typing import TypeVar, Generic, Union, Any, Callable
import numpy as np
import casadi as ca
from beartype import beartype

__all__ = [
    "symbolic",
    "state",
    "param",
    "input_var",
    "output_var",
    "algebraic_var",
    "dependent_var",
    "quadrature_var",
    "discrete_state",
    "discrete_var",
    "event_indicator",
    "ModelSX",
    "ModelMX",
]

# ============================================================================
# Field Creation Helpers
# ============================================================================


def state(dim: int = 1, default: Union[float, list, None] = None, desc: str = ""):
    """Create a continuous state variable field (dx/dt in equations).

    Args:
        dim: Dimension (1 for scalar, >1 for vector)
        default: Default value (scalar or list)
        desc: Description string

    Returns:
        dataclass field with metadata

    Example:
        p: ca.SX = state(3, [0, 0, 10], "position (m)")
        v: ca.SX = state(1, 0.0, "velocity (m/s)")
        w: ca.SX = state(3, desc="angular velocity")  # defaults to zeros
    """
    if default is None:
        default = 0.0 if dim == 1 else [0.0] * dim
    elif isinstance(default, (int, float)) and dim > 1:
        default = [float(default)] * dim
    return field(
        default=None, metadata={"dim": dim, "default": default, "desc": desc, "type": "state"}
    )


def algebraic_var(dim: int = 1, default: Union[float, list, None] = None, desc: str = ""):
    """Create algebraic variable for DAE constraints: 0 = g(x, z_alg, u, p).

    Used for implicit constraints in DAE systems (contact forces, Lagrange
    multipliers, kinematic loops, etc.).

    Args:
        dim: Dimension
        default: Initial guess for DAE solver
        desc: Description
    """
    if default is None:
        default = 0.0 if dim == 1 else [0.0] * dim
    elif isinstance(default, (int, float)) and dim > 1:
        default = [float(default)] * dim
    return field(
        default=None, metadata={"dim": dim, "default": default, "desc": desc, "type": "algebraic"}
    )


def dependent_var(dim: int = 1, default: Union[float, list, None] = None, desc: str = ""):
    """Create dependent variable: y = f_y(x, u, p) (computed, not stored).

    For quantities computed from states but not integrated (energy, forces, etc.).

    Args:
        dim: Dimension
        default: Default value for initialization
        desc: Description
    """
    if default is None:
        default = 0.0 if dim == 1 else [0.0] * dim
    elif isinstance(default, (int, float)) and dim > 1:
        default = [float(default)] * dim
    return field(
        default=None, metadata={"dim": dim, "default": default, "desc": desc, "type": "dependent"}
    )


def quadrature_var(dim: int = 1, default: Union[float, list, None] = None, desc: str = ""):
    """Create quadrature state: dq/dt = integrand(x, u, p) (for cost functions).

    Used for tracking accumulated quantities (cost, energy, etc.) that don't
    feed back into dynamics.

    Args:
        dim: Dimension
        default: Initial value q(0)
        desc: Description
    """
    if default is None:
        default = 0.0 if dim == 1 else [0.0] * dim
    elif isinstance(default, (int, float)) and dim > 1:
        default = [float(default)] * dim
    return field(
        default=None, metadata={"dim": dim, "default": default, "desc": desc, "type": "quadrature"}
    )


def discrete_state(dim: int = 1, default: Union[float, list, None] = None, desc: str = ""):
    """Create discrete state z(tₑ): updated only at events, constant between.

    For variables that jump at events (bounce counter, mode switches, etc.).

    Args:
        dim: Dimension
        default: Initial value z(t₀)
        desc: Description
    """
    if default is None:
        default = 0.0 if dim == 1 else [0.0] * dim
    elif isinstance(default, (int, float)) and dim > 1:
        default = [float(default)] * dim
    return field(
        default=None,
        metadata={"dim": dim, "default": default, "desc": desc, "type": "discrete_state"},
    )


def discrete_var(default: int = 0, desc: str = ""):
    """Create discrete variable m(tₑ): integer/boolean updated at events.

    For discrete-valued quantities (flags, modes, counters).

    Args:
        default: Initial integer value
        desc: Description
    """
    return field(
        default=None,
        metadata={"dim": 1, "default": float(default), "desc": desc, "type": "discrete_var"},
    )


def event_indicator(dim: int = 1, desc: str = ""):
    """Create event indicator c: event occurs when c crosses zero.

    Zero-crossing detection for hybrid systems.

    Args:
        dim: Number of indicators
        desc: Description
    """
    return field(
        default=None, metadata={"dim": dim, "default": 0.0, "desc": desc, "type": "event_indicator"}
    )


def param(default: float, desc: str = ""):
    """Create a parameter field (time-independent constant).

    Args:
        default: Default numeric value
        desc: Description string

    Example:
        m: ca.SX = param(1.5, "mass (kg)")
        g: ca.SX = param(9.81, "gravity (m/s^2)")
    """
    return field(
        default=None,
        metadata={"dim": 1, "default": float(default), "desc": desc, "type": "parameter"},
    )


def input_var(default: float = 0.0, desc: str = ""):
    """Create an input variable field (control signal).

    Args:
        default: Default numeric value
        desc: Description string

    Example:
        thrust: ca.SX = input_var(0.0, "thrust command (N)")
        steering: ca.SX = input_var(desc="steering angle (rad)")
    """
    return field(
        default=None, metadata={"dim": 1, "default": float(default), "desc": desc, "type": "input"}
    )


def output_var(dim: int = 1, default: Union[float, list, None] = None, desc: str = ""):
    """Create an output variable field (observable).

    Args:
        dim: Dimension
        default: Default value
        desc: Description

    Example:
        speed: ca.SX = output_var(1, 0.0, "ground speed (m/s)")
        forces: ca.SX = output_var(3, desc="force vector (N)")
    """
    if default is None:
        default = 0.0 if dim == 1 else [0.0] * dim
    elif isinstance(default, (int, float)) and dim > 1:
        default = [float(default)] * dim
    return field(
        default=None, metadata={"dim": dim, "default": default, "desc": desc, "type": "output"}
    )


# ============================================================================
# Symbolic Dataclass Decorator
# ============================================================================


def symbolic(cls):
    """Combined decorator: applies @dataclass and adds CasADi symbolic methods.

    Adds methods:
        - cls.symbolic(sym_type=ca.SX) -> instance with symbolic variables
        - cls.numeric() -> instance with numeric defaults
        - instance.as_vec() -> ca.SX/MX column vector
        - cls.from_vec(vec) -> instance from vector
        - instance.size1() -> number of rows
        - instance.size2() -> number of columns (always 1)

    Usage:
        @symbolic
        class States:
            x: ca.SX = state(1, 0.0, "position")
            v: ca.SX = state(1, 0.0, "velocity")

        # Create symbolic instance
        x_sym = States.symbolic()  # x_sym.x is ca.SX.sym('x')

        # Create numeric instance
        x0 = States.numeric()  # x0.x == 0.0

        # Convert to vector
        vec = x_sym.as_vec()  # ca.vertcat(x_sym.x, x_sym.v)

        # Access instances directly
        x = model.x  # Access as attribute
    """
    # Apply @dataclass if not already applied
    if not hasattr(cls, "__dataclass_fields__"):
        cls = dataclass(cls)

    # Extract field metadata
    field_info = {}
    for f in fields(cls):
        meta = f.metadata or {}
        dim = meta.get("dim", 1)
        default = meta.get("default", 0.0 if dim == 1 else np.zeros(dim))
        desc = meta.get("desc", "")
        var_type = meta.get("type", "unknown")
        field_info[f.name] = {
            "dim": dim,
            "default": default,
            "desc": desc,
            "type": var_type,
            "casadi_type": f.type,
        }

    # Create symbolic instance factory
    @classmethod
    def create_symbolic(cls_obj, sym_type=ca.SX):
        """Create instance with symbolic CasADi variables."""
        kwargs = {}
        for name, info in field_info.items():
            dim = info["dim"]
            if dim == 1:
                kwargs[name] = sym_type.sym(name)
            else:
                kwargs[name] = sym_type.sym(name, dim)
        return cls_obj(**kwargs)

    # Create numeric instance factory
    @classmethod
    def create_numeric(cls_obj):
        """Create instance with numeric default values."""
        kwargs = {}
        for name, info in field_info.items():
            default = info["default"]
            if isinstance(default, (list, tuple)):
                kwargs[name] = np.array(default, dtype=float)
            else:
                kwargs[name] = float(default)
        return cls_obj(**kwargs)

    # Vector conversion
    def as_vec(self):
        """Convert to CasADi column vector."""
        parts = []
        for f in fields(self.__class__):
            val = getattr(self, f.name)
            parts.append(ca.vec(val))
        return ca.vertcat(*parts) if parts else ca.DM.zeros(0, 1)

    # Reconstruct from vector
    @classmethod
    def from_vec(cls_obj, vec):
        """Reconstruct from CasADi vector (works with both numeric and symbolic).

        For numeric vectors (DM), converts to float/numpy arrays.
        For symbolic vectors (SX/MX), preserves symbolic expressions.
        """
        # Handle CasADi Function dict outputs
        if isinstance(vec, dict):
            if len(vec) == 1:
                vec = list(vec.values())[0]
            else:
                raise ValueError(
                    f"from_vec() received dict with multiple outputs: {list(vec.keys())}"
                )

        # Check if this is a symbolic type (SX or MX)
        is_symbolic = hasattr(vec, "__class__") and vec.__class__.__name__ in ("SX", "MX")

        # Convert to proper vector (only for numeric)
        if not is_symbolic and not hasattr(vec, "shape"):
            vec = ca.DM(vec)

        # Ensure column vector
        if hasattr(vec, "shape") and len(vec.shape) == 2 and vec.shape[1] != 1:
            vec = vec.T

        kwargs = {}
        offset = 0
        for f in fields(cls_obj):
            name = f.name
            dim = field_info[name]["dim"]

            if dim == 1:
                val = vec[offset]
                # Convert DM scalar to float (numeric only)
                if not is_symbolic and hasattr(val, "numel") and val.numel() == 1:
                    val = float(val)
                kwargs[name] = val
                offset += 1
            else:
                val = vec[offset : offset + dim]
                # Convert DM vector to numpy array (numeric only)
                if not is_symbolic and hasattr(val, "__class__") and val.__class__.__name__ == "DM":
                    val = np.array(val, dtype=float).flatten()
                kwargs[name] = val
                offset += dim

        return cls_obj(**kwargs)

    # Matrix reconstruction (for trajectories)
    @classmethod
    def from_matrix(cls_obj, matrix):
        """Create instances from matrix where columns are timesteps.

        Args:
            matrix: CasADi matrix or NumPy array with shape (n_vars, n_steps)

        Returns:
            List of instances, one per timestep
        """
        # Convert to NumPy
        if hasattr(matrix, "__class__") and matrix.__class__.__name__ in ("SX", "MX", "DM"):
            matrix = np.array(ca.DM(matrix))
        else:
            matrix = np.asarray(matrix)

        n_steps = matrix.shape[1]
        instances = []

        for i in range(n_steps):
            col = matrix[:, i : i + 1]
            instances.append(cls_obj.from_vec(col))

        return instances

    # CasADi compatibility methods
    def size1(self):
        """Get number of rows when converted to vector."""
        return self.as_vec().size1()

    def size2(self):
        """Get number of columns (always 1)."""
        return 1

    # Custom repr with descriptions
    def custom_repr(self):
        """String representation with field descriptions."""
        parts = []
        for f in fields(self.__class__):
            val = getattr(self, f.name)
            desc = field_info[f.name].get("desc", "")
            if desc:
                parts.append(f"{f.name}={val!r}  # {desc}")
            else:
                parts.append(f"{f.name}={val!r}")
        return f"{cls.__name__}(" + ", ".join(parts) + ")"

    # Attach methods to class
    cls.symbolic = create_symbolic
    cls.numeric = create_numeric
    cls.as_vec = as_vec
    cls.from_vec = from_vec
    cls.from_matrix = from_matrix
    cls.size1 = size1
    cls.size2 = size2
    cls.__repr__ = custom_repr
    cls._field_info = field_info

    return cls


# ============================================================================
# Type-Safe Model with Full Hybrid DAE Support
# ============================================================================

TState = TypeVar("TState")
TInput = TypeVar("TInput")
TParam = TypeVar("TParam")
TOutput = TypeVar("TOutput")
TAlgebraic = TypeVar("TAlgebraic")
TDependent = TypeVar("TDependent")
TQuadrature = TypeVar("TQuadrature")
TDiscreteState = TypeVar("TDiscreteState")
TDiscreteVar = TypeVar("TDiscreteVar")
TEventIndicator = TypeVar("TEventIndicator")


@beartype
class ModelSX(Generic[TState, TInput, TParam]):
    """Type-safe SX model with full hybrid DAE support.

    Supports:
    - Continuous states (x): dx/dt = f_x(...)
    - Algebraic variables (z_alg): 0 = g(x, z_alg, u, p)
    - Outputs (y): y = f_y(x, u, p)
    - Dependent variables (dep): dep = f_dep(x, u, p)
    - Quadratures (q): dq/dt = f_q(x, u, p)
    - Discrete states (z): z⁺ = f_z(...) at events
    - Discrete variables (m): m⁺ = f_m(...) at events
    - Event indicators (c): event when c = 0

    Example:
        model = ModelSX.create(States, Inputs, Params)

        x = model.x()
        u = model.u()
        p = model.p()

        f_x = ca.vertcat(x.v, u.thrust / p.m - p.g)
        model.build(f_x=f_x, integrator='rk4')
    """

    # Required types
    x: TState
    u: TInput
    p: TParam
    x0: TState
    u0: TInput
    p0: TParam

    # Optional types (set if provided)
    z_alg: Any = None  # Algebraic variables
    y: Any = None  # Dependent variables
    q: Any = None  # Quadrature states
    z: Any = None  # Discrete states
    m: Any = None  # Discrete variables
    c: Any = None  # Event indicators
    out: Any = None  # Outputs

    def __init__(
        self,
        state_type: type[TState],
        input_type: type[TInput],
        param_type: type[TParam],
        output_type: type[TOutput] = None,
        algebraic_type: type[TAlgebraic] = None,
        dependent_type: type[TDependent] = None,
        quadrature_type: type[TQuadrature] = None,
        discrete_state_type: type[TDiscreteState] = None,
        discrete_var_type: type[TDiscreteVar] = None,
        event_indicator_type: type[TEventIndicator] = None,
    ):
        """Initialize typed hybrid DAE model.

        Prefer using ModelSX.create() for automatic type inference.
        """
        self._sym = ca.SX
        self.state_type = state_type
        self.input_type = input_type
        self.param_type = param_type
        self.output_type = output_type
        self.algebraic_type = algebraic_type
        self.dependent_type = dependent_type
        self.quadrature_type = quadrature_type
        self.discrete_state_type = discrete_state_type
        self.discrete_var_type = discrete_var_type
        self.event_indicator_type = event_indicator_type

        # Create required symbolic instances
        self.x = state_type.symbolic(ca.SX)
        self.u = input_type.symbolic(ca.SX)
        self.p = param_type.symbolic(ca.SX)

        # Create required numeric defaults
        self.x0 = state_type.numeric()
        self.u0 = input_type.numeric()
        self.p0 = param_type.numeric()

        # Create optional types if provided
        if output_type:
            self.y = output_type.symbolic(ca.SX)
            self.y0 = output_type.numeric()

        if algebraic_type:
            self.z_alg = algebraic_type.symbolic(ca.SX)
            self.z_alg0 = algebraic_type.numeric()

        if dependent_type:
            self.dep = dependent_type.symbolic(ca.SX)
            self.dep0 = dependent_type.numeric()

        if quadrature_type:
            self.q = quadrature_type.symbolic(ca.SX)
            self.q0 = quadrature_type.numeric()

        if discrete_state_type:
            self.z = discrete_state_type.symbolic(ca.SX)
            self.z0 = discrete_state_type.numeric()

        if discrete_var_type:
            self.m = discrete_var_type.symbolic(ca.SX)
            self.m0 = discrete_var_type.numeric()

        if event_indicator_type:
            self.c = event_indicator_type.symbolic(ca.SX)

    @classmethod
    def create(
        cls, state_type: type[TState], input_type: type[TInput], param_type: type[TParam], **kwargs
    ):
        """Create a fully-typed model instance with automatic type inference.

        This factory method ensures proper type inference for IDE autocomplete.

        Args:
            state_type: Dataclass decorated with @symbolic
            input_type: Dataclass decorated with @symbolic
            param_type: Dataclass decorated with @symbolic
            **kwargs: Optional types (output_type, algebraic_type, etc.)

        Returns:
            Fully-typed ModelSX instance

        Usage:
            model = ModelSX.create(States, Inputs, Params)
            # IDE now knows exact types for autocomplete!

            x = model.x()  # x has full autocomplete
            u = model.u()  # u has full autocomplete
            p = model.p()  # p has full autocomplete
        """
        return cls(state_type, input_type, param_type, **kwargs)

    @beartype
    def build(
        self,
        f_x: ca.SX | ca.MX,
        f_y: ca.SX | ca.MX | None = None,
        f_dep: ca.SX | ca.MX | None = None,
        f_alg: ca.SX | ca.MX | None = None,
        f_q: ca.SX | ca.MX | None = None,
        f_z: ca.SX | ca.MX | None = None,
        f_m: ca.SX | ca.MX | None = None,
        f_c: ca.SX | ca.MX | None = None,
        integrator: str = "rk4",
        integrator_options: dict = None,
    ):
        """Build model with all evolution functions.

        Args:
            f_x: Continuous dynamics dx/dt (required)
            f_y: Output expressions y = f_y(x, u, p)
            f_dep: Dependent variable expressions (less commonly used)
            f_alg: Algebraic constraints (for DAE)
            f_q: Quadrature dynamics dq/dt
            f_z: Discrete state update z⁺
            f_m: Discrete variable update m⁺
            f_c: Event indicator functions
            integrator: Integration method ("rk4", "euler", "idas")
            integrator_options: Integrator settings (e.g., {'N': 10})
        """
        if integrator_options is None:
            integrator_options = {}

        # Build f_x (required)
        self._build_f_x(f_x)

        # Build optional functions
        if f_y is not None:
            self._build_f_y(f_y)
        if f_dep is not None:
            self._build_f_dep(f_dep)
        if f_alg is not None:
            self._build_f_alg(f_alg)
        if f_q is not None:
            self._build_f_q(f_q)
        if f_z is not None:
            self._build_f_z(f_z)
        if f_m is not None:
            self._build_f_m(f_m)
        if f_c is not None:
            self._build_f_c(f_c)

        # Build integrator
        if integrator == "rk4":
            self._build_rk4_integrator(integrator_options)
        elif integrator == "euler":
            self._build_euler_integrator()
        elif integrator == "idas":
            self._build_idas_integrator(integrator_options)
        else:
            raise ValueError(f"Unknown integrator: {integrator}")

        # Build index maps
        self._build_index_maps()

    def _build_f_x(self, f_x_expr):
        """Build continuous dynamics function.

        Note: The 'dep' input here refers to dependent variables (if present),
        not outputs. Dependent variables are computed values that feed into
        dynamics but are not integrated states.
        """
        inputs = [self.x.as_vec()]
        input_names = ["x"]

        if hasattr(self, "dep") and self.dep is not None:
            inputs.append(self.dep.as_vec())
            input_names.append("dep")
        if self.z is not None:
            inputs.append(self.z.as_vec())
            input_names.append("z")
        if self.m is not None:
            inputs.append(self.m.as_vec())
            input_names.append("m")

        inputs.append(self.u.as_vec())
        inputs.append(self.p.as_vec())
        input_names.extend(["u", "p"])

        self.f_x = ca.Function("f_x", inputs, [f_x_expr], input_names, ["dx_dt"])

    def _build_f_y(self, f_y_expr):
        """Build output function.

        Outputs (y) are observables/diagnostics computed from states.
        """
        inputs, names = self._get_standard_inputs()
        self.f_y = ca.Function("f_y", inputs, [f_y_expr], names, ["y"])

    def _build_f_dep(self, f_dep_expr):
        """Build dependent variable function."""
        inputs, names = self._get_standard_inputs(include_dep=False)
        self.f_dep = ca.Function("f_dep", inputs, [f_dep_expr], names, ["dep"])

    def _build_f_alg(self, f_alg_expr):
        """Build algebraic constraint function: 0 = g(x, z_alg, u, p)."""
        inputs = [self.x.as_vec(), self.z_alg.as_vec(), self.u.as_vec(), self.p.as_vec()]
        names = ["x", "z_alg", "u", "p"]
        self.f_alg = ca.Function("f_alg", inputs, [f_alg_expr], names, ["residual"])

    def _build_f_q(self, f_q_expr):
        """Build quadrature dynamics."""
        inputs, names = self._get_standard_inputs()
        self.f_q = ca.Function("f_q", inputs, [f_q_expr], names, ["dq_dt"])

    def _build_f_z(self, f_z_expr):
        """Build discrete state update function."""
        inputs, names = self._get_standard_inputs()
        self.f_z = ca.Function("f_z", inputs, [f_z_expr], names, ["z_plus"])

    def _build_f_m(self, f_m_expr):
        """Build discrete variable update function."""
        inputs, names = self._get_standard_inputs()
        self.f_m = ca.Function("f_m", inputs, [f_m_expr], names, ["m_plus"])

    def _build_f_c(self, f_c_expr):
        """Build event indicator function."""
        inputs, names = self._get_standard_inputs()
        self.f_c = ca.Function("f_c", inputs, [f_c_expr], names, ["indicators"])

    def _get_standard_inputs(self, include_dep=True):
        """Get standard input list for functions."""
        inputs = [self.x.as_vec()]
        names = ["x"]

        if include_dep and hasattr(self, "dep") and self.dep is not None:
            inputs.append(self.dep.as_vec())
            names.append("dep")
        if self.z is not None:
            inputs.append(self.z.as_vec())
            names.append("z")
        if self.m is not None:
            inputs.append(self.m.as_vec())
            names.append("m")

        inputs.append(self.u.as_vec())
        inputs.append(self.p.as_vec())
        names.extend(["u", "p"])

        return inputs, names

    def _build_rk4_integrator(self, options: dict):
        """Build RK4 integrator."""
        from . import integrators

        dt_sym = ca.SX.sym("dt")
        N = options.get("N", 10)

        rk4_step = integrators.rk4(self.f_x, dt_sym, name="rk4", N=N)

        # Extract inputs to handle SX vs MX
        if rk4_step.is_a("SXFunction"):
            xin = rk4_step.sx_in(0)
            uin = rk4_step.sx_in(1)
            pin = rk4_step.sx_in(2)
        else:
            xin = rk4_step.mx_in(0)
            uin = rk4_step.mx_in(1)
            pin = rk4_step.mx_in(2)

        self.f_step = ca.Function(
            "f_step",
            [xin, uin, pin, dt_sym],
            [rk4_step(xin, uin, pin, dt_sym)],
            ["x", "u", "p", "dt"],
            ["x_next"],
        )

    def _build_euler_integrator(self):
        """Build Euler integrator."""
        dt_sym = ca.SX.sym("dt")

        # For composed models, use the combined state size
        if hasattr(self, "_composed") and self._composed:
            x_sym = ca.SX.sym("x", self._total_composed_states)
        else:
            x_sym = self.x.as_vec()

        u_sym = self.u.as_vec()
        p_sym = self.p.as_vec()

        # Build argument list matching f_x signature
        f_x_args = [x_sym]

        # Add dependent variables if present (not currently used in integrators)
        if hasattr(self, "dep") and self.dep is not None:
            # For now, use zero/default values for dep in integrator
            # In a full implementation, would evaluate f_dep here
            f_x_args.append(self.dep0.as_vec())

        # Add discrete states if present (constant during integration)
        if self.z is not None:
            z_sym = ca.SX.sym("z", self.z.size1())
            f_x_args.append(z_sym)
        else:
            z_sym = None

        # Add discrete variables if present (constant during integration)
        if self.m is not None:
            m_sym = ca.SX.sym("m", self.m.size1())
            f_x_args.append(m_sym)
        else:
            m_sym = None

        f_x_args.extend([u_sym, p_sym])

        # Evaluate dynamics
        dx_dt = self.f_x(*f_x_args)
        x_next = x_sym + dt_sym * dx_dt

        # Build f_step with appropriate inputs
        step_inputs = [x_sym]
        step_names = ["x"]

        if z_sym is not None:
            step_inputs.append(z_sym)
            step_names.append("z")
        if m_sym is not None:
            step_inputs.append(m_sym)
            step_names.append("m")

        step_inputs.extend([u_sym, p_sym, dt_sym])
        step_names.extend(["u", "p", "dt"])

        self.f_step = ca.Function("f_step", step_inputs, [x_next], step_names, ["x_next"])

    def _build_idas_integrator(self, options: dict):
        """Build IDAS DAE integrator."""
        # TODO: Implement IDAS for DAE systems with algebraic constraints
        print("Warning: IDAS not yet implemented, falling back to RK4")
        self._build_rk4_integrator(options)

    def _build_index_maps(self):
        """Build index maps for state/input/parameter/output access (internal use only)."""
        # State indices (private - internal use only)
        self._state_index = {}
        offset = 0
        for fname, finfo in self.state_type._field_info.items():
            dim = finfo["dim"]
            if dim == 1:
                self._state_index[fname] = offset
                offset += 1
            else:
                for i in range(dim):
                    self._state_index[f"{fname}[{i}]"] = offset + i
                offset += dim

        # Input indices (private - internal use only)
        self._input_index = {}
        offset = 0
        for fname, finfo in self.input_type._field_info.items():
            dim = finfo["dim"]
            if dim == 1:
                self._input_index[fname] = offset
                offset += 1
            else:
                for i in range(dim):
                    self._input_index[f"{fname}[{i}]"] = offset + i
                offset += dim

        # Parameter indices (private - internal use only)
        self._parameter_index = {}
        offset = 0
        for fname, finfo in self.param_type._field_info.items():
            dim = finfo["dim"]
            if dim == 1:
                self._parameter_index[fname] = offset
                offset += 1
            else:
                for i in range(dim):
                    self._parameter_index[f"{fname}[{i}]"] = offset + i
                offset += dim

        # Output indices (if outputs exist) (private - internal use only)
        if self.output_type is not None:
            self._output_index = {}
            offset = 0
            for fname, finfo in self.output_type._field_info.items():
                dim = finfo["dim"]
                self._output_index[fname] = offset
                if dim > 1:
                    for i in range(dim):
                        self._output_index[f"{fname}[{i}]"] = offset + i
                offset += dim

        # Create ordered name lists
        self.state_names = [n for n, _ in sorted(self._state_index.items(), key=lambda kv: kv[1])]
        self.input_names = (
            [n for n, _ in sorted(self._input_index.items(), key=lambda kv: kv[1])]
            if self._input_index
            else []
        )
        self.parameter_names = (
            [n for n, _ in sorted(self._parameter_index.items(), key=lambda kv: kv[1])]
            if self._parameter_index
            else []
        )
        self.output_names = (
            [n for n, _ in sorted(self._output_index.items(), key=lambda kv: kv[1])]
            if hasattr(self, "_output_index")
            else []
        )

    def simulate(
        self,
        t0: float,
        tf: float,
        dt: float,
        u_func: Callable = None,
        p_vec=None,
        x0_vec=None,
        detect_events: bool = False,
    ):
        """Simulate model from t0 to tf with event detection.

        Args:
            t0: Initial time
            tf: Final time
            dt: Timestep
            u_func: Optional control function (t, x, p) -> u_vec
            p_vec: Optional parameter vector (uses p0 if None)
            x0_vec: Optional initial state vector (uses x0 if None)
            detect_events: Whether to detect and handle zero-crossings

        Returns:
            Dictionary with 't', 'x', and optionally 'z', 'm', 'q', 'out' arrays
        """
        if not hasattr(self, "f_step"):
            raise ValueError("Model not built. Call build() first.")

        # Handle composed models
        if hasattr(self, "_composed") and self._composed:
            p_vec = self.p0.as_vec() if p_vec is None else p_vec
            x_curr = self.x0_composed if x0_vec is None else x0_vec
        else:
            p_vec = self.p0.as_vec() if p_vec is None else p_vec
            x_curr = self.x0.as_vec() if x0_vec is None else x0_vec

        z_curr = self.z0.as_vec() if self.z is not None else None
        m_curr = self.m0.as_vec() if self.m is not None else None
        q_curr = self.q0.as_vec() if self.q is not None else None

        t_hist = [t0]
        x_hist = [x_curr]
        if self.z is not None:
            z_hist = [z_curr]
        if self.m is not None:
            m_hist = [m_curr]
        if self.q is not None:
            q_hist = [q_curr]
        out_hist = [] if hasattr(self, "f_y") else None

        # Track previous event indicator for zero-crossing detection
        c_prev = None
        if detect_events and hasattr(self, "f_c"):
            args = self._build_eval_args(x_curr, z_curr, m_curr, self.u0.as_vec(), p_vec)
            c_prev = float(self.f_c(*args))

        t = t0
        while t < tf - dt / 2:
            # Get control
            if u_func is not None:
                u_curr = u_func(t, x_curr, p_vec)
            else:
                u_curr = self.u0.as_vec()

            # Evaluate outputs before step
            if out_hist is not None:
                args = self._build_eval_args(x_curr, z_curr, m_curr, u_curr, p_vec)
                out_val = self.f_y(*args)
                out_hist.append(out_val)

            # Integration step - build arguments for f_step
            step_args = [x_curr]
            if self.z is not None:
                step_args.append(z_curr)
            if self.m is not None:
                step_args.append(m_curr)
            step_args.extend([u_curr, p_vec, dt])

            x_next = self.f_step(*step_args)

            # Check for events
            if detect_events and hasattr(self, "f_c"):
                args = self._build_eval_args(x_next, z_curr, m_curr, u_curr, p_vec)
                c_curr = float(self.f_c(*args))

                # Detect zero-crossing
                if c_prev is not None and c_prev > 0 and c_curr <= 0:
                    # Event occurred!
                    if hasattr(self, "f_z") and z_curr is not None:
                        z_curr = self.f_z(*args)
                    if hasattr(self, "f_m"):
                        # f_m can reset either discrete variables or continuous states
                        # Check output dimension to determine which
                        m_reset = self.f_m(*args)
                        if m_curr is not None and m_reset.size1() == m_curr.size1():
                            # Reset discrete variables
                            m_curr = m_reset
                        elif m_reset.size1() == x_next.size1():
                            # Reset continuous states
                            x_next = m_reset
                        else:
                            # Assume it's for continuous states if sizes don't match m
                            x_next = m_reset

                c_prev = c_curr

            # Integrate quadratures if present
            if self.q is not None and hasattr(self, "f_q"):
                args = self._build_eval_args(x_curr, z_curr, m_curr, u_curr, p_vec)
                dq_dt = self.f_q(*args)
                q_curr = q_curr + dt * dq_dt

            # Store
            t += dt
            t_hist.append(t)
            x_hist.append(x_next)
            if self.z is not None:
                z_hist.append(z_curr)
            if self.m is not None:
                m_hist.append(m_curr)
            if self.q is not None:
                q_hist.append(q_curr)

            x_curr = x_next

        # Final output
        if out_hist is not None:
            args = self._build_eval_args(x_curr, z_curr, m_curr, u_curr, p_vec)
            out_val = self.f_y(*args)
            out_hist.append(out_val)

        # Build result dictionary
        result = {"t": np.array(t_hist), "x": ca.hcat(x_hist).full()}
        if self.z is not None:
            result["z"] = ca.hcat(z_hist).full()
        if self.m is not None:
            result["m"] = ca.hcat(m_hist).full()
        if self.q is not None:
            result["q"] = ca.hcat(q_hist).full()
        if out_hist is not None:
            result["out"] = ca.hcat(out_hist).full()

        return result

    def _build_eval_args(self, x, z, m, u, p):
        """Build argument list for function evaluation.

        This matches the signature of functions built with _get_standard_inputs(),
        which may include dependent variables (dep) if they exist.
        """
        args = [x]
        if hasattr(self, "dep") and self.dep is not None:
            # Evaluate dependent variables and add to argument list
            # f_dep has signature (x, u, p) -> dep
            if hasattr(self, "f_dep"):
                dep_val = self.f_dep(x, u, p)
                args.append(dep_val)
            else:
                # If dep exists but f_dep not built, this is an error
                raise ValueError("Model has dep but f_dep was not built")
        if z is not None:
            args.append(z)
        if m is not None:
            args.append(m)
        args.append(u)
        args.append(p)
        return args  # ============================================================================

    # Hierarchical Model Composition
    # ============================================================================

    def add_submodel(
        self,
        name: str,
        submodel: "ModelSX",
        state_connections: dict[str, str] = None,
        input_connections: dict[str, str] = None,
        output_connections: dict[str, str] = None,
    ):
        """Add a submodel for hierarchical composition.

        This allows building complex systems from simpler components. Submodels
        are stored and their states/inputs/outputs can be connected to the parent
        model or other submodels.

        Args:
            name: Unique identifier for this submodel
            submodel: ModelSX instance to add as a component
            state_connections: Map submodel states to parent states
                Example: {"controller.i_pitch": "x.pitch_integral"}
            input_connections: Map submodel inputs to parent inputs or submodel outputs
                Example: {"controller.pitch_ref": "u.pitch_cmd",
                         "controller.pitch_meas": "aircraft.pitch"}
            output_connections: Map submodel outputs to parent outputs
                Example: {"controller.elevator": "y.elevator_cmd"}

        Example:
            >>> parent = ModelSX.create(ParentStates, ParentInputs, ParentParams)
            >>> aircraft = sportcub()
            >>> controller = pid_controller()
            >>> parent.add_submodel("aircraft", aircraft,
            ...     input_connections={"aircraft.thr": "controller.thr"})
            >>> parent.add_submodel("controller", controller,
            ...     input_connections={"controller.pitch_meas": "aircraft.theta"})
        """
        if not hasattr(self, "_submodels"):
            self._submodels = {}
            self._state_connections = {}
            self._input_connections = {}
            self._output_connections = {}

        if name in self._submodels:
            raise ValueError(f"Submodel '{name}' already exists")

        self._submodels[name] = submodel
        if state_connections:
            self._state_connections[name] = state_connections
        if input_connections:
            self._input_connections[name] = input_connections
        if output_connections:
            self._output_connections[name] = output_connections

    def build_composed(self, integrator: str = "rk4", integrator_options: dict = None):
        """Build a composed model from added submodels.

        This creates a unified dynamics function that integrates all submodels,
        routing signals according to the connection maps specified in add_submodel().

        The composed model will have:
        - States: concatenation of all submodel states
        - Inputs: parent inputs (submodel inputs are internal)
        - Outputs: parent outputs mapped from submodel outputs
        - Dynamics: combined f_x from all submodels with signal routing

        Example:
            >>> parent = ModelSX.create(ParentStates, ParentInputs, ParentParams)
            >>> parent.add_submodel("aircraft", aircraft, ...)
            >>> parent.add_submodel("controller", controller, ...)
            >>> parent.build_composed(integrator="rk4")
            >>> # Now parent.f_step() integrates both aircraft and controller
        """
        if not hasattr(self, "_submodels") or not self._submodels:
            raise ValueError("No submodels added. Use add_submodel() first.")

        if integrator_options is None:
            integrator_options = {}

        # Build index maps for parent model (needed for connection resolution)
        self._build_index_maps()

        # Build combined state vector from all submodels
        submodel_state_slices = {}
        offset = 0
        for name, submodel in self._submodels.items():
            n_states = submodel.x.size1()
            submodel_state_slices[name] = (offset, offset + n_states)
            offset += n_states

        total_states = offset

        # Create combined dynamics function
        x_combined = ca.SX.sym("x_combined", total_states)
        u_parent = self.u.as_vec()
        p_parent = self.p.as_vec()

        # Extract submodel states from combined vector
        submodel_states = {}
        for name, (start, end) in submodel_state_slices.items():
            submodel_states[name] = x_combined[start:end]

        # First pass: Evaluate all submodel outputs to establish signal sources
        submodel_outputs = {}
        for name, submodel in self._submodels.items():
            if hasattr(submodel, "f_y"):
                x_sub = submodel_states[name]
                u_sub = submodel.u0.as_vec()  # Placeholder, will be resolved
                p_sub = submodel.p0.as_vec()

                # Evaluate outputs
                y_sub = submodel.f_y(x_sub, u_sub, p_sub)
                submodel_outputs[name] = y_sub

        # Second pass: Build input vectors for each submodel by resolving connections
        submodel_inputs = {}
        for name, submodel in self._submodels.items():
            u_sub_list = []
            input_conns = self._input_connections.get(name, {})

            for idx, input_name in enumerate(submodel.input_names):
                full_name = f"{name}.{input_name}"

                if full_name in input_conns:
                    # Input is connected to something
                    source = input_conns[full_name]

                    if source.startswith("u."):
                        # Connected to parent input
                        parent_input = source[2:]
                        if parent_input in self._input_index:
                            u_sub_list.append(u_parent[self._input_index[parent_input]])
                        else:
                            # Use default if parent input doesn't exist
                            u_sub_list.append(submodel.u0.as_vec()[idx])

                    elif "." in source:
                        # Connected to another submodel's output or state
                        parts = source.split(".")
                        source_model = parts[0]
                        source_signal = ".".join(parts[1:])

                        # Check if it's an output
                        if source_model in submodel_outputs:
                            source_submodel = self._submodels[source_model]
                            if (
                                hasattr(source_submodel, "_output_index")
                                and source_signal in source_submodel._output_index
                            ):
                                output_idx = source_submodel._output_index[source_signal]
                                u_sub_list.append(submodel_outputs[source_model][output_idx])
                            else:
                                # Try as state
                                if source_signal in source_submodel._state_index:
                                    state_idx = source_submodel._state_index[source_signal]
                                    u_sub_list.append(submodel_states[source_model][state_idx])
                                else:
                                    # Default value
                                    u_sub_list.append(submodel.u0.as_vec()[idx])
                        else:
                            # Use default
                            u_sub_list.append(submodel.u0.as_vec()[idx])
                    else:
                        # Use default value
                        u_sub_list.append(submodel.u0.as_vec()[idx])
                else:
                    # No connection, use default value
                    u_sub_list.append(submodel.u0.as_vec()[idx])

            # Build input vector for this submodel
            submodel_inputs[name] = ca.vertcat(*u_sub_list) if u_sub_list else ca.DM.zeros(0, 1)

        # Third pass: Evaluate each submodel's dynamics with resolved inputs
        f_x_parts = []
        for name, submodel in self._submodels.items():
            x_sub = submodel_states[name]
            u_sub = submodel_inputs[name]
            p_sub = submodel.p0.as_vec()

            # Evaluate submodel dynamics
            dx_sub = submodel.f_x(x_sub, u_sub, p_sub)
            f_x_parts.append(dx_sub)

        # Combine all submodel derivatives
        f_x_combined = ca.vertcat(*f_x_parts)

        # Create composed dynamics function
        self.f_x = ca.Function(
            "f_x_composed",
            [x_combined, u_parent, p_parent],
            [f_x_combined],
            ["x", "u", "p"],
            ["dx_dt"],
        )

        # Build composed output function if parent has output type
        if self.output_type is not None:
            # Map parent outputs from submodel outputs
            y_parts = []
            output_conns = {}
            for name in self._submodels.keys():
                if name in self._output_connections:
                    output_conns.update(self._output_connections[name])

            # For now, create a simple output that concatenates submodel outputs
            # TODO: Implement proper output mapping based on output_connections
            if hasattr(self, "y"):
                self.f_y = ca.Function(
                    "f_y_composed",
                    [x_combined, u_parent, p_parent],
                    [self.y.as_vec()],  # Placeholder
                    ["x", "u", "p"],
                    ["y"],
                )

        # Store composition metadata BEFORE building integrator
        # (integrator builders check _composed and _total_composed_states)
        self._composed = True
        self._submodel_state_slices = submodel_state_slices
        self._total_composed_states = total_states

        # Build integrator
        if integrator == "rk4":
            self._build_rk4_integrator(integrator_options)
        elif integrator == "euler":
            self._build_euler_integrator()
        else:
            raise ValueError(f"Unknown integrator: {integrator}")

        # Update x0 to be concatenated initial states
        x0_parts = []
        for name in self._submodels.keys():
            x0_parts.append(self._submodels[name].x0.as_vec())

        # Create a compatible x0 structure
        self.x0_composed = ca.vertcat(*x0_parts) if x0_parts else ca.DM.zeros(0, 1)

    # Utility methods
    def saturate(self, val, lower, upper):
        """Saturate value between bounds."""
        return ca.fmin(ca.fmax(val, lower), upper)

    # Backwards compatibility properties for legacy API
    @property
    def State(self):
        """Legacy API: access to state type class."""
        return self.state_type

    @property
    def Input(self):
        """Legacy API: access to input type class."""
        return self.input_type

    @property
    def Parameters(self):
        """Legacy API: access to parameter type class."""
        return self.param_type

    @property
    def output_sizes(self):
        """Legacy API: dictionary mapping output names to their dimensions."""
        if self.output_type is None:
            return {}
        return {fname: finfo["dim"] for fname, finfo in self.output_type._field_info.items()}


@beartype
class ModelMX(ModelSX[TState, TInput, TParam]):
    """Type-safe MX model for large-scale optimization.

    Same API as ModelSX but uses MX symbolic type for better
    performance with large-scale optimization problems.

    Usage:
        model = ModelMX.create(States, Inputs, Params)

        x = model.x()
        u = model.u()
        p = model.p()

        f_x = ca.vertcat(x.v, u.thrust / p.m - p.g)
        model.build(f_x=f_x, integrator='rk4')
    """

    @classmethod
    def create(
        cls, state_type: type[TState], input_type: type[TInput], param_type: type[TParam], **kwargs
    ):
        """Create a fully-typed MX model instance with automatic type inference."""
        return cls(state_type, input_type, param_type, **kwargs)

    def __init__(
        self, state_type: type[TState], input_type: type[TInput], param_type: type[TParam], **kwargs
    ):
        """Initialize MX-based model."""
        # Set MX before calling parent __init__
        self._sym = ca.MX

        # Store types
        self.state_type = state_type
        self.input_type = input_type
        self.param_type = param_type
        self.output_type = kwargs.get("output_type")
        self.algebraic_type = kwargs.get("algebraic_type")
        self.dependent_type = kwargs.get("dependent_type")
        self.quadrature_type = kwargs.get("quadrature_type")
        self.discrete_state_type = kwargs.get("discrete_state_type")
        self.discrete_var_type = kwargs.get("discrete_var_type")
        self.event_indicator_type = kwargs.get("event_indicator_type")

        # Create symbolic instances with MX
        self.x = state_type.symbolic(ca.MX)
        self.u = input_type.symbolic(ca.MX)
        self.p = param_type.symbolic(ca.MX)

        # Create numeric defaults
        self.x0 = state_type.numeric()
        self.u0 = input_type.numeric()
        self.p0 = param_type.numeric()

        # Create optional types if provided
        if self.output_type:
            self.y = self.output_type.symbolic(ca.MX)
            self.y0 = self.output_type.numeric()

        if self.algebraic_type:
            self.z_alg = self.algebraic_type.symbolic(ca.MX)
            self.z_alg0 = self.algebraic_type.numeric()

        if self.dependent_type:
            self.dep = self.dependent_type.symbolic(ca.MX)
            self.dep0 = self.dependent_type.numeric()

        if self.quadrature_type:
            self.q = self.quadrature_type.symbolic(ca.MX)
            self.q0 = self.quadrature_type.numeric()

        if self.discrete_state_type:
            self.z = self.discrete_state_type.symbolic(ca.MX)
            self.z0 = self.discrete_state_type.numeric()

        if self.discrete_var_type:
            self.m = self.discrete_var_type.symbolic(ca.MX)
            self.m0 = self.discrete_var_type.numeric()

        if self.event_indicator_type:
            self.c = self.event_indicator_type.symbolic(ca.MX)

    def _build_rk4_integrator(self, options: dict):
        """Build RK4 integrator with MX."""
        from . import integrators

        dt_sym = ca.MX.sym("dt")
        N = options.get("N", 10)

        rk4_step = integrators.rk4(self.f_x, dt_sym, name="rk4", N=N)

        xin = rk4_step.mx_in(0)
        uin = rk4_step.mx_in(1)
        pin = rk4_step.mx_in(2)

        self.f_step = ca.Function(
            "f_step",
            [xin, uin, pin, dt_sym],
            [rk4_step(xin, uin, pin, dt_sym)],
            ["x", "u", "p", "dt"],
            ["x_next"],
        )

    def _build_euler_integrator(self):
        """Build Euler integrator with MX."""
        dt_sym = ca.MX.sym("dt")

        # For composed models, use the combined state size
        if hasattr(self, "_composed") and self._composed:
            x_sym = ca.MX.sym("x", self._total_composed_states)
        else:
            x_sym = self.x.as_vec()

        u_sym = self.u.as_vec()
        p_sym = self.p.as_vec()

        dx_dt = self.f_x(x_sym, u_sym, p_sym)
        x_next = x_sym + dt_sym * dx_dt

        self.f_step = ca.Function(
            "f_step", [x_sym, u_sym, p_sym, dt_sym], [x_next], ["x", "u", "p", "dt"], ["x_next"]
        )
