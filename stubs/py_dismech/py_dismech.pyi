from __future__ import annotations
import numpy
import typing
__all__: list[str] = ['BACKWARD_EULER', 'BaseForce', 'ContactForce', 'DampingForce', 'ElasticRod', 'FORWARD_EULER', 'FloorContactForce', 'ForceContainer', 'GOLDSTEIN', 'GravityForce', 'HEADLESS', 'IMPLICIT_MIDPOINT', 'IntegratorMethod', 'LineSearchType', 'MAGNUM', 'MaxIterations', 'NO_LS', 'OPENGL', 'RenderEngine', 'RenderParams', 'SimParams', 'SimulationManager', 'SoftRobots', 'SymbolicEquations', 'VERLET_POSITION', 'WOLFE']
class BaseForce:
    pass
class ContactForce(BaseForce):
    def __init__(self, soft_robots: SoftRobots, col_limit: float, delta: float, k_scaler: float, friction: bool, nu: float, self_contact: bool, symbolic_equations: SymbolicEquations = None) -> None:
        ...
class DampingForce(BaseForce):
    def __init__(self, soft_robots: SoftRobots, viscosity: float) -> None:
        ...
class ElasticRod:
    def freeVertexBoundaryCondition(self, arg0: int) -> None:
        ...
    def getEdgeTheta(self, arg0: int) -> float:
        ...
    def getM1(self, arg0: int) -> numpy.ndarray[numpy.float64[3, 1]]:
        ...
    def getM2(self, arg0: int) -> numpy.ndarray[numpy.float64[3, 1]]:
        ...
    def getThetas(self) -> numpy.ndarray[numpy.float64[m, 1]]:
        ...
    def getVelocities(self) -> numpy.ndarray[numpy.float64[m, 3]]:
        ...
    def getVertexPos(self, arg0: int) -> numpy.ndarray[numpy.float64[3, 1]]:
        ...
    def getVertexVel(self, arg0: int) -> numpy.ndarray[numpy.float64[3, 1]]:
        ...
    def getVertices(self) -> numpy.ndarray[numpy.float64[m, 3]]:
        ...
    def setThetaBoundaryCondition(self, arg0: float, arg1: int) -> None:
        ...
    def setVertexBoundaryCondition(self, arg0: numpy.ndarray[numpy.float64[3, 1]], arg1: int) -> None:
        ...
class FloorContactForce(BaseForce):
    def __init__(self, soft_robots: SoftRobots, floor_delta: float, floor_slipTol: float, floor_z: float, floor_mu: float = 0.0) -> None:
        ...
class ForceContainer:
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, m_forces: list[BaseForce]) -> None:
        ...
    def addForce(self, force: BaseForce) -> None:
        ...
class GravityForce(BaseForce):
    def __init__(self, soft_robots: SoftRobots, g_vector: numpy.ndarray[numpy.float64[3, 1]]) -> None:
        ...
class IntegratorMethod:
    """
    Members:
    
      FORWARD_EULER
    
      VERLET_POSITION
    
      BACKWARD_EULER
    
      IMPLICIT_MIDPOINT
    """
    BACKWARD_EULER: typing.ClassVar[IntegratorMethod]  # value = <IntegratorMethod.BACKWARD_EULER: 2>
    FORWARD_EULER: typing.ClassVar[IntegratorMethod]  # value = <IntegratorMethod.FORWARD_EULER: 0>
    IMPLICIT_MIDPOINT: typing.ClassVar[IntegratorMethod]  # value = <IntegratorMethod.IMPLICIT_MIDPOINT: 3>
    VERLET_POSITION: typing.ClassVar[IntegratorMethod]  # value = <IntegratorMethod.VERLET_POSITION: 1>
    __members__: typing.ClassVar[dict[str, IntegratorMethod]]  # value = {'FORWARD_EULER': <IntegratorMethod.FORWARD_EULER: 0>, 'VERLET_POSITION': <IntegratorMethod.VERLET_POSITION: 1>, 'BACKWARD_EULER': <IntegratorMethod.BACKWARD_EULER: 2>, 'IMPLICIT_MIDPOINT': <IntegratorMethod.IMPLICIT_MIDPOINT: 3>}
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: int) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: int) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class LineSearchType:
    """
    Members:
    
      NO_LS
    
      GOLDSTEIN
    
      WOLFE
    """
    GOLDSTEIN: typing.ClassVar[LineSearchType]  # value = <LineSearchType.GOLDSTEIN: 1>
    NO_LS: typing.ClassVar[LineSearchType]  # value = <LineSearchType.NO_LS: 0>
    WOLFE: typing.ClassVar[LineSearchType]  # value = <LineSearchType.WOLFE: 2>
    __members__: typing.ClassVar[dict[str, LineSearchType]]  # value = {'NO_LS': <LineSearchType.NO_LS: 0>, 'GOLDSTEIN': <LineSearchType.GOLDSTEIN: 1>, 'WOLFE': <LineSearchType.WOLFE: 2>}
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: int) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: int) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class MaxIterations:
    num_iters: int
    terminate_at_max: bool
    def __init__(self) -> None:
        ...
class RenderEngine:
    """
    Members:
    
      HEADLESS
    
      OPENGL
    
      MAGNUM
    """
    HEADLESS: typing.ClassVar[RenderEngine]  # value = <RenderEngine.HEADLESS: 0>
    MAGNUM: typing.ClassVar[RenderEngine]  # value = <RenderEngine.MAGNUM: 2>
    OPENGL: typing.ClassVar[RenderEngine]  # value = <RenderEngine.OPENGL: 1>
    __members__: typing.ClassVar[dict[str, RenderEngine]]  # value = {'HEADLESS': <RenderEngine.HEADLESS: 0>, 'OPENGL': <RenderEngine.OPENGL: 1>, 'MAGNUM': <RenderEngine.MAGNUM: 2>}
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: int) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: int) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class RenderParams:
    cmd_line_per: int
    debug_verbosity: int
    render_per: int
    render_record_path: str
    render_scale: float
    renderer: RenderEngine
    show_mat_frames: bool
    def __init__(self) -> None:
        ...
class SimParams:
    adaptive_time_stepping: int
    dt: float
    dtol: float
    enable_2d_sim: bool
    ftol: float
    integrator: IntegratorMethod
    line_search: LineSearchType
    max_iter: MaxIterations
    sim_time: float
    def __init__(self) -> None:
        ...
class SimulationManager:
    logger: BaseLogger
    def __init__(self) -> None:
        ...
    def initialize(self, arg0: list[str]) -> None:
        ...
    def run_simulation(self) -> None:
        ...
    def simulation_completed(self) -> bool:
        ...
    @typing.overload
    def step_simulation(self) -> None:
        ...
    @typing.overload
    def step_simulation(self, arg0: dict) -> None:
        ...
    @property
    def forces(self) -> ForceContainer:
        ...
    @property
    def render_params(self) -> RenderParams:
        ...
    @property
    def sim_params(self) -> SimParams:
        ...
    @property
    def soft_robots(self) -> SoftRobots:
        ...
class SoftRobots:
    def __init__(self) -> None:
        ...
    @typing.overload
    def addLimb(self, start: numpy.ndarray[numpy.float64[3, 1]], end: numpy.ndarray[numpy.float64[3, 1]], num_nodes: int, rho: float, rod_radius: float, youngs_modulus: float, poisson_ratio: float, mu: float = 0.0, col_group: int = 65535) -> None:
        ...
    @typing.overload
    def addLimb(self, nodes: list[numpy.ndarray[numpy.float64[3, 1]]], rho: float, rod_radius: float, youngs_modulus: float, poisson_ratio: float, mu: float = 0.0, col_group: int = 65535) -> None:
        ...
    def addToJoint(self, joint_idx: int, limb_idx: int, node_idx: int) -> None:
        ...
    def applyInitialVelocities(self, limb_idx: int, velocities: list[numpy.ndarray[numpy.float64[3, 1]]]) -> None:
        ...
    def createJoint(self, limb_idx: int, node_idx: int) -> None:
        ...
    def lockEdge(self, limb_idx: int, edge_idx: int) -> None:
        ...
    def setup(self) -> None:
        ...
    @property
    def joints(self) -> list[ElasticJoint]:
        ...
    @property
    def limbs(self) -> list[ElasticRod]:
        ...
class SymbolicEquations:
    def __init__(self) -> None:
        ...
    def generateContactPotentialPiecewiseFunctions(self) -> None:
        ...
    def generateFloorFrictionJacobianFunctions(self) -> None:
        ...
    def generateFrictionJacobianPiecewiseFunctions(self) -> None:
        ...
BACKWARD_EULER: IntegratorMethod  # value = <IntegratorMethod.BACKWARD_EULER: 2>
FORWARD_EULER: IntegratorMethod  # value = <IntegratorMethod.FORWARD_EULER: 0>
GOLDSTEIN: LineSearchType  # value = <LineSearchType.GOLDSTEIN: 1>
HEADLESS: RenderEngine  # value = <RenderEngine.HEADLESS: 0>
IMPLICIT_MIDPOINT: IntegratorMethod  # value = <IntegratorMethod.IMPLICIT_MIDPOINT: 3>
MAGNUM: RenderEngine  # value = <RenderEngine.MAGNUM: 2>
NO_LS: LineSearchType  # value = <LineSearchType.NO_LS: 0>
OPENGL: RenderEngine  # value = <RenderEngine.OPENGL: 1>
VERLET_POSITION: IntegratorMethod  # value = <IntegratorMethod.VERLET_POSITION: 1>
WOLFE: LineSearchType  # value = <LineSearchType.WOLFE: 2>
