import numpy as np
import argparse
import py_dismech
from pathlib import Path
from functools import partial


def extract_n_nodes(qs: np.ndarray, n: int) -> np.ndarray:
    """
    Extracts N equidistant nodes along the centerline using arc-length interpolation.

    Args:
        qs (np.ndarray): The full array of vertex positions (V x 3).
        n (int): The number of nodes to extract.

    Returns:
        np.ndarray: A flattened array of [node_0, theta_0, node_1, theta_1, ..., node_n].
    """
    if n < 2:
        raise ValueError("Number of extracted nodes (n) must be at least 2.")

    # 1. Calculate cumulative arc length (s) of the discrete curve
    segments = np.diff(qs, axis=0)
    lengths = np.linalg.norm(segments, axis=1)
    s = np.concatenate(([0.0], np.cumsum(lengths)))

    # 2. Define target arc lengths for N evenly spaced points
    s_target = np.linspace(0.0, s[-1], n)

    # 3. Interpolate x, y, and z coordinates independently
    nodes = np.zeros((n, 3))
    for i in range(3):
        nodes[:, i] = np.interp(s_target, s, qs[:, i])

    # 4. Interleave nodes with theta=0.0
    result = []
    for i in range(n - 1):
        result.extend(nodes[i])
        result.append(0.0)
    result.extend(nodes[-1])

    return np.array(result)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run slinky simulation in DER.")
    parser.add_argument("--name", type=str, default="out", help="output file prefix.")
    parser.add_argument("--n", type=int, default=3, help="Number of centerline nodes.")
    parser.add_argument("--radius", type=float, default=5e-3, help="Radius of the rod")
    parser.add_argument("--young_mod", type=float, default=1e7, help="Young's Modulus")
    parser.add_argument("--density", type=float, default=1273.52, help="Density")
    parser.add_argument("--poisson", type=float, default=0.5, help="Poisson ratio")
    parser.add_argument("--dt", type=float, default=5e-3, help="Time step")
    parser.add_argument("--dtol", type=float, default=1e-3, help="Tolerance")
    parser.add_argument("--render", action="store_true", help="Run with OpenGL")
    parser.add_argument("--steps", type=int, default=10, help="Number of saved steps")
    parser.add_argument(
        "--final_disp",
        type=float,
        nargs=3,
        default=[0.0, 0.0, 0.0],
        help="Final displacement (x y z)",
    )
    args = parser.parse_args()

    # Aliases
    sim_manager = py_dismech.SimulationManager()
    soft_robots = sim_manager.soft_robots
    sim_params = sim_manager.sim_params
    add_force = sim_manager.forces.addForce

    # Enable/disable render
    if args.render:
        sim_manager.render_params.renderer = py_dismech.OPENGL
        sim_manager.render_params.render_scale = 1.0
    else:
        sim_manager.render_params.renderer = py_dismech.HEADLESS

    # Parameters
    final_disp = np.array(args.final_disp)
    sim_params.dt = args.dt
    sim_params.dtol = args.dtol

    # At minimum take dt as step
    steps_per_move = max(int((1.0 / args.steps) / sim_params.dt), 1)
    ddisp = final_disp / (args.steps * steps_per_move)

    # Constants
    sim_params.integrator = py_dismech.BACKWARD_EULER
    vertices = np.loadtxt(Path(__file__).parent / "slinky_save.txt")
    gravity = np.array([0.0, 0.0, -9.81])
    damping = np.array([2.0])
    velocity_tolerance = 1e-3
    max_settle_steps = 10000

    # Create the helix limb with custom configuration
    add_limb = partial(
        sim_manager.soft_robots.addLimb,
        rho=args.density,
        rod_radius=args.radius,
        youngs_modulus=args.young_mod,
        poisson_ratio=args.poisson,
    )

    # Add the helical structure
    add_limb(vertices)
    helix = sim_manager.soft_robots.limbs[0]

    soft_robots.lockEdge(0, 0)  # first edge
    soft_robots.lockEdge(0, vertices.shape[0] - 2)  # last edge

    add_force(py_dismech.GravityForce(soft_robots, gravity))
    add_force(py_dismech.DampingForce(soft_robots, damping))
    add_force(
        py_dismech.ContactForce(
            soft_robots,
            2.01 * args.radius,
            0.1 * args.radius,
            0.1 * args.young_mod,
            True,
            0.3,
            True,
        )
    )

    sim_manager.initialize([])
    raw = []
    qs = []

    def step_until_static() -> None:
        """Progress simulation until |v| < v_tol or iters > max_settle_steps.

        Raises:
            RuntimeError: If iters > max_settle_steps and |v| > v_tol.
        """
        for i in range(max_settle_steps):
            sim_manager.step_simulation()
            if np.linalg.norm(helix.getVelocities()) < velocity_tolerance:
                vertices = helix.getVertices()
                qs.append(extract_n_nodes(vertices, args.n))
                raw.append(vertices)
                break

        if np.linalg.norm(helix.getVelocities()) > velocity_tolerance:
            raise RuntimeError(
                f"Unable to find a static state: v_norm={np.linalg.norm(helix.getVelocities()):.4f}"
            )

    # Wait until static under gravity
    step_until_static()
    for i in range(args.steps):
        # Take small steps for stability under contact
        for _ in range(steps_per_move):
            sim_manager.step_simulation(
                {
                    "delta_position": np.array(
                        [
                            [0, vertices.shape[0] - 1, *ddisp],
                            [0, vertices.shape[0] - 2, *ddisp],
                        ]
                    )
                }
            )
        step_until_static()

    np.savez(
        f"/app/{args.name}.npz",
        raw=np.asarray(raw),
        qs=np.asarray(qs),
    )
