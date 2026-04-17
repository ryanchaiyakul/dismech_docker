from __future__ import annotations

from dataclasses import dataclass
from functools import partial
from pathlib import Path

import numpy as np
import py_dismech

from sim.core import SimulationResult, prepare_output_dir, validate_output_name


@dataclass(frozen=True)
class SlinkyConfig:
    name: str = "out"
    n_nodes: int = 3
    radius: float = 0.005
    young_mod: float = 1e7
    density: float = 1273.52
    poisson: float = 0.5
    dt: float = 5e-3
    dtol: float = 1e-3
    steps: int = 10
    final_disp: tuple[float, float, float] = (0.0, 0.0, 0.0)
    render: bool = False
    output_dir: str | Path = "/app/output"


def extract_n_nodes(qs: np.ndarray, n: int) -> np.ndarray:
    if n < 2:
        raise ValueError("Number of extracted nodes (n) must be at least 2.")

    segments = np.diff(qs, axis=0)
    lengths = np.linalg.norm(segments, axis=1)
    s = np.concatenate(([0.0], np.cumsum(lengths)))
    s_target = np.linspace(0.0, s[-1], n)

    nodes = np.zeros((n, 3))
    for i in range(3):
        nodes[:, i] = np.interp(s_target, s, qs[:, i])

    result = []
    for i in range(n - 1):
        result.extend(nodes[i])
        result.append(0.0)
    result.extend(nodes[-1])
    return np.array(result)


def run_slinky(config: SlinkyConfig) -> SimulationResult:
    safe_name = validate_output_name(config.name)
    output_dir = prepare_output_dir(config.output_dir)

    sim_manager = py_dismech.SimulationManager()
    soft_robots = sim_manager.soft_robots
    sim_params = sim_manager.sim_params
    add_force = sim_manager.forces.addForce

    if config.render:
        sim_manager.render_params.renderer = py_dismech.OPENGL
        sim_manager.render_params.render_scale = 1.0
    else:
        sim_manager.render_params.renderer = py_dismech.HEADLESS

    final_disp = np.array(config.final_disp)
    sim_params.dt = config.dt
    sim_params.dtol = config.dtol
    steps_per_move = max(int((1.0 / config.steps) / sim_params.dt), 1)
    ddisp = final_disp / (config.steps * steps_per_move)

    sim_params.integrator = py_dismech.BACKWARD_EULER
    vertices = np.loadtxt(Path(__file__).resolve().parents[2] / "vertices/slinky.txt")
    gravity = np.array([0.0, 0.0, -9.81])
    damping = np.array([2.0])
    velocity_tolerance = 1e-3
    max_settle_steps = 10000

    add_limb = partial(
        sim_manager.soft_robots.addLimb,
        rho=config.density,
        rod_radius=config.radius,
        youngs_modulus=config.young_mod,
        poisson_ratio=config.poisson,
    )

    add_limb(vertices)
    helix = sim_manager.soft_robots.limbs[0]

    soft_robots.lockEdge(0, 0)
    soft_robots.lockEdge(0, vertices.shape[0] - 2)

    add_force(py_dismech.GravityForce(soft_robots, gravity))
    add_force(py_dismech.DampingForce(soft_robots, damping))
    add_force(
        py_dismech.ContactForce(
            soft_robots,
            2.01 * config.radius,
            0.1 * config.radius,
            0.1 * config.young_mod,
            True,
            0.3,
            True,
        )
    )

    sim_manager.initialize([])
    raw = []
    qs = []

    def step_until_static() -> None:
        for _ in range(max_settle_steps):
            sim_manager.step_simulation()
            if np.linalg.norm(helix.getVelocities()) < velocity_tolerance:
                current_vertices = helix.getVertices()
                qs.append(extract_n_nodes(current_vertices, config.n_nodes))
                raw.append(current_vertices)
                break

        if np.linalg.norm(helix.getVelocities()) > velocity_tolerance:
            raise RuntimeError(
                f"Unable to find a static state: v_norm={np.linalg.norm(helix.getVelocities()):.4f}"
            )

    step_until_static()
    for _ in range(config.steps):
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

    output_path = output_dir / f"{safe_name}.npz"
    np.savez(output_path, raw=np.asarray(raw), qs=np.asarray(qs))
    return SimulationResult(output_path=output_path)
