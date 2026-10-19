"""Cantilever under gravity should converge to Euler-Bernoulli: q*L^4 / (8*E*I).

uv run python tests/test_cantilever.py [image]
"""

import asyncio
import json
import sys
from itertools import pairwise

from fastmcp import Client
from fastmcp.client.transports import StdioTransport

IMAGE = sys.argv[1] if len(sys.argv) > 1 else "dismech_docker:latest"
NODES = [26, 51, 101, 201]
TOLERANCE = 0.02
SANDBOX = [
    "--network", "none", "--read-only", "--tmpfs", "/tmp", "--cap-drop", "ALL",
    "--security-opt", "no-new-privileges", "--pids-limit", "256", "--memory", "4g",
]

SCRIPT = f"""
import json
import numpy as np
import py_dismech

L, r, E, rho, g = 1.0, 0.02, 1e9, 1000.0, 9.8
tips = {{}}
for n in {NODES}:
    sim = py_dismech.SimulationManager()
    sim.sim_params.dt = 1e-2
    sim.sim_params.sim_time = 3.0
    sim.sim_params.dtol = 1e-6
    sim.sim_params.enable_2d_sim = True
    sim.sim_params.integrator = py_dismech.BACKWARD_EULER
    sim.render_params.renderer = py_dismech.HEADLESS
    sim.render_params.cmd_line_per = 0

    sim.soft_robots.addLimb(np.zeros(3), np.array([L, 0.0, 0.0]), n, rho, r, E, 0.5)
    sim.soft_robots.lockEdge(0, 0)
    sim.forces.addForce(py_dismech.GravityForce(sim.soft_robots, np.array([0.0, 0.0, -g])))
    sim.forces.addForce(py_dismech.DampingForce(sim.soft_robots, 5.0))

    sim.initialize([])
    while not sim.simulation_completed():
        sim.step_simulation()
    tips[n] = -float(sim.soft_robots.limbs[0].getVertices()[-1, 2])

I = np.pi * r**4 / 4
q = rho * np.pi * r**2 * g
print(json.dumps({{"tips": tips, "eb": q * L**4 / (8 * E * I)}}))
"""


async def main() -> int:
    transport = StdioTransport(command="docker", args=["run", "-i", "--rm", *SANDBOX, IMAGE])
    async with Client(transport, timeout=600) as client:
        result = (await client.call_tool("run_script", {"code": SCRIPT})).data
    if result["exit_code"] != 0:
        print(result["stderr"])
        return 1

    data = json.loads(result["stdout"].splitlines()[-1])
    errors = [abs(data["tips"][str(n)] - data["eb"]) / data["eb"] for n in NODES]
    for n, err in zip(NODES, errors):
        print(f"n={n:4d}  rel_err={err:.3%}")

    ok = all(a > b for a, b in pairwise(errors)) and errors[-1] < TOLERANCE
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
