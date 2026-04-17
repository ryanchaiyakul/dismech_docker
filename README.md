## Quick Start

The container entrypoint is [mcp_server.py](mcp_server.py), which exposes the `run_slinky_simulation` tool over stdio. To run the MCP server:

```bash
docker build -t dismesh .
docker run --rm --volume "$(pwd)":/app dismesh
```

Then, connect via an MCP client (such as the [Node.js MCP client library](https://github.com/modelcontextprotocol/sdk-python) or Claude Desktop with MCP config) and call:

```json
{
  "tool": "run_slinky_simulation",
  "parameters": {
    "name": "my_sim",
    "n_nodes": 5,
    "final_disp_x": -0.1,
    "final_disp_y": 0.0,
    "final_disp_z": 0.0,
    "steps": 20
  }
}
```

The output `.npz` file will be written to `/app/output` inside the container, which maps to your workspace directory when using the volume mount.

The shared scenario implementation lives under [sim/scenarios](sim/scenarios), which is the place to add new simulation cases without changing the entrypoints.

## Adding A Scenario

1. Create a new module under [sim/scenarios](sim/scenarios), for example [sim/scenarios/new_case.py](sim/scenarios/new_case.py).
2. Put the scenario config and a `run_<name>()` function in that module, following the pattern in [sim/scenarios/slinky.py](sim/scenarios/slinky.py).
3. Re-export the new scenario from [sim/scenarios/__init__.py](sim/scenarios/__init__.py).
4. If you want MCP access, add one tool in [mcp_server.py](mcp_server.py) that calls the scenario function.
5. If the scenario needs extra shared helpers, put them in [sim/core.py](sim/core.py) so the entrypoints stay thin.

> Based on the Docker image in [dismech-rods](https://github.com/StructuresComp/dismech-rods/blob/main/docker/Dockerfile) with modified MKL.
