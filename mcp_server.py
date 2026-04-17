from mcp.server.fastmcp import FastMCP
from sim.scenarios import SlinkyConfig, run_slinky

mcp = FastMCP("Dismech Docker")

@mcp.tool()
def run_slinky_simulation(
    name: str = "out",
    n_nodes: int = 3,
    radius: float = 0.005,
    young_mod: float = 1e7,
    final_disp_x: float = 0.0,
    final_disp_y: float = 0.0,
    final_disp_z: float = 0.0,
    steps: int = 10
) -> str:
    """
    Runs a Discrete Elastic Rod (DER) physical simulation of a slinky.
    Use this tool to calculate physical trajectories and deformation of elastic rods.
    
    Returns:
        A string indicating success or failure, and the path to the .npz output.
    """
    try:
        result = run_slinky(
            SlinkyConfig(
                name=name,
                n_nodes=n_nodes,
                radius=radius,
                young_mod=young_mod,
                final_disp=(final_disp_x, final_disp_y, final_disp_z),
                steps=steps,
            )
        )
        return f"Simulation successful! Data saved to {result.output_path}."
    except Exception as error:
        return f"Simulation failed with error:\n{error}"

if __name__ == "__main__":
    mcp.run()