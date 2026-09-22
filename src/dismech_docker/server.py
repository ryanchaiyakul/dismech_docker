import py_dismech
from fastmcp import FastMCP

mcp = FastMCP("Dismech Docker")


@mcp.tool()
def dismech_version() -> str:
    return f"py_dismech loaded from {py_dismech.__file__}"


def main() -> None:
    mcp.run()
