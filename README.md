## DiSMech Docker

Minimal MCP server that lets an agent run [DiSMech](https://github.com/StructuresComp/dismech-rods) (`py_dismech`) scripts in a sandboxed container.

### Tools

- `api_reference()`: the `py_dismech` API.
- `run_script(code, timeout_s=600)`: runs a complete Python script and returns `exit_code`, `stdout`, `stderr` and `timed_out`. Relative paths write to `/app/output`.

### Register with Claude Code

```bash
claude mcp add dismech -- docker run -i --rm \
  --network none --read-only --tmpfs /tmp \
  --cap-drop ALL --security-opt no-new-privileges \
  --pids-limit 256 --memory 4g \
  -v "$PWD/output:/app/output" \
  ghcr.io/ryanchaiyakul/dismech_docker:latest
```
