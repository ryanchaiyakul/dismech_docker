### Quick Start

1. Build local Docker image for [DISMech](https://github.com/StructuresComp/dismech-rods).
2. Run the provided [main.sh](main.sh) script.

```bash
docker build -t dismesh .
./main.sh --final_disp -0.05 0.0 0.0
```

> Based on the Docker image in [dismech-rods](https://github.com/StructuresComp/dismech-rods/blob/main/docker/Dockerfile) with modified MKL.
