# syntax=docker/dockerfile:1

# ---------------------------------------------------------------------------
# builder: compiles the DiSMech C++ stack + its py_dismech pybind11 module.
# ---------------------------------------------------------------------------
FROM ubuntu:jammy AS builder

RUN apt update && apt install -y --no-install-recommends \
        libeigen3-dev libglu1-mesa-dev freeglut3-dev mesa-common-dev llvm llvm-dev \
        wget ca-certificates gnupg build-essential cmake pybind11-dev ffmpeg git \
        libfcl-dev libgmp-dev libblas-dev liblapack-dev zlib1g-dev libpng-dev libsdl2-dev \
        libpython3-dev python3-pip python3-numpy \
    && rm -rf /var/lib/apt/lists/*

# Intel MKL -- dismech-rods' linear algebra backend
RUN wget -O- https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB | gpg --dearmor | tee /usr/share/keyrings/oneapi-archive-keyring.gpg > /dev/null \
    && echo "deb [signed-by=/usr/share/keyrings/oneapi-archive-keyring.gpg] https://apt.repos.intel.com/oneapi all main" | tee /etc/apt/sources.list.d/oneAPI.list \
    && apt update && apt install -y --no-install-recommends intel-oneapi-mkl-devel \
    && rm -rf /var/lib/apt/lists/*

# symengine -- symbolic differentiation used to generate dismech-rods' elastic energy gradients/Hessians
RUN git clone --depth 1 https://github.com/symengine/symengine && mkdir symengine/build && cd symengine/build \
    && cmake -D WITH_LLVM=on -D BUILD_BENCHMARKS=off -D BUILD_TESTS=off .. \
    && make -j"$(nproc)" && make install && cd / && rm -rf symengine

# VIEWER: Magnum + dependencies, only needed so dismech-rods can build its optional OpenGL renderer
RUN git clone --depth 1 https://github.com/mosra/corrade.git && mkdir corrade/build && cd corrade/build \
    && cmake .. && make -j"$(nproc)" && make install && cd / && rm -rf corrade
RUN git clone --depth 1 https://github.com/mosra/magnum.git && mkdir magnum/build && cd magnum/build \
    && cmake -DMAGNUM_WITH_SDL2APPLICATION=ON .. && make -j"$(nproc)" && make install && cd / && rm -rf magnum
RUN git clone --depth 1 https://github.com/mosra/magnum-plugins && mkdir magnum-plugins/build && cd magnum-plugins/build \
    && cmake -DMAGNUM_WITH_PNGIMAGECONVERTER=ON .. && make -j"$(nproc)" && make install && cd / && rm -rf magnum-plugins

# dismech-rods -- builds py_dismech.so (the pybind11 extension) and pip-installs it in editable
# mode, so /dismech-rods (the source tree) must ship alongside the compiled .so at runtime.
ENV MKL_DIR=/opt/intel/oneapi/mkl/latest/lib/cmake/mkl
RUN git clone --depth 1 https://github.com/StructuresComp/dismech-rods.git && mkdir dismech-rods/build && cd dismech-rods/build \
    && cmake -DWITH_MAGNUM=ON -DWITH_PYBIND=ON -DCREATE_EXAMPLES=on .. && make -j"$(nproc)" \
    && cd .. && pip install --no-cache-dir -e .

# ---------------------------------------------------------------------------
# stubgen: runs pybind11-stubgen against the built py_dismech.so to produce .pyi type stubs.
# ---------------------------------------------------------------------------
FROM builder AS stubgen
RUN pip install --no-cache-dir pybind11-stubgen \
    && pybind11-stubgen py_dismech -o /stubs

# stubs: an empty (scratch) image holding only /stubs's contents. Needed
# because `docker build --output` exports a target stage's *entire* rootfs;
# without this, exporting `stubgen` directly would dump the whole Ubuntu
# builder filesystem onto the host instead of just the generated .pyi files.
#
# Regenerate/refresh the checked-in stubs/ with:
#   DOCKER_BUILDKIT=1 docker build --target stubs --output type=local,dest=stubs .
FROM scratch AS stubs
COPY --from=stubgen /stubs /

# ---------------------------------------------------------------------------
# runtime: slim image that only ships compiled artifacts + shared libs from
# `builder`, plus this project's Python source -- no compilers/build deps.
# ---------------------------------------------------------------------------
FROM ubuntu:jammy AS runtime

# Runtime-only shared libs (the -dev/-devel/build-essential packages from the
# builder stage are intentionally left out)
RUN apt update && apt install -y --no-install-recommends \
        libglu1-mesa freeglut3 libgomp1 ffmpeg \
        libfcl0.7 libgmp10 libblas3 liblapack3 zlib1g libpng16-16 libsdl2-2.0-0 \
        libllvm14 \
        python3 python3-pip python3-numpy ca-certificates \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir uv

# Compiled libraries/headers installed by cmake (symengine, corrade, magnum, magnum-plugins),
# and py_dismech's compiled .so + editable-install metadata (all under /usr/local)
COPY --from=builder /usr/local /usr/local

# Intel MKL runtime libraries only (not the -devel headers/static libs)
COPY --from=builder /opt/intel/oneapi/mkl /opt/intel/oneapi/mkl

# Intel OpenMP runtime (libiomp5.so) -- MKL's threading layer links against
# this, but it ships under compiler/, not mkl/, so it needs its own copy.
COPY --from=builder /opt/intel/oneapi/compiler /opt/intel/oneapi/compiler

# py_dismech's editable install points back at this source tree (the .pth/egg-link
# itself lives under /usr/local, already copied above) -- must ship alongside it
COPY --from=builder /dismech-rods /dismech-rods

ENV LD_LIBRARY_PATH=/usr/local/lib:/opt/intel/oneapi/mkl/latest/lib:/opt/intel/oneapi/compiler/latest/lib
ENV MKL_DIR=/opt/intel/oneapi/mkl/latest/lib/cmake/mkl

RUN ldconfig

# This project's own code (everything under src/dismech_docker) -- the ONLY
# layer invalidated by day-to-day development, since it's copied last.
WORKDIR /app
COPY pyproject.toml README.md /app/
COPY src /app/src

RUN uv pip install --system --no-cache /app

# Listen to stdin/stdout as an MCP server (console script from [project.scripts])
ENTRYPOINT ["dismech-mcp-server"]
