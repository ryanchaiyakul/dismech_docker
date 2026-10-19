# syntax=docker/dockerfile:1

# builder: compiles dismech-rods and its py_dismech module
FROM ubuntu:jammy AS builder

RUN apt update && apt install -y --no-install-recommends \
        libeigen3-dev libglu1-mesa-dev freeglut3-dev mesa-common-dev llvm llvm-dev \
        wget ca-certificates gnupg build-essential cmake pybind11-dev ffmpeg git \
        libfcl-dev libgmp-dev libblas-dev liblapack-dev zlib1g-dev libpng-dev libsdl2-dev \
        libpython3-dev python3-pip python3-numpy \
    && rm -rf /var/lib/apt/lists/*

RUN wget -O- https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB | gpg --dearmor | tee /usr/share/keyrings/oneapi-archive-keyring.gpg > /dev/null \
    && echo "deb [signed-by=/usr/share/keyrings/oneapi-archive-keyring.gpg] https://apt.repos.intel.com/oneapi all main" | tee /etc/apt/sources.list.d/oneAPI.list \
    && apt update && apt install -y --no-install-recommends intel-oneapi-mkl-devel \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 https://github.com/symengine/symengine && mkdir symengine/build && cd symengine/build \
    && cmake -D WITH_LLVM=on -D BUILD_BENCHMARKS=off -D BUILD_TESTS=off .. \
    && make -j"$(nproc)" && make install && cd / && rm -rf symengine

RUN git clone --depth 1 https://github.com/mosra/corrade.git && mkdir corrade/build && cd corrade/build \
    && cmake .. && make -j"$(nproc)" && make install && cd / && rm -rf corrade
RUN git clone --depth 1 https://github.com/mosra/magnum.git && mkdir magnum/build && cd magnum/build \
    && cmake -DMAGNUM_WITH_SDL2APPLICATION=ON .. && make -j"$(nproc)" && make install && cd / && rm -rf magnum
RUN git clone --depth 1 https://github.com/mosra/magnum-plugins && mkdir magnum-plugins/build && cd magnum-plugins/build \
    && cmake -DMAGNUM_WITH_PNGIMAGECONVERTER=ON .. && make -j"$(nproc)" && make install && cd / && rm -rf magnum-plugins

ENV MKL_DIR=/opt/intel/oneapi/mkl/latest/lib/cmake/mkl
RUN git clone --depth 1 https://github.com/StructuresComp/dismech-rods.git && mkdir dismech-rods/build && cd dismech-rods/build \
    && cmake -DWITH_MAGNUM=ON -DWITH_PYBIND=ON -DCREATE_EXAMPLES=on .. && make -j"$(nproc)" \
    && cd .. && pip install --no-cache-dir -e .

# stubs: regenerate stubs/ with
#   docker build --target stubs --output type=local,dest=stubs .
FROM builder AS stubgen
RUN pip install --no-cache-dir pybind11-stubgen \
    && pybind11-stubgen py_dismech -o /stubs

FROM scratch AS stubs
COPY --from=stubgen /stubs /

# runtime: shared libs + compiled artifacts only
FROM ubuntu:jammy AS runtime

RUN apt update && apt install -y --no-install-recommends \
        libglu1-mesa freeglut3 libgomp1 ffmpeg \
        libfcl0.7 libgmp10 libblas3 liblapack3 zlib1g libpng16-16 libsdl2-2.0-0 \
        libllvm14 \
        python3 python3-pip python3-numpy ca-certificates \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir uv

COPY --from=builder /usr/local /usr/local
COPY --from=builder /opt/intel/oneapi/mkl /opt/intel/oneapi/mkl
# libiomp5 (needed by MKL) lives under compiler/
COPY --from=builder /opt/intel/oneapi/compiler /opt/intel/oneapi/compiler
# py_dismech is an editable install pointing at this tree
COPY --from=builder /dismech-rods /dismech-rods

ENV LD_LIBRARY_PATH=/usr/local/lib:/opt/intel/oneapi/mkl/latest/lib:/opt/intel/oneapi/compiler/latest/lib
ENV MKL_DIR=/opt/intel/oneapi/mkl/latest/lib/cmake/mkl

RUN ldconfig

WORKDIR /app
COPY pyproject.toml README.md /app/
COPY src /app/src

RUN uv pip install --system --no-cache /app

RUN useradd --create-home --uid 1000 dismech \
    && mkdir -p /app/output && chown dismech:dismech /app/output
USER dismech

ENTRYPOINT ["dismech-mcp-server"]
