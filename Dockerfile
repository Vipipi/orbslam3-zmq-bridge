FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git pkg-config \
    libopencv-dev \
    libeigen3-dev \
    libzmq3-dev && \
    rm -rf /var/lib/apt/lists/*

# cppzmq header-only
RUN apt-get update && apt-get install -y --no-install-recommends \
    libcppzmq-dev && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY . /app

# Configure with mock by default; user can mount ORB-SLAM3 and set -DUSE_MOCK_SLAM=OFF
RUN cmake -B build -S . -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build -j

CMD ["/app/build/rgbd_zmq_bridge", "--help"]


