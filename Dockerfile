FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git pkg-config wget unzip \
    libeigen3-dev \
    # Pangolin prerequisites (subset of recommended)
    libglfw3-dev libglew-dev libgl1-mesa-dev libegl1-mesa-dev \
    libglu1-mesa-dev libx11-dev \
    libxkbcommon-dev libxi-dev libxinerama-dev libxcursor-dev \
    libwayland-dev wayland-protocols \
    libjpeg-dev libpng-dev libtiff-dev libopenexr-dev liblz4-dev \
    libepoxy-dev \
    libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libavdevice-dev \
    libv4l-dev libxvidcore-dev libx264-dev libgtk-3-dev \
    libatlas-base-dev gfortran libboost-serialization-dev libssl-dev \
    libpython3-dev python3-pip \
    libudev-dev \
    ca-certificates && \
    rm -rf /var/lib/apt/lists/*

# Ensure CA certificates are registered for TLS (git over HTTPS)
RUN update-ca-certificates

# Build and install OpenCV (pinned)
RUN mkdir -p /deps/opencv && cd /deps/opencv && \
    git clone https://github.com/opencv/opencv.git --branch 4.5.5 --depth 1 . && \
    git clone https://github.com/opencv/opencv_contrib.git --branch 4.5.5 --depth 1 ./contrib && \
    mkdir build && cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release \
          -D CMAKE_INSTALL_PREFIX=/usr/local \
          -D OPENCV_EXTRA_MODULES_PATH=../contrib/modules \
          -D BUILD_EXAMPLES=OFF -D BUILD_TESTS=OFF \
          -D BUILD_PERF_TESTS=OFF -D BUILD_DOCS=OFF .. && \
    make -j$(nproc) && make install && ldconfig && \
    rm -rf /deps/opencv

# Build and install Pangolin (viewer dependency for ORB-SLAM3, pinned)
RUN mkdir -p /deps/Pangolin && cd /deps/Pangolin && \
    git clone https://github.com/stevenlovegrove/Pangolin.git --branch v0.6 --depth 1 . && \
    mkdir build && cd build && \
    cmake .. -DBUILD_PANGOLIN_PYTHON=OFF -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && make install && ldconfig && \
    rm -rf /deps/Pangolin

# Make /usr/local/lib visible to the runtime loader and processes
ENV LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}

# Build and install libzmq (ZeroMQ C library)
RUN git clone --depth=1 https://github.com/zeromq/libzmq.git /tmp/libzmq && \
    cmake -S /tmp/libzmq -B /tmp/libzmq/build -DCMAKE_BUILD_TYPE=Release && \
    cmake --build /tmp/libzmq/build -j && \
    cmake --install /tmp/libzmq/build && ldconfig && \
    rm -rf /tmp/libzmq

# Build and install cppzmq (header-only C++ bindings)
RUN git clone --depth=1 https://github.com/zeromq/cppzmq.git /tmp/cppzmq && \
    cmake -S /tmp/cppzmq -B /tmp/cppzmq/build -DCPPZMQ_BUILD_TESTS=OFF && \
    cmake --build /tmp/cppzmq/build -j && \
    cmake --install /tmp/cppzmq/build && ldconfig && \
    rm -rf /tmp/cppzmq

WORKDIR /app
COPY . /app

# Configure with mock by default; user can mount ORB-SLAM3 and set -DUSE_MOCK_SLAM=OFF
RUN cmake -B build -S . -DCMAKE_BUILD_TYPE=Release -DUSE_MOCK_SLAM=OFF \
    && cmake --build build -j

CMD ["/app/build/rgbd_zmq_bridge", "--help"]


