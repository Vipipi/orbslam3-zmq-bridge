# ORB-SLAM3 ZeroMQ RGB-D Bridge (Docker Build Guide)

This project provides a standalone C++17 bridge that:
- Subscribes RGB-D frames over ZeroMQ
- Runs ORB-SLAM3 in RGB-D mode
- Publishes SLAM poses over ZeroMQ

## 1) Prerequisites
- Docker (Linux/macOS/Windows)
- ORB-SLAM3 sources available in the repository at `orb_slam3/` OR provide an installed ORB-SLAM3 via CMake variables
- Input/Output ports available on your host (e.g., 5555 for input, 6000 for output)

Notes:
- The build requires ORB-SLAM3 (no mock). The Docker image expects either:
  - ORB-SLAM3 sources present in `orb_slam3/` so CMake can `add_subdirectory(orb_slam3)` automatically, or
  - You pass include/lib paths at configure time (less convenient in Docker; see Option B below).

## 2) Prepare Sources (Option A: build ORB-SLAM3 from subdirectory)
Ensure the ORB-SLAM3 source tree is present at `orb_slam3/` in this repo.

Example layout:
```
orbslam3-zmq-wrapper/
  CMakeLists.txt
  Dockerfile
  src/
  include/
  tools/
  orb_slam3/
    CMakeLists.txt
    src/
    include/
    Thirdparty/
    ...
```
CMake will detect `orb_slam3/` and build/link it automatically inside the container.

## 3) Build the Docker Image
Linux/macOS:
```bash
docker build -t rgbd_zmq_bridge:latest .
```
Windows (PowerShell):
```powershell
docker build -t rgbd_zmq_bridge:latest .
```
This produces `/app/build/rgbd_zmq_bridge` inside the image.

## 4) Run the Container
You need to provide the vocabulary and settings YAML. The easiest way is to mount them in the container and pass their absolute paths to the binary.

Linux example:
```bash
docker run --rm \
  -v /abs/path/ORBvoc.txt.bin:/data/ORBvoc.txt.bin:ro \
  -v /abs/path/RGBD.yaml:/data/RGBD.yaml:ro \
  -p 5555:5555 -p 6000:6000 \
  rgbd_zmq_bridge:latest \
  /app/build/rgbd_zmq_bridge \
  --voc /data/ORBvoc.txt.bin \
  --settings /data/RGBD.yaml \
  --sub tcp://0.0.0.0:5555 \
  --pub tcp://*:6000
```

Windows (PowerShell) example:
```powershell
docker run --rm `
  -v C:\abs\path\ORBvoc.txt.bin:/data/ORBvoc.txt.bin:ro `
  -v C:\abs\path\RGBD.yaml:/data/RGBD.yaml:ro `
  -p 5555:5555 -p 6000:6000 `
  rgbd_zmq_bridge:latest `
  /app/build/rgbd_zmq_bridge `
  --voc /data/ORBvoc.txt.bin `
  --settings /data/RGBD.yaml `
  --sub tcp://0.0.0.0:5555 `
  --pub tcp://*:6000
```

Tips:
- On Linux you may also use `--network host` instead of `-p ...`, then use `tcp://127.0.0.1:5555` inside the app.
- ZMQ PUB/SUB: ensure your publisher connects to the SUB endpoint (`--sub`) and your subscribers connect to the PUB endpoint (`--pub`).

## 5) Test with the Provided Tools
Install dependencies on the host:
```bash
python3 -m pip install --user pyzmq opencv-python
```
Start the bridge container (Section 4), then in separate host terminals:

- Publish synthetic RGB-D to the container’s SUB:
```bash
python3 tools/publish_rgbd.py --endpoint tcp://127.0.0.1:5555 --hz 10
```
- Subscribe to the bridge’s output (tracking poses):
```bash
python3 tools/sub_debug.py --endpoint tcp://127.0.0.1:6000 --topic slam/tracking_pose
```

- Subscribe to keyframe packets (images + depth + intrinsics):
```bash
python3 d435i/zmq_debug.py -e tcp://127.0.0.1:6000 -t /slam/kf_packet
```

You should see pose matrices printed for each frame.

## 6) Alternative (Option B): Provide Installed ORB-SLAM3
If you have an ORB-SLAM3 build installed in your base image, configure CMake with explicit include/lib paths instead of using `orb_slam3/`.

Edit the Dockerfile build step (example):
```Dockerfile
RUN cmake -B build -S . -DCMAKE_BUILD_TYPE=Release \
    -DORB_SLAM3_INCLUDE_DIR=/opt/orbslam3/include \
    -DORB_SLAM3_LIBRARIES=/opt/orbslam3/lib/libORB_SLAM3.so \
 && cmake --build build -j
```

## 7) ZMQ Message Model
Input (publisher → bridge)
- Topic: `rgbd/input`
- Frames:
  1. `uint64 t_ns` (LE)
  2. `bytes rgb_jpeg`
  3. `bytes depth_16UC1_raw` (row-major, W×H×2)

Output (bridge → subscribers)
- `slam/tracking_pose`: `[ uint64 t_ns, float32[16] Twc_row_major ]`
- `slam/kf_pose`: `[ uint64 kf_id, uint64 t_ns, float32[16] Twc_row_major ]`

Standardized topics (leading slash) added for map lifecycle and KF updates:
- `/slam/kf_packet` (RGB-D data for keyframes):
  - Frames: `[ uint64 map_id, uint64 kf_id, uint64 t_ns, float32[9] K_row_major, bytes rgb_jpeg, bytes depth_16UC1_raw ]`
  - Notes: K is 3×3 row-major; depth buffer is raw z16 (W×H×2) matching the JPEG’s dimensions. Single-map backend uses `map_id = 0`.
- `/slam/kf_pose_update` (burst after optimizations):
  - Frames: `[ uint64 map_id, uint64 kf_id, float32[16] T_wc_map_new ]`
  - For single-map backend, `map_id = 0`.
- `/slam/map_new`:
  - Frames: `[ uint64 map_id, uint64 created_at_ns ]`
  - For single-map backend, emitted once at startup with `map_id = 0`.
- `/slam/map_switch`:
  - Frames: `[ uint64 from_map_id, uint64 to_map_id, bytes reason_ascii ]`
  - Emits `reason = "relocalized"` when tracking recovers (single-map: `0 -> 0`).
- `/slam/map_merge`:
  - Frames: `[ uint64 kept_map_id, uint64 merged_map_id, float32[7] S_kept_merged_sim3 ]`
  - Not emitted by this backend (no map merges currently).
- `/slam/map_correction`:
  - Frames: `[ uint64 map_id, float32[16] T_global_map_new_SE3 ]`
  - Not emitted by this backend (no global frame maintenance currently).

Notes:
- All numeric fields are little-endian (LE).
- Convert depth to meters using `DepthMapFactor` from your RGB-D YAML (typ. 1000.0).

## 8) Common Issues
- Container builds but binary fails at runtime: check that `ORBvoc.txt.bin` and `RGBD.yaml` are mounted correctly and paths match the CLI args.
- No output poses: verify timestamps are sensible (nanoseconds), JPEG decodes, and depth buffer size equals `W*H*2`.
- Networking: ensure ports 5555/6000 are exposed and your host firewall allows them.
