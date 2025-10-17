#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <chrono>
#include <unordered_map>
#include <cstring>
#include <deque>
#include <mutex>
#include <cmath>

#include <opencv2/opencv.hpp>
#include <zmq.hpp>

#include "orbslam3_rgbd_zmq_bridge/zmq_util.hpp"
#include "orbslam3_rgbd_zmq_bridge/slam_runner.hpp"

using namespace std::chrono_literals;
using orbslam3_rgbd_zmq_bridge::PublisherEndpoints;
using orbslam3_rgbd_zmq_bridge::PubSockets;

namespace {

std::string getEnv(const char* key) {
  const char* v = std::getenv(key);
  return v ? std::string(v) : std::string();
}

bool parseBool(const std::string& s, bool defVal) {
  if (s.empty()) return defVal;
  std::string t = s;
  for (auto& c : t) c = static_cast<char>(::tolower(c));
  if (t == "1" || t == "true" || t == "yes" || t == "on") return true;
  if (t == "0" || t == "false" || t == "no" || t == "off") return false;
  return defVal;
}

std::uint64_t parseUint64LE(const void* data, size_t len) {
  if (len < 8) return 0;
  const std::uint8_t* b = static_cast<const std::uint8_t*>(data);
  std::uint64_t v = 0;
  for (int i = 0; i < 8; ++i) v |= (static_cast<std::uint64_t>(b[i]) << (8 * i));
  return v;
}

void usage() {
  std::cerr << "Usage: rgbd_zmq_bridge --voc /path/ORBvoc.txt.bin --settings /path/RGBD.yaml --sub tcp://host:port [--pub tcp://:6000 | --pub-tracking tcp://:6001 --pub-kf-pose tcp://:6003] [--bind true]" << std::endl;
}

struct Config {
  std::string voc;
  std::string settings;
  std::string subEndpoint;
  std::string pubSingle;
  std::string pubTracking;
  std::string pubKfPose;
  bool bindPub = true;
};

Config parseArgs(int argc, char** argv) {
  Config c;
  // Env fallback
  c.voc = getEnv("RGBD_VOC");
  c.settings = getEnv("RGBD_SETTINGS");
  c.subEndpoint = getEnv("RGBD_SUB");
  c.pubSingle = getEnv("RGBD_PUB");
  c.pubTracking = getEnv("RGBD_PUB_TRACKING");
  c.pubKfPose = getEnv("RGBD_PUB_KF_POSE");
  c.bindPub = parseBool(getEnv("RGBD_BIND"), true);

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto val = [&](int& i) -> std::string { return (i + 1 < argc) ? std::string(argv[++i]) : std::string(); };
    if (a == "--voc") c.voc = val(i);
    else if (a == "--settings") c.settings = val(i);
    else if (a == "--sub") c.subEndpoint = val(i);
    else if (a == "--pub") c.pubSingle = val(i);
    else if (a == "--pub-tracking") c.pubTracking = val(i);
    else if (a == "--pub-kf-pose") c.pubKfPose = val(i);
    else if (a == "--bind") c.bindPub = parseBool(val(i), true);
    else if (a == "-h" || a == "--help") { usage(); std::exit(0); }
  }
  return c;
}

float readDepthMapFactor(const std::string& settingsPath) {
  cv::FileStorage fs(settingsPath, cv::FileStorage::READ);
  if (!fs.isOpened()) return 1000.0f;
  double v = 1000.0;
  cv::FileNode n = fs["DepthMapFactor"];
  if (!n.empty()) v = static_cast<double>(n);
  fs.release();
  if (v <= 0.0) v = 1000.0;
  return static_cast<float>(v);
}

struct CameraK {
  float data[9] = {0};
  bool valid = false;
};

CameraK readCameraK(const std::string& settingsPath) {
  CameraK K;
  cv::FileStorage fs(settingsPath, cv::FileStorage::READ);
  if (!fs.isOpened()) return K;
  auto getf = [&](const char* key, double defv) -> float {
    cv::FileNode n = fs[key];
    double v = defv;
    if (!n.empty()) v = static_cast<double>(n);
    return static_cast<float>(v);
  };
  float fx = getf("Camera.fx", 0.0);
  float fy = getf("Camera.fy", 0.0);
  float cx = getf("Camera.cx", 0.0);
  float cy = getf("Camera.cy", 0.0);
  if (fx <= 0.0f || fy <= 0.0f) {
    fx = getf("Camera1.fx", fx);
    fy = getf("Camera1.fy", fy);
    cx = getf("Camera1.cx", cx);
    cy = getf("Camera1.cy", cy);
  }
  fs.release();
  if (fx > 0.0f && fy > 0.0f) {
    K.data[0] = fx; K.data[1] = 0.0f; K.data[2] = cx;
    K.data[3] = 0.0f; K.data[4] = fy; K.data[5] = cy;
    K.data[6] = 0.0f; K.data[7] = 0.0f; K.data[8] = 1.0f;
    K.valid = true;
  }
  return K;
}

} // namespace

int main(int argc, char** argv) {
  auto cfg = parseArgs(argc, argv);
  if (cfg.voc.empty() || cfg.settings.empty() || cfg.subEndpoint.empty()) {
    usage();
    return 1;
  }

  // Initialize SLAM
  orbslam3_rgbd_zmq_bridge::SlamInitParams init{cfg.voc, cfg.settings};
  orbslam3_rgbd_zmq_bridge::SlamRunner slam(init);
  if (!slam.isOk()) {
    std::cerr << "Failed to initialize ORB-SLAM3 (or mock runner)." << std::endl;
  }

  const float depthMapFactor = readDepthMapFactor(cfg.settings);
  const double invDepthFactor = 1.0 / static_cast<double>(depthMapFactor);
  const CameraK cameraK = readCameraK(cfg.settings);

  // ZMQ context and sockets
  zmq::context_t ctx(1);
  // SUB binds per project spec; publishers connect to this endpoint
  zmq::socket_t sub(ctx, zmq::socket_type::sub);
  orbslam3_rgbd_zmq_bridge::bindOrConnect(sub, cfg.subEndpoint, /*bindMode=*/true);
  sub.set(zmq::sockopt::subscribe, std::string("rgbd/input"));

  PublisherEndpoints eps;
  eps.singlePubEndpoint = cfg.pubSingle;
  if (!cfg.pubTracking.empty()) eps.trackingEndpoint = cfg.pubTracking;
  if (!cfg.pubKfPose.empty()) eps.kfPoseEndpoint = cfg.pubKfPose;
  PubSockets pubs = orbslam3_rgbd_zmq_bridge::makePubs(ctx, eps, cfg.bindPub);

  // Emit single-map creation event for single-map backend (map_id = 0)
  {
    std::uint64_t mapId = 0;
    auto now = std::chrono::system_clock::now().time_since_epoch();
    std::uint64_t createdNs = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
    auto mapIdBytes = orbslam3_rgbd_zmq_bridge::packUint64LE(mapId);
    auto createdBytes = orbslam3_rgbd_zmq_bridge::packUint64LE(createdNs);
    std::vector<zmq::const_buffer> frames{
      zmq::buffer(mapIdBytes.data(), mapIdBytes.size()),
      zmq::buffer(createdBytes.data(), createdBytes.size())
    };
    if (pubs.singlePub) {
      orbslam3_rgbd_zmq_bridge::sendMultipart(*pubs.singlePub, "/slam/map_new", frames);
    } else if (pubs.trackingPub) {
      orbslam3_rgbd_zmq_bridge::sendMultipart(*pubs.trackingPub, "/slam/map_new", frames);
    } else if (pubs.kfPosePub) {
      orbslam3_rgbd_zmq_bridge::sendMultipart(*pubs.kfPosePub, "/slam/map_new", frames);
    }
  }

  std::atomic<bool> running{true};
  std::unordered_map<std::uint64_t, std::array<float, 16>> kfCache;

  struct FrameCacheEntry {
    std::uint64_t tNs;
    std::array<float, 16> Twc;
    std::vector<std::uint8_t> rgbJpeg;
    std::vector<std::uint8_t> depthRaw;
  };
  std::deque<FrameCacheEntry> frameCache;
  std::mutex frameCacheMutex;
  constexpr std::size_t kFrameCacheMax = 128;

  auto poseError = [](const std::array<float, 16>& A, const std::array<float, 16>& B) -> double {
    double dx = static_cast<double>(A[12]) - static_cast<double>(B[12]);
    double dy = static_cast<double>(A[13]) - static_cast<double>(B[13]);
    double dz = static_cast<double>(A[14]) - static_cast<double>(B[14]);
    double tErr = dx*dx + dy*dy + dz*dz;
    double tr = 0.0;
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) tr += static_cast<double>(A[r*4 + c]) * static_cast<double>(B[r*4 + c]);
    }
    double rErr = (3.0 - tr);
    if (rErr < 0.0) rErr = 0.0;
    return tErr + 0.01 * rErr;
  };

  std::thread kfThread([&]() {
    while (running.load()) {
      auto kfs = slam.getAllKeyframePoses();
      if (!kfs.empty()) {
        for (const auto& kf : kfs) {
          auto it = kfCache.find(kf.id);
          bool changed = false;
          if (it == kfCache.end()) {
            changed = true;
            kfCache[kf.id] = kf.TwcRowMajor;
          } else if (std::memcmp(it->second.data(), kf.TwcRowMajor.data(), sizeof(float) * 16) != 0) {
            changed = true;
            it->second = kf.TwcRowMajor;
          }
          if (changed) {
            if (pubs.kfPosePub) {
              auto tnsBytes = orbslam3_rgbd_zmq_bridge::packUint64LE(kf.tNs);
              std::vector<std::uint8_t> poseBytes = orbslam3_rgbd_zmq_bridge::packFloat32ArrayLE(kf.TwcRowMajor.data(), 16);
              auto idBytes = orbslam3_rgbd_zmq_bridge::packUint64LE(kf.id);
              std::vector<zmq::const_buffer> frames{
                zmq::buffer(idBytes.data(), idBytes.size()),
                zmq::buffer(tnsBytes.data(), tnsBytes.size()),
                zmq::buffer(poseBytes.data(), poseBytes.size())
              };
              orbslam3_rgbd_zmq_bridge::sendMultipart(*pubs.kfPosePub, "slam/kf_pose", frames);
              // Also emit standardized kf pose update with map_id
              {
                std::uint64_t mapId = 0;
                auto mapIdBytes = orbslam3_rgbd_zmq_bridge::packUint64LE(mapId);
                std::vector<zmq::const_buffer> framesUpd{
                  zmq::buffer(mapIdBytes.data(), mapIdBytes.size()),
                  zmq::buffer(idBytes.data(), idBytes.size()),
                  zmq::buffer(poseBytes.data(), poseBytes.size())
                };
              orbslam3_rgbd_zmq_bridge::sendMultipart(*pubs.kfPosePub, "/slam/kf_pose_update", framesUpd);
              if (cameraK.valid) {
                FrameCacheEntry best{};
                bool found = false;
                double bestErr = 0.0;
                {
                  std::lock_guard<std::mutex> lg(frameCacheMutex);
                  for (const auto& e : frameCache) {
                    double err = poseError(kf.TwcRowMajor, e.Twc);
                    if (!found || err < bestErr) { best = e; bestErr = err; found = true; }
                  }
                }
                if (found) {
                  std::uint64_t mapId = 0;
                  auto mapIdBytes = orbslam3_rgbd_zmq_bridge::packUint64LE(mapId);
                  auto kfIdBytes = orbslam3_rgbd_zmq_bridge::packUint64LE(kf.id);
                  auto tnsBytes = orbslam3_rgbd_zmq_bridge::packUint64LE(best.tNs);
                  std::vector<std::uint8_t> kBytes = orbslam3_rgbd_zmq_bridge::packFloat32ArrayLE(cameraK.data, 9);
                  std::vector<zmq::const_buffer> packetFrames{
                    zmq::buffer(mapIdBytes.data(), mapIdBytes.size()),
                    zmq::buffer(kfIdBytes.data(), kfIdBytes.size()),
                    zmq::buffer(tnsBytes.data(), tnsBytes.size()),
                    zmq::buffer(kBytes.data(), kBytes.size()),
                    zmq::buffer(best.rgbJpeg.data(), best.rgbJpeg.size()),
                    zmq::buffer(best.depthRaw.data(), best.depthRaw.size())
                  };
                  orbslam3_rgbd_zmq_bridge::sendMultipart(*pubs.kfPosePub, "/slam/kf_packet", packetFrames);
                }
              }
              }
            } else if (pubs.singlePub) {
              auto tnsBytes = orbslam3_rgbd_zmq_bridge::packUint64LE(kf.tNs);
              std::vector<std::uint8_t> poseBytes = orbslam3_rgbd_zmq_bridge::packFloat32ArrayLE(kf.TwcRowMajor.data(), 16);
              auto idBytes = orbslam3_rgbd_zmq_bridge::packUint64LE(kf.id);
              std::vector<zmq::const_buffer> frames{
                zmq::buffer(idBytes.data(), idBytes.size()),
                zmq::buffer(tnsBytes.data(), tnsBytes.size()),
                zmq::buffer(poseBytes.data(), poseBytes.size())
              };
              orbslam3_rgbd_zmq_bridge::sendMultipart(*pubs.singlePub, "slam/kf_pose", frames);
              // Also emit standardized kf pose update with map_id
              {
                std::uint64_t mapId = 0;
                auto mapIdBytes = orbslam3_rgbd_zmq_bridge::packUint64LE(mapId);
                std::vector<zmq::const_buffer> framesUpd{
                  zmq::buffer(mapIdBytes.data(), mapIdBytes.size()),
                  zmq::buffer(idBytes.data(), idBytes.size()),
                  zmq::buffer(poseBytes.data(), poseBytes.size())
                };
              orbslam3_rgbd_zmq_bridge::sendMultipart(*pubs.singlePub, "/slam/kf_pose_update", framesUpd);
              if (cameraK.valid) {
                FrameCacheEntry best{};
                bool found = false;
                double bestErr = 0.0;
                {
                  std::lock_guard<std::mutex> lg(frameCacheMutex);
                  for (const auto& e : frameCache) {
                    double err = poseError(kf.TwcRowMajor, e.Twc);
                    if (!found || err < bestErr) { best = e; bestErr = err; found = true; }
                  }
                }
                if (found) {
                  std::uint64_t mapId = 0;
                  auto mapIdBytes = orbslam3_rgbd_zmq_bridge::packUint64LE(mapId);
                  auto kfIdBytes = orbslam3_rgbd_zmq_bridge::packUint64LE(kf.id);
                  auto tnsBytes = orbslam3_rgbd_zmq_bridge::packUint64LE(best.tNs);
                  std::vector<std::uint8_t> kBytes = orbslam3_rgbd_zmq_bridge::packFloat32ArrayLE(cameraK.data, 9);
                  std::vector<zmq::const_buffer> packetFrames{
                    zmq::buffer(mapIdBytes.data(), mapIdBytes.size()),
                    zmq::buffer(kfIdBytes.data(), kfIdBytes.size()),
                    zmq::buffer(tnsBytes.data(), tnsBytes.size()),
                    zmq::buffer(kBytes.data(), kBytes.size()),
                    zmq::buffer(best.rgbJpeg.data(), best.rgbJpeg.size()),
                    zmq::buffer(best.depthRaw.data(), best.depthRaw.size())
                  };
                  orbslam3_rgbd_zmq_bridge::sendMultipart(*pubs.singlePub, "/slam/kf_packet", packetFrames);
                }
              }
              }
            }
          }
        }
      }
      std::this_thread::sleep_for(200ms);
    }
  });

  bool lastTrackingOk = false;
  int lostStreak = 0;

  while (running.load()) {
    std::vector<zmq::message_t> parts;
    while (true) {
      zmq::message_t msg;
      if (!sub.recv(msg, zmq::recv_flags::none)) {
        continue;
      }
      bool more = sub.get(zmq::sockopt::rcvmore);
      parts.emplace_back(std::move(msg));
      if (!more) break;
    }
    if (parts.size() < 4) continue;
    std::string topic(static_cast<char*>(parts[0].data()), parts[0].size());
    if (topic != "rgbd/input") continue;

    const auto& tMsg = parts[1];
    const auto& jpegMsg = parts[2];
    const auto& depthMsg = parts[3];

    std::uint64_t tNs = parseUint64LE(tMsg.data(), tMsg.size());

    // Decode RGB JPEG -> BGR8
    std::vector<uchar> jpegBuf((uchar*)jpegMsg.data(), (uchar*)jpegMsg.data() + jpegMsg.size());
    cv::Mat rgbBgr = cv::imdecode(jpegBuf, cv::IMREAD_COLOR);
    if (rgbBgr.empty()) continue;

    // Depth raw 16UC1; assume same WxH as RGB for MVP
    int W = rgbBgr.cols;
    int H = rgbBgr.rows;
    if (static_cast<int>(depthMsg.size()) != W * H * 2) {
      // size mismatch; skip
      continue;
    }
    cv::Mat depthU16(H, W, CV_16UC1, const_cast<void*>(depthMsg.data()));
    cv::Mat depthClone = depthU16.clone();
    cv::Mat depthF32;
    depthClone.convertTo(depthF32, CV_32FC1, invDepthFactor);

    std::array<float, 16> Twc{};
    bool ok = slam.trackRgbd(rgbBgr, depthF32, tNs, Twc);
    if (ok) {
      // If we were previously lost, emit a relocalization map_switch event (single-map: 0->0)
      if (!lastTrackingOk && lostStreak > 0) {
        std::uint64_t fromMapId = 0;
        std::uint64_t toMapId = 0;
        auto fromBytes = orbslam3_rgbd_zmq_bridge::packUint64LE(fromMapId);
        auto toBytes = orbslam3_rgbd_zmq_bridge::packUint64LE(toMapId);
        const std::string reason = "relocalized";
        std::vector<zmq::const_buffer> switchFrames{
          zmq::buffer(fromBytes.data(), fromBytes.size()),
          zmq::buffer(toBytes.data(), toBytes.size()),
          zmq::buffer(reason.data(), reason.size())
        };
        if (pubs.singlePub) {
          orbslam3_rgbd_zmq_bridge::sendMultipart(*pubs.singlePub, "/slam/map_switch", switchFrames);
        } else if (pubs.trackingPub) {
          orbslam3_rgbd_zmq_bridge::sendMultipart(*pubs.trackingPub, "/slam/map_switch", switchFrames);
        } else if (pubs.kfPosePub) {
          orbslam3_rgbd_zmq_bridge::sendMultipart(*pubs.kfPosePub, "/slam/map_switch", switchFrames);
        }
      }

      auto tnsBytes = orbslam3_rgbd_zmq_bridge::packUint64LE(tNs);
      std::vector<std::uint8_t> poseBytes = orbslam3_rgbd_zmq_bridge::packFloat32ArrayLE(Twc.data(), 16);
      std::vector<zmq::const_buffer> frames{
        zmq::buffer(tnsBytes.data(), tnsBytes.size()),
        zmq::buffer(poseBytes.data(), poseBytes.size())
      };
      if (pubs.trackingPub) {
        orbslam3_rgbd_zmq_bridge::sendMultipart(*pubs.trackingPub, "slam/tracking_pose", frames);
      } else if (pubs.singlePub) {
        orbslam3_rgbd_zmq_bridge::sendMultipart(*pubs.singlePub, "slam/tracking_pose", frames);
      }
      {
        FrameCacheEntry entry;
        entry.tNs = tNs;
        entry.Twc = Twc;
        entry.rgbJpeg.assign(jpegBuf.begin(), jpegBuf.end());
        entry.depthRaw.resize(depthMsg.size());
        std::memcpy(entry.depthRaw.data(), depthMsg.data(), depthMsg.size());
        std::lock_guard<std::mutex> lg(frameCacheMutex);
        frameCache.push_back(std::move(entry));
        while (frameCache.size() > kFrameCacheMax) frameCache.pop_front();
      }
      lostStreak = 0;
    } else {
      lostStreak++;
    }
    lastTrackingOk = ok;
  }

  running.store(false);
  if (kfThread.joinable()) kfThread.join();
  return 0;
}


