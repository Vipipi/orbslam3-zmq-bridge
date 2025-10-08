#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>
#include <optional>
#include <opencv2/core.hpp>

namespace orbslam3_rgbd_zmq_bridge {

struct SlamInitParams {
  std::string vocPath;
  std::string settingsPath;
};

struct KeyframePose {
  std::uint64_t id;       // surrogate if real id not available
  std::uint64_t tNs;      // timestamp if available, else 0
  std::array<float, 16> TwcRowMajor;
};

class SlamRunner {
 public:
  explicit SlamRunner(const SlamInitParams& params);
  ~SlamRunner();

  bool isOk() const;

  // Track one RGB-D frame; returns true if pose available
  bool trackRgbd(const cv::Mat& rgbBgr8, const cv::Mat& depthF32, std::uint64_t tNs,
                 std::array<float, 16>& TwcRowMajorOut);

  // Return all keyframe poses if supported; otherwise empty
  std::vector<KeyframePose> getAllKeyframePoses();

 private:
  class Impl;
  Impl* impl_ = nullptr;
  bool ok_ = true;
};

}  // namespace orbslam3_rgbd_zmq_bridge


