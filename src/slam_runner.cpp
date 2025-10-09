#include "orbslam3_rgbd_zmq_bridge/slam_runner.hpp"

#include <opencv2/opencv.hpp>

#include <System.h>

namespace orbslam3_rgbd_zmq_bridge {

class SlamRunner::Impl {
 public:
  Impl(const SlamInitParams& params) {
    try {
      system_ = new ORB_SLAM3::System(params.vocPath, params.settingsPath, ORB_SLAM3::System::RGBD, false);
    } catch (...) {
      system_ = nullptr;
    }
  }
  ~Impl() {
    if (system_) {
      system_->Shutdown();
      delete system_;
    }
  }

  bool isOk() const { return system_ != nullptr; }

  bool track(const cv::Mat& rgbBgr8, const cv::Mat& depthF32, std::uint64_t tNs,
             std::array<float, 16>& TwcRowMajorOut) {
    if (!system_) return false;
    // ORB-SLAM3 expects Tcw (camera pose in world coordinates)
    Sophus::SE3f Tcw = system_->TrackRGBD(rgbBgr8, depthF32, static_cast<double>(tNs) * 1e-9);
    if (!system_->GetTrackingState() || system_->GetTrackingState() == ORB_SLAM3::Tracking::eTrackingState::LOST) {
      return false;
    }
    Sophus::SE3f Twc = Tcw.inverse();
    Eigen::Matrix4f M = Twc.matrix();
    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 4; ++c) {
        TwcRowMajorOut[r * 4 + c] = M(r, c);
      }
    }
    return true;
  }

  std::vector<KeyframePose> getAllKfPoses() {
    std::vector<KeyframePose> out;
    if (!system_) return out;
#ifdef ENABLE_KF_POLLING
    auto poses = system_->GetAllKeyframePoses();
    out.reserve(poses.size());
    for (size_t i = 0; i < poses.size(); ++i) {
      KeyframePose kf{};
      kf.id = static_cast<std::uint64_t>(i);
      kf.tNs = 0;
      const Eigen::Matrix4f T = poses[i].matrix();
      for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) kf.TwcRowMajor[r * 4 + c] = T(r, c);
      }
      out.emplace_back(kf);
    }
#endif
    return out;
  }

 private:
  ORB_SLAM3::System* system_ = nullptr;
};

SlamRunner::SlamRunner(const SlamInitParams& params) {
  impl_ = new Impl(params);
  ok_ = impl_ && impl_->isOk();
}

SlamRunner::~SlamRunner() {
  delete impl_;
  impl_ = nullptr;
}

bool SlamRunner::isOk() const { return ok_; }

bool SlamRunner::trackRgbd(const cv::Mat& rgbBgr8, const cv::Mat& depthF32, std::uint64_t tNs,
                           std::array<float, 16>& TwcRowMajorOut) {
  if (!impl_ || !impl_->isOk()) return false;
  return impl_->track(rgbBgr8, depthF32, tNs, TwcRowMajorOut);
}

std::vector<KeyframePose> SlamRunner::getAllKeyframePoses() {
  if (!impl_ || !impl_->isOk()) return {};
  return impl_->getAllKfPoses();
}

}  // namespace orbslam3_rgbd_zmq_bridge


