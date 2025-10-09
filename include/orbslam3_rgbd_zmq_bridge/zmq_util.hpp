#pragma once

#include <string>
#include <vector>
#include <array>
#include <cstdint>
#include <optional>
#include <memory>
#include <zmq.hpp>

namespace orbslam3_rgbd_zmq_bridge {

struct PublisherEndpoints {
  std::string singlePubEndpoint; // if empty, use dedicated below
  std::optional<std::string> trackingEndpoint;
  std::optional<std::string> kfPoseEndpoint;
};

struct PubSockets {
  std::unique_ptr<zmq::socket_t> singlePub;
  std::unique_ptr<zmq::socket_t> trackingPub;
  std::unique_ptr<zmq::socket_t> kfPosePub;
};

void bindOrConnect(zmq::socket_t& socket, const std::string& endpoint, bool bindMode);

zmq::socket_t makeSub(zmq::context_t& ctx, const std::string& endpoint, const std::string& topic, bool bindMode);

PubSockets makePubs(zmq::context_t& ctx, const PublisherEndpoints& eps, bool bindMode);

// Helpers for LE packing
std::array<std::uint8_t, 8> packUint64LE(std::uint64_t v);
std::vector<std::uint8_t> packFloat32ArrayLE(const float* data, std::size_t count);

// Send multipart [topic][payload...]
void sendMultipart(zmq::socket_t& sock, const std::string& topic, const std::vector<zmq::const_buffer>& frames);

} // namespace orbslam3_rgbd_zmq_bridge


