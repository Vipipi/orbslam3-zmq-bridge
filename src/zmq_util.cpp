#include "orbslam3_rgbd_zmq_bridge/zmq_util.hpp"

#include <cstring>
#include <stdexcept>

namespace orbslam3_rgbd_zmq_bridge {

void bindOrConnect(zmq::socket_t& socket, const std::string& endpoint, bool bindMode) {
  if (bindMode) {
    socket.bind(endpoint);
  } else {
    socket.connect(endpoint);
  }
}

zmq::socket_t makeSub(zmq::context_t& ctx, const std::string& endpoint, const std::string& topic, bool bindMode) {
  zmq::socket_t sub(ctx, zmq::socket_type::sub);
  bindOrConnect(sub, endpoint, bindMode);
  sub.set(zmq::sockopt::subscribe, topic);
  return sub;
}

PubSockets makePubs(zmq::context_t& ctx, const PublisherEndpoints& eps, bool bindMode) {
  PubSockets pubs;
  if (!eps.singlePubEndpoint.empty()) {
    pubs.singlePub = std::make_unique<zmq::socket_t>(ctx, zmq::socket_type::pub);
    bindOrConnect(*pubs.singlePub, eps.singlePubEndpoint, bindMode);
  }
  if (eps.trackingEndpoint.has_value()) {
    pubs.trackingPub = std::make_unique<zmq::socket_t>(ctx, zmq::socket_type::pub);
    bindOrConnect(*pubs.trackingPub, *eps.trackingEndpoint, bindMode);
  }
  if (eps.kfPoseEndpoint.has_value()) {
    pubs.kfPosePub = std::make_unique<zmq::socket_t>(ctx, zmq::socket_type::pub);
    bindOrConnect(*pubs.kfPosePub, *eps.kfPoseEndpoint, bindMode);
  }
  return pubs;
}

std::array<std::uint8_t, 8> packUint64LE(std::uint64_t v) {
  std::array<std::uint8_t, 8> b{};
  for (int i = 0; i < 8; ++i) b[i] = static_cast<std::uint8_t>((v >> (8 * i)) & 0xFF);
  return b;
}

std::vector<std::uint8_t> packFloat32ArrayLE(const float* data, std::size_t count) {
  std::vector<std::uint8_t> bytes(count * sizeof(float));
  // Assuming host is LE (common on x86). For other archs, add byte-swap.
  std::memcpy(bytes.data(), data, count * sizeof(float));
  return bytes;
}

void sendMultipart(zmq::socket_t& sock, const std::string& topic, const std::vector<zmq::const_buffer>& frames) {
  zmq::message_t topicMsg(topic.data(), topic.size());
  const bool more = !frames.empty();
  sock.send(topicMsg, more ? zmq::send_flags::sndmore : zmq::send_flags::none);
  for (std::size_t i = 0; i < frames.size(); ++i) {
    const bool last = (i + 1 == frames.size());
    zmq::message_t part(frames[i].size());
    std::memcpy(part.data(), frames[i].data(), frames[i].size());
    sock.send(part, last ? zmq::send_flags::none : zmq::send_flags::sndmore);
  }
}

} // namespace orbslam3_rgbd_zmq_bridge


