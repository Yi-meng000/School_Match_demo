#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

namespace robot_comm
{
namespace chassis_feedback
{

constexpr std::size_t kFrameSize = 16U;
constexpr std::uint8_t kHeader0 = 0xA5U;
constexpr std::uint8_t kHeader1 = 0x51U;
constexpr std::uint8_t kTail0 = 0x15U;
constexpr std::uint8_t kTail1 = 0x5AU;

struct VelocitySample
{
  float vx{0.0F};
  float vy{0.0F};
  float vw{0.0F};
};

inline float decodeLittleEndianFloat(const std::uint8_t * data)
{
  static_assert(
    sizeof(float) == sizeof(std::uint32_t),
    "float32 protocol requires a 32-bit float");
  const std::uint32_t bits =
    static_cast<std::uint32_t>(data[0]) |
    (static_cast<std::uint32_t>(data[1]) << 8U) |
    (static_cast<std::uint32_t>(data[2]) << 16U) |
    (static_cast<std::uint32_t>(data[3]) << 24U);
  float value = 0.0F;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

inline bool decodeFrame(const std::uint8_t * frame, VelocitySample * sample)
{
  if (frame == nullptr || sample == nullptr ||
    frame[0] != kHeader0 || frame[1] != kHeader1 ||
    frame[14] != kTail0 || frame[15] != kTail1)
  {
    return false;
  }

  VelocitySample decoded;
  decoded.vx = decodeLittleEndianFloat(frame + 2U);
  decoded.vy = decodeLittleEndianFloat(frame + 6U);
  decoded.vw = decodeLittleEndianFloat(frame + 10U);
  if (!std::isfinite(decoded.vx) || !std::isfinite(decoded.vy) ||
    !std::isfinite(decoded.vw))
  {
    return false;
  }

  *sample = decoded;
  return true;
}

class StreamParser
{
public:
  std::vector<VelocitySample> append(
    const std::uint8_t * data, std::size_t size)
  {
    if (data != nullptr && size > 0U) {
      buffer_.insert(buffer_.end(), data, data + size);
    }

    std::vector<VelocitySample> samples;
    while (true) {
      const auto header = std::search(
        buffer_.begin(), buffer_.end(), kHeader.begin(), kHeader.end());
      if (header == buffer_.end()) {
        const bool keep_header_prefix =
          !buffer_.empty() && buffer_.back() == kHeader0;
        buffer_.clear();
        if (keep_header_prefix) {
          buffer_.push_back(kHeader0);
        }
        break;
      }

      if (header != buffer_.begin()) {
        buffer_.erase(buffer_.begin(), header);
      }
      if (buffer_.size() < kFrameSize) {
        break;
      }

      VelocitySample sample;
      if (decodeFrame(buffer_.data(), &sample)) {
        samples.push_back(sample);
        buffer_.erase(buffer_.begin(), buffer_.begin() + kFrameSize);
      } else {
        // Move by one byte and search again. This recovers from startup in the
        // middle of a frame, dropped bytes, and false header bytes in payloads.
        buffer_.erase(buffer_.begin());
      }
    }

    return samples;
  }

  std::size_t bufferedBytes() const
  {
    return buffer_.size();
  }

private:
  static constexpr std::array<std::uint8_t, 2U> kHeader{
    kHeader0, kHeader1};
  std::vector<std::uint8_t> buffer_;
};

}  // namespace chassis_feedback
}  // namespace robot_comm
