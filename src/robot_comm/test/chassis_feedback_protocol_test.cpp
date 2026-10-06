#include <array>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "robot_comm/chassis_feedback_protocol.hpp"

namespace feedback = robot_comm::chassis_feedback;

namespace
{

void encodeLittleEndianFloat(float value, std::uint8_t * data)
{
  std::uint32_t bits = 0U;
  std::memcpy(&bits, &value, sizeof(bits));
  data[0] = static_cast<std::uint8_t>(bits & 0xFFU);
  data[1] = static_cast<std::uint8_t>((bits >> 8U) & 0xFFU);
  data[2] = static_cast<std::uint8_t>((bits >> 16U) & 0xFFU);
  data[3] = static_cast<std::uint8_t>((bits >> 24U) & 0xFFU);
}

std::array<std::uint8_t, feedback::kFrameSize> makeFrame(
  float vx, float vy, float vw)
{
  std::array<std::uint8_t, feedback::kFrameSize> frame{};
  frame[0] = feedback::kHeader0;
  frame[1] = feedback::kHeader1;
  encodeLittleEndianFloat(vx, frame.data() + 2U);
  encodeLittleEndianFloat(vy, frame.data() + 6U);
  encodeLittleEndianFloat(vw, frame.data() + 10U);
  frame[14] = feedback::kTail0;
  frame[15] = feedback::kTail1;
  return frame;
}

}  // namespace

TEST(ChassisFeedbackProtocol, DecodesCompleteFrame)
{
  const auto frame = makeFrame(1.25F, -0.75F, 2.5F);
  feedback::VelocitySample sample;
  ASSERT_TRUE(feedback::decodeFrame(frame.data(), &sample));
  EXPECT_FLOAT_EQ(sample.vx, 1.25F);
  EXPECT_FLOAT_EQ(sample.vy, -0.75F);
  EXPECT_FLOAT_EQ(sample.vw, 2.5F);
}

TEST(ChassisFeedbackProtocol, ReassemblesFragmentedFrame)
{
  const auto frame = makeFrame(0.1F, 0.2F, -0.3F);
  feedback::StreamParser parser;
  EXPECT_TRUE(parser.append(frame.data(), 5U).empty());
  const auto samples =
    parser.append(frame.data() + 5U, frame.size() - 5U);
  ASSERT_EQ(samples.size(), 1U);
  EXPECT_FLOAT_EQ(samples.front().vx, 0.1F);
  EXPECT_FLOAT_EQ(samples.front().vy, 0.2F);
  EXPECT_FLOAT_EQ(samples.front().vw, -0.3F);
}

TEST(ChassisFeedbackProtocol, RecoversAfterNoiseAndInvalidFrame)
{
  auto invalid = makeFrame(7.0F, 8.0F, 9.0F);
  invalid[14] = 0x00U;
  const auto valid = makeFrame(-1.0F, 2.0F, -3.0F);
  std::vector<std::uint8_t> stream{0x00U, 0xA5U, 0x00U, 0x51U};
  stream.insert(stream.end(), invalid.begin(), invalid.end());
  stream.insert(stream.end(), valid.begin(), valid.end());

  feedback::StreamParser parser;
  const auto samples = parser.append(stream.data(), stream.size());
  ASSERT_EQ(samples.size(), 1U);
  EXPECT_FLOAT_EQ(samples.front().vx, -1.0F);
  EXPECT_FLOAT_EQ(samples.front().vy, 2.0F);
  EXPECT_FLOAT_EQ(samples.front().vw, -3.0F);
}

TEST(ChassisFeedbackProtocol, PreservesSplitHeader)
{
  const auto frame = makeFrame(0.4F, 0.5F, 0.6F);
  feedback::StreamParser parser;
  const std::array<std::uint8_t, 2U> first_chunk{0x00U, 0xA5U};
  EXPECT_TRUE(
    parser.append(first_chunk.data(), first_chunk.size()).empty());
  const auto samples =
    parser.append(frame.data() + 1U, frame.size() - 1U);
  ASSERT_EQ(samples.size(), 1U);
  EXPECT_FLOAT_EQ(samples.front().vx, 0.4F);
}

TEST(ChassisFeedbackProtocol, RejectsNonFinitePayload)
{
  const auto invalid = makeFrame(
    std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F);
  const auto valid = makeFrame(0.4F, 0.5F, 0.6F);
  std::vector<std::uint8_t> stream(invalid.begin(), invalid.end());
  stream.insert(stream.end(), valid.begin(), valid.end());

  feedback::StreamParser parser;
  const auto samples = parser.append(stream.data(), stream.size());
  ASSERT_EQ(samples.size(), 1U);
  EXPECT_FLOAT_EQ(samples.front().vx, 0.4F);
}
