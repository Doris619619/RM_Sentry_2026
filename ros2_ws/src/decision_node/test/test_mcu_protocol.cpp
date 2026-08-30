#include "decision_node/mcu_protocol.hpp"

#include <gtest/gtest.h>

#include <cstring>
#include <vector>

namespace
{
std::vector<uint8_t> gameFrame()
{
  MCUDataFrame frame{};
  frame.header.sof[0] = HK_FRAME_SOF_H; frame.header.sof[1] = HK_FRAME_SOF_K; frame.header.length = sizeof(frame);
  frame.header.packet_type = HK_PACKET_TYPE_GAME; frame.header.packet_seq = 23;
  frame.header.header_crc8 = calculateCRC8(reinterpret_cast<const uint8_t*>(&frame.header), 8);
  frame.data.game_progress = 4; frame.data.occupy_status = 1; frame.data.robot_id = 7; frame.data.robot_color = 1;
  frame.data.red_1_hp = 101; frame.data.blue_7_hp = 707; frame.data.red_dead_bits = 0x12; frame.data.blue_dead_bits = 0x34;
  frame.data.enemy_hero_x = -123; frame.data.enemy_hero_y = 456; frame.data.enemy_sentry_x = -8888; frame.data.enemy_sentry_y = 22;
  frame.data.suggested_target = 4; frame.data.radar_flags = 0x42; frame.data.can_free_revive = 1; frame.data.can_instant_revive = 2;
  frame.data.self_hp = 333; frame.data.self_max_hp = 400; frame.data.bullet_remain = 150; frame.data.operator_x = 1.25F; frame.data.operator_y = -0.5F; frame.data.hurt_info = 0xa4;
  frame.packet_crc16 = calculateCRC16(reinterpret_cast<const uint8_t*>(&frame), sizeof(HKFrameHeader) + sizeof(HKGameData));
  frame.trailer[0] = HK_FRAME_TRAILER_K; frame.trailer[1] = HK_FRAME_TRAILER_H;
  const auto* begin = reinterpret_cast<const uint8_t*>(&frame); return {begin, begin + sizeof(frame)};
}
}

TEST(McuProtocol, SizesAndCrcKnownVectors)
{
  EXPECT_EQ(sizeof(HKFrameHeader), 9U); EXPECT_EQ(sizeof(HKGameData), 65U); EXPECT_EQ(sizeof(MCUDataFrame), 78U);
  EXPECT_EQ(sizeof(NavigationCommandFrame), 21U); EXPECT_EQ(sizeof(MotionCommandFrame), 7U);
  const uint8_t vector[] = {0x48, 0x4b, 0x15, 0x00, 0x02, 0x00, 0x07, 0x00};
  EXPECT_EQ(calculateCRC8(vector, sizeof(vector)), 0x16U);
  EXPECT_EQ(calculateCRC16(vector, sizeof(vector)), 0xa70cU);
}

TEST(McuProtocol, ParsesFragmentedAndNoisyFrames)
{
  const auto valid = gameFrame(); decision_node::mcu::GameFrameParser parser;
  std::vector<uint8_t> noise = {0x00, 0x48, 0x48, 0x7f}; noise.insert(noise.end(), valid.begin(), valid.end());
  std::vector<decision_node::mcu::DecodedGameData> results;
  for (const auto byte : noise) { const auto next = parser.push(&byte, 1); results.insert(results.end(), next.begin(), next.end()); }
  ASSERT_EQ(results.size(), 1U); EXPECT_EQ(results[0].game_progress, 4U); EXPECT_EQ(results[0].self_hp, 333U); EXPECT_EQ(results[0].bullet_remain, 150U);
  EXPECT_DOUBLE_EQ(results[0].enemy_hero_x, -1.23); EXPECT_DOUBLE_EQ(results[0].enemy_sentry_x, -88.88); EXPECT_EQ(results[0].suggested_target, 4U);
}

TEST(McuProtocol, RejectsEveryHeaderAndPacketIntegrityFailureThenResyncs)
{
  const auto valid = gameFrame(); auto bad_length = valid; bad_length[2] = 1; auto bad_header_crc = valid; ++bad_header_crc[8];
  auto bad_type = valid; bad_type[4] = HK_PACKET_TYPE_NAV; auto bad_packet_crc = valid; ++bad_packet_crc[74]; auto bad_trailer = valid; bad_trailer[77] = 0;
  decision_node::mcu::GameFrameParser parser;
  for (const auto& broken : {bad_length, bad_header_crc, bad_type, bad_packet_crc, bad_trailer}) EXPECT_TRUE(parser.push(broken.data(), broken.size()).empty());
  const auto parsed = parser.push(valid.data(), valid.size()); ASSERT_EQ(parsed.size(), 1U); EXPECT_EQ(parsed.front().robot_id, 7U);
}

TEST(McuProtocol, NavigationAndMotionFramesHaveExactLayoutAndSaturation)
{
  const auto nav = decision_node::mcu::makeNavigationFrame(7, 2, 1.234F, -2.5F, 0.75F);
  ASSERT_EQ(nav.size(), 21U); EXPECT_EQ(nav[0], 'H'); EXPECT_EQ(nav[1], 'K'); EXPECT_EQ(nav[2], 21); EXPECT_EQ(nav[4], HK_PACKET_TYPE_NAV); EXPECT_EQ(nav[6], 7); EXPECT_EQ(nav[9], 2);
  EXPECT_EQ(static_cast<int16_t>(nav[11] | (nav[12] << 8)), 1234); EXPECT_EQ(static_cast<int16_t>(nav[13] | (nav[14] << 8)), -2500); EXPECT_EQ(static_cast<int16_t>(nav[15] | (nav[16] << 8)), 75);
  EXPECT_EQ(calculateCRC8(nav.data(), 8), nav[8]); EXPECT_EQ(calculateCRC16(nav.data(), 17), static_cast<uint16_t>(nav[17] | (nav[18] << 8))); EXPECT_EQ(nav[19], 'K'); EXPECT_EQ(nav[20], 'H');
  const auto saturated = decision_node::mcu::makeNavigationFrame(8, 0, 100.0F, -100.0F, 1000.0F); EXPECT_EQ(static_cast<int16_t>(saturated[11] | (saturated[12] << 8)), 32767); EXPECT_EQ(static_cast<int16_t>(saturated[13] | (saturated[14] << 8)), -32768); EXPECT_EQ(static_cast<int16_t>(saturated[15] | (saturated[16] << 8)), 32767);
  const auto motion = decision_node::mcu::makeMotionFrame(3, 0, 1, 150); ASSERT_EQ(motion.size(), 7U); EXPECT_EQ(motion[0], MOTION_FRAME_SOF); EXPECT_EQ(motion[1], 3); EXPECT_EQ(motion[3], 1); EXPECT_EQ(motion[4], 150); EXPECT_EQ(motion[5], calculateCRC8(motion.data(), 5)); EXPECT_EQ(motion[6], MCU_FRAME_EOF);
}
