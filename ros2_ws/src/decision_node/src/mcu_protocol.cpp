#include "decision_node/mcu_protocol.hpp"

#include <algorithm>
#include <cstring>

namespace decision_node::mcu
{
namespace
{
uint16_t littleEndianU16(const uint8_t* bytes)
{
  return static_cast<uint16_t>(bytes[0]) | (static_cast<uint16_t>(bytes[1]) << 8U);
}

int16_t saturatedScaled(const float value, const float scale)
{
  const auto scaled = static_cast<int32_t>(value * scale);  // ROS1 truncation semantics.
  return static_cast<int16_t>(std::clamp(scaled, int32_t{-32768}, int32_t{32767}));
}
}  // namespace

std::vector<DecodedGameData> GameFrameParser::push(const uint8_t* data, const std::size_t size)
{
  buffer_.insert(buffer_.end(), data, data + size);
  std::vector<DecodedGameData> decoded;

  while (true) {
    constexpr uint8_t kSof[] = {HK_FRAME_SOF_H, HK_FRAME_SOF_K};
    const auto sof = std::search(buffer_.begin(), buffer_.end(), std::begin(kSof), std::end(kSof));
    if (sof == buffer_.end()) {
      if (!buffer_.empty() && buffer_.back() == HK_FRAME_SOF_H) {
        buffer_.erase(buffer_.begin(), buffer_.end() - 1);
      } else {
        buffer_.clear();
      }
      break;
    }
    if (sof != buffer_.begin()) buffer_.erase(buffer_.begin(), sof);
    if (buffer_.size() < sizeof(HKFrameHeader)) break;

    const auto declared_length = littleEndianU16(buffer_.data() + 2);
    const bool valid_header = declared_length == sizeof(MCUDataFrame) &&
      buffer_[4] == HK_PACKET_TYPE_GAME &&
      calculateCRC8(buffer_.data(), 8, 0xFF) == buffer_[8];
    if (!valid_header) {
      buffer_.erase(buffer_.begin());
      continue;
    }
    if (buffer_.size() < sizeof(MCUDataFrame)) break;

    DecodedGameData output;
    const bool valid_packet = decodeFront(output);
    if (valid_packet) decoded.push_back(output);
    // Advance one byte on a bad packet as well, allowing an overlapping HK to resync.
    buffer_.erase(buffer_.begin(), buffer_.begin() + (valid_packet ? sizeof(MCUDataFrame) : 1));
  }
  return decoded;
}

void GameFrameParser::reset()
{
  buffer_.clear();
}

bool GameFrameParser::decodeFront(DecodedGameData& output) const
{
  if (buffer_.size() < sizeof(MCUDataFrame) ||
      buffer_[sizeof(MCUDataFrame) - 2] != HK_FRAME_TRAILER_K ||
      buffer_[sizeof(MCUDataFrame) - 1] != HK_FRAME_TRAILER_H) {
    return false;
  }
  if (calculateCRC16(buffer_.data(), sizeof(HKFrameHeader) + sizeof(HKGameData), 0xFFFF) !=
      littleEndianU16(buffer_.data() + sizeof(HKFrameHeader) + sizeof(HKGameData))) {
    return false;
  }

  MCUDataFrame frame{};
  std::memcpy(&frame, buffer_.data(), sizeof(frame));
  const auto& d = frame.data;
  output.game_progress = d.game_progress;
  output.occupy_status = d.occupy_status;
  output.robot_id = d.robot_id;
  output.robot_color = d.robot_color;
  output.red_1_hp = d.red_1_hp; output.red_3_hp = d.red_3_hp; output.red_7_hp = d.red_7_hp;
  output.red_dead_bits = d.red_dead_bits;
  output.blue_1_hp = d.blue_1_hp; output.blue_3_hp = d.blue_3_hp; output.blue_7_hp = d.blue_7_hp;
  output.blue_dead_bits = d.blue_dead_bits;
  output.enemy_hero_x = d.enemy_hero_x / 100.0; output.enemy_hero_y = d.enemy_hero_y / 100.0;
  output.enemy_engineer_x = d.enemy_engineer_x / 100.0; output.enemy_engineer_y = d.enemy_engineer_y / 100.0;
  output.enemy_std3_x = d.enemy_std3_x / 100.0; output.enemy_std3_y = d.enemy_std3_y / 100.0;
  output.enemy_std4_x = d.enemy_std4_x / 100.0; output.enemy_std4_y = d.enemy_std4_y / 100.0;
  output.enemy_sentry_x = d.enemy_sentry_x / 100.0; output.enemy_sentry_y = d.enemy_sentry_y / 100.0;
  output.suggested_target = d.suggested_target; output.radar_flags = d.radar_flags;
  output.can_free_revive = d.can_free_revive; output.can_instant_revive = d.can_instant_revive;
  output.self_hp = d.self_hp; output.self_max_hp = d.self_max_hp; output.bullet_remain = d.bullet_remain;
  output.operator_x = d.operator_x; output.operator_y = d.operator_y; output.hurt_info = d.hurt_info;
  return true;
}

std::vector<uint8_t> makeNavigationFrame(
  const uint8_t sequence, const uint8_t heroes_never_die, const float vx_mps, const float vy_mps, const float wz_rads)
{
  NavigationCommandFrame frame{};
  frame.header.sof[0] = HK_FRAME_SOF_H; frame.header.sof[1] = HK_FRAME_SOF_K;
  frame.header.length = sizeof(frame); frame.header.packet_type = HK_PACKET_TYPE_NAV;
  frame.header.packet_seq = sequence;
  frame.header.header_crc8 = calculateCRC8(reinterpret_cast<const uint8_t*>(&frame.header), 8, 0xFF);
  frame.data.heroes_never_die = heroes_never_die;
  frame.data.vx = saturatedScaled(vx_mps, 1000.0F);
  frame.data.vy = saturatedScaled(vy_mps, 1000.0F);
  frame.data.wz = saturatedScaled(wz_rads, 100.0F);
  frame.packet_crc16 = calculateCRC16(reinterpret_cast<const uint8_t*>(&frame), sizeof(frame) - 4, 0xFFFF);
  frame.trailer[0] = HK_FRAME_TRAILER_K; frame.trailer[1] = HK_FRAME_TRAILER_H;
  const auto* first = reinterpret_cast<const uint8_t*>(&frame);
  return {first, first + sizeof(frame)};
}

std::vector<uint8_t> makeMotionFrame(
  const uint8_t motion_mode, const uint8_t hp_up, const uint8_t bullet_up, const uint8_t bullet_num)
{
  MotionCommandFrame frame{};
  frame.sof = MOTION_FRAME_SOF; frame.motion_mode_up = motion_mode; frame.hp_up = hp_up;
  frame.bullet_up = bullet_up; frame.bullet_num = bullet_num;
  frame.crc8 = calculateCRC8(reinterpret_cast<const uint8_t*>(&frame), sizeof(frame) - 2, 0xFF);
  frame.eof = MCU_FRAME_EOF;
  const auto* first = reinterpret_cast<const uint8_t*>(&frame);
  return {first, first + sizeof(frame)};
}

}  // namespace decision_node::mcu
