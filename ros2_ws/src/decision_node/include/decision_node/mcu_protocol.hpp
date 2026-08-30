#pragma once

// ROS-independent HK serial protocol core.  The layout and CRC lookup tables
// are copied verbatim from the ROS1 source-of-truth header.
#include "decision_node/mcu_protocol_legacy_tables.hpp"

#include <cstdint>
#include <vector>

namespace decision_node::mcu
{

struct DecodedGameData
{
  uint8_t game_progress{};
  uint8_t occupy_status{};
  uint8_t robot_id{};
  uint8_t robot_color{};
  uint16_t red_1_hp{}, red_3_hp{}, red_7_hp{}, red_dead_bits{};
  uint16_t blue_1_hp{}, blue_3_hp{}, blue_7_hp{}, blue_dead_bits{};
  double enemy_hero_x{}, enemy_hero_y{}, enemy_engineer_x{}, enemy_engineer_y{};
  double enemy_std3_x{}, enemy_std3_y{}, enemy_std4_x{}, enemy_std4_y{};
  double enemy_sentry_x{}, enemy_sentry_y{};
  uint8_t suggested_target{};
  uint16_t radar_flags{};
  uint8_t can_free_revive{}, can_instant_revive{};
  uint16_t self_hp{}, self_max_hp{}, bullet_remain{};
  float operator_x{}, operator_y{};
  uint8_t hurt_info{};
};

class GameFrameParser
{
public:
  std::vector<DecodedGameData> push(const uint8_t* data, std::size_t size);
  void reset();

private:
  bool decodeFront(DecodedGameData& output) const;
  std::vector<uint8_t> buffer_;
};

std::vector<uint8_t> makeNavigationFrame(
  uint8_t sequence, uint8_t heroes_never_die, float vx_mps, float vy_mps, float wz_rads);
std::vector<uint8_t> makeMotionFrame(
  uint8_t motion_mode, uint8_t hp_up, uint8_t bullet_up, uint8_t bullet_num);

}  // namespace decision_node::mcu
