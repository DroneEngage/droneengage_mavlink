#ifndef FCB_PRECLAND_MANAGER_H_
#define FCB_PRECLAND_MANAGER_H_

/**
 * @file fcb_precland_manager.hpp
 * @brief Receives semantic TYPE_AndruavMessage_PRECLAND_TARGET / _STATUS from
 *        de_precland and emits MAVLink LANDING_TARGET to the flight controller
 *        behind a safety gate. de_mavlink remains the sole owner of the FC link.
 */

#include <cstdint>
#include <string>
#include <vector>

#include "../de_common/de_databus/messages.hpp"

namespace de {
namespace fcb {
namespace precland {

class CPreclandManager {
public:
  static CPreclandManager &getInstance() {
    static CPreclandManager instance;

    return instance;
  }

  CPreclandManager(CPreclandManager const &) = delete;
  void operator=(CPreclandManager const &) = delete;

private:
  CPreclandManager() {}

public:
  ~CPreclandManager() {}

public:
  void init();
  void onPreclandTarget(const double x, const double y, const double z,
                        const double ax, const double ay, const int n_tags,
                        const double rmse, const int64_t capture_time_us,
                        const bool valid, const int target_num);
  void onPreclandStatus(const int state);
  void reloadParametersIfConfigChanged();

public:
  inline int getState() const { return m_state; }
  inline const std::string &getGateReason() const { return m_gate_reason; }
  inline uint64_t getGateRejectCount() const { return m_gate_reject_count; }

private:
  void readConfigParameters();

  /**
   * @brief §C3 safety gate. Refuse to emit LANDING_TARGET unless all hold:
   * enabled, armed, allowed flight mode, fresh capture timestamp, send rate,
   * min tags, valid position, and tracker not actively controlling.
   * On rejection sets m_gate_reason and increments m_gate_reject_count.
   */
  bool fn_isGateOpen(const int64_t capture_time_us, const int n_tags,
                     const bool valid);

private:
  bool m_enabled = false; // default OFF - opt-in, safety-critical path
  std::vector<int> m_allowed_modes; // default: LAND + RTL
  int64_t m_max_target_age_us = 300000; // default 300 ms
  int64_t m_min_send_interval_us = 100000; // 1 / max_send_hz, default 10 Hz
  int m_min_tags = 1;
  uint8_t m_frame = 12; // MAV_FRAME_BODY_FRD (8 = MAV_FRAME_BODY_NED)

  int64_t m_last_send_time_us = 0;
  int m_state = PRECLAND_STATUS_DISABLED;
  std::string m_gate_reason;
  uint64_t m_gate_reject_count = 0;
  uint64_t m_sent_count = 0;
};

} // namespace precland
} // namespace fcb
} // namespace de

#endif
