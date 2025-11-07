#ifndef FCB_TRACKING_MANAGER_H_
#define FCB_TRACKING_MANAGER_H_

#include <chrono> // For high-resolution timing
#include <iostream>
#include <mavlink_sdk.h>

#include "pic_controller.hpp"

#include "../de_common/helpers/json_nlohmann.hpp"
using Json_de = nlohmann::json;

namespace de {
namespace fcb {
namespace tracking {
class CTrackingManager {
public:
  static CTrackingManager &getInstance() {
    static CTrackingManager instance;

    return instance;
  }

  CTrackingManager(CTrackingManager const &) = delete;
  void operator=(CTrackingManager const &) = delete;

private:
  CTrackingManager() {}

public:
  ~CTrackingManager() {}

public:
  void init();
  void onTrack(const double x, const double yz, const bool is_xy);
  void onStatusChanged(const int status);

public:
  inline void setParameters(const double x_PID_P, const double yz_PID_P,
                            const double x_PID_I, const double yz_PID_I) {
    m_x_PID_P = x_PID_P;
    m_yz_PID_P = yz_PID_P;
    m_x_PID_I = x_PID_I;
    m_yz_PID_I = yz_PID_I;
   
  }

private:
  void readConfigParameters();

private:
  bool m_tracking_running = false;
  bool m_object_detected = false;

  CPIDController m_PID_X;
  CPIDController m_PID_YZ;

  double m_x;
  double m_yz;
  double m_x_PID_P = 1.0;
  double m_yz_PID_P = 1.0;
  double m_x_PID_I = 1.0;
  double m_yz_PID_I = 1.0;
  double m_alpha = 0.1;

  // Legacy single-axis settings (used as fallback)
  double m_expo_factor = 0.3;
  double m_deadband = 0.01;

  // Per-axis shaping (preferred if configured)
  double m_deadband_x  = 0.01;
  double m_deadband_yz = 0.01;
  double m_expo_x      = 0.3;
  double m_expo_yz     = 0.3;
  double m_rate_limit = 0.05; // max change per update in normalized units

  double m_prev_dx = 0.0;
  double m_prev_dy = 0.0;
  bool m_prev_initialized = false;

  std::chrono::high_resolution_clock::time_point m_last_message_time;

};
} // namespace tracking
} // namespace fcb
} // namespace de

#endif