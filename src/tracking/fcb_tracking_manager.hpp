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

typedef enum TRACKING_TYPE {
  TRACKING_TARGET = 0,
  TRACKING_FOLLOW_ME = 1,
  TRACKING_STANDING = 2       // when camera is looking at ground vertically on a drone.
} TRACKING_TYPE;

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
  void onTrack(const double x, const double yz, const bool is_forward_camera);
  void onStatusChanged(const int status);
  void reloadParametersIfConfigChanged();

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

  void trackingFollowMe(const double tracking_x, const double tracking_yz);
  void trackingTarget(const double tracking_x, const double tracking_yz); 
  void trackingStanding(const double tracking_x, const double tracking_yz);
  void trackingDroneForward(const double tracking_x, const double tracking_yz);


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
  double m_x_PID_D = 0.0;
  double m_yz_PID_D = 0.0;
  double m_alpha = 0.1;

  // Per-axis shaping (preferred if configured)
  double m_deadband_x = 0.01;
  double m_deadband_yz = 0.01;
  double m_expo_x = 0.3;
  double m_expo_yz = 0.3;
  double m_rate_limit = 0.05; // max change per update in normalized units

  double m_prev_dx = 0.0;
  double m_prev_dy = 0.0;
  bool m_prev_initialized = false;

  // Optional center-hold for plane pitch
  bool m_center_hold_enabled = false;
  double m_center_hold_y_band = 0.02;
  double m_center_hold_decay = 0.02;
  double m_pitch_hold = 0.0;

  std::chrono::high_resolution_clock::time_point m_last_message_time;

  TRACKING_TYPE m_tracking_type = TRACKING_STANDING;
};
} // namespace tracking
} // namespace fcb
} // namespace de

#endif