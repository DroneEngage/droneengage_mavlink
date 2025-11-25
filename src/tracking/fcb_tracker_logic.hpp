#ifndef FCB_TRACKER_LOGIC_HPP
#define FCB_TRACKER_LOGIC_HPP

#include "pic_controller.hpp"
#include <chrono> // For high-resolution timing
#include <iostream>

#include "../de_common/de_databus/messages.hpp"
#include "kalman_filter.hpp"
#include "pic_controller.hpp"

#include "../de_common/helpers/json_nlohmann.hpp"
using Json_de = nlohmann::json;

namespace de {
namespace fcb {
namespace tracking {
typedef enum TRACKING_TYPE {
  TRACKING_TARGET = 0,
  TRACKING_FOLLOW_ME = 1,
  TRACKING_STANDING =
      2 // when camera is looking at ground vertically on a drone.
} TRACKING_TYPE;

class CTrackerLogic {
public:
  CTrackerLogic();
  virtual ~CTrackerLogic();

  virtual void init();
  virtual void onTrack(const double x, const double yz,
                       const bool is_forward_camera);
  virtual void onStatusChanged(const int status);
  virtual void reloadParametersIfConfigChanged();


public:
  inline void setParameters(const double x_PID_P, const double yz_PID_P,
                            const double x_PID_I, const double yz_PID_I) {
    m_x_PID_P = x_PID_P;
    m_yz_PID_P = yz_PID_P;
    m_x_PID_I = x_PID_I;
    m_yz_PID_I = yz_PID_I;
  }

protected:
  virtual void readConfigParameters() = 0;

  void processTrackingInput(const double raw_x, const double raw_yz,
                            const bool is_forward_camera);


protected:
  double clampStepDt(const double dt, const double prev,
                     const double cur) const;
  double applyDeadbandX(const double v) const;
  double applyDeadbandYZ(const double v) const;
  double expoX(const double v) const;
  double expoYZ(const double v) const;
  static double round3(const double v);
  double clampInputStep(const double dt, const double prev,
                        const double cur) const;
  static double projectSign(const double err, const double out);

protected:
  CPIDController m_PID_X;
  CPIDController m_PID_YZ;

  // Kalman filters for smooth tracking
  SimpleKalmanFilter m_kalman_x;
  SimpleKalmanFilter m_kalman_yz;

  bool m_prev_initialized = false;

  std::chrono::high_resolution_clock::time_point m_last_message_time;

protected:
  TRACKING_TYPE m_tracking_type = TRACKING_TARGET;

  double m_raw_x = 0.0;
  double m_raw_yz = 0.0;

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

  // Optional center-hold for plane pitch
  bool m_center_hold_enabled = false;
  double m_center_hold_y_band = 0.02;
  double m_center_hold_decay = 0.02;
  double m_pitch_hold = 0.0;

  // Kalman filter tuning parameters
  double m_kalman_process_noise_q = 0.01;
  double m_kalman_measurement_noise_r = 0.1;
  bool m_kalman_enabled = true;
};
} // namespace tracking
} // namespace fcb
} // namespace de
#endif // FCB_TRACKER_LOGIC_HPP