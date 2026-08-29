#ifndef FCB_TRACKER_PLAN_LOGIC_H_
#define FCB_TRACKER_PLAN_LOGIC_H_

#include "fcb_tracker_logic.hpp"
#include "../de_pilot/advanced_pid_controller.hpp"
#include <cstdint>

namespace de {
namespace fcb {
namespace tracking {

class CTrackerPlanLogic : public CTrackerLogic {
public:
  static CTrackerPlanLogic &getInstance() {
    static CTrackerPlanLogic instance;

    return instance;
  }

  CTrackerPlanLogic(CTrackerPlanLogic const &) = delete;
  void operator=(CTrackerPlanLogic const &) = delete;

public:
  void onStatusChanged(const int status, const uint8_t tracking_camera_direction = 0, const bool ai_priority = false) override;

  void onTrack(const double x, const double yz) override;

protected:
  void readConfigParameters() override;

private:
  CTrackerPlanLogic()
    : m_forward_pid_x(0.5, 0.0, 0.0, 0.01, 100.0, 500.0, 200.0, false, 0.2),
      m_forward_pid_yz(0.5, 0.0, 0.0, 0.01, 100.0, 500.0, 200.0, false, 0.2) {}

public:
  ~CTrackerPlanLogic() {}

protected:
  void trackingFollowMe(const double x, const double yz);
  void trackingTarget(const double x, const double yz);

private:
  // Advanced PID controllers for forward-camera tracking (plane only has a
  // forward-looking camera).
  de::fcb::depilot::CAdvancedPIDController m_forward_pid_x;
  de::fcb::depilot::CAdvancedPIDController m_forward_pid_yz;

  // Configuration parameters for forward tracking
  double m_forward_pid_p_x = 0.8;
  double m_forward_pid_i_x = 0.0;
  double m_forward_pid_d_x = 0.1;
  double m_forward_ff_scale_x = 200.0;
  double m_forward_max_accel_x = 3.0;
  double m_forward_max_rate_x = 1.5;
  double m_forward_deadband_x = 0.001;
  double m_forward_integral_limit_x = 100.0;
  double m_forward_output_limit_x = 500.0;

  double m_forward_pid_p_yz = 0.6;
  double m_forward_pid_i_yz = 0.1;
  double m_forward_pid_d_yz = 0.5;
  double m_forward_ff_scale_yz = 200.0;
  double m_forward_max_accel_yz = 2.5;
  double m_forward_max_rate_yz = 1.2;
  double m_forward_deadband_yz = 0.001;
  double m_forward_integral_limit_yz = 100.0;
  double m_forward_output_limit_yz = 500.0;

  // Rate tracking for forward mode
  double m_forward_last_x_for_rate = 0.0;
  double m_forward_last_yz_for_rate = 0.0;
  uint64_t m_forward_last_rate_time = 0;
  double m_forward_current_rate_x = 0.0;
  double m_forward_current_rate_yz = 0.0;
};
} // namespace tracking
} // namespace fcb
} // namespace de

#endif