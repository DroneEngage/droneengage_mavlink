#ifndef FCB_TRACKER_QUAD_LOGIC_H_
#define FCB_TRACKER_QUAD_LOGIC_H_

#include "fcb_tracker_logic.hpp"
#include "../de_pilot/advanced_pid_controller.hpp"
#include <cstdint>

namespace de {
namespace fcb {
namespace tracking {

class CTrackerQuadLogic : public CTrackerLogic {
public:
  static CTrackerQuadLogic &getInstance() {
    static CTrackerQuadLogic instance;

    return instance;
  }

  CTrackerQuadLogic(CTrackerQuadLogic const &) = delete;
  void operator=(CTrackerQuadLogic const &) = delete;

private:
  CTrackerQuadLogic() 
    : m_standing_pid_x(0.5, 0.0, 0.0, 0.01, 100.0, 500.0, 200.0, true, 0.2),
      m_standing_pid_yz(0.5, 0.0, 0.0, 0.01, 100.0, 500.0, 200.0, true, 0.2) {}

public:
  ~CTrackerQuadLogic() {}

public:
  void onStatusChanged(const int status, const uint8_t tracking_camera_direction = 0, const bool ai_priority = false) override;
  
  void onTrack(const double x, const double yz) override;

  
public:
  void readConfigParameters() override;
  void reloadParametersIfConfigChanged() override;

  void trackingStanding(const double x, const double yz, const double tracking_x, const double tracking_yz);
  void trackingDroneForward(const double x, const double yz, const double tracking_x, const double tracking_yz);

  inline void resetTrackerCopterStatus() {
    m_tracker_copter_status = 0;
    // Reset advanced PID controllers for standing mode
    m_standing_pid_x.reset();
    m_standing_pid_yz.reset();
    // Reset rate tracking
    m_last_x_for_rate = 0.0;
    m_last_yz_for_rate = 0.0;
    m_last_rate_time = 0;
    m_current_rate_x = 0.0;
    m_current_rate_yz = 0.0;
  }

  inline void setTrackerCopterStatus(uint8_t status)
  {
    m_tracker_copter_status = status;
  }

  inline void addTrackerCopterStatus(uint8_t status)
  {
    m_tracker_copter_status |= status;
  }

  inline bool isTrackerCopterStatus(uint8_t status)
  {
    return m_tracker_copter_status & status;
  }

private:
  bool m_loose_altitude = false;

  uint8_t m_tracker_copter_status = 0;

  // Advanced PID controllers for standing tracking mode (camera looking UP)
  de::fcb::depilot::CAdvancedPIDController m_standing_pid_x;
  de::fcb::depilot::CAdvancedPIDController m_standing_pid_yz;

  // Configuration parameters for standing tracking mode
  double m_standing_pid_p_x = 0.5;
  double m_standing_pid_i_x = 0.0;
  double m_standing_pid_d_x = 0.0;
  double m_standing_ff_scale_x = 200.0;
  double m_standing_max_accel_x = 3.0;
  double m_standing_max_rate_x = 1.5;
  double m_standing_deadband_x = 0.02;
  double m_standing_integral_limit_x = 100.0;
  double m_standing_output_limit_x = 500.0;

  double m_standing_pid_p_yz = 0.5;
  double m_standing_pid_i_yz = 0.0;
  double m_standing_pid_d_yz = 0.0;
  double m_standing_ff_scale_yz = 200.0;
  double m_standing_max_accel_yz = 2.5;
  double m_standing_max_rate_yz = 1.2;
  double m_standing_deadband_yz = 0.02;
  double m_standing_integral_limit_yz = 100.0;
  double m_standing_output_limit_yz = 500.0;

  // Rate tracking for standing mode
  double m_last_x_for_rate = 0.0;
  double m_last_yz_for_rate = 0.0;
  uint64_t m_last_rate_time = 0;
  double m_current_rate_x = 0.0;
  double m_current_rate_yz = 0.0;
};
} // namespace tracking
} // namespace fcb
} // namespace de

#endif
