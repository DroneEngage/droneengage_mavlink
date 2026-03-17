#ifndef FCB_DE_PILOT_STABILIZATION_H_
#define FCB_DE_PILOT_STABILIZATION_H_

#include "fcb_de_pilot_operation_base.hpp"
#include <cstdint>
#include "advanced_pid_controller.hpp"

namespace de {
namespace fcb {
namespace depilot {

class CDEPilotStabilization : public CDEPilotOperationBase {
public:
  static CDEPilotStabilization &getInstance() {
    static CDEPilotStabilization instance;
    return instance;
  }

  CDEPilotStabilization(CDEPilotStabilization const &) = delete;
  void operator=(CDEPilotStabilization const &) = delete;

private:
  CDEPilotStabilization() 
    : m_yaw_pid_controller(15.0, 0.5, 0.1, 0.01, 500.0, 600.0, 143.2, true, 0.2) {
    // Initialize with default yaw PID parameters
    // Parameters: kp, ki, kd, dt, integral_max, max_min_value, feedforward_gain, advanced_antiwindup, derivative_filter_alpha
    // integral_max: 500.0 (matches original yaw_integral_limit * 10 scale factor from original code)
    // max_min_value: 600.0 (allows ±300 PWM from center, matching original 1200-1800 range)
    // Will be reconfigured in readConfigParameters()
  }

public:
  ~CDEPilotStabilization() {}

  // Base class interface implementation
  void init() override;
  void update() override;
  void uninit() override;
  void readConfigParameters() override;
  void setPhase(int phase) override;
  int getPhase() const override;
  void setActive(bool active) override;
  bool getActive() const override;
  bool isCompleted() override;

  // Class-specific interface
  void startStabilization();
  void updateStabilization();
  void stopStabilization();
  bool isStabilizationActive() const;

  // Yaw control interface
  void setYawTarget(double angle, double rate, bool is_clockwise,
                    bool is_relative);
  void clearYawTarget();
  bool isYawControlActive() const;

private:
  // Helper to compute desired yaw rate using sqrt_controller
  float calculateDesiredYawRate(double angle_error_rad) const;

private:
  enum StabilizationPhase { PHASE_IDLE, PHASE_STABILIZING, PHASE_COMPLETE };

  // Stabilization-specific member variables (base class provides m_active,
  // m_generic_phase, m_phase_start_time, m_last_update_time)
  StabilizationPhase m_phase = PHASE_IDLE;

  // Yaw control member variables
  bool m_yaw_control_enabled = false;
  double m_target_yaw_angle = 0.0;

  // Yaw PID controller variables (replaced by advanced PID controller)
  // double m_yaw_error = 0.0;           // Now managed by CAdvancedPIDController
  // double m_yaw_error_integral = 0.0;  // Now managed by CAdvancedPIDController
  // double m_yaw_error_derivative = 0.0; // Now managed by CAdvancedPIDController
  // double m_yaw_previous_error = 0.0;  // Now managed by CAdvancedPIDController
  // uint64_t m_yaw_last_time = 0;        // Now managed by CAdvancedPIDController

  // Advanced PID controller for yaw control
  CAdvancedPIDController m_yaw_pid_controller;

  // Yaw rate tracking for rate-based control
  double m_last_heading_for_rate = 0.0;
  uint64_t m_yaw_rate_check_time = 0;
  double m_current_yaw_rate = 0.0;

  // Configuration parameters
  double m_default_yaw_rate = 30.0; // deg/sec

  // Yaw PID parameters
  double m_yaw_p = 15.0;              // Proportional gain
  double m_yaw_i = 0.5;               // Integral gain
  double m_yaw_d = 0.1;               // Derivative gain
  double m_yaw_integral_limit = 50.0; // Integral windup limit
  double m_yaw_max_accel =
      180.0; // deg/s^2 - max angular acceleration for sqrt_controller
  double m_yaw_ff_scale = 429.6;      // PWM per rad/s feedforward (3x original 143.2 for tripled response)
};

} // namespace depilot
} // namespace fcb
} // namespace de

#endif
