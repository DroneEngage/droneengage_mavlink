#ifndef FCB_DE_PILOT_YAW_CONTROL_H_
#define FCB_DE_PILOT_YAW_CONTROL_H_

#include "fcb_de_pilot_task_base.hpp"
#include "advanced_pid_controller.hpp"
#include <cstdint>
#include "../defines.hpp"

namespace de {
namespace fcb {
namespace depilot {

class CDEPilotYawControl : public CDEPilotTaskBase {
public:
  static CDEPilotYawControl &getInstance() {
    static CDEPilotYawControl instance;
    return instance;
  }

  CDEPilotYawControl(CDEPilotYawControl const &) = delete;
  void operator=(CDEPilotYawControl const &) = delete;

private:
  CDEPilotYawControl() 
    : m_yaw_pid_controller(8.0, 0.2, 0.5, 0.01, 500.0, 600.0, 120.0, true, 0.2) {
    // Initialize with balanced yaw PID parameters
    // Parameters: kp, ki, kd, dt, integral_max, max_min_value, feedforward_gain, advanced_antiwindup, derivative_filter_alpha
    // kp: 8.0 (balanced for good response without excessive oscillation)
    // ki: 0.2 (moderate integral for steady-state error correction)
    // kd: 0.5 (strong damping to prevent oscillation)
    // feedforward_gain: 120.0 (sufficient for responsive control)
  }

public:
  ~CDEPilotYawControl() {}

  public:
  // Base class interface implementation
  void init() override;
  void update() override;
  void uninit() override;
  void readConfigParameters() override;
  void reloadParametersIfConfigChanged() override;
  void setPhase(int phase) override;
  int getPhase() const override;
  void setActive(bool active) override;
  bool getActive() const override;
  bool isCompleted();

  // Task interface
  std::string getName() const override { return std::string("yaw_control"); }
  DEPILOT_TASK_STATE getTaskState() const override { return m_task_state; }
  void setTaskState(DEPILOT_TASK_STATE state) override { m_task_state = state; }
  std::string getEventContext() const override;

  // YAW control interface
  void setYawTarget(double angle, double rate, bool is_clockwise, bool is_relative);
  void clearYawTarget();
  bool isYawControlActive() const;
  void updateYawControl();
  void applyYawToRCChannels(uint16_t rc_channels[], int flying_mode);

private:
  // Helper to compute desired yaw rate using sqrt_controller
  float calculateDesiredYawRate(double angle_error_rad) const;

private:
  enum YawControlPhase { PHASE_IDLE, PHASE_ACTIVE, PHASE_COMPLETE };

  // YAW control phase
  YawControlPhase m_phase = PHASE_IDLE;

  // Yaw control member variables
  bool m_yaw_control_enabled = false;
  double m_target_yaw_angle = 0.0;
  bool m_is_clockwise = true;      // Direction: true = clockwise, false = counter-clockwise
  bool m_is_relative = false;      // Mode: true = relative, false = absolute

  // Advanced PID controller for yaw control
  CAdvancedPIDController m_yaw_pid_controller;

  // Yaw rate tracking for rate-based control
  double m_last_heading_for_rate = 0.0;
  uint64_t m_yaw_rate_check_time = 0;
  double m_current_yaw_rate = 0.0;

  // Configuration parameters
  double m_default_yaw_rate = 30.0; // deg/sec
  double m_yaw_deadband = 0.05;     // rad (~2.86 degrees) - angle error deadband

  // Yaw PID parameters - balanced for good response with damping
  double m_yaw_p = 8.0;               // Proportional gain
  double m_yaw_i = 0.2;               // Integral gain
  double m_yaw_d = 0.5;               // Derivative gain for damping
  double m_yaw_integral_limit = 50.0; // Integral windup limit
  double m_yaw_max_accel = 180.0;     // deg/s^2 - max angular acceleration for sqrt_controller
  double m_yaw_ff_scale = 120.0;      // PWM per rad/s feedforward
};

} // namespace depilot
} // namespace fcb
} // namespace de

#endif // FCB_DE_PILOT_YAW_CONTROL_H_
