#include "fcb_de_pilot_yaw_control.hpp"
#include "../de_common/de_databus/configFile.hpp"
#include "../de_common/helpers/colors.hpp"
#include "../de_common/helpers/helpers.hpp"
#include "../fcb_main.hpp"
#include <sstream>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <mavlink_command.h>
#include <mavlink_sdk.h>
#include <vehicle.h>
#include "../defines.hpp"

using namespace de::fcb::depilot;

void CDEPilotYawControl::init() {
  readConfigParameters();

  m_active = false;
  m_phase = PHASE_IDLE;
  m_phase_start_time = get_time_usec() / 1000;
  m_last_update_time = m_phase_start_time;
  m_generic_phase = static_cast<int>(m_phase);

  m_yaw_control_enabled = false;
  m_target_yaw_angle = 0.0;

  m_yaw_pid_controller.reset();

  m_last_heading_for_rate = 0.0;
  m_yaw_rate_check_time = 0;
  m_current_yaw_rate = 0.0;

  std::cout << _INFO_CONSOLE_BOLD_TEXT << "DEPilotYawControl: Initialized"
            << _NORMAL_CONSOLE_TEXT_ << std::endl;
}

void CDEPilotYawControl::update() {
  updateYawControl();
}

void CDEPilotYawControl::uninit() {
  m_yaw_control_enabled = false;
  m_target_yaw_angle = 0.0;
  m_active = false;
  m_phase = PHASE_IDLE;
  m_generic_phase = static_cast<int>(m_phase);
}

void CDEPilotYawControl::readConfigParameters() {
  de::CConfigFile &cConfigFile = de::CConfigFile::getInstance();
  const Json_de &jsonConfig = cConfigFile.GetConfigJSON();

  if (jsonConfig.contains("de_pilot")) {
    const Json_de &de_pilot_root = jsonConfig["de_pilot"];

    if (de_pilot_root.contains("stabilization")) {
      const Json_de &stabilization_config = de_pilot_root["stabilization"];

      if (stabilization_config.contains("default_yaw_rate")) {
        m_default_yaw_rate = stabilization_config["default_yaw_rate"].get<double>();
      }
      if (stabilization_config.contains("yaw_p")) {
        m_yaw_p = stabilization_config["yaw_p"].get<double>();
      }
      if (stabilization_config.contains("yaw_i")) {
        m_yaw_i = stabilization_config["yaw_i"].get<double>();
      }
      if (stabilization_config.contains("yaw_d")) {
        m_yaw_d = stabilization_config["yaw_d"].get<double>();
      }
      if (stabilization_config.contains("yaw_integral_limit")) {
        m_yaw_integral_limit = stabilization_config["yaw_integral_limit"].get<double>();
      }
      if (stabilization_config.contains("yaw_max_accel")) {
        m_yaw_max_accel = stabilization_config["yaw_max_accel"].get<double>();
      }
      if (stabilization_config.contains("yaw_ff_scale")) {
        m_yaw_ff_scale = stabilization_config["yaw_ff_scale"].get<double>();
      }
      if (stabilization_config.contains("yaw_deadband")) {
        m_yaw_deadband = stabilization_config["yaw_deadband"].get<double>();
      }
      
      m_yaw_pid_controller.setPID(m_yaw_p, m_yaw_i, m_yaw_d);
      m_yaw_pid_controller.setFeedforwardGain(m_yaw_ff_scale);
      m_yaw_pid_controller.setDeltaTime(0.01);
    }
  }
}

void CDEPilotYawControl::reloadParametersIfConfigChanged() {
  readConfigParameters();
  m_yaw_pid_controller.setPID(m_yaw_p, m_yaw_i, m_yaw_d);
  m_yaw_pid_controller.setFeedforwardGain(m_yaw_ff_scale);
  m_yaw_pid_controller.setDeltaTime(0.01);
}

void CDEPilotYawControl::setPhase(int phase) {
  m_generic_phase = phase;
  m_phase = static_cast<YawControlPhase>(phase);
  m_phase_start_time = get_time_usec() / 1000;
}

int CDEPilotYawControl::getPhase() const {
  return m_generic_phase;
}

void CDEPilotYawControl::setActive(bool active) {
  m_active = active;
  if (active) {
    m_phase = PHASE_ACTIVE;
    m_phase_start_time = get_time_usec() / 1000;
  } else {
    m_phase = PHASE_IDLE;
  }
  m_generic_phase = static_cast<int>(m_phase);
}

bool CDEPilotYawControl::getActive() const {
  return m_active;
}

bool CDEPilotYawControl::isCompleted() {
  return false;
}

void CDEPilotYawControl::setYawTarget(double angle, double rate, bool is_clockwise, bool is_relative) {
  m_target_yaw_angle = angle;
  m_is_clockwise = is_clockwise;
  m_is_relative = is_relative;
  m_yaw_control_enabled = true;
  m_active = true;
  m_phase = PHASE_ACTIVE;
  m_generic_phase = static_cast<int>(m_phase);
  setTaskState(DEPILOT_TASK_STATE::ACTIVE);
#ifdef DEBUG
  std::cout << _INFO_CONSOLE_BOLD_TEXT << "DEPilotYawControl: Yaw target set"
            << _NORMAL_CONSOLE_TEXT_ << std::endl;
  std::cout << "  - Target angle: " << m_target_yaw_angle << " deg" << std::endl;
  std::cout << "  - Direction: " << (is_clockwise ? "Clockwise" : "Counter-clockwise") << std::endl;
  std::cout << "  - Mode: " << (is_relative ? "Relative" : "Absolute") << std::endl;
#endif  
}

void CDEPilotYawControl::clearYawTarget() {
  m_yaw_control_enabled = false;
  m_target_yaw_angle = 0.0;
  m_active = false;
  m_phase = PHASE_IDLE;
  m_generic_phase = static_cast<int>(m_phase);
  setTaskState(DEPILOT_TASK_STATE::CANCELLED);

  std::cout << _INFO_CONSOLE_BOLD_TEXT << "DEPilotYawControl: Yaw control cleared"
            << _NORMAL_CONSOLE_TEXT_ << std::endl;
}

bool CDEPilotYawControl::isYawControlActive() const {
  return m_yaw_control_enabled;
}

void CDEPilotYawControl::updateYawControl() {
  if (!m_yaw_control_enabled) {
    return;
  }

  const auto &attitude = mavlinksdk::CVehicle::getInstance().getMsgAttitude();
  double current_heading = attitude.yaw;

  while (current_heading < 0)
    current_heading += 2 * M_PI;
  while (current_heading >= 2 * M_PI)
    current_heading -= 2 * M_PI;

  const uint64_t now = get_time_usec();
  if (m_yaw_rate_check_time > 0) {
    const double dt = (now - m_yaw_rate_check_time) / 1000000.0;
    if (dt > 0.2) {
      double heading_diff = current_heading - m_last_heading_for_rate;
      if (heading_diff > M_PI)
        heading_diff -= 2 * M_PI;
      if (heading_diff < -M_PI)
        heading_diff += 2 * M_PI;
      m_current_yaw_rate = heading_diff / dt;
      m_last_heading_for_rate = current_heading;
      m_yaw_rate_check_time = now;
    }
  } else {
    m_last_heading_for_rate = current_heading;
    m_yaw_rate_check_time = now;
    m_current_yaw_rate = 0.0;
  }

  m_last_update_time = get_time_usec() / 1000;
}

void CDEPilotYawControl::applyYawToRCChannels(uint16_t rc_channels[], int flying_mode) {
  if (!m_yaw_control_enabled) {
    return;
  }


  if (flying_mode != VEHICLE_MODE_GUIDED && flying_mode != VEHICLE_MODE_BRAKE) {
    const auto &attitude = mavlinksdk::CVehicle::getInstance().getMsgAttitude();
    double current_heading = attitude.yaw;

    while (current_heading < 0)
      current_heading += 2 * M_PI;
    while (current_heading >= 2 * M_PI)
      current_heading -= 2 * M_PI;

    double target_angle_rad = m_target_yaw_angle * M_PI / 180.0;
    double target_heading = target_angle_rad;

    while (target_heading < 0)
      target_heading += 2 * M_PI;
    while (target_heading >= 2 * M_PI)
      target_heading -= 2 * M_PI;

    double angle_error = target_heading - current_heading;

    while (angle_error > M_PI)
      angle_error -= 2 * M_PI;
    while (angle_error < -M_PI)
      angle_error += 2 * M_PI;

    int16_t pwm = 1500;
    double pid_output = 0.0;
    float desired_yaw_rate = 0.0;
    double rate_error = 0.0;

    // Check if within deadband - if so, send neutral and let ArduPilot stabilize
    if (std::abs(angle_error) < m_yaw_deadband) {
      // Within deadband - send neutral PWM and reset PID
      pwm = 1500;
      m_yaw_pid_controller.reset();
#ifdef DEBUG      
      std::cout << "  - YAW Control (RC): WITHIN DEADBAND - Neutral" << std::endl;
#endif
    } else {
      // Outside deadband - apply PID control
      desired_yaw_rate = calculateDesiredYawRate(angle_error);
      rate_error = desired_yaw_rate - m_current_yaw_rate;

      // Use rate error for PID control with feedforward for desired rate
      // The sqrt_controller already handles angle-to-rate conversion
      pid_output = m_yaw_pid_controller.calculate(rate_error, desired_yaw_rate);

      pwm = 1500 + (int16_t)pid_output;

      if (pwm > 1800)
        pwm = 1800;
      if (pwm < 1200)
        pwm = 1200;
    }

#ifdef DEBUG
    std::cout << "  - YAW Control (RC):" << std::endl;
    std::cout << "    - Current heading: " << current_heading << " rad" << std::endl;
    std::cout << "    - Target heading: " << target_heading << " rad" << std::endl;
    std::cout << "    - Angle error: " << angle_error << " rad" << std::endl;
    std::cout << "    - Current yaw rate: " << m_current_yaw_rate << " rad/s" << std::endl;
    std::cout << "    - Desired yaw rate: " << desired_yaw_rate << " rad/s" << std::endl;
    std::cout << "    - Rate error: " << rate_error << " rad/s" << std::endl;
    std::cout << "    - Advanced PID output: " << pid_output << " PWM" << std::endl;
    std::cout << "    - PWM: " << pwm << std::endl;
#endif

    rc_channels[m_fcbMain.getRCChannelsMapInfo().rcmap_yaw] = pwm;
  } else {
#ifdef DEBUG
    std::cout << "  - YAW Control (MAVLink): angle=" << m_target_yaw_angle << std::endl;
#endif
    double target_angle_rad = m_target_yaw_angle * M_PI / 180.0;
    double turn_rate_rad = m_default_yaw_rate * M_PI / 180.0;

    mavlinksdk::CMavlinkCommand::getInstance().setYawCondition(
        target_angle_rad, turn_rate_rad, m_is_clockwise, m_is_relative);
  }
}

std::string CDEPilotYawControl::getEventContext() const {
  std::ostringstream oss;
  oss << "{"
      << "\"angle\":" << m_target_yaw_angle << ","
      << "\"rate\":" << m_default_yaw_rate << ","
      << "\"is_clockwise\":" << (m_is_clockwise ? "true" : "false") << ","
      << "\"is_relative\":" << (m_is_relative ? "true" : "false")
      << "}";
  return oss.str();
}

float CDEPilotYawControl::calculateDesiredYawRate(double angle_error_rad) const {
  const double error = angle_error_rad * 180.0 / M_PI;
  const double p_gain = m_yaw_p;
  const double max_accel = m_yaw_max_accel;
  const double max_rate = m_default_yaw_rate;
      
  double desired_rate_deg = CAdvancedPIDController::sqrt_controller(
      error, p_gain, max_accel, max_rate);

  return static_cast<float>(desired_rate_deg * M_PI / 180.0);
}
