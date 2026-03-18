#include "fcb_de_pilot_stabilization.hpp"
#include "../de_common/de_databus/configFile.hpp"
#include "../de_common/helpers/colors.hpp"
#include "../de_common/helpers/helpers.hpp"
#include "../fcb_main.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <mavlink_command.h>
#include <mavlink_sdk.h>
#include <vehicle.h>

using namespace de::fcb::depilot;

// Base class interface implementation
void CDEPilotStabilization::init() {
  // Read configuration parameters first
  readConfigParameters();

  // Initialize stabilization system
  m_active = false;
  m_phase = PHASE_IDLE;
  m_phase_start_time = get_time_usec() / 1000;
  m_last_update_time = m_phase_start_time;
  m_generic_phase = static_cast<int>(m_phase);

  // Initialize yaw control
  m_yaw_control_enabled = false;
  m_target_yaw_angle = 0.0;

  // Initialize advanced PID controller
  m_yaw_pid_controller.reset();

  // Initialize yaw rate tracking
  m_last_heading_for_rate = 0.0;
  m_yaw_rate_check_time = 0;
  m_current_yaw_rate = 0.0;

  startStabilization();
}

void CDEPilotStabilization::update() { updateStabilization(); }

void CDEPilotStabilization::uninit() {
  stopStabilization();
  m_active = false;
  m_phase = PHASE_IDLE;
  m_generic_phase = static_cast<int>(m_phase);
}

bool CDEPilotStabilization::isCompleted() {
  return true; // you can always get out of this.
}

void CDEPilotStabilization::setPhase(int phase) {
  m_generic_phase = phase;
  m_phase = static_cast<StabilizationPhase>(phase);
  m_phase_start_time = get_time_usec() / 1000;
}

int CDEPilotStabilization::getPhase() const { return m_generic_phase; }

void CDEPilotStabilization::setActive(bool active) {
  m_active = active;
  if (active) {
    m_phase_start_time = get_time_usec() / 1000;
  }
}

bool CDEPilotStabilization::getActive() const { return m_active; }

void CDEPilotStabilization::readConfigParameters() {
  de::CConfigFile &cConfigFile = de::CConfigFile::getInstance();
  const Json_de &jsonConfig = cConfigFile.GetConfigJSON();

  if (jsonConfig.contains("de_pilot")) {
    const Json_de &de_pilot_root = jsonConfig["de_pilot"];

    if (de_pilot_root.contains("stabilization")) {
      const Json_de &stabilization_config = de_pilot_root["stabilization"];

      if (stabilization_config.contains("default_yaw_rate")) {
        m_default_yaw_rate =
            stabilization_config["default_yaw_rate"].get<double>();
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
        m_yaw_integral_limit =
            stabilization_config["yaw_integral_limit"].get<double>();
      }
      if (stabilization_config.contains("yaw_max_accel")) {
        m_yaw_max_accel = stabilization_config["yaw_max_accel"].get<double>();
      }
      if (stabilization_config.contains("yaw_ff_scale")) {
        m_yaw_ff_scale = stabilization_config["yaw_ff_scale"].get<double>();
      }
      
      // Configure the advanced PID controller with loaded parameters
      m_yaw_pid_controller.setPID(m_yaw_p, m_yaw_i, m_yaw_d);
      // Set feedforward gain for yaw from configuration
      m_yaw_pid_controller.setFeedforwardGain(m_yaw_ff_scale);
      // Set delta time to 0.01s (10ms) which matches typical update rate
      m_yaw_pid_controller.setDeltaTime(0.01);
    }
  }
}

void CDEPilotStabilization::reloadParametersIfConfigChanged() {
  readConfigParameters();
  // Reconfigure the advanced PID controller with loaded parameters
  m_yaw_pid_controller.setPID(m_yaw_p, m_yaw_i, m_yaw_d);
  // Set feedforward gain for yaw from configuration
  m_yaw_pid_controller.setFeedforwardGain(m_yaw_ff_scale);
  // Set delta time to 0.01s (10ms) which matches typical update rate
  m_yaw_pid_controller.setDeltaTime(0.01);
}

void CDEPilotStabilization::startStabilization() {
  if (m_active) {
    std::cout << _INFO_CONSOLE_BOLD_TEXT
              << "DEPILOT: Stabilization already in progress"
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
    return;
  }

  std::cout << _INFO_CONSOLE_BOLD_TEXT << "DEPILOT: Starting stabilization"
            << _NORMAL_CONSOLE_TEXT_ << std::endl;

  m_active = true;
  m_phase = PHASE_STABILIZING;
  m_phase_start_time = get_time_usec();
  m_last_update_time = m_phase_start_time;

  de::fcb::CFCBMain &fcbMain = de::fcb::CFCBMain::getInstance();
  const ANDRUAV_VEHICLE_INFO &vehicle_info = fcbMain.getAndruavVehicleInfo();

  std::cout << "  - Current flight mode: " << vehicle_info.flying_mode
            << std::endl;
  std::cout << "  - Armed: " << (vehicle_info.is_armed ? "YES" : "NO")
            << std::endl;
  std::cout << "  - Flying: " << (vehicle_info.is_flying ? "YES" : "NO")
            << std::endl;
  // Don't set operation here - it's already set by the manager when this
  // operation is started

  std::cout << _SUCCESS_CONSOLE_TEXT_
            << "DEPILOT: Stabilization initialized successfully"
            << _NORMAL_CONSOLE_TEXT_ << std::endl;
}

void CDEPilotStabilization::updateStabilization() {
  if (m_phase == PHASE_IDLE || m_phase == PHASE_COMPLETE) {
    return;
  }

  de::fcb::CFCBMain &fcbMain = de::fcb::CFCBMain::getInstance();
  const ANDRUAV_VEHICLE_INFO &vehicle_info = fcbMain.getAndruavVehicleInfo();
  const uint64_t now = get_time_usec();

  if (!vehicle_info.is_armed) {
    std::cout << _INFO_CONSOLE_BOLD_TEXT << "DEPILOT: Waiting for arm..."
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
    return;
  }

  const double current_altitude = mavlinksdk::CVehicle::getInstance()
                                      .getMsgGlobalPositionInt()
                                      .relative_alt /
                                  1000.0;

  switch (m_phase) {
  case PHASE_STABILIZING: {
    std::cout << _INFO_CONSOLE_BOLD_TEXT << "DEPILOT: Stabilizing altitude"
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
    std::cout << "  - Current flight mode: " << vehicle_info.flying_mode
              << std::endl;
    std::cout << "  - Current altitude: " << current_altitude << "m"
              << std::endl;

    // In ALT-HOLD mode, send 1500 to stop climbing/descending
    if (vehicle_info.flying_mode != VEHICLE_MODE_GUIDED &&
        vehicle_info.flying_mode != VEHICLE_MODE_BRAKE) {
      std::cout << "  - ALT-HOLD mode: Sending stop command (1500) to maintain "
                   "altitude"
                << std::endl;
      std::cout << "  - Yaw control enabled: "
                << (m_yaw_control_enabled ? "YES" : "NO") << std::endl;

      int16_t rc_channels[RC_CHANNELS_MAX];
      std::fill_n(rc_channels, RC_CHANNELS_MAX, SKIP_RC_CHANNEL);
      rc_channels[fcbMain.getRCChannelsMapInfo().rcmap_throttle] =
          1500; // Stop climbing
      // Force roll and pitch to neutral (1500) to prevent max PWM values
      rc_channels[fcbMain.getRCChannelsMapInfo().rcmap_roll] = 1500;
      rc_channels[fcbMain.getRCChannelsMapInfo().rcmap_pitch] = 1500;

      // Handle yaw control via RC channel in ALT-HOLD mode
      if (m_yaw_control_enabled) {
        std::cout << "  - Yaw control (RC): angle=" << m_target_yaw_angle
                  << std::endl;

        // Get current heading from MAVLink attitude message (already in
        // radians)
        const auto &attitude =
            mavlinksdk::CVehicle::getInstance().getMsgAttitude();
        double current_heading = attitude.yaw;

        // Normalize to 0-2π range
        while (current_heading < 0)
          current_heading += 2 * M_PI;
        while (current_heading >= 2 * M_PI)
          current_heading -= 2 * M_PI;

        // Calculate current yaw rate
        const uint64_t now = get_time_usec();
        if (m_yaw_rate_check_time > 0) {
          const double dt =
              (now - m_yaw_rate_check_time) / 1000000.0; // seconds
          if (dt > 0.2) { // Update yaw rate every 0.2 seconds
            double heading_diff = current_heading - m_last_heading_for_rate;
            // Handle wraparound for shortest path
            if (heading_diff > M_PI)
              heading_diff -= 2 * M_PI;
            if (heading_diff < -M_PI)
              heading_diff += 2 * M_PI;
            m_current_yaw_rate = heading_diff / dt;
            m_last_heading_for_rate = current_heading;
            m_yaw_rate_check_time = now;
          }
        } else {
          // Initialize yaw rate tracking
          m_last_heading_for_rate = current_heading;
          m_yaw_rate_check_time = now;
          m_current_yaw_rate = 0.0;
        }

        // Convert target angle from degrees to radians
        double target_angle_rad = m_target_yaw_angle * M_PI / 180.0;

        // Calculate target heading (absolute only)
        double target_heading = target_angle_rad;

        // Normalize angles to 0-2π range
        while (target_heading < 0)
          target_heading += 2 * M_PI;
        while (target_heading >= 2 * M_PI)
          target_heading -= 2 * M_PI;

        // Calculate yaw error (shortest path) - FIXED wraparound logic
        double angle_error = target_heading - current_heading;

        // Normalize to [-π, +π] range for shortest path
        while (angle_error > M_PI)
          angle_error -= 2 * M_PI;
        while (angle_error < -M_PI)
          angle_error += 2 * M_PI;

        // Calculate desired yaw rate using sqrt_controller
        const float desired_yaw_rate = calculateDesiredYawRate(angle_error);
        const double rate_error = desired_yaw_rate - m_current_yaw_rate;

        // PID calculation using rate error with advanced PID controller
        if (true) { // Always execute since advanced PID controller handles dt internally
          // Use advanced PID controller with feedforward
          // The controller handles PID calculation, anti-windup, and feedforward internally
          double pid_output = m_yaw_pid_controller.calculate(rate_error, desired_yaw_rate);

          // Convert combined output to PWM (1000-2000 range)
          // The advanced PID controller already includes feedforward, so just add to 1500
          int16_t pwm = 1500 + (int16_t)pid_output;

          // Limit PWM to valid range (widened from 1400-1600 to 1200-1800 to
          // allow overcoming deadband)
          if (pwm > 1800)
            pwm = 1800;
          if (pwm < 1200)
            pwm = 1200;

          std::cout << "    - Current heading: " << current_heading << " rad"
                    << std::endl;
          std::cout << "    - Target heading: " << target_heading << " rad"
                    << std::endl;
          std::cout << "    - Angle error: " << angle_error << " rad"
                    << std::endl;
          std::cout << "    - Current yaw rate: " << m_current_yaw_rate
                    << " rad/s" << std::endl;
          std::cout << "    - Desired yaw rate: " << desired_yaw_rate
                    << " rad/s" << std::endl;
          std::cout << "    - Rate error: " << rate_error << " rad/s"
                    << std::endl;
          std::cout << "    - Advanced PID output: " << pid_output << " PWM" << std::endl;
          std::cout << "    - PWM: " << pwm << std::endl;

          rc_channels[fcbMain.getRCChannelsMapInfo().rcmap_yaw] = pwm;
        } else {
          rc_channels[fcbMain.getRCChannelsMapInfo().rcmap_yaw] = 1500;
        }
      } else {
        rc_channels[fcbMain.getRCChannelsMapInfo().rcmap_yaw] = 1500;
      }

      mavlinksdk::CMavlinkCommand::getInstance().sendRCChannels(
          rc_channels, RC_CHANNELS_MAX);
    } else {
      std::cout << "  - GUIDED mode: Flight controller handles stabilization"
                << std::endl;

      // Handle yaw control via MAVLink command in GUIDED mode
      if (m_yaw_control_enabled) {
        std::cout << "  - Yaw control (MAVLink): angle=" << m_target_yaw_angle
                  << std::endl;

        // Convert target angle from degrees to radians for MAVLink command
        double target_angle_rad = m_target_yaw_angle * M_PI / 180.0;
        double turn_rate_rad = m_default_yaw_rate * M_PI / 180.0;

        mavlinksdk::CMavlinkCommand::getInstance().setYawCondition(
            target_angle_rad, turn_rate_rad, true, false);
      }
    }

    m_last_update_time = now;

    // Stabilization runs indefinitely - no timeout
    // It will continue sending 1500 throttle to maintain altitude
    // until manually stopped or switched to another operation
  } break;

  default:
    break;
  }
}

void CDEPilotStabilization::stopStabilization() {
  std::cout << _INFO_CONSOLE_BOLD_TEXT << "DEPILOT: Stopping stabilization"
            << _NORMAL_CONSOLE_TEXT_ << std::endl;

  m_active = false;
  setPhase(PHASE_COMPLETE);
}

bool CDEPilotStabilization::isStabilizationActive() const {
  return m_phase != PHASE_IDLE && m_phase != PHASE_COMPLETE;
}

void CDEPilotStabilization::setYawTarget(double angle, double rate,
                                         bool is_clockwise, bool is_relative) {
  m_target_yaw_angle = angle;
  m_yaw_control_enabled = true;

  std::cout << _INFO_CONSOLE_BOLD_TEXT << "DEPILOT: Yaw target set"
            << _NORMAL_CONSOLE_TEXT_ << std::endl;
  std::cout << "  - Target angle: " << m_target_yaw_angle << " deg"
            << std::endl;
  std::cout << "  - Direction: "
            << (is_clockwise ? "Clockwise" : "Counter-clockwise") << std::endl;
  std::cout << "  - Mode: " << (is_relative ? "Relative" : "Absolute")
            << std::endl;
}

void CDEPilotStabilization::clearYawTarget() {
  m_yaw_control_enabled = false;
  m_target_yaw_angle = 0.0;

  std::cout << _INFO_CONSOLE_BOLD_TEXT << "DEPILOT: Yaw control cleared"
            << _NORMAL_CONSOLE_TEXT_ << std::endl;
}

bool CDEPilotStabilization::isYawControlActive() const {
  return m_yaw_control_enabled;
}

float CDEPilotStabilization::calculateDesiredYawRate(
    double angle_error_rad) const {
  // Convert angle error to degrees for the sqrt_controller
  const double error = angle_error_rad * 180.0 / M_PI;
  const double p_gain = m_yaw_p;
  const double max_accel = m_yaw_max_accel;
  const double max_rate = m_default_yaw_rate;

  // Use the advanced PID controller's static sqrt_controller method
  // This provides the same ArduPilot-style piecewise controller behavior
  double desired_rate_deg = CAdvancedPIDController::sqrt_controller(
      error, p_gain, max_accel, max_rate);

  // Convert to radians/sec for control
  return static_cast<float>(desired_rate_deg * M_PI / 180.0);
}
