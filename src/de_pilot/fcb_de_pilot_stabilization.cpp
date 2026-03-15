#include "fcb_de_pilot_stabilization.hpp"
#include "../fcb_main.hpp"
#include "../de_common/helpers/colors.hpp"
#include "../de_common/helpers/helpers.hpp"
#include "../de_common/de_databus/configFile.hpp"
#include "fcb_de_pilot_manager.hpp"
#include <mavlink_command.h>
#include <mavlink_sdk.h>
#include <vehicle.h>
#include <iostream>
#include <cmath>

using namespace de::fcb::depilot;

// Base class interface implementation
void CDEPilotStabilization::init() {
    // Read configuration parameters first
    readConfigParameters();
    
    // Initialize stabilization system
    m_phase = PHASE_IDLE;
    m_phase_start_time = get_time_usec() / 1000;
    m_last_update_time = m_phase_start_time;
    m_generic_phase = static_cast<int>(m_phase);

    // Initialize yaw control
    m_yaw_control_enabled = false;
    m_target_yaw_angle = 0.0;
    m_yaw_turn_rate = 0.0;
    m_yaw_is_clockwise = true;
    m_yaw_is_relative = false;
    
    // Initialize yaw PID variables
    m_yaw_error = 0.0;
    m_yaw_error_integral = 0.0;
    m_yaw_error_derivative = 0.0;
    m_yaw_previous_error = 0.0;
    m_yaw_last_time = get_time_usec() / 1000;

    // Initialize yaw rate tracking
    m_last_heading_for_rate = 0.0;
    m_yaw_rate_check_time = 0;
    m_current_yaw_rate = 0.0;

    startStabilization();
}

void CDEPilotStabilization::update() {
    updateStabilization();
}

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

int CDEPilotStabilization::getPhase() const {
    return m_generic_phase;
}

void CDEPilotStabilization::setActive(bool active) {
    m_active = active;
    if (active) {
        m_phase_start_time = get_time_usec() / 1000;
    }
}

bool CDEPilotStabilization::getActive() const {
    return m_active;
}

void CDEPilotStabilization::readConfigParameters() {
    de::CConfigFile &cConfigFile = de::CConfigFile::getInstance();
    const Json_de &jsonConfig = cConfigFile.GetConfigJSON();

    if (jsonConfig.contains("de_pilot")) {
        const Json_de &de_pilot_root = jsonConfig["de_pilot"];

        if (de_pilot_root.contains("stabilization")) {
            const Json_de &stabilization_config = de_pilot_root["stabilization"];

            if (stabilization_config.contains("stabilize_time_ms")) {
                m_stabilize_time_ms = stabilization_config["stabilize_time_ms"].get<double>();
            }
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
            if (stabilization_config.contains("slowdown_angle")) {
                m_slowdown_angle = stabilization_config["slowdown_angle"].get<double>();
            }
        }
    }
}

void CDEPilotStabilization::startStabilization() {
    if (m_active) {
        std::cout << _INFO_CONSOLE_BOLD_TEXT << "DEPILOT: Stabilization already in progress" 
                  << _NORMAL_CONSOLE_TEXT_ << std::endl;
        return;
    }

    m_target_altitude = mavlinksdk::CVehicle::getInstance().getMsgGlobalPositionInt().relative_alt / 1000.0;
    
    std::cout << _INFO_CONSOLE_BOLD_TEXT << "DEPILOT: Starting stabilization" 
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
    std::cout << "  - Target altitude: " << m_target_altitude << "m" << std::endl;

    m_active = true;
    m_phase = PHASE_STABILIZING;
    m_phase_start_time = get_time_usec();
    m_last_update_time = m_phase_start_time;

    de::fcb::CFCBMain &fcbMain = de::fcb::CFCBMain::getInstance();
    const ANDRUAV_VEHICLE_INFO &vehicle_info = fcbMain.getAndruavVehicleInfo();
    
    std::cout << "  - Current flight mode: " << vehicle_info.flying_mode << std::endl;
    std::cout << "  - Armed: " << (vehicle_info.is_armed ? "YES" : "NO") << std::endl;
    std::cout << "  - Flying: " << (vehicle_info.is_flying ? "YES" : "NO") << std::endl;

    de::fcb::depilot::CDEPilotManager::getInstance().setTargetAltitude(m_target_altitude);
    // Don't set operation here - it's already set by the manager when this operation is started
    
    std::cout << _SUCCESS_CONSOLE_TEXT_ << "DEPILOT: Stabilization initialized successfully" 
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
                // Do nothing I am not armed.
        return;
    }

    const double current_altitude = mavlinksdk::CVehicle::getInstance().getMsgGlobalPositionInt().relative_alt / 1000.0;

    switch (m_phase) {
        case PHASE_STABILIZING: {
            std::cout << _INFO_CONSOLE_BOLD_TEXT << "DEPILOT: Stabilizing altitude" 
                      << _NORMAL_CONSOLE_TEXT_ << std::endl;
            std::cout << "  - Current flight mode: " << vehicle_info.flying_mode << std::endl;
            std::cout << "  - Current altitude: " << current_altitude << "m" << std::endl;
            std::cout << "  - Target altitude: " << m_target_altitude << "m" << std::endl;
            
            // In ALT-HOLD mode, send 1500 to stop climbing/descending
            if (vehicle_info.flying_mode != VEHICLE_MODE_GUIDED && vehicle_info.flying_mode != VEHICLE_MODE_BRAKE) {
                std::cout << "  - ALT-HOLD mode: Sending stop command (1500) to maintain altitude" << std::endl;
                std::cout << "  - Yaw control enabled: " << (m_yaw_control_enabled ? "YES" : "NO") << std::endl;
                
                int16_t rc_channels[RC_CHANNELS_MAX] = {SKIP_RC_CHANNEL};
                rc_channels[fcbMain.getRCChannelsMapInfo().rcmap_throttle] = 1500;  // Stop climbing
                // Force roll and pitch to neutral (1500) to prevent max PWM values
                rc_channels[fcbMain.getRCChannelsMapInfo().rcmap_roll] = 1500;
                rc_channels[fcbMain.getRCChannelsMapInfo().rcmap_pitch] = 1500;
                
                // Handle yaw control via RC channel in ALT-HOLD mode
                if (m_yaw_control_enabled) {
                    std::cout << "  - Yaw control (RC): angle=" << m_target_yaw_angle 
                              << " clockwise=" << (m_yaw_is_clockwise ? "YES" : "NO") << std::endl;
                    
                    // Get current heading from MAVLink attitude message (already in radians)
                    const mavlink_attitude_t& attitude = mavlinksdk::CVehicle::getInstance().getMsgAttitude();
                    double current_heading = attitude.yaw;
                    
                    // Normalize to 0-2π range
                    while (current_heading < 0) current_heading += 2 * M_PI;
                    while (current_heading >= 2 * M_PI) current_heading -= 2 * M_PI;
                    
                    // Calculate current yaw rate
                    const uint64_t now = get_time_usec();
                    if (m_yaw_rate_check_time > 0) {
                        const double dt = (now - m_yaw_rate_check_time) / 1000000.0; // seconds
                        if (dt > 0.2) { // Update yaw rate every 0.2 seconds
                            double heading_diff = current_heading - m_last_heading_for_rate;
                            // Handle wraparound for shortest path
                            if (heading_diff > M_PI) heading_diff -= 2 * M_PI;
                            if (heading_diff < -M_PI) heading_diff += 2 * M_PI;
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
                    
                    // Calculate target heading (absolute or relative)
                    double target_heading = target_angle_rad;
                    if (m_yaw_is_relative) {
                        target_heading = current_heading + target_angle_rad;
                    }
                    
                    // Normalize angles to 0-2π range
                    while (target_heading < 0) target_heading += 2 * M_PI;
                    while (target_heading >= 2 * M_PI) target_heading -= 2 * M_PI;
                    
                    // Calculate yaw error (shortest path)
                    double angle_error = target_heading - current_heading;
                    if (angle_error > M_PI) angle_error -= 2 * M_PI;
                    if (angle_error < -M_PI) angle_error += 2 * M_PI;
                    
                    // Apply direction constraint if specified
                    if (m_yaw_is_clockwise && angle_error < 0) {
                        angle_error = 2 * M_PI + angle_error; // Force clockwise rotation
                    } else if (!m_yaw_is_clockwise && angle_error > 0) {
                        angle_error = angle_error - 2 * M_PI; // Force counter-clockwise rotation
                    }
                    
                    // Calculate desired yaw rate with linear taper
                    const float desired_yaw_rate = calculateDesiredYawRate(angle_error);
                    const double rate_error = desired_yaw_rate - m_current_yaw_rate;
                    
                    // PID calculation using rate error
                    uint64_t current_time = now / 1000;
                    double dt = (current_time - m_yaw_last_time) / 1000.0; // Convert to seconds
                    
                    if (dt > 0.001) { // Avoid division by zero
                        m_yaw_error = rate_error;
                        m_yaw_error_integral += rate_error * dt;
                        
                        // Integral windup protection
                        if (m_yaw_error_integral > m_yaw_integral_limit) {
                            m_yaw_error_integral = m_yaw_integral_limit;
                        } else if (m_yaw_error_integral < -m_yaw_integral_limit) {
                            m_yaw_error_integral = -m_yaw_integral_limit;
                        }
                        
                        m_yaw_error_derivative = (rate_error - m_yaw_previous_error) / dt;
                        m_yaw_previous_error = rate_error;
                        m_yaw_last_time = current_time;
                        
                        // PID output
                        double pid_output = m_yaw_p * m_yaw_error + 
                                          m_yaw_i * m_yaw_error_integral + 
                                          m_yaw_d * m_yaw_error_derivative;
                        
                        // Convert PID output to PWM (1000-2000 range)
                        // Positive error = clockwise, Negative error = counter-clockwise
                        int16_t pwm = 1500 + (int16_t)(pid_output * 10); // Scale factor for PWM
                        
                        // Limit PWM to valid range
                        if (pwm > 1600) pwm = 1600;
                        if (pwm < 1400) pwm = 1400;
                        
                        std::cout << "    - Current heading: " << current_heading << " rad" << std::endl;
                        std::cout << "    - Target heading: " << target_heading << " rad" << std::endl;
                        std::cout << "    - Angle error: " << angle_error << " rad" << std::endl;
                        std::cout << "    - Current yaw rate: " << m_current_yaw_rate << " rad/s" << std::endl;
                        std::cout << "    - Desired yaw rate: " << desired_yaw_rate << " rad/s" << std::endl;
                        std::cout << "    - Rate error: " << rate_error << " rad/s" << std::endl;
                        std::cout << "    - PID output: " << pid_output << std::endl;
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
                std::cout << "  - GUIDED mode: Flight controller handles stabilization" << std::endl;
                
                // Handle yaw control via MAVLink command in GUIDED mode
                if (m_yaw_control_enabled) {
                    std::cout << "  - Yaw control (MAVLink): angle=" << m_target_yaw_angle 
                              << " rate=" << m_yaw_turn_rate 
                              << " clockwise=" << (m_yaw_is_clockwise ? "YES" : "NO")
                              << " relative=" << (m_yaw_is_relative ? "YES" : "NO") << std::endl;
                    
                    // Convert target angle from degrees to radians for MAVLink command
                    double target_angle_rad = m_target_yaw_angle * M_PI / 180.0;
                    double turn_rate_rad = m_yaw_turn_rate * M_PI / 180.0;
                    
                    mavlinksdk::CMavlinkCommand::getInstance().setYawCondition(
                        target_angle_rad, turn_rate_rad, m_yaw_is_clockwise, m_yaw_is_relative);
                }
            }
            
            m_last_update_time = now;
            
            // Stabilization runs indefinitely - no timeout
            // It will continue sending 1500 throttle to maintain altitude
            // until manually stopped or switched to another operation
        }
        break;

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

void CDEPilotStabilization::setYawTarget(double angle, double rate, bool is_clockwise, bool is_relative) {
    m_target_yaw_angle = angle;
    m_yaw_turn_rate = (rate > 0.0) ? rate : m_default_yaw_rate;
    m_yaw_is_clockwise = is_clockwise;
    m_yaw_is_relative = is_relative;
    m_yaw_control_enabled = true;
    
    std::cout << _INFO_CONSOLE_BOLD_TEXT << "DEPILOT: Yaw target set" 
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
    std::cout << "  - Target angle: " << m_target_yaw_angle << " deg" << std::endl;
    std::cout << "  - Turn rate: " << m_yaw_turn_rate << " deg/sec" << std::endl;
    std::cout << "  - Direction: " << (m_yaw_is_clockwise ? "Clockwise" : "Counter-clockwise") << std::endl;
    std::cout << "  - Mode: " << (m_yaw_is_relative ? "Relative" : "Absolute") << std::endl;
}

void CDEPilotStabilization::clearYawTarget() {
    m_yaw_control_enabled = false;
    m_target_yaw_angle = 0.0;
    m_yaw_turn_rate = 0.0;
    
    std::cout << _INFO_CONSOLE_BOLD_TEXT << "DEPILOT: Yaw control cleared" 
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
}

bool CDEPilotStabilization::isYawControlActive() const {
    return m_yaw_control_enabled;
}

float CDEPilotStabilization::calculateDesiredYawRate(double angle_error_rad) const {
    // Convert angle error to degrees for comparison with slowdown_angle
    const double angle_error_deg = angle_error_rad * 180.0 / M_PI;
    const double abs_error_deg = std::abs(angle_error_deg);
    
    // Linear taper: desired rate = sign(error) * max_rate * min(abs_error / slowdown_angle, 1)
    const float desired_rate_deg = static_cast<float>(std::copysign(1.0, angle_error_deg) * m_default_yaw_rate * std::min(abs_error_deg / m_slowdown_angle, 1.0));
    
    // Convert back to radians/sec for control
    return desired_rate_deg * static_cast<float>(M_PI / 180.0);
}
