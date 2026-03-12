#include "fcb_de_pilot_altitude.hpp"
#include "../fcb_main.hpp"
#include "../de_common/helpers/colors.hpp"
#include "../de_common/helpers/helpers.hpp"
#include "../de_common/de_databus/configFile.hpp"
#include <mavlink_command.h>
#include <vehicle.h>
#include <iostream>
#include <cmath>

using namespace de::fcb::depilot;

void CDEPilotAltitude::readConfigParameters() {
    de::CConfigFile &cConfigFile = de::CConfigFile::getInstance();
    const Json_de &jsonConfig = cConfigFile.GetConfigJSON();

    if (jsonConfig.contains("de_pilot")) {
        const Json_de &de_pilot_root = jsonConfig["de_pilot"];

        if (de_pilot_root.contains("altitude_control")) {
            const Json_de &altitude_config = de_pilot_root["altitude_control"];

            if (altitude_config.contains("pid_p")) {
                m_pid_p = altitude_config["pid_p"].get<double>();
            }
            if (altitude_config.contains("pid_i")) {
                m_pid_i = altitude_config["pid_i"].get<double>();
            }
            if (altitude_config.contains("pid_d")) {
                m_pid_d = altitude_config["pid_d"].get<double>();
            }
            if (altitude_config.contains("deadband_m")) {
                m_deadband = altitude_config["deadband_m"].get<double>();
            }
            if (altitude_config.contains("max_throttle_adjustment")) {
                m_max_throttle_adjustment = altitude_config["max_throttle_adjustment"].get<double>();
            }
            if (altitude_config.contains("max_climb_rate")) {
                m_max_climb_rate = altitude_config["max_climb_rate"].get<double>();
            }
            if (altitude_config.contains("rate_limit")) {
                m_rate_limit = altitude_config["rate_limit"].get<double>();
            }
            if (altitude_config.contains("timeout_ms")) {
                m_timeout_ms = altitude_config["timeout_ms"].get<uint64_t>();
            }
        }
    }
}

void CDEPilotAltitude::startAltitudeChange(double target_altitude) {
    if (m_active) {
        std::cout << _INFO_CONSOLE_BOLD_TEXT << "DEPILOT: Altitude control already active" 
                  << _NORMAL_CONSOLE_TEXT_ << std::endl;
        return;
    }

    m_start_altitude = mavlinksdk::CVehicle::getInstance().getMsgGlobalPositionInt().relative_alt / 1000.0;
    
    std::cout << _INFO_CONSOLE_BOLD_TEXT << "DEPILOT: Starting altitude change" 
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
    std::cout << "  - Start altitude: " << m_start_altitude << "m" << std::endl;
    std::cout << "  - Target altitude: " << target_altitude << "m" << std::endl;
    std::cout << "  - Altitude difference: " << (target_altitude - m_start_altitude) << "m" << std::endl;

    m_target_altitude = target_altitude;
    m_start_time = get_time_usec();
    m_last_update_time = m_start_time;
    m_last_error = 0.0;
    m_integral = 0.0;
    m_active = true;

    de::fcb::CFCBMain &fcbMain = de::fcb::CFCBMain::getInstance();
    const ANDRUAV_VEHICLE_INFO &vehicle_info = fcbMain.getAndruavVehicleInfo();
    
    std::cout << "  - Current flight mode: " << vehicle_info.flying_mode << std::endl;
    std::cout << "  - Armed: " << (vehicle_info.is_armed ? "YES" : "NO") << std::endl;
    std::cout << "  - Flying: " << (vehicle_info.is_flying ? "YES" : "NO") << std::endl;

    fcbMain.setDEPilotOperation(DEPILOT_OP_ALTITUDE_CONTROL, true);
    fcbMain.setDEPilotTargetAltitude(target_altitude);
    
    std::cout << _SUCCESS_CONSOLE_TEXT_ << "DEPILOT: Altitude control initialized successfully" 
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
}

void CDEPilotAltitude::updateAltitudeControl() {
    if (!m_active) {
        return;
    }

    de::fcb::CFCBMain &fcbMain = de::fcb::CFCBMain::getInstance();
    const ANDRUAV_VEHICLE_INFO &vehicle_info = fcbMain.getAndruavVehicleInfo();
    const uint64_t now = get_time_usec();
    const double current_altitude = mavlinksdk::CVehicle::getInstance().getMsgGlobalPositionInt().relative_alt / 1000.0;

    // Check timeout
    if ((now - m_start_time) > (m_timeout_ms * 1000)) {
        std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "DEPILOT: Altitude control timeout" 
                  << _NORMAL_CONSOLE_TEXT_ << std::endl;
        stopAltitudeControl();
        return;
    }

    // Check if target reached
    const double altitude_error = m_target_altitude - current_altitude;
    
    std::cout << _INFO_CONSOLE_BOLD_TEXT << "DEPILOT: Altitude control update" 
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
    std::cout << "  - Current altitude: " << current_altitude << "m" << std::endl;
    std::cout << "  - Target altitude: " << m_target_altitude << "m" << std::endl;
    std::cout << "  - Altitude error: " << altitude_error << "m" << std::endl;
    std::cout << "  - Deadband: " << m_deadband << "m" << std::endl;
    
    if (std::abs(altitude_error) < m_deadband) {
        std::cout << _SUCCESS_CONSOLE_TEXT_ << "DEPILOT: Target altitude reached" 
                  << _NORMAL_CONSOLE_TEXT_ << std::endl;
        stopAltitudeControl();
        return;
    }

    // Send control command based on mode
    if (vehicle_info.flying_mode == VEHICLE_MODE_GUIDED) {
        // In GUIDED mode, use direct MAVLink changeAltitude command
        std::cout << "  - GUIDED mode: Sending changeAltitude(" << m_target_altitude << ") command" << std::endl;
        mavlinksdk::CMavlinkCommand::getInstance().changeAltitude(m_target_altitude);
    } else {
        // ALT-HOLD or STABILIZE - use RC override
        int16_t throttle_adj = calculateThrottleAdjustment(current_altitude);
        std::cout << "  - Manual mode: Calculated throttle adjustment: " << throttle_adj << std::endl;
        std::cout << "  - Manual mode: Sending throttle PWM: " << (1500 + throttle_adj) << std::endl;
        
        int16_t rc_channels[RC_CHANNELS_MAX] = {SKIP_RC_CHANNEL};
        rc_channels[fcbMain.getRCChannelsMapInfo().rcmap_throttle] = 1500 + throttle_adj;
        mavlinksdk::CMavlinkCommand::getInstance().sendRCChannels(
            rc_channels, RC_CHANNELS_MAX);
    }

    m_last_update_time = now;
}

void CDEPilotAltitude::stopAltitudeControl() {
    if (!m_active) {
        return;
    }

    std::cout << _INFO_CONSOLE_BOLD_TEXT << "DEPILOT: Stopping altitude control" 
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
    
    m_active = false;
    
    de::fcb::CFCBMain &fcbMain = de::fcb::CFCBMain::getInstance();
    const ANDRUAV_VEHICLE_INFO &vehicle_info = fcbMain.getAndruavVehicleInfo();
    
    // Send neutral control for manual modes only
    if (vehicle_info.flying_mode != VEHICLE_MODE_GUIDED) {
        int16_t rc_channels[RC_CHANNELS_MAX] = {SKIP_RC_CHANNEL};
        rc_channels[fcbMain.getRCChannelsMapInfo().rcmap_throttle] = 1500;
        mavlinksdk::CMavlinkCommand::getInstance().sendRCChannels(
            rc_channels, RC_CHANNELS_MAX);
    }
    
    // Clear operation flag
    fcbMain.setDEPilotOperation(DEPILOT_OP_ALTITUDE_CONTROL, false);
}

bool CDEPilotAltitude::isAltitudeReached() const {
    if (!m_active) {
        return true;
    }
    
    const double current_altitude = mavlinksdk::CVehicle::getInstance().getMsgGlobalPositionInt().relative_alt / 1000.0;
    const double altitude_error = m_target_altitude - current_altitude;
    return std::abs(altitude_error) < m_deadband;
}

bool CDEPilotAltitude::isAltitudeControlActive() const {
    return m_active;
}

int16_t CDEPilotAltitude::calculateThrottleAdjustment(double current_altitude) {
    const double error = m_target_altitude - current_altitude;
    const uint64_t now = get_time_usec();
    const double dt = (now - m_last_update_time) / 1000000.0; // seconds

    if (dt <= 0) return 0;

    // PID calculation
    m_integral += error * dt;
    const double derivative = (error - m_last_error) / dt;
    
    const double output = m_pid_p * error + m_pid_i * m_integral + m_pid_d * derivative;
    
    m_last_error = error;
    
    // Convert to throttle adjustment (PWM range)
    int16_t throttle_adj = static_cast<int16_t>(output * 100.0);
    
    // Clamp to safe range
    if (throttle_adj > m_max_throttle_adjustment) throttle_adj = static_cast<int16_t>(m_max_throttle_adjustment);
    if (throttle_adj < -m_max_throttle_adjustment) throttle_adj = static_cast<int16_t>(-m_max_throttle_adjustment);
    
    return throttle_adj;
}

float CDEPilotAltitude::calculateClimbRate(double current_altitude) {
    const double error = m_target_altitude - current_altitude;
    
    // Simple proportional control for climb rate
    float climb_rate = static_cast<float>(error * m_pid_p);
    
    // Clamp to max climb rate
    if (climb_rate > m_max_climb_rate) climb_rate = m_max_climb_rate;
    if (climb_rate < -m_max_climb_rate) climb_rate = -m_max_climb_rate;
    
    return climb_rate;
}
