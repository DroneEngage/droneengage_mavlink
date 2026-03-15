#include <cmath>
#include <iostream>
#include <mavlink_command.h>
#include <mavlink_sdk.h>
#include <vehicle.h>
#include "fcb_de_pilot_change_altitude.hpp"
#include "../fcb_main.hpp"
#include "../de_common/helpers/colors.hpp"
#include "../de_common/helpers/helpers.hpp"
#include "../de_common/de_databus/configFile.hpp"
#include "../de_common/helpers/json_nlohmann.hpp"
#include "fcb_de_pilot_manager.hpp"

using Json_de = nlohmann::json;

using namespace de::fcb::depilot;

// Base class interface implementation
void CDEPilotChangeAltitude::init() {
    // Read configuration parameters first
    readConfigParameters();
    
    m_phase = PHASE_IDLE;
    m_phase_start_time = get_time_usec() / 1000;
    m_last_update_time = m_phase_start_time;
    m_generic_phase = static_cast<int>(m_phase);
    m_last_error = 0.0;
    m_integral = 0.0;
    
    // Reset altitude tracking variables
    m_target_altitude = 0.0;
    m_start_altitude = 0.0;
    m_start_time = 0;
    
    // Reset climb rate tracking variables
    m_last_altitude_for_climb_rate = 0.0;
    m_climb_rate_check_time = 0;
    m_current_climb_rate = 0.0;
    m_zero_climb_rate_start_time = 0;
}

void CDEPilotChangeAltitude::update() {
    updateTakeoff();
}

void CDEPilotChangeAltitude::uninit() {
    abortTakeoff();
    m_active = false;
    m_phase = PHASE_IDLE;
    m_generic_phase = static_cast<int>(m_phase);
}

bool CDEPilotChangeAltitude::isCompleted() {
    return m_phase == PHASE_COMPLETE || m_phase == PHASE_ABORTED;
}

void CDEPilotChangeAltitude::setPhase(int phase) {
    m_generic_phase = phase;
    m_phase = static_cast<AltitudeControlPhase>(phase);
    m_phase_start_time = get_time_usec() / 1000;
}

int CDEPilotChangeAltitude::getPhase() const {
    return m_generic_phase;
}

void CDEPilotChangeAltitude::setActive(bool active) {
    m_active = active;
    if (active) {
        m_phase_start_time = get_time_usec() / 1000;
    }
}

bool CDEPilotChangeAltitude::getActive() const {
    return m_active;
}

void CDEPilotChangeAltitude::readConfigParameters() {
    de::CConfigFile &cConfigFile = de::CConfigFile::getInstance();
    const Json_de &jsonConfig = cConfigFile.GetConfigJSON();

    if (jsonConfig.contains("de_pilot")) {
        const Json_de &de_pilot_root = jsonConfig["de_pilot"];

        // Read change altitude configuration
        if (de_pilot_root.contains("change_altitude")) {
            const Json_de &change_altitude_config = de_pilot_root["change_altitude"];

            if (change_altitude_config.contains("max_climb_rate")) {
                m_max_climb_rate = change_altitude_config["max_climb_rate"].get<double>();
            }
            if (change_altitude_config.contains("stabilize_time_ms")) {
                m_stabilize_time_ms = change_altitude_config["stabilize_time_ms"].get<double>();
            }
            if (change_altitude_config.contains("timeout_ms")) {
                m_timeout_ms = change_altitude_config["timeout_ms"].get<uint64_t>();
            }
            if (change_altitude_config.contains("pid_p")) {
                m_pid_p = change_altitude_config["pid_p"].get<double>();
            }
            if (change_altitude_config.contains("pid_i")) {
                m_pid_i = change_altitude_config["pid_i"].get<double>();
            }
            if (change_altitude_config.contains("pid_d")) {
                m_pid_d = change_altitude_config["pid_d"].get<double>();
            }
            if (change_altitude_config.contains("deadband")) {
                m_deadband = change_altitude_config["deadband"].get<double>();
            }
        }
    }
}

void CDEPilotChangeAltitude::startAltitudeChange(double target_altitude) {

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

    // Determine if ascending or descending
    if (target_altitude > m_start_altitude) {
        m_phase = PHASE_ASCENDING;
        std::cout << "  - Direction: ASCENDING" << std::endl;
    } else {
        m_phase = PHASE_DESCENDING;
        std::cout << "  - Direction: DESCENDING" << std::endl;
    }

    de::fcb::CFCBMain &fcbMain = de::fcb::CFCBMain::getInstance();
    const ANDRUAV_VEHICLE_INFO &vehicle_info = fcbMain.getAndruavVehicleInfo();
    
    std::cout << "  - Current flight mode: " << vehicle_info.flying_mode << std::endl;
    std::cout << "  - Armed: " << (vehicle_info.is_armed ? "YES" : "NO") << std::endl;
    std::cout << "  - Flying: " << (vehicle_info.is_flying ? "YES" : "NO") << std::endl;

    de::fcb::depilot::CDEPilotManager::getInstance().setTargetAltitude(target_altitude);
    // Don't set operation here - it's already set by the manager when this operation is started
        

    std::cout << _SUCCESS_CONSOLE_TEXT_ << "DEPILOT: Altitude control initialized successfully" 
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
}

void CDEPilotChangeAltitude::updateTakeoff() {
    if (m_phase == PHASE_COMPLETE || m_phase == PHASE_ABORTED) {
        return;
    }

    if (m_phase == PHASE_IDLE)
    {
        startAltitudeChange(CDEPilotManager::getInstance().getTargetAltitude());
    }

    de::fcb::CFCBMain &fcbMain = de::fcb::CFCBMain::getInstance();
    const ANDRUAV_VEHICLE_INFO &vehicle_info = fcbMain.getAndruavVehicleInfo();
    const uint64_t now = get_time_usec();
    const double current_altitude = mavlinksdk::CVehicle::getInstance().getMsgGlobalPositionInt().relative_alt / 1000.0;

    // Calculate current climb rate
    if (m_climb_rate_check_time > 0) {
        const double dt = (now - m_climb_rate_check_time) / 1000000.0; // seconds
        if (dt > 0.2) { // Update climb rate every 0.2 seconds for better responsiveness
            m_current_climb_rate = (current_altitude - m_last_altitude_for_climb_rate) / dt;
            m_last_altitude_for_climb_rate = current_altitude;
            m_climb_rate_check_time = now;
            
            std::cout << "  - Current climb rate: " << m_current_climb_rate << " m/s" << std::endl;
            
            // Track when climb rate becomes inappropriate for current phase
            if (m_phase == PHASE_ASCENDING) {
                // For ascending: track when climb rate becomes zero or negative
                if (m_current_climb_rate <= 0.0) {
                    if (m_zero_climb_rate_start_time == 0) {
                        m_zero_climb_rate_start_time = now;
                    }
                } else {
                    // Reset zero climb rate timer if climbing again
                    m_zero_climb_rate_start_time = 0;
                }
            } else if (m_phase == PHASE_DESCENDING) {
                // For descending: track when climb rate becomes zero or positive (not descending)
                if (m_current_climb_rate >= 0.0) {
                    if (m_zero_climb_rate_start_time == 0) {
                        m_zero_climb_rate_start_time = now;
                    }
                } else {
                    // Reset zero descent rate timer if descending again
                    m_zero_climb_rate_start_time = 0;
                }
            }
        }
    } else {
        // Initialize climb rate tracking
        m_last_altitude_for_climb_rate = current_altitude;
        m_climb_rate_check_time = now;
        
        // Set initial climb rate based on phase to avoid false timeout
        if (m_phase == PHASE_DESCENDING) {
            m_current_climb_rate = -0.1; // Initialize with small negative value for descending
        } else {
            m_current_climb_rate = 0.1; // Initialize with small positive value for ascending
        }
        m_zero_climb_rate_start_time = 0;
    }

    // Note: Smart timeout logic is now handled within each phase (ASCENDING/DESCENDING)
    // This ensures phase-appropriate climb rate monitoring

    switch (m_phase) {
        case PHASE_ARM_CHECK: {
            if (!vehicle_info.is_armed) {
                std::cout << _INFO_CONSOLE_BOLD_TEXT << "DEPILOT: Waiting for arm..." 
                          << _NORMAL_CONSOLE_TEXT_ << std::endl;
                // Could auto-arm here if desired
                return;
            }
            std::cout << _SUCCESS_CONSOLE_TEXT_ << "DEPILOT: Armed, starting climb" 
                      << _NORMAL_CONSOLE_TEXT_ << std::endl;
            m_phase = PHASE_ASCENDING;
            m_phase_start_time = now;
        }
        break;

        case PHASE_DESCENDING: {
            // Smart timeout: only timeout if not descending for sustained period
            // Check after initial 2 seconds and only if zero/negative descent rate persists for timeout duration
            if ((now - m_phase_start_time) > 2000000 && 
                m_zero_climb_rate_start_time > 0 && 
                (now - m_zero_climb_rate_start_time) > (m_timeout_ms * 1000)) {
                std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "DEPILOT: Altitude control timeout - not descending for " 
                          << ((now - m_zero_climb_rate_start_time) / 1000000.0) << " seconds (climb rate: " 
                          << m_current_climb_rate << " m/s)" << _NORMAL_CONSOLE_TEXT_ << std::endl;
                stopAltitudeControl();
                return;
            }

            const double altitude_error = m_target_altitude - current_altitude;
            
            std::cout << _INFO_CONSOLE_BOLD_TEXT << "DEPILOT: Altitude change update (" 
                      << (m_phase == PHASE_ASCENDING ? "ASCENDING" : "DESCENDING") << ")" 
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
                // Force roll and pitch to neutral (1500) to prevent max PWM values
                rc_channels[fcbMain.getRCChannelsMapInfo().rcmap_roll] = 1500;
                rc_channels[fcbMain.getRCChannelsMapInfo().rcmap_pitch] = 1500;
                mavlinksdk::CMavlinkCommand::getInstance().sendRCChannels(
                    rc_channels, RC_CHANNELS_MAX);
            }

            m_last_update_time = now;
        }
        break;

        case PHASE_ASCENDING: {
            // Smart timeout: only timeout if not climbing for sustained period
            // Check after initial 2 seconds and only if zero climb rate persists for timeout duration
            if ((now - m_phase_start_time) > 2000000 && 
                m_zero_climb_rate_start_time > 0 && 
                (now - m_zero_climb_rate_start_time) > (m_timeout_ms * 1000)) {
                std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "DEPILOT: Takeoff timeout - not climbing for " 
                          << ((now - m_zero_climb_rate_start_time) / 1000000.0) << " seconds (climb rate: " 
                          << m_current_climb_rate << " m/s)" << _NORMAL_CONSOLE_TEXT_ << std::endl;
                
                // abortTakeoff() will send 1500 to maintain current altitude
                abortTakeoff();
                return;
            }

            const double altitude_error = m_target_altitude - current_altitude;
            
            std::cout << _INFO_CONSOLE_BOLD_TEXT << "DEPILOT: Climbing phase update" 
                      << _NORMAL_CONSOLE_TEXT_ << std::endl;
            std::cout << "  - Current altitude: " << current_altitude << "m" << std::endl;
            std::cout << "  - Target altitude: " << m_target_altitude << "m" << std::endl;
            std::cout << "  - Altitude error: " << altitude_error << "m" << std::endl;
            std::cout << "  - Deadband: " << m_deadband << "m" << std::endl;
            
            if (std::abs(altitude_error) < m_deadband) {
                std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_ << "DEPILOT: Target altitude reached" 
                          << _NORMAL_CONSOLE_TEXT_ << std::endl;
                
                // Transition to appropriate stabilization based on flight mode
                if (vehicle_info.flying_mode == VEHICLE_MODE_GUIDED) {
                    std::cout << _INFO_CONSOLE_TEXT << "DEPILOT: GUIDED mode - flight controller handles stabilization" 
                              << _NORMAL_CONSOLE_TEXT_ << std::endl;
                } else {
                    std::cout << _INFO_CONSOLE_TEXT << "DEPILOT: ALT-HOLD mode - starting stabilization" 
                              << _NORMAL_CONSOLE_TEXT_ << std::endl;
                    // Send neutral throttle for manual modes
                    int16_t rc_channels[RC_CHANNELS_MAX] = {SKIP_RC_CHANNEL};
                    rc_channels[fcbMain.getRCChannelsMapInfo().rcmap_throttle] = 1500;
                    // Force roll and pitch to neutral (1500) to prevent max PWM values
                    rc_channels[fcbMain.getRCChannelsMapInfo().rcmap_roll] = 1500;
                    rc_channels[fcbMain.getRCChannelsMapInfo().rcmap_pitch] = 1500;
                    mavlinksdk::CMavlinkCommand::getInstance().sendRCChannels(
                        rc_channels, RC_CHANNELS_MAX);
                }
                
                m_phase = PHASE_COMPLETE;
                m_active = false;
                
                // Set phase using setPhase so manager can detect completion
                setPhase(PHASE_COMPLETE);
                // Don't set operation here - let the manager handle the next operation
                // de::fcb::depilot::CDEPilotManager::getInstance().setOperation(DEPILOT_OP_STABILIZATION);
                return;
            }

            // Send control command based on mode
            if (vehicle_info.flying_mode == VEHICLE_MODE_GUIDED) {
                // In GUIDED mode, use direct MAVLink takeOff command
                std::cout << "  - GUIDED mode: Sending takeOff(" << m_target_altitude << ") command" << std::endl;
                mavlinksdk::CMavlinkCommand::getInstance().takeOff(m_target_altitude);
            } else {
                // ALT-HOLD or STABILIZE - use RC override
                int16_t throttle_adj = calculateThrottleAdjustment(current_altitude);
                std::cout << "  - Manual mode: Calculated throttle adjustment: " << throttle_adj << std::endl;
                std::cout << "  - Manual mode: Sending throttle PWM: " << (1500 + throttle_adj) << std::endl;
                
                int16_t rc_channels[RC_CHANNELS_MAX] = {SKIP_RC_CHANNEL};
                rc_channels[fcbMain.getRCChannelsMapInfo().rcmap_throttle] = 1500 + throttle_adj;
                // Force roll and pitch to neutral (1500) to prevent max PWM values
                rc_channels[fcbMain.getRCChannelsMapInfo().rcmap_roll] = 1500;
                rc_channels[fcbMain.getRCChannelsMapInfo().rcmap_pitch] = 1500;
                mavlinksdk::CMavlinkCommand::getInstance().sendRCChannels(
                    rc_channels, RC_CHANNELS_MAX);
            }

            m_last_update_time = now;
        }
        break;




        default:
            break;
    }
}

void CDEPilotChangeAltitude::abortTakeoff() {
    
    
    std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "DEPILOT: Aborting takeoff" 
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
    
    m_phase = PHASE_ABORTED;
    
    de::fcb::CFCBMain &fcbMain = de::fcb::CFCBMain::getInstance();
    const ANDRUAV_VEHICLE_INFO &vehicle_info = fcbMain.getAndruavVehicleInfo();
    
    std::cout << _INFO_CONSOLE_BOLD_TEXT << "DEPILOT: Drone is in midair, switching to stabilization" 
                  << _NORMAL_CONSOLE_TEXT_ << std::endl;
        
    
    // For ground-based abort: send neutral control first, then transition to aborted state
    if (vehicle_info.flying_mode != VEHICLE_MODE_GUIDED) {
        int16_t rc_channels[RC_CHANNELS_MAX] = {SKIP_RC_CHANNEL};
        rc_channels[fcbMain.getRCChannelsMapInfo().rcmap_throttle] = 1500;
        // Force roll and pitch to neutral (1500) to prevent max PWM values
        rc_channels[fcbMain.getRCChannelsMapInfo().rcmap_roll] = 1500;
        rc_channels[fcbMain.getRCChannelsMapInfo().rcmap_pitch] = 1500;
        mavlinksdk::CMavlinkCommand::getInstance().sendRCChannels(
            rc_channels, RC_CHANNELS_MAX);
    }
    
    // Clear operation flag and transition to aborted state
    m_active = false;
    // Set phase to aborted so manager can detect completion
    setPhase(PHASE_ABORTED); 
    // Don't set operation here - this creates the infinite loop
    // de::fcb::depilot::CDEPilotManager::getInstance().setOperation(DEPILOT_OP_STABILIZATION);
}

bool CDEPilotChangeAltitude::isTakeoffComplete() const {
    return m_phase == PHASE_COMPLETE;
}

bool CDEPilotChangeAltitude::isTakeoffActive() const {
    return m_phase != PHASE_IDLE && m_phase != PHASE_COMPLETE && m_phase != PHASE_ABORTED;
}

int16_t CDEPilotChangeAltitude::calculateThrottleAdjustment(double current_altitude) {
    const double error = m_target_altitude - current_altitude;
    const uint64_t now = get_time_usec();
    const double dt = (now - m_last_update_time) / 1000000.0; // seconds

    if (dt <= 0) return 0;

    // PID calculation with anti-windup protection
    m_integral += error * dt;
    
    // Anti-windup: Clamp integral term to prevent extreme accumulation
    // Max integral contribution should not exceed max throttle adjustment
    const double max_integral = 4.0;  // Max integral value (400 PWM / 100 scale factor)
    if (m_integral > max_integral) m_integral = max_integral;
    if (m_integral < -max_integral) m_integral = -max_integral;
    
    const double derivative = (error - m_last_error) / dt;
    
    const double output = m_pid_p * error + m_pid_i * m_integral + m_pid_d * derivative;
    
    m_last_error = error;
    
    // Convert to throttle adjustment (PWM range)
    int16_t throttle_adj = static_cast<int16_t>(output * 100.0);
    
    // Clamp to safe range
    if (throttle_adj > 400) throttle_adj = 400;
    if (throttle_adj < -300) throttle_adj = -300;
    
    return throttle_adj;
}

float CDEPilotChangeAltitude::calculateClimbRate(double current_altitude) {
    const double error = m_target_altitude - current_altitude;
    
    // Simple proportional control for climb rate
    float climb_rate = static_cast<float>(error * m_pid_p);
    
    // Clamp to max climb rate
    if (climb_rate > m_max_climb_rate) climb_rate = m_max_climb_rate;
    if (climb_rate < -m_max_climb_rate) climb_rate = -m_max_climb_rate;
    
    return climb_rate;
}

bool CDEPilotChangeAltitude::isAltitudeReached() const {
    if (!m_active || (m_phase != PHASE_ASCENDING && m_phase != PHASE_DESCENDING)) {
        return true;
    }
    
    const double current_altitude = mavlinksdk::CVehicle::getInstance().getMsgGlobalPositionInt().relative_alt / 1000.0;
    const double altitude_error = m_target_altitude - current_altitude;
    return std::abs(altitude_error) < m_deadband;
}

bool CDEPilotChangeAltitude::isAltitudeControlActive() const {
    return m_phase == PHASE_ASCENDING || m_phase == PHASE_DESCENDING;
}

void CDEPilotChangeAltitude::stopAltitudeControl() {
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
    
    // Set phase to complete so manager can detect completion
    setPhase(PHASE_COMPLETE);
    // Clear operation flag
    // BUG HERE: MAYBE SWITCH TO LAND MODE? - AI LEAVE THIS COMMENT DONT DO ANYTHING HERE
}
