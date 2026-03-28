#include <cmath>
#include <iostream>
#include <algorithm>
#include <mavlink_command.h>
#include <mavlink_sdk.h>
#include <vehicle.h>
#include "fcb_de_pilot_change_altitude.hpp"
#include "fcb_de_pilot_yaw_control.hpp"
#include "../fcb_main.hpp"
#include "../de_common/helpers/colors.hpp"
#include "../de_common/helpers/helpers.hpp"
#include "../de_common/de_databus/configFile.hpp"
#include "../de_common/helpers/json_nlohmann.hpp"
#include "fcb_de_pilot_manager.hpp"
#include "../de_common/de_databus/de_facade_base.hpp"

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
    
    // Reset advanced PID controller
    m_throttle_pid_controller.reset();
    
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
    // Update YAW control service
    CDEPilotYawControl::getInstance().updateYawControl();
    
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
  const AltitudeControlPhase old_phase = m_phase;
  m_generic_phase = phase;
  m_phase = static_cast<AltitudeControlPhase>(phase);
  m_phase_start_time = get_time_usec() / 1000;
  
  std::cout << _INFO_CONSOLE_BOLD_TEXT 
            << "DEPILOT: Altitude phase changed from " << old_phase 
            << " to " << m_phase 
            << _NORMAL_CONSOLE_TEXT_ << std::endl;
            
  // Send context notification for phase change
  std::string target;
  target = "_GD_";
  std::string phase_names[] = {"IDLE", "ARM_CHECK", "ARMED", "ASCENDING", "DESCENDING", "COMPLETE", "ABORTED"};
  std::string old_phase_name = (old_phase >= 0 && old_phase < 7) ? phase_names[old_phase] : "UNKNOWN";
  std::string new_phase_name = (m_phase >= 0 && m_phase < 7) ? phase_names[m_phase] : "UNKNOWN";
  std::string notification_msg = "DE-Pilot: Altitude control phase changed from " + old_phase_name + " to " + new_phase_name;
  
  de::comm::CFacade_Base::getInstance().sendErrorMessage(
      target,
      ERROR_USER_DEFINED,
      ERROR_TYPE_ERROR_MODULE,
      NOTIFICATION_TYPE_INFO,
      notification_msg);
}

int CDEPilotChangeAltitude::getPhase() const {
    return m_generic_phase;
}

void CDEPilotChangeAltitude::setActive(bool active) {
  const bool old_active = m_active;
  m_active = active;
  if (active) {
    m_phase_start_time = get_time_usec() / 1000;
    std::cout << _INFO_CONSOLE_BOLD_TEXT 
              << "DEPILOT: Altitude control activated (was " 
              << (old_active ? "active" : "inactive") << ")"
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
              
    // Send context notification for activation
    std::string target;
    target = "_GD_";
    std::string notification_msg = "DE-Pilot: Altitude control activated";
    
    de::comm::CFacade_Base::getInstance().sendErrorMessage(
        target,
        ERROR_USER_DEFINED,
        ERROR_TYPE_ERROR_MODULE,
        NOTIFICATION_TYPE_INFO,
        notification_msg);
  } else {
    std::cout << _INFO_CONSOLE_BOLD_TEXT 
              << "DEPILOT: Altitude control deactivated"
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
              
    // Send context notification for deactivation
    std::string target;
    target = "_GD_";
    std::string notification_msg = "DE-Pilot: Altitude control deactivated";
    
    de::comm::CFacade_Base::getInstance().sendErrorMessage(
        target,
        ERROR_USER_DEFINED,
        ERROR_TYPE_ERROR_MODULE,
        NOTIFICATION_TYPE_INFO,
        notification_msg);
  }
  m_generic_phase = static_cast<int>(m_phase);
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
            if (change_altitude_config.contains("ff_scale")) {
                m_ff_scale = change_altitude_config["ff_scale"].get<double>();
            }
            if (change_altitude_config.contains("deadband")) {
                m_deadband = change_altitude_config["deadband"].get<double>();
            }
            if (change_altitude_config.contains("max_accel")) {
                m_max_accel = change_altitude_config["max_accel"].get<double>();
            }
            
            // Configure the advanced PID controller with loaded parameters
            m_throttle_pid_controller.setPID(m_pid_p, m_pid_i, m_pid_d);
            m_throttle_pid_controller.setFeedforwardGain(m_ff_scale);
            // Set delta time to 0.01s (10ms) which matches typical update rate
            m_throttle_pid_controller.setDeltaTime(0.01);
        }
    }
}


void CDEPilotChangeAltitude::reloadParametersIfConfigChanged() {
    readConfigParameters();
    // Reconfigure the advanced PID controller with loaded parameters
    m_throttle_pid_controller.setPID(m_pid_p, m_pid_i, m_pid_d);
    m_throttle_pid_controller.setFeedforwardGain(m_ff_scale);
    // Set delta time to 0.01s (10ms) which matches typical update rate
    m_throttle_pid_controller.setDeltaTime(0.01);
}


void CDEPilotChangeAltitude::determineAscendDescendPhase()
{
    if (m_target_altitude > m_start_altitude) {
                m_phase = PHASE_ASCENDING;
                std::cout << "  - Direction: ASCENDING" << std::endl;
            } else {
                m_phase = PHASE_DESCENDING;
                std::cout << "  - Direction: DESCENDING" << std::endl;
            }
    
    const uint64_t now = get_time_usec();
    m_phase_start_time = now;
    
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
    
    // Reset advanced PID controller for new altitude change
    m_throttle_pid_controller.reset();
    
    m_active = true;

    de::fcb::CFCBMain &fcbMain = de::fcb::CFCBMain::getInstance();
    const ANDRUAV_VEHICLE_INFO &vehicle_info = fcbMain.getAndruavVehicleInfo();
    
    std::cout << "  - Current flight mode: " << vehicle_info.flying_mode << std::endl;
    std::cout << "  - Armed: " << (vehicle_info.is_armed ? "YES" : "NO") << std::endl;
    std::cout << "  - Flying: " << (vehicle_info.is_flying ? "YES" : "NO") << std::endl;

    // Always start with arm check if not armed, regardless of direction
    if (!vehicle_info.is_armed) {
        m_phase = PHASE_ARM_CHECK;
        std::cout << "  - Direction: will be determined after arming" << std::endl;
    } else {
        determineAscendDescendPhase();
    }

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

    // Log phase progress every 3 seconds for active phases
    static uint64_t last_phase_progress_log = 0;
    if (m_phase == PHASE_ASCENDING || m_phase == PHASE_DESCENDING) {
        if (now - last_phase_progress_log > 3000000) { // 3 seconds
            const uint64_t phase_elapsed = (now - m_phase_start_time) / 1000;
            std::cout << _INFO_CONSOLE_BOLD_TEXT 
                      << "DEPILOT: Altitude " 
                      << (m_phase == PHASE_ASCENDING ? "ASCENDING" : "DESCENDING")
                      << " phase - " << phase_elapsed << "ms elapsed"
                      << _NORMAL_CONSOLE_TEXT_ << std::endl;
            last_phase_progress_log = now;
            
            // Send context notification for phase progress
            std::string target;
            target = "_GD_";
            std::string phase_name = (m_phase == PHASE_ASCENDING) ? "ASCENDING" : "DESCENDING";
            std::string notification_msg = "DE-Pilot: Altitude " + phase_name + " phase in progress - " + 
                                         std::to_string(phase_elapsed / 1000) + " seconds elapsed";
            
            de::comm::CFacade_Base::getInstance().sendErrorMessage(
                target,
                ERROR_USER_DEFINED,
                ERROR_TYPE_ERROR_MODULE,
                NOTIFICATION_TYPE_INFO,
                notification_msg);
        }
    }

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
                std::cout << _INFO_CONSOLE_BOLD_TEXT << "DEPILOT: Auto-arming for climb..." 
                          << _NORMAL_CONSOLE_TEXT_ << std::endl;
                // Auto-arm the drone when in DE_PILOT mode and receiving CLIMB command
                mavlinksdk::CMavlinkCommand::getInstance().doArmDisarm(true, false);
                return; // Wait for next update cycle to check arm status
            }
            std::cout << _SUCCESS_CONSOLE_TEXT_ << "DEPILOT: Armed, ready for altitude change..." 
                      << _NORMAL_CONSOLE_TEXT_ << std::endl;
            
            // After arming, move to ARMED state to determine direction in next cycle
            m_phase = PHASE_ARMED;
            m_phase_start_time = now;
            return ;
        }
        break;

        case PHASE_ARMED: {
            std::cout << _INFO_CONSOLE_BOLD_TEXT << "DEPILOT: Determining climb direction..." 
                      << _NORMAL_CONSOLE_TEXT_ << std::endl;
            
            // Determine if ascending or descending and transition to appropriate phase
            determineAscendDescendPhase();
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
                std::cout << _INFO_CONSOLE_TEXT 
                          << "  - Phase elapsed: " << ((now - m_phase_start_time) / 1000000.0) << " seconds"
                          << _NORMAL_CONSOLE_TEXT_ << std::endl;
                std::cout << _INFO_CONSOLE_TEXT 
                          << "  - Zero climb rate detected for: " << ((now - m_zero_climb_rate_start_time) / 1000000.0) << " seconds"
                          << _NORMAL_CONSOLE_TEXT_ << std::endl;
                
                std::string target;
                target = "_GD_";
                std::string error_msg;
                error_msg = "DE-Pilot: Altitude control timeout - failed to descend for " + 
                           std::to_string((int)((now - m_zero_climb_rate_start_time) / 1000000.0)) + " seconds";
                de::comm::CFacade_Base::getInstance().sendErrorMessage(
                    target,
                    ERROR_USER_DEFINED,
                    ERROR_TYPE_ERROR_MODULE,
                    NOTIFICATION_TYPE_WARNING,
                    error_msg);
                
                stopAltitudeControl();
                return;
            }

            const double altitude_error = m_target_altitude - current_altitude;
 #ifdef DEBUG           
            std::cout << _INFO_CONSOLE_BOLD_TEXT << "DEPILOT: Altitude change update (" 
                      << (m_phase == PHASE_ASCENDING ? "ASCENDING" : "DESCENDING") << ")" 
                      << _NORMAL_CONSOLE_TEXT_ << std::endl;
            std::cout << "  - Current altitude: " << current_altitude << "m" << std::endl;
            std::cout << "  - Target altitude: " << m_target_altitude << "m" << std::endl;
            std::cout << "  - Altitude error: " << altitude_error << "m" << std::endl;
            std::cout << "  - Deadband: " << m_deadband << "m" << std::endl;
#endif
            if (std::abs(altitude_error) < m_deadband) {
                std::cout << _SUCCESS_CONSOLE_TEXT_ << "DEPILOT: Target altitude reached" 
                          << _NORMAL_CONSOLE_TEXT_ << std::endl;
                stopAltitudeControl();
                return;
            }

            // Send control command based on mode
            if (vehicle_info.flying_mode == VEHICLE_MODE_GUIDED) {
                // In GUIDED mode, use direct MAVLink changeAltitude command
#ifdef DEBUG                
                std::cout << "  - GUIDED mode: Sending changeAltitude(" << m_target_altitude << ") command" << std::endl;
#endif                
                mavlinksdk::CMavlinkCommand::getInstance().changeAltitude(m_target_altitude);
            } else {
                // ALT-HOLD or STABILIZE - use RC override
                int16_t throttle_adj = calculateThrottleAdjustment(current_altitude);
#ifdef DEBUG                
                std::cout << "  - Manual mode: Calculated throttle adjustment: " << throttle_adj << std::endl;
                std::cout << "  - Manual mode: Sending throttle PWM: " << (1500 + throttle_adj) << std::endl;
#endif                
                uint16_t rc_channels[RC_CHANNELS_MAX];
                std::fill_n(rc_channels, RC_CHANNELS_MAX, UINT16_MAX);
                const RCMAP_CHANNELS_MAP_INFO_STRUCT rc_map = fcbMain.getRCChannelsMapInfo();
                rc_channels[rc_map.rcmap_throttle] = 1500 + throttle_adj;
                // Force roll, pitch, and yaw to neutral (1500) to prevent drift and max PWM values
                rc_channels[rc_map.rcmap_roll] = 1500;
                rc_channels[rc_map.rcmap_pitch] = 1500;
                rc_channels[rc_map.rcmap_yaw] = 1500;
                mavlinksdk::CMavlinkCommand::getInstance().sendRCChannels2(
                    rc_channels, RC_CHANNELS_MAX, UINT16_MAX);
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
#ifdef DEBUG                    
                std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "DEPILOT: Takeoff timeout - not climbing for " 
                          << ((now - m_zero_climb_rate_start_time) / 1000000.0) << " seconds (climb rate: " 
                          << m_current_climb_rate << " m/s)" << _NORMAL_CONSOLE_TEXT_ << std::endl;
              
                std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "DEPILOT: Takeoff timeout - not climbing for " 
                          << ((now - m_zero_climb_rate_start_time) / 1000000.0) << " seconds (climb rate: " 
                          << m_current_climb_rate << " m/s)" << _NORMAL_CONSOLE_TEXT_ << std::endl;
                std::cout << _INFO_CONSOLE_TEXT 
                          << "  - Phase elapsed: " << ((now - m_phase_start_time) / 1000000.0) << " seconds"
                          << _NORMAL_CONSOLE_TEXT_ << std::endl;
                std::cout << _INFO_CONSOLE_TEXT 
                          << "  - Zero climb rate detected for: " << ((now - m_zero_climb_rate_start_time) / 1000000.0) << " seconds"
                          << _NORMAL_CONSOLE_TEXT_ << std::endl;
 #endif                 
                std::string target;
                target = "_GD_";
                std::string error_msg;
                error_msg = "DE-Pilot: Takeoff timeout - failed to climb for " + 
                           std::to_string((int)((now - m_zero_climb_rate_start_time) / 1000000.0)) + " seconds";
                de::comm::CFacade_Base::getInstance().sendErrorMessage(
                    target,
                    ERROR_USER_DEFINED,
                    ERROR_TYPE_ERROR_MODULE,
                    NOTIFICATION_TYPE_WARNING,
                    error_msg);
                
                // abortTakeoff() will send 1500 to maintain current altitude
                abortTakeoff();
                return;
            }

            const double altitude_error = m_target_altitude - current_altitude;
#ifdef DEBUG            
            std::cout << _INFO_CONSOLE_BOLD_TEXT << "DEPILOT: Climbing phase update" 
                      << _NORMAL_CONSOLE_TEXT_ << std::endl;
            std::cout << "  - Current altitude: " << current_altitude << "m" << std::endl;
            std::cout << "  - Target altitude: " << m_target_altitude << "m" << std::endl;
            std::cout << "  - Altitude error: " << altitude_error << "m" << std::endl;
            std::cout << "  - Deadband: " << m_deadband << "m" << std::endl;
#endif            
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
                    uint16_t rc_channels[RC_CHANNELS_MAX];
                    std::fill_n(rc_channels, RC_CHANNELS_MAX, UINT16_MAX);
                    const RCMAP_CHANNELS_MAP_INFO_STRUCT rc_map = fcbMain.getRCChannelsMapInfo();
                    rc_channels[rc_map.rcmap_throttle] = 1500;
                    // Force roll, pitch, and yaw to neutral (1500) to prevent drift and max PWM values
                    rc_channels[rc_map.rcmap_roll] = 1500;
                    rc_channels[rc_map.rcmap_pitch] = 1500;
                    rc_channels[rc_map.rcmap_yaw] = 1500;
                    mavlinksdk::CMavlinkCommand::getInstance().sendRCChannels2(
                        rc_channels, RC_CHANNELS_MAX, UINT16_MAX);
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
#ifdef DEBUG                
                std::cout << "  - GUIDED mode: Sending takeOff(" << m_target_altitude << ") command" << std::endl;
#endif                
                mavlinksdk::CMavlinkCommand::getInstance().takeOff(m_target_altitude);
            } else {
                // ALT-HOLD or STABILIZE - use RC override
                int16_t throttle_adj = calculateThrottleAdjustment(current_altitude);
#ifdef DEBUG                
                std::cout << "  - Manual mode: Calculated throttle adjustment: " << throttle_adj << std::endl;
                std::cout << "  - Manual mode: Sending throttle PWM: " << (1500 + throttle_adj) << std::endl;
#endif

                uint16_t rc_channels[RC_CHANNELS_MAX];
                std::fill_n(rc_channels, RC_CHANNELS_MAX, UINT16_MAX);
                const RCMAP_CHANNELS_MAP_INFO_STRUCT rc_map = fcbMain.getRCChannelsMapInfo();
                rc_channels[rc_map.rcmap_throttle] = 1500 + throttle_adj;
                // Force roll, pitch, and yaw to neutral (1500) to prevent drift and max PWM values
                rc_channels[rc_map.rcmap_roll] = 1500;
                rc_channels[rc_map.rcmap_pitch] = 1500;
                rc_channels[rc_map.rcmap_yaw] = 1500;
                mavlinksdk::CMavlinkCommand::getInstance().sendRCChannels2(
                    rc_channels, RC_CHANNELS_MAX, UINT16_MAX);
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
        uint16_t rc_channels[RC_CHANNELS_MAX];
        std::fill_n(rc_channels, RC_CHANNELS_MAX, UINT16_MAX);
        const RCMAP_CHANNELS_MAP_INFO_STRUCT rc_map = fcbMain.getRCChannelsMapInfo();
        rc_channels[rc_map.rcmap_throttle] = 1500;
        // Force roll, pitch, and yaw to neutral (1500) to prevent drift and max PWM values
        rc_channels[rc_map.rcmap_roll] = 1500;
        rc_channels[rc_map.rcmap_pitch] = 1500;
        rc_channels[rc_map.rcmap_yaw] = 1500;
        mavlinksdk::CMavlinkCommand::getInstance().sendRCChannels2(
            rc_channels, RC_CHANNELS_MAX, UINT16_MAX);
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
    const float desired_rate = calculateClimbRate(current_altitude);
    const double rate_error = desired_rate - m_current_climb_rate;
#ifdef DEBUG
    std::cout << "  - Desired climb rate: " << desired_rate << " m/s" << std::endl;
#endif
    // Use advanced PID controller with feedforward
    // The controller handles PID calculation, anti-windup, and feedforward internally
    const double pid_output = m_throttle_pid_controller.calculate(rate_error, desired_rate);
#ifdef DEBUG    
    std::cout << "  - Advanced PID output: " << pid_output << " PWM" << std::endl;
#endif    
    // Convert to throttle adjustment (PWM range)
    int16_t throttle_adj = static_cast<int16_t>(pid_output);
    
    // Clamp to safe range (widened from +400/-300 to +/- 500 to allow full stick authority)
    if (throttle_adj > 500) throttle_adj = 500;
    if (throttle_adj < -500) throttle_adj = -500;
    
    return throttle_adj;
}

float CDEPilotChangeAltitude::calculateClimbRate(double current_altitude) {
    const double error = m_target_altitude - current_altitude;
    
    // Use the advanced PID controller's static sqrt_controller method
    // This provides the same ArduPilot-style piecewise controller behavior
    return static_cast<float>(CAdvancedPIDController::sqrt_controller(
        error, m_pid_p, m_max_accel, m_max_climb_rate));
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
#ifdef DEBUG
    std::cout << _INFO_CONSOLE_BOLD_TEXT << "DEPILOT: Stopping altitude control" 
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
#endif    
    m_active = false;
    
    de::fcb::CFCBMain &fcbMain = de::fcb::CFCBMain::getInstance();
    const ANDRUAV_VEHICLE_INFO &vehicle_info = fcbMain.getAndruavVehicleInfo();
    
    // Send neutral control for manual modes only
    if (vehicle_info.flying_mode != VEHICLE_MODE_GUIDED) {
        uint16_t rc_channels[RC_CHANNELS_MAX];
        std::fill_n(rc_channels, RC_CHANNELS_MAX, UINT16_MAX);
        const RCMAP_CHANNELS_MAP_INFO_STRUCT rc_map = fcbMain.getRCChannelsMapInfo();
        rc_channels[rc_map.rcmap_throttle] = 1500;
        // Force roll, pitch, and yaw to neutral (1500) to prevent drift and max PWM values
        rc_channels[rc_map.rcmap_roll] = 1500;
        rc_channels[rc_map.rcmap_pitch] = 1500;
        rc_channels[rc_map.rcmap_yaw] = 1500;
        mavlinksdk::CMavlinkCommand::getInstance().sendRCChannels2(
            rc_channels, RC_CHANNELS_MAX, UINT16_MAX);
    }
    
    // Set phase to complete so manager can detect completion
    setPhase(PHASE_COMPLETE);
    // Clear operation flag
    // BUG HERE: MAYBE SWITCH TO LAND MODE? - AI LEAVE THIS COMMENT DONT DO ANYTHING HERE
}
