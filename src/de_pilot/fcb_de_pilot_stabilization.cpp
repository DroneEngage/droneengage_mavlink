#include "fcb_de_pilot_stabilization.hpp"
#include "fcb_de_pilot_yaw_control.hpp"
#include "../de_common/de_databus/configFile.hpp"
#include "../de_common/helpers/colors.hpp"
#include "../de_common/helpers/helpers.hpp"
#include "../fcb_main.hpp"
#include "../de_common/de_databus/de_facade_base.hpp"
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
  if (m_stabilize_duration_ms == 0) {
    return true; // Infinite stabilization: always considered completed
  }
  
  if (m_phase != PHASE_STABILIZING) {
    return (m_phase == PHASE_COMPLETE);
  }
  
  uint64_t current_time = get_time_usec() / 1000;
  const uint64_t elapsed_time = current_time - m_phase_start_time;
  
  // Log progress every 5 seconds for timed stabilization
  static uint64_t last_progress_log = 0;
  if (current_time - last_progress_log > 5000) {
    std::cout << _INFO_CONSOLE_BOLD_TEXT 
              << "DEPILOT: Stabilization progress - " 
              << elapsed_time << "ms elapsed of " << m_stabilize_duration_ms << "ms"
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
    last_progress_log = current_time;
  }
  
  if (elapsed_time >= m_stabilize_duration_ms) {
    m_phase = PHASE_COMPLETE;
    m_generic_phase = static_cast<int>(m_phase);
    std::cout << _INFO_CONSOLE_BOLD_TEXT 
              << "DEPILOT: Stabilization duration completed after " 
              << elapsed_time << " ms (target: " << m_stabilize_duration_ms << " ms)"
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
    return true;
  }
  
  return false;
}

void CDEPilotStabilization::setPhase(int phase) {
  const StabilizationPhase old_phase = m_phase;
  m_generic_phase = phase;
  m_phase = static_cast<StabilizationPhase>(phase);
  m_phase_start_time = get_time_usec() / 1000;
  
  std::cout << _INFO_CONSOLE_BOLD_TEXT 
            << "DEPILOT: Phase changed from " << old_phase 
            << " to " << m_phase 
            << _NORMAL_CONSOLE_TEXT_ << std::endl;
            
  // Send context notification for phase change
  std::string target;
  target = "_GD_";
  const char* phase_names[] = {"IDLE", "STABILIZING", "COMPLETE"};
  std::string old_phase_name;
  std::string new_phase_name;
  if (old_phase >= 0 && old_phase < 3) {
    old_phase_name = phase_names[old_phase];
  } else {
    old_phase_name = "UNKNOWN";
  }
  if (m_phase >= 0 && m_phase < 3) {
    new_phase_name = phase_names[m_phase];
  } else {
    new_phase_name = "UNKNOWN";
  }
  std::string notification_msg = "DE-Pilot: Stabilization phase changed from " + old_phase_name + " to " + new_phase_name;
  
  de::comm::CFacade_Base::getInstance().sendErrorMessage(
      target,
      ERROR_USER_DEFINED,
      ERROR_TYPE_ERROR_MODULE,
      NOTIFICATION_TYPE_INFO,
      notification_msg);
}

int CDEPilotStabilization::getPhase() const { return m_generic_phase; }

void CDEPilotStabilization::setActive(bool active) {
  const bool old_active = m_active;
  m_active = active;
  if (active) {
    m_phase_start_time = get_time_usec() / 1000;
    std::cout << _INFO_CONSOLE_BOLD_TEXT 
              << "DEPILOT: Stabilization activated (was " 
              << (old_active ? "active" : "inactive") << ")"
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
              
    // Send context notification for activation
    std::string target;
    target = "_GD_";
    std::string notification_msg;
    notification_msg = "DE-Pilot: Stabilization activated";
    
    de::comm::CFacade_Base::getInstance().sendErrorMessage(
        target,
        ERROR_USER_DEFINED,
        ERROR_TYPE_ERROR_MODULE,
        NOTIFICATION_TYPE_INFO,
        notification_msg);
  } else {
    std::cout << _INFO_CONSOLE_BOLD_TEXT 
              << "DEPILOT: Stabilization deactivated"
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
              
    // Send context notification for deactivation
    std::string target;
    target = "_GD_";
    std::string notification_msg;
    notification_msg = "DE-Pilot: Stabilization deactivated";
    
    de::comm::CFacade_Base::getInstance().sendErrorMessage(
        target,
        ERROR_USER_DEFINED,
        ERROR_TYPE_ERROR_MODULE,
        NOTIFICATION_TYPE_INFO,
        notification_msg);
  }
}

bool CDEPilotStabilization::getActive() const { return m_active; }

void CDEPilotStabilization::readConfigParameters() {
}

void CDEPilotStabilization::reloadParametersIfConfigChanged() {
  readConfigParameters();
}

void CDEPilotStabilization::startStabilization() {
  startStabilization(0); // 0 = infinite stabilization
}

void CDEPilotStabilization::startStabilization(uint64_t duration_ms) {
  if (m_active) {
    std::cout << _INFO_CONSOLE_BOLD_TEXT
              << "DEPILOT: Stabilization already in progress - resetting timer"
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
    // Reset timer and duration even if already active
    m_stabilize_duration_ms = duration_ms;
    m_phase_start_time = get_time_usec() / 1000;
    m_last_update_time = m_phase_start_time;
    if (duration_ms == 0) {
      std::cout << _INFO_CONSOLE_BOLD_TEXT << "DEPILOT: Continuing stabilization (infinite)"
                << _NORMAL_CONSOLE_TEXT_ << std::endl;
                
      // Send context notification for continuing stabilization
      std::string target;
      target = "_GD_";
      std::string notification_msg;
      notification_msg = "DE-Pilot: Continuing stabilization (infinite duration)";
      
      de::comm::CFacade_Base::getInstance().sendErrorMessage(
          target,
          ERROR_USER_DEFINED,
          ERROR_TYPE_ERROR_MODULE,
          NOTIFICATION_TYPE_INFO,
          notification_msg);
    } else {
      std::cout << _INFO_CONSOLE_BOLD_TEXT 
                << "DEPILOT: Reset stabilization duration to " 
                << duration_ms << " ms" << _NORMAL_CONSOLE_TEXT_ << std::endl;
                
      // Send context notification for duration reset
      std::string target;
      target = "_GD_";
      std::string notification_msg;
      notification_msg = "DE-Pilot: Reset stabilization duration to " + 
                         std::to_string(duration_ms) + " ms";
      
      de::comm::CFacade_Base::getInstance().sendErrorMessage(
          target,
          ERROR_USER_DEFINED,
          ERROR_TYPE_ERROR_MODULE,
          NOTIFICATION_TYPE_INFO,
          notification_msg);
    }
    return;
  }

  m_stabilize_duration_ms = duration_ms;
  if (duration_ms == 0) {
    std::cout << _INFO_CONSOLE_BOLD_TEXT << "DEPILOT: Starting stabilization"
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
              
    // Send context notification for starting stabilization
    std::string target;
    target = "_GD_";
    std::string notification_msg;
    notification_msg = "DE-Pilot: Starting stabilization (infinite duration)";
    
    de::comm::CFacade_Base::getInstance().sendErrorMessage(
        target,
        ERROR_USER_DEFINED,
        ERROR_TYPE_ERROR_MODULE,
        NOTIFICATION_TYPE_INFO,
        notification_msg);
  } else {
    std::cout << _INFO_CONSOLE_BOLD_TEXT 
              << "DEPILOT: Starting stabilization with minimum duration " 
              << duration_ms << " ms" << _NORMAL_CONSOLE_TEXT_ << std::endl;
              
    // Send context notification for starting stabilization with duration
    std::string target;
    target = "_GD_";
    std::string notification_msg;
    notification_msg = "DE-Pilot: Starting stabilization with " + 
                       std::to_string(duration_ms) + " ms minimum duration";
    
    de::comm::CFacade_Base::getInstance().sendErrorMessage(
        target,
        ERROR_USER_DEFINED,
        ERROR_TYPE_ERROR_MODULE,
        NOTIFICATION_TYPE_INFO,
        notification_msg);
  }

  m_active = true;
  m_phase = PHASE_STABILIZING;
  m_generic_phase = static_cast<int>(m_phase);
  m_phase_start_time = get_time_usec() / 1000;
  m_last_update_time = m_phase_start_time;
}

void CDEPilotStabilization::updateStabilization() {
  if (m_phase == PHASE_IDLE || m_phase == PHASE_COMPLETE) {
    return;
  }

  const ANDRUAV_VEHICLE_INFO &vehicle_info = m_fcbMain.getAndruavVehicleInfo();
  const uint64_t now = get_time_usec();

  if (!vehicle_info.is_armed) {
    std::cout << _INFO_CONSOLE_BOLD_TEXT << "DEPILOT: Waiting for arm..."
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
    // Only send error message if we've been waiting for more than 10 seconds
    static uint64_t arm_wait_start_time = 0;
    if (arm_wait_start_time == 0) {
      arm_wait_start_time = now;
    } else if ((now - arm_wait_start_time) > 20000000) { // 10 seconds
      std::string target;
      target = "_GD_";
      std::string error_msg;
      error_msg = "DE-Pilot: Stabilization waiting for arm - drone may not be armed";
      de::comm::CFacade_Base::getInstance().sendErrorMessage(
          target,
          ERROR_USER_DEFINED,
          ERROR_TYPE_ERROR_MODULE,
          NOTIFICATION_TYPE_INFO,
          error_msg);
      arm_wait_start_time = now; // Reset timer to avoid spam
    }
    return;
  } else {
    // Reset arm wait timer when armed
    static uint64_t *arm_wait_start_time = nullptr;
    if (arm_wait_start_time == nullptr) {
      arm_wait_start_time = new uint64_t(0);
    }
    *arm_wait_start_time = 0;
  }

  
  switch (m_phase) {
  case PHASE_STABILIZING: {
#ifdef DEBUG    
  const double current_altitude = mavlinksdk::CVehicle::getInstance()
                                      .getMsgGlobalPositionInt()
                                      .relative_alt /
                                  1000.0;
  
    std::cout << _INFO_CONSOLE_BOLD_TEXT << "DEPILOT: Stabilizing altitude"
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
    std::cout << "  - Current flight mode: " << vehicle_info.flying_mode
              << std::endl;
    std::cout << "  - Current altitude: " << current_altitude << "m"
              << std::endl;
#endif
    // Update YAW control service
    CDEPilotYawControl::getInstance().updateYawControl();

    // In ALT-HOLD mode, send 1500 to stop climbing/descending
    if (vehicle_info.flying_mode != VEHICLE_MODE_GUIDED &&
        vehicle_info.flying_mode != VEHICLE_MODE_BRAKE) {
#ifdef DEBUG          
      std::cout << "  - ALT-HOLD mode: Sending stop command (1500) to maintain "
                   "altitude"
                << std::endl;
#endif
      uint16_t rc_channels[RC_CHANNELS_MAX];
      std::fill_n(rc_channels, RC_CHANNELS_MAX, UINT16_MAX);
      rc_channels[m_fcbMain.getRCChannelsMapInfo().rcmap_throttle] = 1500;
      rc_channels[m_fcbMain.getRCChannelsMapInfo().rcmap_roll] = 1500;
      rc_channels[m_fcbMain.getRCChannelsMapInfo().rcmap_pitch] = 1500;
      rc_channels[m_fcbMain.getRCChannelsMapInfo().rcmap_yaw] = 1500;

      // Apply YAW control from service
      CDEPilotYawControl::getInstance().applyYawToRCChannels(rc_channels, vehicle_info.flying_mode);

      mavlinksdk::CMavlinkCommand::getInstance().sendRCChannels2(
          rc_channels, RC_CHANNELS_MAX, UINT16_MAX);
    } else {
#ifdef DEBUG      
      std::cout << "  - GUIDED mode: Flight controller handles stabilization"
                << std::endl;
#endif
      // Apply YAW control from service (handles MAVLink commands in GUIDED mode)
      CDEPilotYawControl::getInstance().applyYawToRCChannels(nullptr, vehicle_info.flying_mode);
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

