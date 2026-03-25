#include "fcb_de_pilot_stabilization.hpp"
#include "fcb_de_pilot_yaw_control.hpp"
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
}

void CDEPilotStabilization::reloadParametersIfConfigChanged() {
  readConfigParameters();
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

  const ANDRUAV_VEHICLE_INFO &vehicle_info = m_fcbMain.getAndruavVehicleInfo();

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

  const ANDRUAV_VEHICLE_INFO &vehicle_info = m_fcbMain.getAndruavVehicleInfo();
  const uint64_t now = get_time_usec();

  if (!vehicle_info.is_armed) {
    std::cout << _INFO_CONSOLE_BOLD_TEXT << "DEPILOT: Waiting for arm..."
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
    return;
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

