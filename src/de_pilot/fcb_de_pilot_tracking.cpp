#include "fcb_de_pilot_tracking.hpp"
#include "fcb_de_pilot_yaw_control.hpp"
#include "../de_common/de_databus/configFile.hpp"
#include "../de_common/helpers/colors.hpp"
#include "../de_common/helpers/helpers.hpp"
#include "../fcb_main.hpp"
#include "../tracking/fcb_tracking_manager.hpp"
#include "fcb_de_pilot_manager.hpp"
#include <iostream>

using namespace de::fcb::depilot;

// Base class interface implementation
void CDEPilotTracking::init() {
  // Read configuration parameters first
  readConfigParameters();

  // Initialize tracking system
  m_active = false;
  m_phase = PHASE_IDLE;
  m_phase_start_time = get_time_usec() / 1000;
  m_last_update_time = m_phase_start_time;
  m_generic_phase = static_cast<int>(m_phase);
  m_last_tracking_update_time = 0;

  // Initialize the tracking manager
  de::fcb::tracking::CTrackingManager::getInstance().init();

  std::cout << _INFO_CONSOLE_BOLD_TEXT 
            << "DEPilotTracking: Initialized" 
            << _NORMAL_CONSOLE_TEXT_ << std::endl;
}

void CDEPilotTracking::update() {
  if (!m_active) {
    return;
  }

  // Update YAW control service
  CDEPilotYawControl::getInstance().updateYawControl();

  m_last_update_time = get_time_usec() / 1000;


  // Check for tracking timeout (no tracking updates for specified time)
  if (m_last_tracking_update_time > 0) {
    uint64_t current_time = get_time_usec() / 1000;
    if (current_time - m_last_tracking_update_time > m_tracking_timeout_us) {
      std::cout << _INFO_CONSOLE_BOLD_TEXT 
                << "DEPilotTracking: Timeout, stopping tracking" 
                << _NORMAL_CONSOLE_TEXT_ << std::endl;
      setActive(false);
      return;
    }
  }

  // Update phase based on tracking status
  int tracking_status = de::fcb::tracking::CTrackingManager::getInstance().getTrackingStatus();
  
  switch (m_phase) {
    case PHASE_INITIALIZING:
      if (tracking_status == TrackingTarget_STATUS_TRACKING_ENABLED ||
          tracking_status == TrackingTarget_STATUS_TRACKING_DETECTED) {
        m_phase = PHASE_TRACKING;
        m_generic_phase = static_cast<int>(m_phase);
        std::cout << _INFO_CONSOLE_BOLD_TEXT 
                  << "DEPilotTracking: Active" 
                  << _NORMAL_CONSOLE_TEXT_ << std::endl;
      }
      break;
      
    case PHASE_TRACKING:
      if (tracking_status == TrackingTarget_STATUS_TRACKING_STOPPED ||
          tracking_status == TrackingTarget_STATUS_TRACKING_LOST) {
        m_active = false;
        m_phase = PHASE_IDLE;
        m_generic_phase = static_cast<int>(m_phase);
        m_last_tracking_update_time = 0;
        std::cout << _INFO_CONSOLE_BOLD_TEXT 
                  << "DEPilotTracking: Stopped" 
                  << _NORMAL_CONSOLE_TEXT_ << std::endl;
      }
      break;
      
    case PHASE_IDLE:
    case PHASE_COMPLETE:
      // Do nothing, wait for external activation
      break;
  }
}

void CDEPilotTracking::uninit() {
  stopTracking();
  m_active = false;
  m_phase = PHASE_IDLE;
  m_generic_phase = static_cast<int>(m_phase);
  m_last_tracking_update_time = 0;

  std::cout << _INFO_CONSOLE_BOLD_TEXT 
            << "DEPilotTracking: Uninitialized" 
            << _NORMAL_CONSOLE_TEXT_ << std::endl;
}

void CDEPilotTracking::readConfigParameters() {
  de::CConfigFile &cConfigFile = de::CConfigFile::getInstance();
  const Json_de &jsonConfig = cConfigFile.GetConfigJSON();

  if (jsonConfig.contains("de_pilot")) {
    const Json_de &de_pilot_root = jsonConfig["de_pilot"];

    if (de_pilot_root.contains("tracking")) {
      const Json_de &tracking_config = de_pilot_root["tracking"];

      if (tracking_config.contains("auto_enable_on_detection")) {
        m_auto_enable_on_detection = tracking_config["auto_enable_on_detection"].get<bool>();
      }
      if (tracking_config.contains("tracking_timeout_us")) {
        m_tracking_timeout_us = tracking_config["tracking_timeout_us"].get<uint64_t>();
      }
    }
  }
}

void CDEPilotTracking::reloadParametersIfConfigChanged() {
  readConfigParameters();
  // Tracking parameters are handled deeper in the tracker classes
  // The tracking manager reload is already called in update()
}

void CDEPilotTracking::setPhase(int phase) {
  m_generic_phase = phase;
  // Convert generic phase to tracking phase if needed
  if (phase >= 0 && phase <= 3) {
    m_phase = static_cast<TrackingPhase>(phase);
  }
}

int CDEPilotTracking::getPhase() const {
  return m_generic_phase;
}

void CDEPilotTracking::setActive(bool active) {
  if (m_active == active) {
    return; // No change needed
  }

  if (active) {
    m_active = true;
    startTracking();
  } else {
    m_active = false;
    stopTracking();
  }
}

bool CDEPilotTracking::getActive() const {
  return m_active;
}

bool CDEPilotTracking::isCompleted() {
  // Tracking operation is never "completed" in the traditional sense
  // It's a continuous operation that can be stopped but doesn't have a completion state
  return false;
}

// Class-specific interface
void CDEPilotTracking::startTracking() {
  if (m_phase == PHASE_IDLE) {
    m_phase = PHASE_INITIALIZING;
    m_generic_phase = static_cast<int>(m_phase);
    m_phase_start_time = get_time_usec() / 1000;
    m_last_tracking_update_time = 0;
    
    std::cout << _INFO_CONSOLE_BOLD_TEXT 
              << "DEPilotTracking: Starting tracking" 
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
  }
}

void CDEPilotTracking::stopTracking() {
  if (m_phase != PHASE_IDLE) {
    de::fcb::tracking::CTrackingManager::getInstance().onStatusChanged(
        TrackingTarget_STATUS_TRACKING_STOPPED, TRACKING_CAMERA_DIRECTION_FRONT, false);
    m_phase = PHASE_IDLE;
    m_generic_phase = static_cast<int>(m_phase);
    m_last_tracking_update_time = 0;
    
    std::cout << _INFO_CONSOLE_BOLD_TEXT 
              << "DEPilotTracking: Stopping tracking" 
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
  }
}

bool CDEPilotTracking::isTrackingActive() const {
  return m_active && (m_phase == PHASE_INITIALIZING || m_phase == PHASE_TRACKING);
}

void CDEPilotTracking::updateTrackingTimestamp() {
  m_last_tracking_update_time = get_time_usec() / 1000;
}

void CDEPilotTracking::processTrackingData(double x_ratio, double yz_ratio) {
  // Update timestamp to indicate we received tracking data
  updateTrackingTimestamp();
  
  // Auto-enable tracking if configured and currently idle
  if (m_auto_enable_on_detection && m_phase == PHASE_IDLE && !m_active) {
    // Use pilot manager to properly select tracking operation
    CDEPilotManager::getInstance().do_Tracking();
  }
  
  // Forward to tracking manager
  de::fcb::tracking::CTrackingManager::getInstance().onTrack(x_ratio, yz_ratio);
}
