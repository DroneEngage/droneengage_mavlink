#include "fcb_de_pilot_tracking.hpp"
#include "../de_common/de_databus/configFile.hpp"
#include "../de_common/helpers/colors.hpp"
#include "../de_common/helpers/helpers.hpp"
#include "../fcb_main.hpp"
#include "../tracking/fcb_tracking_manager.hpp"
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

  m_last_update_time = get_time_usec() / 1000;

  // Update tracking manager parameters if needed
  de::fcb::tracking::CTrackingManager::getInstance().reloadParametersIfConfigChanged();

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
  // Load tracking-specific configuration parameters
  // These can be added to the config file as needed
  
  // For now, use default values
  m_auto_enable_on_detection = true;
  m_tracking_timeout_us = 5000000; // 5 seconds
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
        TrackingTarget_STATUS_TRACKING_STOPPED);
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
