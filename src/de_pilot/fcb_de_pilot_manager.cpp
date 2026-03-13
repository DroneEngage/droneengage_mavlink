#include "fcb_de_pilot_manager.hpp"

#include <iostream>
#include "../de_common/helpers/colors.hpp"
#include "../fcb_main.hpp"
#include "../fcb_facade.hpp"
#include "../fcb_modes.hpp"



namespace de {
namespace fcb {
namespace depilot {

void CDEPilotManager::init() {
  // Initialize all pilot operations
  CDEPilotStabilization::getInstance().init();
  CDEPilotTakeoff::getInstance()
      .init(); // Now handles both takeoff and altitude

  m_de_pilot_operation = DEPILOT_OP_DISABLED;
  m_allow_RCControl = true;
  m_operation_instance = nullptr;
  m_target_altitude = 0.0;

  std::cout << _INFO_CONSOLE_BOLD_TEXT << "DEPilotManager: Initialized"
            << _NORMAL_CONSOLE_TEXT_ << std::endl;
}

bool CDEPilotManager::isCompatibleMode() {
  de::fcb::CFCBMain &fcbMain = de::fcb::CFCBMain::getInstance();
  const ANDRUAV_VEHICLE_INFO &vehicle_info = fcbMain.getAndruavVehicleInfo();
  const int flying_mode = vehicle_info.flying_mode;
  const bool depilot_compatible =
      ( flying_mode == VEHICLE_MODE_ALT_HOLD 
     || flying_mode == VEHICLE_MODE_LAND 
     || flying_mode == VEHICLE_MODE_BRAKE 
     || flying_mode == VEHICLE_MODE_GUIDED);

  if (!depilot_compatible) {
    std::cout << "DE-Pilot: Current flight mode [" << flying_mode
              << "] is not compatible with DE pilot." << std::endl;
    return false;
  }
  
  return true;
}

void CDEPilotManager::OnFlightModeChanged() {
  if (!isCompatibleMode()) {
    setActive(false);
    return;
  }
}

void CDEPilotManager::setActive(bool active) {
  if (getActive() == active) {
    // nothing to do
    return;
  }

  if (!active) {
    // shutdown
    init();
    de_pilot_enabled = false;
    CFCBFacade::getInstance().API_IC_sendID(
        std::string(ANDRUAV_PROTOCOL_SENDER_ALL));
    return;
  }

  // Check if mode is compatible
  if (!isCompatibleMode()) {
    init();
    de_pilot_enabled = false;
    CFCBFacade::getInstance().API_IC_sendID(
        std::string(ANDRUAV_PROTOCOL_SENDER_ALL));
    return;
  }
  
  de_pilot_enabled = true;
  m_allow_RCControl = false;
  
  setOperation(DEPILOT_OP_STABILIZATION); // by default switch to
                                          // DEPILOT_OP_STABILIZATION

  
  std::cout << _SUCCESS_CONSOLE_TEXT_ << "DRONEENGAGE_PILOT: " 
              << (de_pilot_enabled ? "Enabled" : "Disabled") 
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
  
  CFCBFacade::getInstance().API_IC_sendID(
      std::string(ANDRUAV_PROTOCOL_SENDER_ALL));
}

void CDEPilotManager::updateOperations() {

  if (!de_pilot_enabled) {
    return;
  }

  const char *operation_name = "UNKNOWN";
  switch (m_de_pilot_operation) {
  case DEPILOT_OP_DISABLED:
    operation_name = "DISABLED";
    break;
  case DEPILOT_OP_CHANGE_ALTITUDE:
    operation_name = "CHANGE-ALTITUDE";
    break;
  case DEPILOT_OP_TRACKING:
    operation_name = "TRACKING";
    break;
  case DEPILOT_OP_STABILIZATION:
    operation_name = "STABILIZATION";
    break;
  }

  std::cout << _INFO_CONSOLE_BOLD_TEXT << "DEOperation: " << operation_name
            << " (" << m_de_pilot_operation << ")" << _NORMAL_CONSOLE_TEXT_
            << std::endl;

  // Update the active operation
  CDEPilotOperationBase *operation = getOperationInstance(m_de_pilot_operation);
  if (operation != nullptr) {
    operation->update();
  }
}

void CDEPilotManager::ChangeAltitude(double target_altitude) {
  // LOGIC HERE for AI to IMplement:
  if (!isCompatibleMode()) {
    init();
    return;
  }

  setTargetAltitude(target_altitude);
  setOperation(DEPILOT_OP_CHANGE_ALTITUDE);
  if (m_operation_instance == nullptr)
  {
    // Failes
    std::cout << "DE-Pilot Failed to call Change Altitude" << std::endl;
    return ;
  }
}

void CDEPilotManager::Land() {
  
  if (!isCompatibleMode()) {
    std::cout << "DE-PILOT: Failed to switch to LAND mode." << std::endl;
    init();
    return;
  }


  de::fcb::CFCBMain &fcbMain = de::fcb::CFCBMain::getInstance();
  uint32_t ardupilot_mode, ardupilot_custom_mode, ardupilot_custom_sub_mode;
    CFCBModes::getArduPilotMode(
        VEHICLE_MODE_LAND, fcbMain.getAndruavVehicleInfo().vehicle_type,
        ardupilot_mode, ardupilot_custom_mode, ardupilot_custom_sub_mode);
    if (ardupilot_mode == E_UNDEFINED_MODE) {
      return;
    }
    mavlinksdk::CMavlinkCommand::getInstance().doSetMode(
        ardupilot_mode, ardupilot_custom_mode, ardupilot_custom_sub_mode);

  setOperation(DEPILOT_OP_CHANGE_ALTITUDE);
}

void CDEPilotManager::setOperation(DRONEENGAGE_PILOT_OPERATION operation) {

  if (!de_pilot_enabled) {
    return;
  }
  
  CDEPilotOperationBase *operation_instance = getOperationInstance(operation);
  if (operation_instance != nullptr &&
      operation_instance != m_operation_instance) {
    std::cout << _INFO_CONSOLE_BOLD_TEXT
            << "DEPilotManager: changing  m_de_pilot_operation from "
            << m_de_pilot_operation << " to " << operation
            << _NORMAL_CONSOLE_TEXT_ << std::endl;

    if (m_operation_instance != nullptr)
    {
        m_operation_instance->setActive(false);
        m_operation_instance->uninit();
    }

    m_de_pilot_operation = operation;
    m_operation_instance = operation_instance;
    m_operation_instance->init();
    m_operation_instance->setActive(true);
  }

  // Set default operation based on current state
  // This logic mirrors the original fcb_main.cpp behavior
  // Note: We would need access to vehicle info to determine if flying
  // For now, default to DISABLED - this can be enhanced later
  
  de::fcb::CFCBFacade::getInstance().API_IC_sendID(
      std::string(ANDRUAV_PROTOCOL_SENDER_ALL));
}

DRONEENGAGE_PILOT_OPERATION CDEPilotManager::getCurrentOperation() const {
  return m_de_pilot_operation;
}

bool CDEPilotManager::isOperationActive(
    DRONEENGAGE_PILOT_OPERATION operation) const {
  CDEPilotOperationBase *operation_instance = getOperationInstance(operation);
  if (operation_instance != nullptr) {
    return operation_instance->getActive();
  }
  return false;
}

void CDEPilotManager::deactivateAllOperations() {
  CDEPilotStabilization::getInstance().setActive(false);
  CDEPilotTakeoff::getInstance().setActive(false);
}

CDEPilotOperationBase *CDEPilotManager::getOperationInstance(
    DRONEENGAGE_PILOT_OPERATION operation) const {
  switch (operation) {
  case DEPILOT_OP_CHANGE_ALTITUDE:
    return &CDEPilotTakeoff::getInstance();
  case DEPILOT_OP_STABILIZATION:
    return &CDEPilotStabilization::getInstance();
  case DEPILOT_OP_TRACKING:
    // Tracking is handled differently - return nullptr for now
    return nullptr;
  case DEPILOT_OP_DISABLED:
  default:
    return nullptr;
  }
}



double CDEPilotManager::getTargetAltitude() const { return m_target_altitude; }

} // namespace depilot
} // namespace fcb
} // namespace de
