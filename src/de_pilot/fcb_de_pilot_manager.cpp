#include "fcb_de_pilot_manager.hpp"

#include "../de_common/helpers/colors.hpp"
#include "../de_common/de_databus/sync_fire_events.hpp"
#include "../de_common/helpers/helpers.hpp"
#include "../fcb_facade.hpp"
#include "../fcb_main.hpp"
#include "../fcb_modes.hpp"
#include "fcb_de_pilot_yaw_control.hpp"
#include "../de_common/de_databus/de_facade_base.hpp"
#include <iostream>
#include <mavlink_sdk.h>
#include <vehicle.h>

namespace de {
namespace fcb {
namespace depilot {

void CDEPilotManager::init() {
  // Initialize all pilot operations
  CDEPilotStabilization::getInstance().init();
  CDEPilotChangeAltitude::getInstance().init();
  CDEPilotTracking::getInstance().init();
  CDEPilotYawControl::getInstance().init();

  m_de_pilot_operation = DEPILOT_OP_DISABLED;
  m_allow_RCControl = true;
  m_operation_instance = nullptr;
  m_target_altitude = 0.0;

  std::cout << _INFO_CONSOLE_BOLD_TEXT << "DEPilotManager: Initialized"
            << _NORMAL_CONSOLE_TEXT_ << std::endl;
}

void CDEPilotManager::reloadParametersIfConfigChanged() {

  std::cout << _INFO_CONSOLE_BOLD_TEXT << "DEPilotManager: " << _LOG_CONSOLE_BOLD_TEXT << "Parameters reloading..."
            << _NORMAL_CONSOLE_TEXT_ << std::endl;

  // Reload parameters for all de_pilot operations
  CDEPilotChangeAltitude::getInstance().reloadParametersIfConfigChanged();
  CDEPilotStabilization::getInstance().reloadParametersIfConfigChanged();
  CDEPilotTracking::getInstance().reloadParametersIfConfigChanged();
  CDEPilotYawControl::getInstance().reloadParametersIfConfigChanged();
  
  std::cout << _INFO_CONSOLE_BOLD_TEXT << "DEPilotManager: " << _LOG_CONSOLE_BOLD_TEXT << "Parameters reloading DONE"
            << _NORMAL_CONSOLE_TEXT_ << std::endl;

}

bool CDEPilotManager::isCompatibleMode() {
  de::fcb::CFCBMain &fcbMain = de::fcb::CFCBMain::getInstance();
  const ANDRUAV_VEHICLE_INFO &vehicle_info = fcbMain.getAndruavVehicleInfo();
  const int flying_mode = vehicle_info.flying_mode;
  const bool depilot_compatible =
      (flying_mode == VEHICLE_MODE_ALT_HOLD ||
       flying_mode == VEHICLE_MODE_LAND || flying_mode == VEHICLE_MODE_BRAKE ||
       flying_mode == VEHICLE_MODE_GUIDED);

  if (!depilot_compatible) {
    std::cout << "DE-Pilot: Current flight mode [" << flying_mode
              << "] is not compatible with DE pilot." << std::endl;
    de::comm::CFacade_Base::getInstance().sendErrorMessage(
        std::string(ANDRUAV_PROTOCOL_SENDER_ALL),
        ERROR_USER_DEFINED,
        ERROR_TYPE_ERROR_MODULE,
        NOTIFICATION_TYPE_INFO,
        "DE-Pilot: Flight mode " + std::to_string(flying_mode) + " detected - pilot operations disabled");
    return false;
  }

  return true;
}

void CDEPilotManager::OnFlightModeChanged() {
  if (!isCompatibleMode()) {
    setActive(false);
    return;
  }

  de::fcb::CFCBMain &fcbMain = de::fcb::CFCBMain::getInstance();
  const ANDRUAV_VEHICLE_INFO &vehicle_info = fcbMain.getAndruavVehicleInfo();
  const int flying_mode = vehicle_info.flying_mode;
  
  switch (flying_mode) {
    case VEHICLE_MODE_LAND:
      setOperation(DEPILOT_OP_IDLE);
    break;
  
    case VEHICLE_MODE_STABILIZE:
      setActive(false);
    break;

    default:
    break;
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
    m_de_pilot_enabled = false;
    CFCBFacade::getInstance().API_IC_sendID(
        std::string(ANDRUAV_PROTOCOL_SENDER_ALL));
    return;
  }

  // Check if mode is compatible
  if (!isCompatibleMode()) {
    init();
    m_de_pilot_enabled = false;
    de::comm::CFacade_Base::getInstance().sendErrorMessage(
        std::string(ANDRUAV_PROTOCOL_SENDER_ALL),
        ERROR_USER_DEFINED,
        ERROR_TYPE_ERROR_MODULE,
        NOTIFICATION_TYPE_INFO,
        "DE-Pilot: System disabled - incompatible flight mode detected");
    CFCBFacade::getInstance().API_IC_sendID(
        std::string(ANDRUAV_PROTOCOL_SENDER_ALL));
    return;
  }

  m_de_pilot_enabled = true;
  m_allow_RCControl = false;

  setOperation(DEPILOT_OP_STABILIZATION); // by default switch to
                                          // DEPILOT_OP_STABILIZATION

  std::cout << _SUCCESS_CONSOLE_TEXT_ << "DRONEENGAGE_PILOT: "
            << (m_de_pilot_enabled ? "Enabled" : "Disabled")
            << _NORMAL_CONSOLE_TEXT_ << std::endl;

  CFCBFacade::getInstance().API_IC_sendID(
      std::string(ANDRUAV_PROTOCOL_SENDER_ALL));
}

void CDEPilotManager::updateOperations() {

  if (!m_de_pilot_enabled) {
    return;
  }

  DRONEENGAGE_PILOT_OPERATION current_op = m_de_pilot_operation;

  // Log queue status at start of update
  static uint64_t last_queue_log_time = 0;
  const uint64_t now = get_time_usec() / 1000;
  if (now - last_queue_log_time > 5000) { // Log every 5 seconds
    std::cout << _INFO_CONSOLE_BOLD_TEXT << "DEPilotManager: Queue status - "
              << m_operation_queue.size() << " operations queued"
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
    last_queue_log_time = now;
  }

  // Update the active operation
  CDEPilotOperationBase *operation = getOperationInstance(current_op);
  if (operation != nullptr) {
    // Step Execution
    operation->update();

    // No queued operation: keep legacy fallback behavior.
    // Completed TRACKING/CHANGE_ALTITUDE returns to STABILIZATION.
    if (m_operation_queue.empty()) {
      if (operation->isCompleted()) {
        std::cout << _INFO_CONSOLE_BOLD_TEXT
                  << "DEPilotManager: Operation " << operation->getName()
                  << " completed with no queue - checking fallback"
                  << _NORMAL_CONSOLE_TEXT_ << std::endl;
        if (current_op == DEPILOT_OP_CHANGE_ALTITUDE ||
            current_op == DEPILOT_OP_TRACKING) {
          setOperation(DEPILOT_OP_STABILIZATION);
        }
      }
      return;
    }

    // Check if we can advance to next queued operation
    if (!m_operation_queue.empty() && canAdvanceFromCurrentOperation()) {
      QueuedOperation next_op = m_operation_queue.front();
      CDEPilotOperationBase *next_operation =
          getOperationInstance(next_op.operation);

      if (next_operation == nullptr) {
        std::cout << _ERROR_CONSOLE_BOLD_TEXT_
                  << "DEPilotManager: Invalid queued operation " 
                  << static_cast<int>(next_op.operation) << " - removing"
                  << _NORMAL_CONSOLE_TEXT_ << std::endl;
        m_operation_queue.pop();
        return;
      }

      // If yaw target is pending, allow transition only to yaw-supporting operations.
      if (isYawTargetPending() && !next_operation->isYawSupported()) {
        std::cout << _INFO_CONSOLE_BOLD_TEXT
                  << "DEPilotManager: Yaw target pending, delaying advance to " 
                  << next_operation->getName()
                  << _NORMAL_CONSOLE_TEXT_ << std::endl;
        return;
      }

      m_operation_queue.pop();
      
      std::cout << _INFO_CONSOLE_BOLD_TEXT
                << "DEPilotManager: advancing from " << operation->getName()
                << " to queued operation " << next_operation->getName()
                << " (" << static_cast<int>(next_op.operation) << ")"
                << _NORMAL_CONSOLE_TEXT_ << std::endl;
      
      // Set target altitude if this operation has altitude parameter
      if (next_op.has_altitude_param) {
        if (next_op.operation == DEPILOT_OP_STABILIZATION) {
          // For STABILIZATION, target_altitude stores duration_ms
          if (m_operation_instance != nullptr && m_operation_instance->getActive()) {
            CDEPilotStabilization *stabilize_op = static_cast<CDEPilotStabilization *>(m_operation_instance);
            stabilize_op->startStabilization(static_cast<uint64_t>(next_op.target_altitude));
            std::cout << "  - Starting stabilization with duration: " 
                      << next_op.target_altitude << " ms" << std::endl;
          }
        } else {
          setTargetAltitude(next_op.target_altitude);
          std::cout << "  - Setting target altitude: " 
                    << next_op.target_altitude << " m" << std::endl;
        }
      }
      
      // Switch to next operation
      setOperation(next_op.operation);
      
      // If the new operation is CHANGE_ALTITUDE and has altitude param, start it
      if (next_op.operation == DEPILOT_OP_CHANGE_ALTITUDE && next_op.has_altitude_param) {
        if (m_operation_instance != nullptr && m_operation_instance->getActive()) {
          CDEPilotChangeAltitude *altitude_op =
              static_cast<CDEPilotChangeAltitude *>(m_operation_instance);
          altitude_op->startAltitudeChange(next_op.target_altitude);
          std::cout << "  - Started altitude change to " 
                    << next_op.target_altitude << " m" << std::endl;
        }
      }
    } else if (!m_operation_queue.empty()) {
      // Log why we're not advancing
      static uint64_t last_block_log_time = 0;
      if (now - last_block_log_time > 10000) { // Log every 10 seconds
        std::cout << _INFO_CONSOLE_BOLD_TEXT
                  << "DEPilotManager: Queue has " << m_operation_queue.size() 
                  << " operations but cannot advance - current operation " 
                  << operation->getName() << " " 
                  << (operation->isCompleted() ? "completed" : "active")
                  << _NORMAL_CONSOLE_TEXT_ << std::endl;
        last_block_log_time = now;
      }
    }
  }
}

void CDEPilotManager::do_ChangeAltitude(double target_altitude, bool enqueue) {
  if (!isCompatibleMode()) {
    de::comm::CFacade_Base::getInstance().sendErrorMessage(
        std::string(ANDRUAV_PROTOCOL_SENDER_ALL),
        ERROR_USER_DEFINED,
        ERROR_TYPE_ERROR_MODULE,
        NOTIFICATION_TYPE_INFO,
        "DE-Pilot: Altitude change request rejected - incompatible flight mode");
    init();
    return;
  }

  if (enqueue) {
    QueuedOperation op;
    op.operation = DEPILOT_OP_CHANGE_ALTITUDE;
    op.target_altitude = target_altitude;
    op.has_altitude_param = true;
    m_operation_queue.push(op);
    std::cout << _INFO_CONSOLE_BOLD_TEXT 
              << "DE-Pilot: Enqueued Change Altitude to " << target_altitude 
              << "m (queue size: " << m_operation_queue.size() << ")"
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
    return;
  }

  // Immediate path: clear queue and execute
  const size_t queue_size_before = m_operation_queue.size();
  clearOperationQueue();
  if (queue_size_before > 0) {
    std::cout << _INFO_CONSOLE_BOLD_TEXT 
              << "DE-Pilot: Cleared " << queue_size_before 
              << " queued operations for immediate Change Altitude"
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
  }
  
  setTargetAltitude(target_altitude);
  setOperation(DEPILOT_OP_CHANGE_ALTITUDE);
  if (m_operation_instance == nullptr) {
    std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "DE-Pilot Failed to call Change Altitude - no instance"
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
    de::comm::CFacade_Base::getInstance().sendErrorMessage(
        std::string(ANDRUAV_PROTOCOL_SENDER_ALL),
        ERROR_USER_DEFINED,
        ERROR_TYPE_ERROR_MODULE,
        NOTIFICATION_TYPE_ERROR,
        "DE-Pilot: Change altitude failed - operation system error");
    return;
  }

  if (m_operation_instance != nullptr && m_operation_instance->getActive()) {
    CDEPilotChangeAltitude *altitude_op =
        static_cast<CDEPilotChangeAltitude *>(m_operation_instance);
    altitude_op->startAltitudeChange(target_altitude);
    std::cout << _INFO_CONSOLE_BOLD_TEXT << "DE-Pilot: Started immediate Change Altitude to "
              << target_altitude << "m" << _NORMAL_CONSOLE_TEXT_ << std::endl;
  }
}

void CDEPilotManager::do_Stabilize(bool enqueue) {
  if (!isCompatibleMode()) {
    de::comm::CFacade_Base::getInstance().sendErrorMessage(
        std::string(ANDRUAV_PROTOCOL_SENDER_ALL),
        ERROR_USER_DEFINED,
        ERROR_TYPE_ERROR_MODULE,
        NOTIFICATION_TYPE_ERROR,
        "DE-Pilot: Cannot stabilize - incompatible flight mode");
    init();
    return;
  }

  if (enqueue) {
    QueuedOperation op;
    op.operation = DEPILOT_OP_STABILIZATION;
    op.target_altitude = 0.0;
    op.has_altitude_param = false;
    m_operation_queue.push(op);
    std::cout << _INFO_CONSOLE_BOLD_TEXT 
              << "DE-Pilot: Enqueued Stabilize (queue size: " 
              << m_operation_queue.size() << ")"
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
    return;
  }

  // Immediate path: clear queue and execute
  const size_t queue_size_before = m_operation_queue.size();
  clearOperationQueue();
  if (queue_size_before > 0) {
    std::cout << _INFO_CONSOLE_BOLD_TEXT 
              << "DE-Pilot: Cleared " << queue_size_before 
              << " queued operations for immediate Stabilize"
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
  }
  
  setOperation(DEPILOT_OP_STABILIZATION);
  if (m_operation_instance == nullptr) {
    std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "DE-Pilot Failed to call STABILIZE - no instance"
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
    de::comm::CFacade_Base::getInstance().sendErrorMessage(
        std::string(ANDRUAV_PROTOCOL_SENDER_ALL),
        ERROR_USER_DEFINED,
        ERROR_TYPE_ERROR_MODULE,
        NOTIFICATION_TYPE_ERROR,
        "DE-Pilot: Stabilization failed - operation system error");
    return;
  }

  std::cout << _INFO_CONSOLE_BOLD_TEXT << "DE-Pilot: Started immediate Stabilize"
            << _NORMAL_CONSOLE_TEXT_ << std::endl;
}

void CDEPilotManager::do_Stabilize(uint64_t duration_ms, bool enqueue) {
  if (!isCompatibleMode()) {
    de::comm::CFacade_Base::getInstance().sendErrorMessage(
        std::string(ANDRUAV_PROTOCOL_SENDER_ALL),
        ERROR_USER_DEFINED,
        ERROR_TYPE_ERROR_MODULE,
        NOTIFICATION_TYPE_ERROR,
        "DE-Pilot: Cannot stabilize - incompatible flight mode");
    init();
    return;
  }

  if (enqueue) {
    QueuedOperation op;
    op.operation = DEPILOT_OP_STABILIZATION;
    op.target_altitude = static_cast<double>(duration_ms); // Reuse field for duration
    op.has_altitude_param = true;
    m_operation_queue.push(op);
    std::cout << _INFO_CONSOLE_BOLD_TEXT 
              << "DE-Pilot: Enqueued Stabilize for " << duration_ms 
              << "ms (queue size: " << m_operation_queue.size() << ")"
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
              
    // Send context notification for enqueued stabilization
    std::string target;
    target = "_GD_";
    std::string notification_msg = "DE-Pilot: Stabilization enqueued for " + 
                                 std::to_string(duration_ms) + "ms (queue size: " + 
                                 std::to_string(m_operation_queue.size()) + ")";
    
    de::comm::CFacade_Base::getInstance().sendErrorMessage(
        target,
        ERROR_USER_DEFINED,
        ERROR_TYPE_ERROR_MODULE,
        NOTIFICATION_TYPE_INFO,
        notification_msg);
    return;
  }

  // Immediate path: clear queue and execute
  const size_t queue_size_before = m_operation_queue.size();
  clearOperationQueue();
  if (queue_size_before > 0) {
    std::cout << _INFO_CONSOLE_BOLD_TEXT 
              << "DE-Pilot: Cleared " << queue_size_before 
              << " queued operations for immediate Stabilize"
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
  }
  
  setOperation(DEPILOT_OP_STABILIZATION);
  if (m_operation_instance == nullptr) {
    std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "DE-Pilot Failed to call STABILIZE - no instance"
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
    de::comm::CFacade_Base::getInstance().sendErrorMessage(
        std::string(ANDRUAV_PROTOCOL_SENDER_ALL),
        ERROR_USER_DEFINED,
        ERROR_TYPE_ERROR_MODULE,
        NOTIFICATION_TYPE_ERROR,
        "DE-Pilot: Stabilization failed - operation system error");
    return;
  }

  // Start stabilization with duration
  if (m_operation_instance != nullptr && m_operation_instance->getActive()) {
    CDEPilotStabilization *stabilize_op = static_cast<CDEPilotStabilization *>(m_operation_instance);
    stabilize_op->startStabilization(duration_ms);
    std::cout << _INFO_CONSOLE_BOLD_TEXT << "DE-Pilot: Started immediate Stabilize for " 
              << duration_ms << " ms" << _NORMAL_CONSOLE_TEXT_ << std::endl;
  }
}

void CDEPilotManager::do_Tracking(bool enqueue) {
  if (!isCompatibleMode()) {
    de::comm::CFacade_Base::getInstance().sendErrorMessage(
        std::string(ANDRUAV_PROTOCOL_SENDER_ALL),
        ERROR_USER_DEFINED,
        ERROR_TYPE_ERROR_MODULE,
        NOTIFICATION_TYPE_INFO,
        "DE-Pilot: Tracking request rejected - incompatible flight mode");
    init();
    return;
  }

  if (enqueue) {
    QueuedOperation op;
    op.operation = DEPILOT_OP_TRACKING;
    op.target_altitude = 0.0;
    op.has_altitude_param = false;
    m_operation_queue.push(op);
    std::cout << _INFO_CONSOLE_BOLD_TEXT 
              << "DE-Pilot: Enqueued Tracking (queue size: " 
              << m_operation_queue.size() << ")"
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
    return;
  }

  // Immediate path: clear queue and execute
  const size_t queue_size_before = m_operation_queue.size();
  clearOperationQueue();
  if (queue_size_before > 0) {
    std::cout << _INFO_CONSOLE_BOLD_TEXT 
              << "DE-Pilot: Cleared " << queue_size_before 
              << " queued operations for immediate Tracking"
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
  }
  
  setOperation(DEPILOT_OP_TRACKING);
  if (m_operation_instance == nullptr) {
    std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "DE-Pilot Failed to call TRACKING - no instance"
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
    de::comm::CFacade_Base::getInstance().sendErrorMessage(
        std::string(ANDRUAV_PROTOCOL_SENDER_ALL),
        ERROR_USER_DEFINED,
        ERROR_TYPE_ERROR_MODULE,
        NOTIFICATION_TYPE_ERROR,
        "DE-Pilot: Tracking failed - operation system error");
    return;
  }

  std::cout << _INFO_CONSOLE_BOLD_TEXT 
            << "DE-Pilot: Started immediate Tracking mode" 
            << _NORMAL_CONSOLE_TEXT_ << std::endl;
}

void CDEPilotManager::do_Wait(uint64_t duration_ms, bool enqueue) {
  // Shadow function that calls do_Stabilize with duration
  do_Stabilize(duration_ms, enqueue);
}

void CDEPilotManager::do_SetYaw(double angle, double rate, bool is_clockwise,
                                bool is_relative) {
  if (!m_de_pilot_enabled) {
    std::cout << _INFO_CONSOLE_BOLD_TEXT
              << "DE-Pilot: Not active, cannot set yaw" << _NORMAL_CONSOLE_TEXT_
              << std::endl;
    de::comm::CFacade_Base::getInstance().sendErrorMessage(
        std::string(ANDRUAV_PROTOCOL_SENDER_ALL),
        ERROR_USER_DEFINED,
        ERROR_TYPE_ERROR_MODULE,
        NOTIFICATION_TYPE_INFO,
        "DE-Pilot: Yaw control request rejected - pilot system not active");
    return;
  }

  if (!isCompatibleMode()) {
    std::cout << _INFO_CONSOLE_BOLD_TEXT
              << "DE-Pilot: Incompatible mode, cannot set yaw"
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
    de::comm::CFacade_Base::getInstance().sendErrorMessage(
        std::string(ANDRUAV_PROTOCOL_SENDER_ALL),
        ERROR_USER_DEFINED,
        ERROR_TYPE_ERROR_MODULE,
        NOTIFICATION_TYPE_INFO,
        "DE-Pilot: Yaw control request rejected - incompatible flight mode");
    return;
  }

  // Set yaw target on YAW control service (works with any operation)
  CDEPilotYawControl::getInstance().setYawTarget(angle, rate, is_clockwise, is_relative);
  emitSubOperationEvent(CDEPilotYawControl::getInstance());

  CFCBFacade::getInstance().API_IC_sendID(
      std::string(ANDRUAV_PROTOCOL_SENDER_ALL));
}

void CDEPilotManager::do_Land(bool enqueue) {
  if (!isCompatibleMode()) {
    std::cout << "DE-PILOT: Failed to switch to LAND mode." << std::endl;
    de::comm::CFacade_Base::getInstance().sendErrorMessage(
        std::string(ANDRUAV_PROTOCOL_SENDER_ALL),
        ERROR_USER_DEFINED,
        ERROR_TYPE_ERROR_MODULE,
        NOTIFICATION_TYPE_INFO,
        "DE-Pilot: Landing request rejected - incompatible flight mode");
    init();
    return;
  }

  if (enqueue) {
    QueuedOperation op;
    op.operation = DEPILOT_OP_IDLE;
    op.target_altitude = 0.0;
    op.has_altitude_param = false;
    m_operation_queue.push(op);
    std::cout << "DE-Pilot: Enqueued Land (mapped to IDLE)" << std::endl;
    return;
  }

  // Immediate path: clear queue and execute
  clearOperationQueue();
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

  setOperation(DEPILOT_OP_IDLE);
}

void CDEPilotManager::setOperation(DRONEENGAGE_PILOT_OPERATION operation) {

  if (!m_de_pilot_enabled) {
    return;
  }

  CDEPilotOperationBase *operation_instance = getOperationInstance(operation);
  if (operation_instance != nullptr &&
      operation_instance != m_operation_instance) {
    std::cout << _INFO_CONSOLE_BOLD_TEXT
              << "DEPilotManager: changing  m_de_pilot_operation from "
              << m_de_pilot_operation << " to " << operation
              << _NORMAL_CONSOLE_TEXT_ << std::endl;

    if (m_operation_instance != nullptr) {
      m_operation_instance->setActive(false);
      m_operation_instance->uninit();
    }

    m_de_pilot_operation = operation;
    m_operation_instance = operation_instance;
    m_operation_instance->init();
    m_operation_instance->setActive(true);

    Json_de event_message = Json_de::object();
    event_message["p"] = m_operation_instance->getPhase();
    event_message["a"] = m_operation_instance->getActive();
    event_message["o"] = m_operation_instance->getOperation();
    m_fcb_facade.sendSyncFireEvent(std::string(), DRONE_DEPILOT_MODE_ACTIVATED, event_message, false);

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

double CDEPilotManager::getTargetAltitude() const { return m_target_altitude; }

void CDEPilotManager::deactivateAllOperations() {
  CDEPilotStabilization::getInstance().setActive(false);
  CDEPilotChangeAltitude::getInstance().setActive(false);
  CDEPilotTracking::getInstance().setActive(false);
}

void CDEPilotManager::emitSubOperationEvent(const CDEPilotTaskBase& task) {
  Json_de event_message = Json_de::object();
  event_message["sub"] = task.getName();
  event_message["state"] = static_cast<int>(task.getTaskState());
  event_message["p"] = task.getPhase();
  event_message["a"] = task.getActive();
  event_message["o"] = m_de_pilot_operation;
  // Parse context JSON string into object
  try {
    Json_de ctx = Json_de::parse(task.getEventContext());
    event_message["ctx"] = ctx;
  } catch (...) {
    event_message["ctx"] = Json_de::object();
  }
  m_fcb_facade.sendSyncFireEvent(std::string(), DRONE_DEPILOT_SUBOPERATION_UPDATED, event_message, false);
}

void CDEPilotManager::clearOperationQueue() {
  const size_t cleared_count = m_operation_queue.size();
  while (!m_operation_queue.empty()) {
    m_operation_queue.pop();
  }
  if (cleared_count > 0) {
    std::cout << _INFO_CONSOLE_BOLD_TEXT 
              << "DEPilotManager: Cleared " << cleared_count << " queued operations"
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
  }
}

size_t CDEPilotManager::getOperationQueueSize() const {
  return m_operation_queue.size();
}

bool CDEPilotManager::canAdvanceFromCurrentOperation() const {
  if (m_operation_instance == nullptr) {
    std::cout << _INFO_CONSOLE_BOLD_TEXT 
              << "DEPilotManager: Can advance - no current operation"
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
    return true; // No current operation, can advance
  }
  
  // Check if current operation is completed
  const bool is_completed = m_operation_instance->isCompleted();
  if (!is_completed) {
    static uint64_t last_advance_log_time = 0;
    const uint64_t now = get_time_usec() / 1000;
    if (now - last_advance_log_time > 15000) { // Log every 15 seconds
      std::cout << _INFO_CONSOLE_BOLD_TEXT 
                << "DEPilotManager: Cannot advance - " 
                << m_operation_instance->getName() << " is still active"
                << _NORMAL_CONSOLE_TEXT_ << std::endl;
      last_advance_log_time = now;
    }
    return false;
  }
  
  std::cout << _INFO_CONSOLE_BOLD_TEXT 
            << "DEPilotManager: Can advance - " 
            << m_operation_instance->getName() << " is completed"
            << _NORMAL_CONSOLE_TEXT_ << std::endl;
  return true;
}

bool CDEPilotManager::isYawTargetPending() const {
  return CDEPilotYawControl::getInstance().isYawControlActive();
}


CDEPilotOperationBase *CDEPilotManager::getOperationInstance(
    DRONEENGAGE_PILOT_OPERATION operation) const {
  switch (operation) {
  case DEPILOT_OP_CHANGE_ALTITUDE:
    return &CDEPilotChangeAltitude::getInstance();
  case DEPILOT_OP_STABILIZATION:
    return &CDEPilotStabilization::getInstance();
  case DEPILOT_OP_TRACKING:
    return &CDEPilotTracking::getInstance();
  case DEPILOT_OP_IDLE:
    return &CDEPilotIdle::getInstance();;
  case DEPILOT_OP_DISABLED:
  default:
    return nullptr;
  }
}

} // namespace depilot
} // namespace fcb
} // namespace de
