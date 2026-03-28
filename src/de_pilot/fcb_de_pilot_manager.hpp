#ifndef FCB_DE_PILOT_MANAGER_H_
#define FCB_DE_PILOT_MANAGER_H_

#include "../defines.hpp"
#include "fcb_de_pilot_change_altitude.hpp"
#include "fcb_de_pilot_idle.hpp"
#include "fcb_de_pilot_operation_base.hpp"
#include "fcb_de_pilot_stabilization.hpp"
#include "fcb_de_pilot_tracking.hpp"
#include "fcb_de_pilot_yaw_control.hpp"
#include "../fcb_facade.hpp"
#include <cstdint>
#include <string>
#include <vector>
#include <queue>

namespace de {
namespace fcb {
namespace depilot {

class CDEPilotManager {
public:
  static CDEPilotManager &getInstance() {
    static CDEPilotManager instance;
    return instance;
  }

  CDEPilotManager(CDEPilotManager const &) = delete;
  void operator=(CDEPilotManager const &) = delete;

public:
  // Manager interface
  void init();
  void updateOperations();
  void reloadParametersIfConfigChanged();

  void setActive(const bool active);

public:
  // Main Functions
  void do_ChangeAltitude(double target_altitude, bool enqueue = false);
  void do_Stabilize(bool enqueue = false);
  void do_Stabilize(uint64_t duration_ms, bool enqueue = false);
  void do_Tracking(bool enqueue = false);
  void do_Land(bool enqueue = false);
  void do_Wait(uint64_t duration_ms, bool enqueue = false);
  void do_SetYaw(double angle, double rate, bool is_clockwise,
                 bool is_relative);
public:
  // Getters & Setters
  inline bool getActive() const {
    return m_de_pilot_operation != DEPILOT_OP_DISABLED;
  }

  inline bool getAllowRCControl() const { return m_allow_RCControl; }

  inline void setTargetAltitude(double altitude) {
    m_target_altitude = altitude;
  }

  // Operation management
  DRONEENGAGE_PILOT_OPERATION getCurrentOperation() const;
  bool isOperationActive(DRONEENGAGE_PILOT_OPERATION operation) const;

  // Queue management
  void clearOperationQueue();
  size_t getOperationQueueSize() const;

  // Target altitude management
  double getTargetAltitude() const;

public:
  void OnFlightModeChanged();

  
private:
  CDEPilotManager() {}
  ~CDEPilotManager() = default;

private:
  // Internal helper methods
  void setOperation(DRONEENGAGE_PILOT_OPERATION operation);
  void deactivateAllOperations();
  bool isCompatibleMode();
  CDEPilotOperationBase *
  getOperationInstance(DRONEENGAGE_PILOT_OPERATION operation) const;

  // Sub-operation event dispatcher
  void emitSubOperationEvent(const CDEPilotTaskBase& task);

  // Transition policy
  bool canAdvanceFromCurrentOperation() const;
  bool isYawTargetPending() const;

  // Queue request type
  struct QueuedOperation {
    DRONEENGAGE_PILOT_OPERATION operation;
    double target_altitude; // Only used for CHANGE_ALTITUDE
    bool has_altitude_param;
  };

  de::fcb::CFCBFacade& m_fcb_facade = de::fcb::CFCBFacade::getInstance();
            
  // Current operation state
  CDEPilotOperationBase *m_operation_instance = nullptr;
  double m_target_altitude = 0.0;

  // Operation queue
  std::queue<QueuedOperation> m_operation_queue;

  bool m_allow_RCControl = true;
  bool m_de_pilot_enabled = false;
  DRONEENGAGE_PILOT_OPERATION m_de_pilot_operation = DEPILOT_OP_DISABLED;
};

} // namespace depilot
} // namespace fcb
} // namespace de

#endif // FCB_DE_PILOT_MANAGER_H_
