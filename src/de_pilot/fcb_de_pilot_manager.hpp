#ifndef FCB_DE_PILOT_MANAGER_H_
#define FCB_DE_PILOT_MANAGER_H_

#include "../defines.hpp"
#include "fcb_de_pilot_change_altitude.hpp"
#include "fcb_de_pilot_operation_base.hpp"
#include "fcb_de_pilot_stabilization.hpp"
#include "fcb_de_pilot_tracking.hpp"
#include <cstdint>

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
  void do_ChangeAltitude(double target_altitude);
  void do_Stabilize();
  void do_Tracking();
  void do_Land();
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
  void setOperation(DRONEENGAGE_PILOT_OPERATION operation);
  DRONEENGAGE_PILOT_OPERATION getCurrentOperation() const;
  bool isOperationActive(DRONEENGAGE_PILOT_OPERATION operation) const;

  // Target altitude management
  double getTargetAltitude() const;

public:
  void OnFlightModeChanged();

private:
  CDEPilotManager() {}
  ~CDEPilotManager() = default;

private:
  // Internal helper methods
  void deactivateAllOperations();
  bool isCompatibleMode();
  CDEPilotOperationBase *
  getOperationInstance(DRONEENGAGE_PILOT_OPERATION operation) const;

  // Current operation state
  CDEPilotOperationBase *m_operation_instance = nullptr;
  double m_target_altitude = 0.0;

  bool m_allow_RCControl = true;
  bool m_de_pilot_enabled = false;
  DRONEENGAGE_PILOT_OPERATION m_de_pilot_operation = DEPILOT_OP_DISABLED;
};

} // namespace depilot
} // namespace fcb
} // namespace de

#endif // FCB_DE_PILOT_MANAGER_H_
