#ifndef FCB_DE_PILOT_IDLE_H_
#define FCB_DE_PILOT_IDLE_H_

#include "fcb_de_pilot_operation_base.hpp"

namespace de {
namespace fcb {
namespace depilot {

class CDEPilotIdle : public CDEPilotOperationBase {
public:
  static CDEPilotIdle &getInstance() {
    static CDEPilotIdle instance;
    return instance;
  }

  void init() override {}
  void update() override {}
  void uninit() override {}
  void readConfigParameters() override {}
  void reloadParametersIfConfigChanged() override {}
  void setPhase(int phase) override {}
  int getPhase() const override { return 0; }
  void setActive(bool active) override {}
  bool getActive() const override { return false; }
  bool isCompleted() override { return false; }
};

} // namespace depilot
} // namespace fcb
} // namespace de

#endif // FCB_DE_PILOT_IDLE_H_