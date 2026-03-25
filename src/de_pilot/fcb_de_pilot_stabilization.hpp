#ifndef FCB_DE_PILOT_STABILIZATION_H_
#define FCB_DE_PILOT_STABILIZATION_H_

#include "fcb_de_pilot_operation_base.hpp"
#include <cstdint>

namespace de {
namespace fcb {
namespace depilot {

class CDEPilotStabilization : public CDEPilotOperationBase {
public:
  static CDEPilotStabilization &getInstance() {
    static CDEPilotStabilization instance;
    return instance;
  }

  CDEPilotStabilization(CDEPilotStabilization const &) = delete;
  void operator=(CDEPilotStabilization const &) = delete;

private:
  CDEPilotStabilization() {}

public:
  ~CDEPilotStabilization() {}

  // Base class interface implementation
  void init() override;
  void update() override;
  void uninit() override;
  void readConfigParameters() override;
  void reloadParametersIfConfigChanged() override;
  void setPhase(int phase) override;
  int getPhase() const override;
  void setActive(bool active) override;
  bool getActive() const override;
  bool isCompleted() override;

  // Class-specific interface
  void startStabilization();
  void updateStabilization();
  void stopStabilization();
  bool isStabilizationActive() const;

private:
  enum StabilizationPhase { PHASE_IDLE, PHASE_STABILIZING, PHASE_COMPLETE };

  // Stabilization-specific member variables (base class provides m_active,
  // m_generic_phase, m_phase_start_time, m_last_update_time)
  StabilizationPhase m_phase = PHASE_IDLE;
};

} // namespace depilot
} // namespace fcb
} // namespace de

#endif
