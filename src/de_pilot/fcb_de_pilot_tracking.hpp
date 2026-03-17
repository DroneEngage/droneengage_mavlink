#ifndef FCB_DE_PILOT_TRACKING_H_
#define FCB_DE_PILOT_TRACKING_H_

#include "fcb_de_pilot_operation_base.hpp"
#include <cstdint>

namespace de {
namespace fcb {
namespace depilot {

class CDEPilotTracking : public CDEPilotOperationBase {
public:
  static CDEPilotTracking &getInstance() {
    static CDEPilotTracking instance;
    return instance;
  }

  CDEPilotTracking(CDEPilotTracking const &) = delete;
  void operator=(CDEPilotTracking const &) = delete;

private:
  CDEPilotTracking() {}

public:
  ~CDEPilotTracking() {}

  // Base class interface implementation
  void init() override;
  void update() override;
  void uninit() override;
  void readConfigParameters() override;
  void setPhase(int phase) override;
  int getPhase() const override;
  void setActive(bool active) override;
  bool getActive() const override;
  bool isCompleted() override;

  // Class-specific interface
  void startTracking();
  void stopTracking();
  bool isTrackingActive() const;
  void updateTrackingTimestamp(); // Called by CTrackingManager when tracking data is received

private:
  enum TrackingPhase { 
    PHASE_IDLE, 
    PHASE_INITIALIZING, 
    PHASE_TRACKING, 
    PHASE_COMPLETE 
  };

  // Tracking-specific member variables
  TrackingPhase m_phase = PHASE_IDLE;
  
  // Configuration parameters
  bool m_auto_enable_on_detection = true;
  uint64_t m_tracking_timeout_us = 5000000; // 5 seconds timeout
  uint64_t m_last_tracking_update_time = 0;
};

} // namespace depilot
} // namespace fcb
} // namespace de

#endif // FCB_DE_PILOT_TRACKING_H_
