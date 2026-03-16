#ifndef FCB_TRACKING_MANAGER_H_
#define FCB_TRACKING_MANAGER_H_

#include "../de_common/de_databus/messages.hpp"

#include "fcb_tracker_logic_plan.hpp"
#include "fcb_tracker_logic_quad.hpp"

#include "../de_common/helpers/json_nlohmann.hpp"
using Json_de = nlohmann::json;

namespace de {
namespace fcb {
namespace tracking {

class CTrackingManager {
public:
  static CTrackingManager &getInstance() {
    static CTrackingManager instance;

    return instance;
  }

  CTrackingManager(CTrackingManager const &) = delete;
  void operator=(CTrackingManager const &) = delete;

private:
  CTrackingManager() {}

public:
  ~CTrackingManager() {}

public:
  void init();
  void onTrack(const double x, const double yz, const bool is_forward_camera);
  void onStatusChanged(const int status);
  void reloadParametersIfConfigChanged();

public:
  inline int getTrackingStatus() const { return m_tracking_status; }

private:
  void readConfigParameters()
  {
    // can be used to load settings if needed.
  };

  CTrackerLogic &getTracker();

private:
  bool m_tracking_running = false;
  bool m_object_detected = false;

  TRACKING_TYPE m_tracking_type = TRACKING_STANDING;

  int m_tracking_status = TrackingTarget_STATUS_TRACKING_STOPPED;

  CTrackerPlanLogic &m_tracker_plan_logic = CTrackerPlanLogic::getInstance();
  CTrackerQuadLogic &m_tracker_quad_logic = CTrackerQuadLogic::getInstance();
};
} // namespace tracking
} // namespace fcb
} // namespace de

#endif