#ifndef FCB_TRACKER_PLAN_LOGIC_H_
#define FCB_TRACKER_PLAN_LOGIC_H_

#include "fcb_tracker_logic.hpp"

namespace de {
namespace fcb {
namespace tracking {

class CTrackerPlanLogic : public CTrackerLogic {
public:
  static CTrackerPlanLogic &getInstance() {
    static CTrackerPlanLogic instance;

    return instance;
  }

  CTrackerPlanLogic(CTrackerPlanLogic const &) = delete;
  void operator=(CTrackerPlanLogic const &) = delete;

public:
  void onTrack(const double x, const double yz,
               const bool is_forward_camera) override;

protected:
  void readConfigParameters() override;

private:
  CTrackerPlanLogic() {}

public:
  ~CTrackerPlanLogic() {}

protected:
  void trackingFollowMe(const double tracking_x, const double tracking_yz);
  void trackingTarget(const double tracking_x, const double tracking_yz);
};
} // namespace tracking
} // namespace fcb
} // namespace de

#endif