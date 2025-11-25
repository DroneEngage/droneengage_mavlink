#ifndef FCB_TRACKER_QUAD_LOGIC_H_
#define FCB_TRACKER_QUAD_LOGIC_H_

#include "fcb_tracker_logic.hpp"

namespace de {
namespace fcb {
namespace tracking {

class CTrackerQuadLogic : public CTrackerLogic {
public:
  static CTrackerQuadLogic &getInstance() {
    static CTrackerQuadLogic instance;

    return instance;
  }

  CTrackerQuadLogic(CTrackerQuadLogic const &) = delete;
  void operator=(CTrackerQuadLogic const &) = delete;

private:
  CTrackerQuadLogic() {}

public:
  ~CTrackerQuadLogic() {}

public:
  void onTrack(const double x, const double yz,
               const bool is_forward_camera) override;

protected:
  void readConfigParameters() override;

  void trackingStanding(const double x, const double yz, const double tracking_x, const double tracking_yz);
  void trackingDroneForward(const double x, const double yz, const double tracking_x, const double tracking_yz);

private:
  bool m_loose_altitude = false;
};
} // namespace tracking
} // namespace fcb
} // namespace de

#endif
