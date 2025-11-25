#ifndef FCB_TRACKER_QUAD_LOGIC_H_
#define FCB_TRACKER_QUAD_LOGIC_H_

#include "fcb_tracker_logic.hpp"
#include <cstdint>

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
  void onStatusChanged(const int status) override;
  
  void onTrack(const double x, const double yz,
               const bool is_forward_camera) override;

  
public:
  void readConfigParameters() override;

  void trackingStanding(const double x, const double yz, const double tracking_x, const double tracking_yz);
  void trackingDroneForward(const double x, const double yz, const double tracking_x, const double tracking_yz);

  inline void resetTrackerCopterStatus() {
    m_tracker_copter_status = 0;
  }

  inline void setTrackerCopterStatus(uint8_t status)
  {
    m_tracker_copter_status = status;
  }

  inline void addTrackerCopterStatus(uint8_t status)
  {
    m_tracker_copter_status |= status;
  }

  inline bool isTrackerCopterStatus(uint8_t status)
  {
    return m_tracker_copter_status & status;
  }

private:
  bool m_loose_altitude = false;

  uint8_t m_tracker_copter_status = 0;
};
} // namespace tracking
} // namespace fcb
} // namespace de

#endif
