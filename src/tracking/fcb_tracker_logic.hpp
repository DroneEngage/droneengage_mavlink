#ifndef FCB_TRACKER_LOGIC_HPP
#define FCB_TRACKER_LOGIC_HPP

#include <cstdint>

#include "../fcb_main.hpp"
#include "../de_common/de_databus/messages.hpp"

#include "../de_common/helpers/json_nlohmann.hpp"
using Json_de = nlohmann::json;

namespace de {
namespace fcb {
namespace tracking {
typedef enum TRACKING_TYPE {
  TRACKING_TARGET = 0,
  TRACKING_FOLLOW_ME = 1,
  TRACKING_STANDING =
      2 // when camera is looking at ground vertically on a drone.
} TRACKING_TYPE;

class CTrackerLogic {
public:
  CTrackerLogic();
  virtual ~CTrackerLogic();

  virtual void init();
  virtual void onTrack(const double x, const double yz) = 0;
  virtual void onStatusChanged(const int status, const uint8_t tracking_camera_direction = 0, const bool ai_priority = false);
  virtual void reloadParametersIfConfigChanged();

protected:
  virtual void readConfigParameters() = 0;

  // True when smart-RC channel mapping is configured and valid, i.e. it is
  // safe to feed shaped control output into the RC channels.
  bool isTrackingActive() const;

protected:
  TRACKING_TYPE m_tracking_type = TRACKING_TARGET;

  uint8_t m_tracking_camera_direction = TRACKING_CAMERA_DIRECTION_FRONT;

  de::fcb::CFCBMain &m_fcbMain = de::fcb::CFCBMain::getInstance();
};
} // namespace tracking
} // namespace fcb
} // namespace de
#endif // FCB_TRACKER_LOGIC_HPP
