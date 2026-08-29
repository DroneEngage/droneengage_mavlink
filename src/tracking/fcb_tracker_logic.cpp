#include "fcb_tracker_logic.hpp"
#include "../de_common/helpers/colors.hpp"
#include "../fcb_main.hpp"

using namespace de::fcb::tracking;

CTrackerLogic::CTrackerLogic() {}

CTrackerLogic::~CTrackerLogic() {}

void CTrackerLogic::init() {
  readConfigParameters();
}

void CTrackerLogic::reloadParametersIfConfigChanged() {
  readConfigParameters();
}

bool CTrackerLogic::isTrackingActive() const {
  const RCMAP_CHANNELS_MAP_INFO_STRUCT rc_map =
      de::fcb::CFCBMain::getInstance().getRCChannelsMapInfo();

  return rc_map.use_smart_rc && rc_map.is_valid;
}

void CTrackerLogic::onStatusChanged(const int status, const uint8_t tracking_camera_direction, const bool ai_priority) {
#ifdef DEBUG
  std::cout << _INFO_CONSOLE_BOLD_TEXT
            << "onTrackStatusChanged:" << _LOG_CONSOLE_BOLD_TEXT
            << std::to_string(status) << _NORMAL_CONSOLE_TEXT_ << std::endl;
#endif

  m_tracking_camera_direction = tracking_camera_direction;
}
