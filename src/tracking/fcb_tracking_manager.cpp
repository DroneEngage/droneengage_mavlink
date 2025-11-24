#include "../de_common/helpers/colors.hpp"
#include "../de_common/helpers/helpers.hpp"

#include "fcb_tracking_manager.hpp"

#include "../de_common/de_databus/configFile.hpp"
#include "../de_common/de_databus/localConfigFile.hpp"

#include "fcb_tracker_logic_plan.hpp"
#include "fcb_tracker_logic_quad.hpp"
#include "../fcb_main.hpp"
#include <cmath>
#include <string>

using Json_de = nlohmann::json;
using namespace de::fcb::tracking;

void CTrackingManager::init() {
  readConfigParameters();
m_tracker_plan_logic.init();
  m_tracker_quad_logic.init();
}

void CTrackingManager::reloadParametersIfConfigChanged() {
  m_tracker_plan_logic.reloadParametersIfConfigChanged();
  m_tracker_quad_logic.reloadParametersIfConfigChanged();
}

CTrackerLogic &CTrackingManager::getTracker() {
  switch (
      de::fcb::CFCBMain::getInstance().getAndruavVehicleInfo().vehicle_type) {
  case ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_TRI:
  case ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_QUAD:
    return m_tracker_quad_logic;
  case ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_PLANE:
    return m_tracker_plan_logic;
  default:
    return m_tracker_plan_logic;
  }
}

void CTrackingManager::onStatusChanged(const int status) {
#ifdef DEBUG
  std::cout << _INFO_CONSOLE_BOLD_TEXT
            << "onTrackStatusChanged:" << _LOG_CONSOLE_BOLD_TEXT
            << std::to_string(status) << _NORMAL_CONSOLE_TEXT_ << std::endl;
#endif

  switch (status) {
  case TrackingTarget_STATUS_TRACKING_LOST:
    m_object_detected = false;
    getTracker().onStatusChanged(status);
    break;

  case TrackingTarget_STATUS_TRACKING_DETECTED:
    m_tracking_running = true;
    m_object_detected = true;
    getTracker().onStatusChanged(status);
    break;

  case TrackingTarget_STATUS_TRACKING_ENABLED:
    m_tracking_running = true;
    getTracker().onStatusChanged(status);
    break;

  case TrackingTarget_STATUS_TRACKING_STOPPED:
    m_object_detected = false;
    m_tracking_running = false;
    getTracker().onStatusChanged(status);
    break;

  default:
    break;
  }

  m_tracking_status = status;

}

void CTrackingManager::onTrack(const double x, const double yz,
                               const bool is_forward_camera) {

  if (!m_tracking_running || !m_object_detected) {
    return;
  }

  getTracker().onTrack(x, yz, is_forward_camera);
}
