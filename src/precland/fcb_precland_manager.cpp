#include "../de_common/helpers/colors.hpp"
#include "../de_common/helpers/helpers.hpp"

#include "fcb_precland_manager.hpp"

#include "../de_common/de_databus/configFile.hpp"
#include "../defines.hpp"
#include "../fcb_main.hpp"
#include "../tracking/fcb_tracking_manager.hpp"

#include <chrono>
#include <iostream>

#include <mavlink_command.h>

using Json_de = nlohmann::json;
using namespace de::fcb::precland;

// PRECLAND_TARGET.t is a CLOCK_MONOTONIC microseconds stamp taken at frame
// grab by de_precland - compare ages against the same clock, NOT epoch-based
// get_time_usec().
static int64_t monotonicUsec()
{
  return std::chrono::duration_cast<std::chrono::microseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

void CPreclandManager::init() { readConfigParameters(); }

void CPreclandManager::reloadParametersIfConfigChanged() {
  readConfigParameters();
}

void CPreclandManager::readConfigParameters() {
  const Json_de &jsonConfig =
      de::CConfigFile::getInstance().GetConfigJSON();

  // defaults
  m_allowed_modes = {VEHICLE_MODE_LAND, VEHICLE_MODE_RTL};
  m_max_target_age_us = 300000;
  m_min_send_interval_us = 100000;
  m_min_tags = 1;
  m_frame = MAV_FRAME_BODY_FRD;

  if (!jsonConfig.contains("precland") || !jsonConfig["precland"].is_object()) {
    m_enabled = false;
    return;
  }

  const Json_de &cfg = jsonConfig["precland"];

  if (cfg.contains("enabled") && cfg["enabled"].is_boolean()) {
    m_enabled = cfg["enabled"].get<bool>();
  }

  if (cfg.contains("allowed_modes") && cfg["allowed_modes"].is_array()) {
    m_allowed_modes.clear();
    for (const auto &m : cfg["allowed_modes"]) {
      if (m.is_number_integer())
        m_allowed_modes.push_back(m.get<int>());
    }
    if (m_allowed_modes.empty())
      m_allowed_modes = {VEHICLE_MODE_LAND, VEHICLE_MODE_RTL};
  }

  if (cfg.contains("max_target_age_ms") &&
      cfg["max_target_age_ms"].is_number()) {
    m_max_target_age_us =
        (int64_t)cfg["max_target_age_ms"].get<double>() * 1000;
  }

  if (cfg.contains("max_send_hz") && cfg["max_send_hz"].is_number()) {
    const double hz = cfg["max_send_hz"].get<double>();
    if (hz > 0)
      m_min_send_interval_us = (int64_t)(1000000.0 / hz);
  }

  if (cfg.contains("min_tags") && cfg["min_tags"].is_number_integer()) {
    m_min_tags = cfg["min_tags"].get<int>();
  }

  if (cfg.contains("frame") && cfg["frame"].is_number_integer()) {
    const int f = cfg["frame"].get<int>();
    if (f == MAV_FRAME_BODY_NED || f == MAV_FRAME_BODY_FRD)
      m_frame = (uint8_t)f;
  }
}

void CPreclandManager::onPreclandStatus(const int state) {
  m_state = state;
}

bool CPreclandManager::fn_isGateOpen(const int64_t capture_time_us,
                                     const int n_tags, const bool valid) {
  // A rejection is a real reason the FC is not being fed. Report it once per
  // transition so an aborted approach is diagnosable, instead of either
  // staying silent or spamming the console every frame.
  auto reject = [this](const char *reason) {
    if (m_gate_reason != reason) {
      m_gate_reason = reason;
      std::cout << _INFO_CONSOLE_BOLD_TEXT << "PRECLAND gate closed: "
                << _ERROR_CONSOLE_BOLD_TEXT_ << reason << _INFO_CONSOLE_TEXT
                << " (rejects=" << (m_gate_reject_count + 1)
                << " sent=" << m_sent_count << ")" << _NORMAL_CONSOLE_TEXT_
                << std::endl;
    }
    ++m_gate_reject_count;
    return false;
  };

  if (!m_enabled)
    return reject("precland disabled");

  const ANDRUAV_VEHICLE_INFO &info =
      de::fcb::CFCBMain::getInstance().getAndruavVehicleInfo();

  if (!info.is_armed)
    return reject("vehicle disarmed");

  bool mode_allowed = false;
  for (const int m : m_allowed_modes) {
    if (info.flying_mode == m) {
      mode_allowed = true;
      break;
    }
  }
  if (!mode_allowed)
    return reject("flight mode not allowed");

  // Mutual exclusion: two controllers must never drive the vehicle at once.
  const int tracking_status =
      de::fcb::tracking::CTrackingManager::getInstance().getTrackingStatus();
  if (tracking_status == TrackingTarget_STATUS_TRACKING_ENABLED ||
      tracking_status == TrackingTarget_STATUS_TRACKING_DETECTED)
    return reject("tracker actively controlling");

  if (!valid)
    return reject("position_valid false");

  if (n_tags < m_min_tags)
    return reject("insufficient tags fused");

  const int64_t now = monotonicUsec();
  if (now - capture_time_us > m_max_target_age_us)
    return reject("stale target");

  // Rate limiting is a normal throttle, not a failure: de_precland already
  // publishes at its own send_hz and this is only a backstop. Counting it
  // would drown the reject counter and latch a misleading gate reason.
  if (now - m_last_send_time_us < m_min_send_interval_us)
    return false;

  if (!m_gate_reason.empty()) {
    std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_ << "PRECLAND gate open"
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
    m_gate_reason.clear();
  }
  return true;
}

void CPreclandManager::onPreclandTarget(
    const double x, const double y, const double z, const double ax,
    const double ay, const int n_tags, const double rmse,
    const int64_t capture_time_us, const bool valid, const int target_num) {
  if (!fn_isGateOpen(capture_time_us, n_tags, valid))
    return;

  mavlink_message_t mavlink_message;
  const float q[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // unused for position target

  // Identity: de_mavlink owns the FC link, so it stamps the vehicle's own
  // sysid and the companion-computer compid. A hardcoded 255/190 is the
  // Mission Planner GCS identity and would collide with a real GCS on the
  // same link.
  const uint8_t src_sysid =
      (uint8_t)mavlinksdk::CVehicle::getInstance().getSysId();
  const uint8_t src_compid = (uint8_t)MAV_COMP_ID_ONBOARD_COMPUTER;

  mavlink_msg_landing_target_pack(
      src_sysid, src_compid,
      &mavlink_message, (uint64_t)capture_time_us, (uint8_t)target_num,
      m_frame, (float)ax, (float)ay,
      0.0f, // distance - unknown, FC derives it from z
      0.0f, 0.0f, // size_x, size_y
      (float)x, (float)y, (float)z, q, LANDING_TARGET_TYPE_VISION_FIDUCIAL,
      1);   // position_valid

  mavlinksdk::CMavlinkCommand::getInstance().sendNative(mavlink_message);

  m_last_send_time_us = monotonicUsec();
  ++m_sent_count;

#ifdef DEBUG
  std::cout << _INFO_CONSOLE_TEXT << "PRECLAND LANDING_TARGET x=" << x
            << " y=" << y << " z=" << z << " n=" << n_tags << " rmse=" << rmse
            << _NORMAL_CONSOLE_TEXT_ << std::endl;
#endif
}
