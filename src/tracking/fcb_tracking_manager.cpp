#include "../de_common/helpers/colors.hpp"
#include "../de_common/helpers/helpers.hpp"

#include "fcb_tracking_manager.hpp"

#include "../de_common/de_databus/configFile.hpp"
#include "../de_common/de_databus/localConfigFile.hpp"

#include "../fcb_main.hpp"
#include <cmath>

using Json_de = nlohmann::json;
using namespace de::fcb::tracking;

de::fcb::CFCBMain &m_fcbMain2 = de::fcb::CFCBMain::getInstance();

void CTrackingManager::init() {
  readConfigParameters();

  m_PID_X.setPID(m_x_PID_P, m_x_PID_I, 0);
  m_PID_YZ.setPID(m_yz_PID_P, m_yz_PID_I, 0);
}

void CTrackingManager::readConfigParameters() {
  de::CConfigFile &cConfigFile = de::CConfigFile::getInstance();
  const Json_de &jsonConfig = cConfigFile.GetConfigJSON();

  if (jsonConfig.contains("follow_me")) {
    const Json_de &follow_me = jsonConfig["follow_me"];
    if (follow_me.contains("PID_P_X")) {
      m_x_PID_P = follow_me["PID_P_X"].get<double>();
      std::cout << _SUCCESS_CONSOLE_TEXT_ << "Apply:  " << _LOG_CONSOLE_BOLD_TEXT
                << "PID_P_X:" << _INFO_CONSOLE_BOLD_TEXT
                << std::to_string(m_x_PID_P) << _NORMAL_CONSOLE_TEXT_
                << std::endl;
    }

    if (follow_me.contains("PID_P_Y")) {
      m_yz_PID_P = follow_me["PID_P_Y"].get<double>();
      std::cout << _SUCCESS_CONSOLE_TEXT_ << "Apply:  " << _LOG_CONSOLE_BOLD_TEXT
                << "PID_P_Y:" << _INFO_CONSOLE_BOLD_TEXT
                << std::to_string(m_yz_PID_P) << _NORMAL_CONSOLE_TEXT_
                << std::endl;
    }

    if (follow_me.contains("PID_I_X")) {
      m_x_PID_I = follow_me["PID_I_X"].get<double>();
      std::cout << _SUCCESS_CONSOLE_TEXT_ << "Apply:  " << _LOG_CONSOLE_BOLD_TEXT
                << "PID_I_X:" << _INFO_CONSOLE_BOLD_TEXT
                << std::to_string(m_x_PID_I) << _NORMAL_CONSOLE_TEXT_
                << std::endl;
    }

    if (follow_me.contains("PID_I_Y")) {
      m_yz_PID_I = follow_me["PID_I_Y"].get<double>();
      std::cout << _SUCCESS_CONSOLE_TEXT_ << "Apply:  " << _LOG_CONSOLE_BOLD_TEXT
                << "PID_I_Y:" << _INFO_CONSOLE_BOLD_TEXT
                << std::to_string(m_yz_PID_I) << _NORMAL_CONSOLE_TEXT_
                << std::endl;
    }

    

  if (jsonConfig.contains("expo_factor")) {
    m_expo_factor = jsonConfig["expo_factor"].get<double>();
  }

  if (jsonConfig.contains("deadband")) {
    m_deadband = jsonConfig["deadband"].get<double>();
  }

  if (jsonConfig.contains("rate_limit")) {
    m_rate_limit = jsonConfig["rate_limit"].get<double>();
  }

  // Optional per-axis overrides
  if (jsonConfig.contains("deadband_x")) {
    m_deadband_x = jsonConfig["deadband_x"].get<double>();
  } else {
    m_deadband_x = m_deadband;
  }
  if (jsonConfig.contains("deadband_yz")) {
    m_deadband_yz = jsonConfig["deadband_yz"].get<double>();
  } else {
    m_deadband_yz = m_deadband;
  }
  if (jsonConfig.contains("expo_x")) {
    m_expo_x = jsonConfig["expo_x"].get<double>();
  } else {
    m_expo_x = m_expo_factor;
  }
  if (jsonConfig.contains("expo_yz")) {
    m_expo_yz = jsonConfig["expo_yz"].get<double>();
  } else {
    m_expo_yz = m_expo_factor;
  }
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
    m_prev_initialized = false; // reset shaping state
    m_PID_X.setPID(m_x_PID_P, m_x_PID_I, 0); // reset integrator
    m_PID_YZ.setPID(m_yz_PID_P, m_yz_PID_I, 0);
    de::fcb::CFCBMain::getInstance().adjustRemoteJoystickByMode(
        RC_SUB_ACTION::RC_SUB_ACTION_RELEASED);
    break;

  case TrackingTarget_STATUS_TRACKING_DETECTED:
    m_tracking_running = true;
    m_object_detected = true;
    de::fcb::CFCBMain::getInstance().adjustRemoteJoystickByMode(
        RC_SUB_ACTION::RC_SUB_ACTION_JOYSTICK_CHANNELS);
    break;

  case TrackingTarget_STATUS_TRACKING_ENABLED:
    m_tracking_running = true;
    m_PID_X.setPID(m_x_PID_P, m_x_PID_I, 0);
    m_PID_YZ.setPID(m_yz_PID_P, m_yz_PID_I, 0);
    break;

  case TrackingTarget_STATUS_TRACKING_STOPPED:
    m_object_detected = false;
    m_tracking_running = false;
    m_PID_X.setPID(m_x_PID_P, m_x_PID_I, 0);
    m_PID_YZ.setPID(m_yz_PID_P, m_yz_PID_I, 0);
    m_prev_initialized = false; // reset shaping state
    de::fcb::CFCBMain::getInstance().adjustRemoteJoystickByMode(
        RC_SUB_ACTION::RC_SUB_ACTION_RELEASED);
    break;

  default:
    break;
  }
}

void CTrackingManager::onTrack(const double x, const double yz,
                               const bool is_xy) {

  if (!m_tracking_running || !m_object_detected) {
    return;
  }
  std::chrono::high_resolution_clock::time_point now =
      std::chrono::high_resolution_clock::now();

  // Compute dt (seconds) for time-based shaping
  double dt = 0.0;
  if (m_prev_initialized) {
    dt = std::chrono::duration<double>(now - m_last_message_time).count();
  }
  m_last_message_time = now;

#ifdef DDEBUG
  std::cout << _INFO_CONSOLE_BOLD_TEXT << "onTrack >> "
            << _LOG_CONSOLE_BOLD_TEXT << "  x:" << _INFO_CONSOLE_BOLD_TEXT << x
            << _LOG_CONSOLE_BOLD_TEXT << "  yz:" << _INFO_CONSOLE_BOLD_TEXT
            << yz << _LOG_CONSOLE_BOLD_TEXT
            << "  is_xy:" << _INFO_CONSOLE_BOLD_TEXT << is_xy
            << _NORMAL_CONSOLE_TEXT_ << std::endl;
#endif

  const RCMAP_CHANNELS_MAP_INFO_STRUCT rc_map =
      m_fcbMain2.getRCChannelsMapInfo();
  
  std::cout << _INFO_CONSOLE_BOLD_TEXT << "rc_map.use_smart_rc:" << rc_map.use_smart_rc << "     rc_map.is_valid:" << rc_map.is_valid << _NORMAL_CONSOLE_TEXT_ << std::endl;
  if ((!rc_map.use_smart_rc) || (!rc_map.is_valid))
   
  {
    return;
  }
  
  // values: [-0.5,0.5]

  // value: [0,1000] IMPORTANT: SKIP_RC_CHANNEL (-999) means channel release
  // 'R': Rudder
  // 'T': Throttle
  // 'A': Aileron
  // 'E': Elevator
  int16_t rc_channels[RC_CHANNELS_MAX] = {SKIP_RC_CHANNEL};

  // Post-processing: rate limiting, deadband, and expo response

  // Initialize previous state once
  if (!m_prev_initialized) {
    m_prev_dx = x;
    m_prev_dy = yz;
    m_prev_initialized = true;
  }
  
  // 1) Rate limiting (outlier rejection) - time based
  // m_rate_limit is interpreted as normalized units per second
  auto clampStepDt = [this, dt](double prev, double cur) {
    if (dt <= 0.0) return cur; // first sample after init
    const double max_step = m_rate_limit * dt;
    const double step = cur - prev;
    if (std::abs(step) > max_step) {
      return prev + std::copysign(max_step, step);
    }
    return cur;
  };
  double sx = clampStepDt(m_prev_dx, x);
  double sy = clampStepDt(m_prev_dy, yz);
  
  // 2) Deadband (per-axis if provided)
  auto applyDeadbandX = [this](double v) {
    return (std::abs(v) < m_deadband_x) ? 0.0 : v;
  };
  auto applyDeadbandYZ = [this](double v) {
    return (std::abs(v) < m_deadband_yz) ? 0.0 : v;
  };
  sx = applyDeadbandX(sx);
  sy = applyDeadbandYZ(sy);
  
  // 3) Expo response (per-axis if provided)
  auto expoX = [this](double v) {
    return v * (1.0 - m_expo_x) + std::pow(v, 3) * m_expo_x;
  };
  auto expoYZ = [this](double v) {
    return v * (1.0 - m_expo_yz) + std::pow(v, 3) * m_expo_yz;
  };
  sx = expoX(sx);
  sy = expoYZ(sy);
  
  // 4) Optional precision limiting
  auto round3 = [](double v) { return std::round(v * 1000.0) / 1000.0; };
  sx = round3(sx);
  sy = round3(sy);

  // Clamp shaped inputs to [-0.5, 0.5] BEFORE PID to protect integrators
  sx = std::clamp(sx, -0.5, 0.5);
  sy = std::clamp(sy, -0.5, 0.5);

  // Update previous for next invocation
  m_prev_dx = sx;
  m_prev_dy = sy;

  // Feed PID with shaped values (normalized [-0.5, 0.5])
  m_x = m_PID_X.calculate(sx);
  m_yz = m_PID_YZ.calculate(sy);

  // Clamp PID outputs back to [-0.5, 0.5]
  m_x = std::clamp(m_x, -0.5, 0.5);
  m_yz = std::clamp(m_yz, -0.5, 0.5);
  
  int tracking_x = static_cast<int>(m_x * 1000 + 500);
  int tracking_yz = static_cast<int>(m_yz * 1000 + 500);

  switch (
      de::fcb::CFCBMain::getInstance().getAndruavVehicleInfo().vehicle_type) {
  case ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_TRI:
  case ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_QUAD:
    break;

  case ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_BOAT:
    break;

  case ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_ROVER:
    break;
  }
  if (is_xy) {
    // x & y .... forward camera.
    rc_channels[rc_map.rcmap_roll] = 1000 - tracking_x;
    rc_channels[rc_map.rcmap_pitch] = 1000 - tracking_yz;
    rc_channels[rc_map.rcmap_yaw] =
        500; // to be aligned with default settings of Ardu
    rc_channels[rc_map.rcmap_throttle] = 500;
  } else {
    // x & z .... vertical camera.
    rc_channels[rc_map.rcmap_roll] =
        1000 - tracking_x; // to be aligned with default settings of Ardu
    rc_channels[rc_map.rcmap_pitch] = 1000 - tracking_yz;
    rc_channels[rc_map.rcmap_yaw] = 500;
    rc_channels[rc_map.rcmap_throttle] = 500;
  }

#ifdef DEBUG
  std::cout << _INFO_CONSOLE_BOLD_TEXT << "onTrack >> "
            << _LOG_CONSOLE_BOLD_TEXT << "  rcmap_roll:" << rc_map.rcmap_roll
            << ":" << _INFO_CONSOLE_BOLD_TEXT << rc_channels[rc_map.rcmap_roll]
            << _LOG_CONSOLE_BOLD_TEXT << "  rcmap_pitch:" << rc_map.rcmap_pitch
            << ":" << _INFO_CONSOLE_BOLD_TEXT << rc_channels[rc_map.rcmap_pitch]
            << _LOG_CONSOLE_BOLD_TEXT << "  rcmap_yaw:" << rc_map.rcmap_yaw
            << ":" << _INFO_CONSOLE_BOLD_TEXT << rc_channels[rc_map.rcmap_yaw]
            << _LOG_CONSOLE_BOLD_TEXT
            << "  rcmap_throttle:" << rc_map.rcmap_throttle << ":"
            << _INFO_CONSOLE_BOLD_TEXT << rc_channels[rc_map.rcmap_throttle]
            << _NORMAL_CONSOLE_TEXT_ << std::endl;
#endif

  m_fcbMain2.updateRemoteControlChannels(rc_channels);
}
