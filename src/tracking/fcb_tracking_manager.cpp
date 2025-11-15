#include "../de_common/helpers/colors.hpp"
#include "../de_common/helpers/helpers.hpp"

#include "fcb_tracking_manager.hpp"

#include "../de_common/de_databus/configFile.hpp"
#include "../de_common/de_databus/localConfigFile.hpp"

#include "../fcb_main.hpp"
#include <cmath>
#include <string>

using Json_de = nlohmann::json;
using namespace de::fcb::tracking;

de::fcb::CFCBMain &m_fcbMain2 = de::fcb::CFCBMain::getInstance();

void CTrackingManager::init() {
  readConfigParameters();

  m_PID_X.setPID(m_x_PID_P, m_x_PID_I, m_x_PID_D);
  m_PID_YZ.setPID(m_yz_PID_P, m_yz_PID_I, m_yz_PID_D);
  
  // Initialize Kalman filters with configured parameters
  m_kalman_x.setParameters(m_kalman_process_noise_q, m_kalman_measurement_noise_r);
  m_kalman_yz.setParameters(m_kalman_process_noise_q, m_kalman_measurement_noise_r);
}

void CTrackingManager::readConfigParameters() {
  de::CConfigFile &cConfigFile = de::CConfigFile::getInstance();
  const Json_de &jsonConfig = cConfigFile.GetConfigJSON();
  bool expo_x_from_follow_me = false;
  bool expo_y_from_follow_me = false;

  if (jsonConfig.contains("follow_me")) {
    const Json_de &follow_me = jsonConfig["follow_me"];
    if (follow_me.contains("PID_P_X")) {
      m_x_PID_P = follow_me["PID_P_X"].get<double>();
      std::cout << _SUCCESS_CONSOLE_TEXT_
                << "Apply:  " << _LOG_CONSOLE_BOLD_TEXT
                << "PID_P_X:" << _INFO_CONSOLE_BOLD_TEXT
                << std::to_string(m_x_PID_P) << _NORMAL_CONSOLE_TEXT_
                << std::endl;
    }

    if (follow_me.contains("PID_P_Y")) {
      m_yz_PID_P = follow_me["PID_P_Y"].get<double>();
      std::cout << _SUCCESS_CONSOLE_TEXT_
                << "Apply:  " << _LOG_CONSOLE_BOLD_TEXT
                << "PID_P_Y:" << _INFO_CONSOLE_BOLD_TEXT
                << std::to_string(m_yz_PID_P) << _NORMAL_CONSOLE_TEXT_
                << std::endl;
    }

    if (follow_me.contains("PID_I_X")) {
      m_x_PID_I = follow_me["PID_I_X"].get<double>();
#ifdef  DEBUG
      std::cout << _SUCCESS_CONSOLE_TEXT_
                << "Apply:  " << _LOG_CONSOLE_BOLD_TEXT
                << "PID_I_X:" << _INFO_CONSOLE_BOLD_TEXT
                << std::to_string(m_x_PID_I) << _NORMAL_CONSOLE_TEXT_
                << std::endl;
#endif
    }

    if (follow_me.contains("PID_I_Y")) {
      m_yz_PID_I = follow_me["PID_I_Y"].get<double>();
#ifdef  DEBUG
      std::cout << _SUCCESS_CONSOLE_TEXT_
                << "Apply:  " << _LOG_CONSOLE_BOLD_TEXT
                << "PID_I_Y:" << _INFO_CONSOLE_BOLD_TEXT
                << std::to_string(m_yz_PID_I) << _NORMAL_CONSOLE_TEXT_
                << std::endl;
#endif
    }

    if (follow_me.contains("PID_D_X")) {
      m_x_PID_D = follow_me["PID_D_X"].get<double>();
#ifdef  DEBUG
      std::cout << _SUCCESS_CONSOLE_TEXT_
                << "Apply:  " << _LOG_CONSOLE_BOLD_TEXT
                << "PID_D_X:" << _INFO_CONSOLE_BOLD_TEXT
                << std::to_string(m_x_PID_D) << _NORMAL_CONSOLE_TEXT_
                << std::endl;
#endif
    }

    if (follow_me.contains("PID_D_Y")) {
      m_yz_PID_D = follow_me["PID_D_Y"].get<double>();
#ifdef  DEBUG
      std::cout << _SUCCESS_CONSOLE_TEXT_
                << "Apply:  " << _LOG_CONSOLE_BOLD_TEXT
                << "PID_D_Y:" << _INFO_CONSOLE_BOLD_TEXT
                << std::to_string(m_yz_PID_D) << _NORMAL_CONSOLE_TEXT_
                << std::endl;
#endif
    }

    // Expo can be provided under follow_me; give it priority over top-level
    // keys
    if (follow_me.contains("expo_x")) {
      m_expo_x = follow_me["expo_x"].get<double>();
      expo_x_from_follow_me = true;
#ifdef  DEBUG
      std::cout << _SUCCESS_CONSOLE_TEXT_
                << "Apply:  " << _LOG_CONSOLE_BOLD_TEXT
                << "expo_x:" << _INFO_CONSOLE_BOLD_TEXT
                << std::to_string(m_expo_x) << _NORMAL_CONSOLE_TEXT_
                << std::endl;
#endif
    }
    if (follow_me.contains("expo_y")) {
      m_expo_yz = follow_me["expo_y"].get<double>();
      expo_y_from_follow_me = true;
#ifdef  DEBUG
      std::cout << _SUCCESS_CONSOLE_TEXT_
                << "Apply:  " << _LOG_CONSOLE_BOLD_TEXT
                << "expo_y:" << _INFO_CONSOLE_BOLD_TEXT
                << std::to_string(m_expo_yz) << _NORMAL_CONSOLE_TEXT_
                << std::endl;
#endif
    }

    if (follow_me.contains("center_hold_enabled")) {
      m_center_hold_enabled = follow_me["center_hold_enabled"].get<bool>();
#ifdef  DEBUG
      std::cout << _SUCCESS_CONSOLE_TEXT_
                << "Apply:  " << _LOG_CONSOLE_BOLD_TEXT
                << "center_hold_enabled:" << _INFO_CONSOLE_BOLD_TEXT
                << (m_center_hold_enabled ? "true" : "false")
                << _NORMAL_CONSOLE_TEXT_ << std::endl;
#endif
    }
    if (follow_me.contains("center_hold_y_band")) {
      m_center_hold_y_band = follow_me["center_hold_y_band"].get<double>();
#ifdef  DEBUG
      std::cout << _SUCCESS_CONSOLE_TEXT_
                << "Apply:  " << _LOG_CONSOLE_BOLD_TEXT
                << "center_hold_y_band:" << _INFO_CONSOLE_BOLD_TEXT
                << std::to_string(m_center_hold_y_band) << _NORMAL_CONSOLE_TEXT_
                << std::endl;
#endif
    }
    if (follow_me.contains("center_hold_decay")) {
      m_center_hold_decay = follow_me["center_hold_decay"].get<double>();
#ifdef  DEBUG
      std::cout << _SUCCESS_CONSOLE_TEXT_
                << "Apply:  " << _LOG_CONSOLE_BOLD_TEXT
                << "center_hold_decay:" << _INFO_CONSOLE_BOLD_TEXT
                << std::to_string(m_center_hold_decay) << _NORMAL_CONSOLE_TEXT_
                << std::endl;
#endif
    }

    if (follow_me.contains("rate_limit")) {
      m_rate_limit = follow_me["rate_limit"].get<double>();
#ifdef  DEBUG
      std::cout << _SUCCESS_CONSOLE_TEXT_
                << "Apply:  " << _LOG_CONSOLE_BOLD_TEXT
                << "rate_limit:" << _INFO_CONSOLE_BOLD_TEXT
                << std::to_string(m_rate_limit) << _NORMAL_CONSOLE_TEXT_
                << std::endl;
#endif
    }

    // Optional per-axis overrides
    if (follow_me.contains("deadband_x")) {
      m_deadband_x = follow_me["deadband_x"].get<double>();
    }
    if (follow_me.contains("deadband_y")) {
      m_deadband_yz = follow_me["deadband_y"].get<double>();
    }
#ifdef  DEBUG
    std::cout << _SUCCESS_CONSOLE_TEXT_ << "Apply:  " << _LOG_CONSOLE_BOLD_TEXT
              << "deadband_x:" << _INFO_CONSOLE_BOLD_TEXT
              << std::to_string(m_deadband_x) << _NORMAL_CONSOLE_TEXT_
              << std::endl;
    std::cout << _SUCCESS_CONSOLE_TEXT_ << "Apply:  " << _LOG_CONSOLE_BOLD_TEXT
              << "deadband_y:" << _INFO_CONSOLE_BOLD_TEXT
              << std::to_string(m_deadband_yz) << _NORMAL_CONSOLE_TEXT_
              << std::endl;
#endif        
    // Kalman filter parameters
    if (follow_me.contains("kalman_enabled")) {
      m_kalman_enabled = follow_me["kalman_enabled"].get<bool>();
#ifdef  DEBUG
      std::cout << _SUCCESS_CONSOLE_TEXT_
                << "Apply:  " << _LOG_CONSOLE_BOLD_TEXT
                << "kalman_enabled:" << _INFO_CONSOLE_BOLD_TEXT
                << (m_kalman_enabled ? "true" : "false")
                << _NORMAL_CONSOLE_TEXT_ << std::endl;
#endif
    }
    if (follow_me.contains("kalman_process_noise_q")) {
      m_kalman_process_noise_q = follow_me["kalman_process_noise_q"].get<double>();
#ifdef  DEBUG
      std::cout << _SUCCESS_CONSOLE_TEXT_
                << "Apply:  " << _LOG_CONSOLE_BOLD_TEXT
                << "kalman_process_noise_q:" << _INFO_CONSOLE_BOLD_TEXT
                << std::to_string(m_kalman_process_noise_q) << _NORMAL_CONSOLE_TEXT_
                << std::endl;
#endif
    }
    if (follow_me.contains("kalman_measurement_noise_r")) {
      m_kalman_measurement_noise_r = follow_me["kalman_measurement_noise_r"].get<double>();
#ifdef  DEBUG
      std::cout << _SUCCESS_CONSOLE_TEXT_
                << "Apply:  " << _LOG_CONSOLE_BOLD_TEXT
                << "kalman_measurement_noise_r:" << _INFO_CONSOLE_BOLD_TEXT
                << std::to_string(m_kalman_measurement_noise_r) << _NORMAL_CONSOLE_TEXT_
                << std::endl;
#endif
    }
  }
}

void CTrackingManager::reloadParametersIfConfigChanged() {
  readConfigParameters();
  m_PID_X.setPID(m_x_PID_P, m_x_PID_I, m_x_PID_D);
  m_PID_YZ.setPID(m_yz_PID_P, m_yz_PID_I, m_yz_PID_D);
  m_kalman_x.setParameters(m_kalman_process_noise_q, m_kalman_measurement_noise_r);
  m_kalman_yz.setParameters(m_kalman_process_noise_q, m_kalman_measurement_noise_r);
  m_kalman_x.reset();
  m_kalman_yz.reset();
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
    m_prev_initialized = false;              // reset shaping state
    m_PID_X.setPID(m_x_PID_P, m_x_PID_I, 0); // reset integrator
    m_PID_YZ.setPID(m_yz_PID_P, m_yz_PID_I, 0);
    // Reset Kalman filters for clean state on next acquisition
    m_kalman_x.reset();
    m_kalman_yz.reset();
    // de::fcb::CFCBMain::getInstance().adjustRemoteJoystickByMode(
    //     RC_SUB_ACTION::RC_SUB_ACTION_RELEASED);
    break;

  case TrackingTarget_STATUS_TRACKING_DETECTED:
    m_tracking_running = true;
    m_object_detected = true;
    // de::fcb::CFCBMain::getInstance().adjustRemoteJoystickByMode(
    //     RC_SUB_ACTION::RC_SUB_ACTION_JOYSTICK_CHANNELS_GUIDED);
    break;

  case TrackingTarget_STATUS_TRACKING_ENABLED:
    m_tracking_running = true;
    m_PID_X.setPID(m_x_PID_P, m_x_PID_I, 0);
    m_PID_YZ.setPID(m_yz_PID_P, m_yz_PID_I, 0);
    m_kalman_x.reset();
    m_kalman_yz.reset();
    break;

  case TrackingTarget_STATUS_TRACKING_STOPPED:
    m_object_detected = false;
    m_tracking_running = false;
    m_PID_X.setPID(m_x_PID_P, m_x_PID_I, 0);
    m_PID_YZ.setPID(m_yz_PID_P, m_yz_PID_I, 0);
    m_kalman_x.reset();
    m_kalman_yz.reset();
    m_prev_initialized = false; // reset shaping state
    // de::fcb::CFCBMain::getInstance().adjustRemoteJoystickByMode(
    //     RC_SUB_ACTION::RC_SUB_ACTION_RELEASED);
    break;

  default:
    break;
  }

  m_trcking_status = status;
}

void CTrackingManager::onTrack(const double x, const double yz,
                               const bool is_forward_camera) {

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

  // pass dt to the PID controllers
  m_PID_X.setDeltaTime(dt);
  m_PID_YZ.setDeltaTime(dt);

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

  std::cout << _INFO_CONSOLE_BOLD_TEXT
            << "rc_map.use_smart_rc:" << rc_map.use_smart_rc
            << "     rc_map.is_valid:" << rc_map.is_valid
            << _NORMAL_CONSOLE_TEXT_ << std::endl;
  if ((!rc_map.use_smart_rc) || (!rc_map.is_valid))

  {
    return;
  }

  // Apply Kalman filtering before existing processing if enabled
  double kalman_x = x;
  double kalman_yz = yz;
  if (m_kalman_enabled && m_prev_initialized) {
    kalman_x = m_kalman_x.update(x, dt);
    kalman_yz = m_kalman_yz.update(yz, dt);
    
    // Use velocity estimates for adaptive response
    double vel_x = m_kalman_x.getVelocity();
    double vel_yz = m_kalman_yz.getVelocity();
    
    // Reduce rate limiting when we have confident velocity estimates
    if (std::abs(vel_x) > 0.01) {
      // Target is moving, allow faster response
      m_rate_limit = std::min(m_rate_limit * 1.5, 0.15);
    }
    if (std::abs(vel_yz) > 0.01) {
      // Target is moving, allow faster response  
      m_rate_limit = std::min(m_rate_limit * 1.5, 0.15);
    }
  }
  
  // Use Kalman-filtered values for subsequent processing
  double processed_x = kalman_x;
  double processed_yz = kalman_yz;
  
  // Post-processing: rate limiting, deadband, and expo response

  // Initialize previous state once
  if (!m_prev_initialized) {
    m_prev_dx = processed_x;
    m_prev_dy = processed_yz;
    m_prev_initialized = true;
  }

  // 1) Rate limiting (outlier rejection) - time based
  // m_rate_limit is interpreted as normalized units per second
  auto clampStepDt = [this, dt](double prev, double cur) {
    if (dt <= 0.0)
      return cur; // first sample after init
    const double max_step = m_rate_limit * dt;
    const double step = cur - prev;
    if (std::abs(step) > max_step) {
      return prev + std::copysign(max_step, step);
    }
    return cur;
  };
  double sx = clampStepDt(m_prev_dx, processed_x);
  double sy = clampStepDt(m_prev_dy, processed_yz);

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

  // 5) Input slew rate limiting BEFORE PID (prevents integrator corruption)
  auto clampInputStep = [this, dt](double prev, double cur) {
    if (dt <= 0.0)
      return cur;
    const double max_input_step = 0.3 * dt; // max 0.3 units per second
    const double step = cur - prev;
    if (std::abs(step) > max_input_step) {
      return prev + std::copysign(max_input_step, step);
    }
    return cur;
  };
  static double prev_input_x = 0.0;
  static double prev_input_yz = 0.0;
  sx = clampInputStep(prev_input_x, sx);
  sy = clampInputStep(prev_input_yz, sy);
  prev_input_x = sx;
  prev_input_yz = sy;

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

  // Enforce sign consistency: project output to match error sign when outside
  // deadband
  const double epsx = m_deadband_x;
  const double epsyz = m_deadband_yz;
  auto projectSign = [](double err, double out) {
    if (err > 0.0) {
      // ensure strictly positive output side (> 500 after mapping)
      const double mag = std::max(std::abs(out), 1e-3);
      return +mag;
    } else {
      // ensure strictly negative output side (< 500 after mapping)
      const double mag = std::max(std::abs(out), 1e-3);
      return -mag;
    }
  };
  if (std::abs(sx) > epsx && (m_x * sx) <= 0.0) {
    m_x = projectSign(sx, m_x);
  }
  if (std::abs(sy) > epsyz && (m_yz * sy) <= 0.0) {
    m_yz = projectSign(sy, m_yz);
  }

  // Optional center-hold for plane pitch when target is centered
  if (m_center_hold_enabled && is_forward_camera &&
      (de::fcb::CFCBMain::getInstance().getAndruavVehicleInfo().vehicle_type ==
       ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_PLANE)) {
    if (std::abs(sy) < m_center_hold_y_band) {
      const double decay = std::clamp(m_center_hold_decay, 0.0, 1.0);
      m_yz = (1.0 - decay) * m_pitch_hold + decay * m_yz;
    } else {
      m_pitch_hold = m_yz;
    }
  }

  // Re-enforce sign after any blending to guarantee final RC side correctness
  if (std::abs(sx) > epsx && (m_x * sx) <= 0.0) {
    m_x = projectSign(sx, m_x);
  }
  if (std::abs(sy) > epsyz && (m_yz * sy) <= 0.0) {
    m_yz = projectSign(sy, m_yz);
  }

  int tracking_x = static_cast<int>(m_x * 1000 + 500);
  int tracking_yz = static_cast<int>(m_yz * 1000 + 500);

  switch (
      de::fcb::CFCBMain::getInstance().getAndruavVehicleInfo().vehicle_type) {
  case ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_TRI:
  case ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_QUAD:
  if (is_forward_camera) {
      // x & y .... forward camera.
      trackingDroneForward(tracking_x, tracking_yz);
#ifdef DEBUG
  std::cout << "is_forward_camera: true" << std::endl;
#endif
    } else {
      // x & z .... vertical camera.
      m_tracking_type = TRACKING_STANDING;
      trackingStanding(tracking_x, tracking_yz);
    }
    break;

  case ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_BOAT:
    break;

  case ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_ROVER:
    break;

  case ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_PLANE:
    m_tracking_type = TRACKING_TARGET;
    if (is_forward_camera) {

      switch (m_tracking_type) {
      case TRACKING_TYPE::TRACKING_FOLLOW_ME:
        trackingFollowMe(tracking_x, tracking_yz);
        break;
      case TRACKING_TYPE::TRACKING_TARGET:
        trackingTarget(tracking_x, tracking_yz);
        break;
      }

    } else {
      // Q-Plan ONLY
      //  x & z .... vertical camera.
      //  TO BE HANDLED
      //  rc_channels[RC_CHANNEL_TRACKING_ROLL] = 1000 - tracking_x; // to be aligned
      //  with default settings of Ardu rc_channels[RC_CHANNEL_TRACKING_PITCH] = 1000 -
      //  tracking_yz; rc_channels[RC_CHANNEL_TRACKING_YAW] = 500;
      //  rc_channels[RC_CHANNEL_TRACKING_THROTTLE] = 500;
    }
    break;

  default:
    // x & y .... forward camera.
      // TODO: TO BE IMPLEMENTED
#ifdef DEBUG
  std::cout << "undefined tracking algorithm" << std::endl;
#endif
      return ;
    break;
  }

#ifdef DEBUG
  std::cout << "tracking_x:" << std::to_string(tracking_x)
            << " tracking_yz:" << std::to_string(tracking_yz) << std::endl;
#endif
}

void CTrackingManager::trackingFollowMe(const double tracking_x,
                                        const double tracking_yz) {
  
#ifdef DEBUG
  std::cout << "trackingFollowMe" << std::endl;
#endif
                                          // x & y .... forward camera.
  const RCMAP_CHANNELS_MAP_INFO_STRUCT rc_map =
      m_fcbMain2.getRCChannelsMapInfo();

  // value: [0,1000] IMPORTANT: SKIP_RC_CHANNEL (-999) means channel release
  // 'R': Rudder
  // 'T': Throttle
  // 'A': Aileron
  // 'E': Elevator
  int16_t rc_channels[RC_CHANNEL_TRACKING_COUNT] = {SKIP_RC_CHANNEL};
  rc_channels[RC_CHANNEL_TRACKING_ROLL] = 500;
  rc_channels[RC_CHANNEL_TRACKING_PITCH] = 1000 - tracking_yz;
  rc_channels[RC_CHANNEL_TRACKING_YAW] =
      1000 - tracking_x; // to be aligned with default settings of Ardu
  rc_channels[RC_CHANNEL_TRACKING_THROTTLE] = 500;

#ifdef DEBUG
  std::cout << _INFO_CONSOLE_BOLD_TEXT << "onTrack >> "
            << _LOG_CONSOLE_BOLD_TEXT << "  rcmap_roll:" << RC_CHANNEL_TRACKING_ROLL
            << ":" << _INFO_CONSOLE_BOLD_TEXT << rc_channels[RC_CHANNEL_TRACKING_ROLL]
            << _LOG_CONSOLE_BOLD_TEXT << "  rcmap_pitch:" << RC_CHANNEL_TRACKING_PITCH
            << ":" << _INFO_CONSOLE_BOLD_TEXT << rc_channels[RC_CHANNEL_TRACKING_PITCH]
            << _LOG_CONSOLE_BOLD_TEXT << "  rcmap_yaw:" << RC_CHANNEL_TRACKING_YAW
            << ":" << _INFO_CONSOLE_BOLD_TEXT << rc_channels[RC_CHANNEL_TRACKING_YAW]
            << _LOG_CONSOLE_BOLD_TEXT
            << "  rcmap_throttle:" << RC_CHANNEL_TRACKING_THROTTLE << ":"
            << _INFO_CONSOLE_BOLD_TEXT << rc_channels[RC_CHANNEL_TRACKING_THROTTLE]
            << _NORMAL_CONSOLE_TEXT_ << std::endl;
#endif

  m_fcbMain2.updateTrackingControlChannels(rc_channels);
}

void CTrackingManager::trackingTarget(const double tracking_x,
                                      const double tracking_yz) {
#ifdef DEBUG
  std::cout << "trackingTarget" << std::endl;
#endif
  
  // x & y .... forward camera.
  const RCMAP_CHANNELS_MAP_INFO_STRUCT rc_map =
      m_fcbMain2.getRCChannelsMapInfo();

  // value: [0,1000] IMPORTANT: SKIP_RC_CHANNEL (-999) means channel release
  // 'R': Rudder
  // 'T': Throttle
  // 'A': Aileron
  // 'E': Elevator
  int16_t rc_channels[RC_CHANNEL_TRACKING_COUNT] = {SKIP_RC_CHANNEL};

  double pitch = 1000 - tracking_yz;
  if (pitch < 500) 
  {
    // dont adapt pitch if target still higher than zero level.
    rc_channels[RC_CHANNEL_TRACKING_PITCH] = SKIP_RC_CHANNEL;
  }
  else
  {
    rc_channels[RC_CHANNEL_TRACKING_PITCH] = pitch;
  }
  
  rc_channels[RC_CHANNEL_TRACKING_YAW] = 1000 - tracking_x; // to be aligned with default settings of Ardu
  rc_channels[RC_CHANNEL_TRACKING_ROLL] = 500;
  rc_channels[RC_CHANNEL_TRACKING_THROTTLE] = 500;

#ifdef DEBUG
  std::cout << _INFO_CONSOLE_BOLD_TEXT << "onTrack >> "
            << _LOG_CONSOLE_BOLD_TEXT << "  rcmap_roll:" << RC_CHANNEL_TRACKING_ROLL
            << ":" << _INFO_CONSOLE_BOLD_TEXT << rc_channels[RC_CHANNEL_TRACKING_ROLL]
            << _LOG_CONSOLE_BOLD_TEXT << "  rcmap_pitch:" << RC_CHANNEL_TRACKING_PITCH
            << ":" << _INFO_CONSOLE_BOLD_TEXT << rc_channels[RC_CHANNEL_TRACKING_PITCH]
            << _LOG_CONSOLE_BOLD_TEXT << "  rcmap_yaw:" << RC_CHANNEL_TRACKING_YAW
            << ":" << _INFO_CONSOLE_BOLD_TEXT << rc_channels[RC_CHANNEL_TRACKING_YAW]
            << _LOG_CONSOLE_BOLD_TEXT
            << "  rcmap_throttle:" << RC_CHANNEL_TRACKING_THROTTLE << ":"
            << _INFO_CONSOLE_BOLD_TEXT << rc_channels[RC_CHANNEL_TRACKING_THROTTLE]
            << _NORMAL_CONSOLE_TEXT_ << std::endl;
#endif

  m_fcbMain2.updateTrackingControlChannels(rc_channels);
}

void CTrackingManager::trackingDroneForward(const double tracking_x,
                                        const double tracking_yz) {
#ifdef DEBUG
  std::cout << "trackingDroneForward" << std::endl;
#endif
const RCMAP_CHANNELS_MAP_INFO_STRUCT rc_map =
      m_fcbMain2.getRCChannelsMapInfo();

  // values: [-0.5,0.5]

  // value: [0,1000] IMPORTANT: SKIP_RC_CHANNEL (-999) means channel release
  // 'R': Rudder
  // 'T': Throttle
  // 'A': Aileron
  // 'E': Elevator
  int16_t rc_channels[RC_CHANNEL_TRACKING_COUNT] = {SKIP_RC_CHANNEL};

  rc_channels[RC_CHANNEL_TRACKING_ROLL] = 500; // to be aligned with default settings of Ardu
  rc_channels[RC_CHANNEL_TRACKING_PITCH] = tracking_yz;
  rc_channels[RC_CHANNEL_TRACKING_YAW] = 1000 - tracking_x;
  rc_channels[RC_CHANNEL_TRACKING_THROTTLE] = 500;

#ifdef DEBUG
  std::cout << _INFO_CONSOLE_BOLD_TEXT << "onTrack >> "
            << _LOG_CONSOLE_BOLD_TEXT << "  rcmap_roll:" << RC_CHANNEL_TRACKING_ROLL
            << ":" << _INFO_CONSOLE_BOLD_TEXT << rc_channels[RC_CHANNEL_TRACKING_ROLL]
            << _LOG_CONSOLE_BOLD_TEXT << "  rcmap_pitch:" << RC_CHANNEL_TRACKING_PITCH
            << ":" << _INFO_CONSOLE_BOLD_TEXT << rc_channels[RC_CHANNEL_TRACKING_PITCH]
            << _LOG_CONSOLE_BOLD_TEXT << "  rcmap_yaw:" << RC_CHANNEL_TRACKING_YAW
            << ":" << _INFO_CONSOLE_BOLD_TEXT << rc_channels[RC_CHANNEL_TRACKING_YAW]
            << _LOG_CONSOLE_BOLD_TEXT
            << "  rcmap_throttle:" << RC_CHANNEL_TRACKING_THROTTLE << ":"
            << _INFO_CONSOLE_BOLD_TEXT << rc_channels[RC_CHANNEL_TRACKING_THROTTLE]
            << _NORMAL_CONSOLE_TEXT_ << std::endl;
#endif

  m_fcbMain2.updateTrackingControlChannels(rc_channels);

}
void CTrackingManager::trackingStanding(const double tracking_x,
                                        const double tracking_yz) {
#ifdef DEBUG
  std::cout << "trackingStanding" << std::endl;
#endif

  const RCMAP_CHANNELS_MAP_INFO_STRUCT rc_map =
      m_fcbMain2.getRCChannelsMapInfo();

  // values: [-0.5,0.5]

  // value: [0,1000] IMPORTANT: SKIP_RC_CHANNEL (-999) means channel release
  // 'R': Rudder
  // 'T': Throttle
  // 'A': Aileron
  // 'E': Elevator
  int16_t rc_channels[RC_CHANNEL_TRACKING_COUNT] = {SKIP_RC_CHANNEL};

  rc_channels[RC_CHANNEL_TRACKING_ROLL] = 1000 - tracking_x; // to be aligned with default settings of Ardu
  rc_channels[RC_CHANNEL_TRACKING_PITCH] = 1000 - tracking_yz;
  rc_channels[RC_CHANNEL_TRACKING_YAW] = 500;
  rc_channels[RC_CHANNEL_TRACKING_THROTTLE] = 500;

#ifdef DEBUG
  std::cout << _INFO_CONSOLE_BOLD_TEXT << "onTrack >> "
            << _LOG_CONSOLE_BOLD_TEXT << "  rcmap_roll:" << RC_CHANNEL_TRACKING_ROLL
            << ":" << _INFO_CONSOLE_BOLD_TEXT << rc_channels[RC_CHANNEL_TRACKING_ROLL]
            << _LOG_CONSOLE_BOLD_TEXT << "  rcmap_pitch:" << RC_CHANNEL_TRACKING_PITCH
            << ":" << _INFO_CONSOLE_BOLD_TEXT << rc_channels[RC_CHANNEL_TRACKING_PITCH]
            << _LOG_CONSOLE_BOLD_TEXT << "  rcmap_yaw:" << RC_CHANNEL_TRACKING_YAW
            << ":" << _INFO_CONSOLE_BOLD_TEXT << rc_channels[RC_CHANNEL_TRACKING_YAW]
            << _LOG_CONSOLE_BOLD_TEXT
            << "  rcmap_throttle:" << RC_CHANNEL_TRACKING_THROTTLE << ":"
            << _INFO_CONSOLE_BOLD_TEXT << rc_channels[RC_CHANNEL_TRACKING_THROTTLE]
            << _NORMAL_CONSOLE_TEXT_ << std::endl;
#endif

  m_fcbMain2.updateTrackingControlChannels(rc_channels);
}
