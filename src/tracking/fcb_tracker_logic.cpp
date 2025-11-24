#include "fcb_tracker_logic.hpp"
#include "../de_common/helpers/colors.hpp"
#include "../fcb_main.hpp"

using namespace de::fcb::tracking;

CTrackerLogic::CTrackerLogic() {}

CTrackerLogic::~CTrackerLogic() {}

void CTrackerLogic::init() { 
  readConfigParameters();

  m_PID_X.setPID(m_x_PID_P, m_x_PID_I, m_x_PID_D);
  m_PID_YZ.setPID(m_yz_PID_P, m_yz_PID_I, m_yz_PID_D);

  // Initialize Kalman filters with configured parameters
  m_kalman_x.setParameters(m_kalman_process_noise_q,
                           m_kalman_measurement_noise_r);
  m_kalman_yz.setParameters(m_kalman_process_noise_q,
                            m_kalman_measurement_noise_r); }

void CTrackerLogic::reloadParametersIfConfigChanged() {
  readConfigParameters();
  m_PID_X.setPID(m_x_PID_P, m_x_PID_I, m_x_PID_D);
  m_PID_YZ.setPID(m_yz_PID_P, m_yz_PID_I, m_yz_PID_D);
  m_kalman_x.setParameters(m_kalman_process_noise_q,
                           m_kalman_measurement_noise_r);
  m_kalman_yz.setParameters(m_kalman_process_noise_q,
                            m_kalman_measurement_noise_r);
  m_kalman_x.reset();
  m_kalman_yz.reset();
}

void CTrackerLogic::onStatusChanged(const int status) {
#ifdef DEBUG
  std::cout << _INFO_CONSOLE_BOLD_TEXT
            << "onTrackStatusChanged:" << _LOG_CONSOLE_BOLD_TEXT
            << std::to_string(status) << _NORMAL_CONSOLE_TEXT_ << std::endl;
#endif

  switch (status) {
  case TrackingTarget_STATUS_TRACKING_LOST:
    m_prev_initialized = false;              // reset shaping state
    m_PID_X.setPID(m_x_PID_P, m_x_PID_I, 0); // reset integrator
    m_PID_YZ.setPID(m_yz_PID_P, m_yz_PID_I, 0);
    // Reset Kalman filters for clean state on next acquisition
    m_kalman_x.reset();
    m_kalman_yz.reset();
    break;

  case TrackingTarget_STATUS_TRACKING_DETECTED:
    // de::fcb::CFCBMain::getInstance().adjustRemoteJoystickByMode(
    //     RC_SUB_ACTION::RC_SUB_ACTION_JOYSTICK_CHANNELS_GUIDED);
    break;

  case TrackingTarget_STATUS_TRACKING_ENABLED:
    m_PID_X.setPID(m_x_PID_P, m_x_PID_I, 0);
    m_PID_YZ.setPID(m_yz_PID_P, m_yz_PID_I, 0);
    m_kalman_x.reset();
    m_kalman_yz.reset();
    break;

  case TrackingTarget_STATUS_TRACKING_STOPPED:
    m_PID_X.setPID(m_x_PID_P, m_x_PID_I, 0);
    m_PID_YZ.setPID(m_yz_PID_P, m_yz_PID_I, 0);
    m_kalman_x.reset();
    m_kalman_yz.reset();
    m_prev_initialized = false; // reset shaping state
    break;

  default:
    break;
  }
}

void CTrackerLogic::onTrack(const double x, const double yz,
                            const bool is_forward_camera) {
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
            << "  is_xy:" << _INFO_CONSOLE_BOLD_TEXT << is_forward_camera
            << _NORMAL_CONSOLE_TEXT_ << std::endl;
#endif

  const RCMAP_CHANNELS_MAP_INFO_STRUCT rc_map =
      de::fcb::CFCBMain::getInstance().getRCChannelsMapInfo();

  // std::cout << _INFO_CONSOLE_BOLD_TEXT
  //           << "rc_map.use_smart_rc:" << rc_map.use_smart_rc
  //           << "     rc_map.is_valid:" << rc_map.is_valid
  //           << _NORMAL_CONSOLE_TEXT_ << std::endl;
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
}