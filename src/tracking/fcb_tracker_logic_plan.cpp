#include "../de_common/helpers/colors.hpp"
#include "../de_common/helpers/helpers.hpp"
#include <algorithm>

#include "fcb_tracker_logic_plan.hpp"
#include "fcb_tracking_manager.hpp"

#include "../de_common/de_databus/configFile.hpp"
#include "../de_common/de_databus/localConfigFile.hpp"
#include "../defines.hpp"

#include "../fcb_main.hpp"

using namespace de::fcb::tracking;

static de::fcb::CFCBMain &m_fcbMain2 = de::fcb::CFCBMain::getInstance();

void CTrackerPlanLogic::onStatusChanged(const int status, const uint8_t tracking_camera_direction, const bool ai_priority) {
  CTrackerLogic::onStatusChanged(status, tracking_camera_direction, ai_priority);

  switch (status) {
  case TrackingTarget_STATUS_TRACKING_LOST:
  case TrackingTarget_STATUS_TRACKING_ENABLED:
  case TrackingTarget_STATUS_TRACKING_STOPPED:
    m_forward_pid_x.reset();
    m_forward_pid_yz.reset();
    m_forward_last_x_for_rate = 0.0;
    m_forward_last_yz_for_rate = 0.0;
    m_forward_last_rate_time = 0;
    m_forward_current_rate_x = 0.0;
    m_forward_current_rate_yz = 0.0;
    break;

  default:
    break;
  }
}

void CTrackerPlanLogic::readConfigParameters() {
  de::CConfigFile &cConfigFile = de::CConfigFile::getInstance();
  const Json_de &jsonConfig = cConfigFile.GetConfigJSON();

  if (jsonConfig.contains("de_pilot")) {
    const Json_de &de_pilot_root = jsonConfig["de_pilot"];

    if (de_pilot_root.contains("tracking")) {
      const Json_de &tracking_root = de_pilot_root["tracking"];

      if (tracking_root.contains("plane")) {
        const Json_de &plane_config = tracking_root["plane"];

        if (plane_config.contains("forward")) {
          const Json_de &forward_config = plane_config["forward"];

          if (forward_config.contains("x_axis")) {
            const Json_de &x_config = forward_config["x_axis"];

            if (x_config.contains("pid_p")) {
              m_forward_pid_p_x = x_config["pid_p"].get<double>();
            }
            if (x_config.contains("pid_i")) {
              m_forward_pid_i_x = x_config["pid_i"].get<double>();
            }
            if (x_config.contains("pid_d")) {
              m_forward_pid_d_x = x_config["pid_d"].get<double>();
            }
            if (x_config.contains("ff_scale")) {
              m_forward_ff_scale_x = x_config["ff_scale"].get<double>();
            }
            if (x_config.contains("max_accel")) {
              m_forward_max_accel_x = x_config["max_accel"].get<double>();
            }
            if (x_config.contains("max_rate")) {
              m_forward_max_rate_x = x_config["max_rate"].get<double>();
            }
            if (x_config.contains("deadband")) {
              m_forward_deadband_x = x_config["deadband"].get<double>();
            }
            if (x_config.contains("integral_limit")) {
              m_forward_integral_limit_x = x_config["integral_limit"].get<double>();
            }
            if (x_config.contains("output_limit")) {
              m_forward_output_limit_x = x_config["output_limit"].get<double>();
            }
          }

          if (forward_config.contains("yz_axis")) {
            const Json_de &yz_config = forward_config["yz_axis"];

            if (yz_config.contains("pid_p")) {
              m_forward_pid_p_yz = yz_config["pid_p"].get<double>();
            }
            if (yz_config.contains("pid_i")) {
              m_forward_pid_i_yz = yz_config["pid_i"].get<double>();
            }
            if (yz_config.contains("pid_d")) {
              m_forward_pid_d_yz = yz_config["pid_d"].get<double>();
            }
            if (yz_config.contains("ff_scale")) {
              m_forward_ff_scale_yz = yz_config["ff_scale"].get<double>();
            }
            if (yz_config.contains("max_accel")) {
              m_forward_max_accel_yz = yz_config["max_accel"].get<double>();
            }
            if (yz_config.contains("max_rate")) {
              m_forward_max_rate_yz = yz_config["max_rate"].get<double>();
            }
            if (yz_config.contains("deadband")) {
              m_forward_deadband_yz = yz_config["deadband"].get<double>();
            }
            if (yz_config.contains("integral_limit")) {
              m_forward_integral_limit_yz = yz_config["integral_limit"].get<double>();
            }
            if (yz_config.contains("output_limit")) {
              m_forward_output_limit_yz = yz_config["output_limit"].get<double>();
            }
          }
        }
      }
    }
  }

  // Configure advanced PID controllers with loaded parameters
  m_forward_pid_x.setPID(m_forward_pid_p_x, m_forward_pid_i_x, m_forward_pid_d_x);
  m_forward_pid_x.setFeedforwardGain(m_forward_ff_scale_x);
  m_forward_pid_x.setIntegralLimit(m_forward_integral_limit_x);
  m_forward_pid_x.setOutputLimit(m_forward_output_limit_x);

  m_forward_pid_yz.setPID(m_forward_pid_p_yz, m_forward_pid_i_yz, m_forward_pid_d_yz);
  m_forward_pid_yz.setFeedforwardGain(m_forward_ff_scale_yz);
  m_forward_pid_yz.setIntegralLimit(m_forward_integral_limit_yz);
  m_forward_pid_yz.setOutputLimit(m_forward_output_limit_yz);
}

void CTrackerPlanLogic::onTrack(const double x, const double yz) {
  if (!isTrackingActive()) {
    return;
  }

  m_tracking_type = TRACKING_TARGET; // HARD CODED FOR NOW
  if (m_tracking_camera_direction == TRACKING_CAMERA_DIRECTION_FRONT) {

    // Update rate tracking for feedback
    const uint64_t now = get_time_usec();
    if (m_forward_last_rate_time > 0) {
      const double dt = (now - m_forward_last_rate_time) / 1000000.0; // seconds
      if (dt <= 0.05) { // If dt is too small, skip update and keep last RC values
        return;
      }

      m_forward_pid_x.setDeltaTime(dt);
      m_forward_pid_yz.setDeltaTime(dt);

      m_forward_current_rate_x = (x - m_forward_last_x_for_rate) / dt;
      m_forward_current_rate_yz = (yz - m_forward_last_yz_for_rate) / dt;
      m_forward_last_x_for_rate = x;
      m_forward_last_yz_for_rate = yz;
      m_forward_last_rate_time = now;
    } else {
      // Initialize rate tracking
      m_forward_last_x_for_rate = x;
      m_forward_last_yz_for_rate = yz;
      m_forward_last_rate_time = now;
      m_forward_current_rate_x = 0.0;
      m_forward_current_rate_yz = 0.0;

      // Set initial delta time for PID controllers (use reasonable default)
      const double initial_dt = 0.01; // 10ms default
      m_forward_pid_x.setDeltaTime(initial_dt);
      m_forward_pid_yz.setDeltaTime(initial_dt);
    }

    // Apply deadband
    const double filtered_error_x = (std::abs(x) < m_forward_deadband_x) ? 0.0 : x;
    const double filtered_error_yz = (std::abs(yz) < m_forward_deadband_yz) ? 0.0 : yz;

    // Use sqrt_controller to compute desired rates
    const double desired_rate_x = de::fcb::depilot::CAdvancedPIDController::sqrt_controller(
        filtered_error_x, m_forward_pid_p_x, m_forward_max_accel_x, m_forward_max_rate_x);
    const double desired_rate_yz = de::fcb::depilot::CAdvancedPIDController::sqrt_controller(
        filtered_error_yz, m_forward_pid_p_yz, m_forward_max_accel_yz, m_forward_max_rate_yz);

    // Calculate rate errors (desired vs actual)
    const double rate_error_x = desired_rate_x - m_forward_current_rate_x;
    const double rate_error_yz = desired_rate_yz - m_forward_current_rate_yz;

    // Use advanced PID controllers to compute control outputs, with desired
    // rate passed as feedforward for better responsiveness.
    const double control_x = m_forward_pid_x.calculate(rate_error_x, desired_rate_x);
    const double control_yz = m_forward_pid_yz.calculate(rate_error_yz, desired_rate_yz);

    // Convert control outputs (+-output_limit) into RC_CHANNEL_TRACKING space
    // [0,1000] with 500 as neutral, same convention used everywhere else.
    const int tracking_x = static_cast<int>(std::clamp(500.0 + control_x, 0.0, 1000.0));
    const int tracking_yz = static_cast<int>(std::clamp(500.0 + control_yz, 0.0, 1000.0));

    switch (m_tracking_type) {
    case TRACKING_TYPE::TRACKING_FOLLOW_ME:
      trackingFollowMe(tracking_x, tracking_yz);
      break;
    case TRACKING_TYPE::TRACKING_TARGET:
      trackingTarget(tracking_x, tracking_yz);
      break;
    default:
      break;
    }

  } else {
    // Q-Plan ONLY
    //  x & z .... vertical camera.
    //  TO BE HANDLED
    //  rc_channels[RC_CHANNEL_TRACKING_ROLL] = 1000 - tracking_x; // to be
    //  aligned with default settings of Ardu
    //  rc_channels[RC_CHANNEL_TRACKING_PITCH] = 1000 - tracking_yz;
    //  rc_channels[RC_CHANNEL_TRACKING_YAW] = 500;
    //  rc_channels[RC_CHANNEL_TRACKING_THROTTLE] = 500;
  }
}

void CTrackerPlanLogic::trackingFollowMe(const double x,
                                         const double yz) {

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
  int16_t rc_channels[RC_CHANNEL_TRACKING_COUNT];
  std::fill_n(rc_channels, RC_CHANNEL_TRACKING_COUNT, SKIP_RC_CHANNEL);
  rc_channels[RC_CHANNEL_TRACKING_ROLL] = 500;
  rc_channels[RC_CHANNEL_TRACKING_PITCH] = 1000 - yz;
  rc_channels[RC_CHANNEL_TRACKING_YAW] =
      1000 - x; // to be aligned with default settings of Ardu
  rc_channels[RC_CHANNEL_TRACKING_THROTTLE] = 500;

#ifdef DEBUG
  std::cout << _INFO_CONSOLE_BOLD_TEXT << "onTrack >> "
            << _LOG_CONSOLE_BOLD_TEXT
            << "  rcmap_roll:" << RC_CHANNEL_TRACKING_ROLL << ":"
            << _INFO_CONSOLE_BOLD_TEXT << rc_channels[RC_CHANNEL_TRACKING_ROLL]
            << _LOG_CONSOLE_BOLD_TEXT
            << "  rcmap_pitch:" << RC_CHANNEL_TRACKING_PITCH << ":"
            << _INFO_CONSOLE_BOLD_TEXT << rc_channels[RC_CHANNEL_TRACKING_PITCH]
            << _LOG_CONSOLE_BOLD_TEXT
            << "  rcmap_yaw:" << RC_CHANNEL_TRACKING_YAW << ":"
            << _INFO_CONSOLE_BOLD_TEXT << rc_channels[RC_CHANNEL_TRACKING_YAW]
            << _LOG_CONSOLE_BOLD_TEXT
            << "  rcmap_throttle:" << RC_CHANNEL_TRACKING_THROTTLE << ":"
            << _INFO_CONSOLE_BOLD_TEXT
            << rc_channels[RC_CHANNEL_TRACKING_THROTTLE]
            << _NORMAL_CONSOLE_TEXT_ << std::endl;
#endif

  m_fcbMain2.updateTrackingControlChannels(rc_channels);
}

void CTrackerPlanLogic::trackingTarget(const double x,
                                       const double yz) {
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
  int16_t rc_channels[RC_CHANNEL_TRACKING_COUNT];
  std::fill_n(rc_channels, RC_CHANNEL_TRACKING_COUNT, SKIP_RC_CHANNEL);

  double pitch = 1000 - yz;
  if (pitch < 500) {
    // dont adapt pitch if target still higher than zero level.
    rc_channels[RC_CHANNEL_TRACKING_PITCH] = SKIP_RC_CHANNEL;
  } else {
    rc_channels[RC_CHANNEL_TRACKING_PITCH] = pitch;
  }

  rc_channels[RC_CHANNEL_TRACKING_YAW] =
      1000 - x; // to be aligned with default settings of Ardu
  rc_channels[RC_CHANNEL_TRACKING_ROLL] = 500;
  rc_channels[RC_CHANNEL_TRACKING_THROTTLE] = 500;

#ifdef DEBUG
  std::cout << _INFO_CONSOLE_BOLD_TEXT << "onTrack >> "
            << _LOG_CONSOLE_BOLD_TEXT
            << "  rcmap_roll:" << RC_CHANNEL_TRACKING_ROLL << ":"
            << _INFO_CONSOLE_BOLD_TEXT << rc_channels[RC_CHANNEL_TRACKING_ROLL]
            << _LOG_CONSOLE_BOLD_TEXT
            << "  rcmap_pitch:" << RC_CHANNEL_TRACKING_PITCH << ":"
            << _INFO_CONSOLE_BOLD_TEXT << rc_channels[RC_CHANNEL_TRACKING_PITCH]
            << _LOG_CONSOLE_BOLD_TEXT
            << "  rcmap_yaw:" << RC_CHANNEL_TRACKING_YAW << ":"
            << _INFO_CONSOLE_BOLD_TEXT << rc_channels[RC_CHANNEL_TRACKING_YAW]
            << _LOG_CONSOLE_BOLD_TEXT
            << "  rcmap_throttle:" << RC_CHANNEL_TRACKING_THROTTLE << ":"
            << _INFO_CONSOLE_BOLD_TEXT
            << rc_channels[RC_CHANNEL_TRACKING_THROTTLE]
            << _NORMAL_CONSOLE_TEXT_ << std::endl;
#endif

  m_fcbMain2.updateTrackingControlChannels(rc_channels);
}
