#include "../de_common/helpers/colors.hpp"
#include "../de_common/helpers/helpers.hpp"
#include <algorithm>

#include "fcb_tracker_logic_quad.hpp"
#include "fcb_tracking_manager.hpp"

#include "../de_common/de_databus/configFile.hpp"
#include "../de_common/de_databus/localConfigFile.hpp"
#include "../defines.hpp"
#include "../fcb_main.hpp"

using Json_de = nlohmann::json;
using namespace de::fcb::tracking;

#define TRACK_COPTER_STATUS_NONE 0b00000000
#define TRACK_COPTER_STATUS_VERTICAL_CENTER_REACHED 0b00000001
#define TRACK_COPTER_STATUS_HORIZONTAL_CENTER_REACHED 0b00000010

static de::fcb::CFCBMain &m_fcbMain2 = de::fcb::CFCBMain::getInstance();

void CTrackerQuadLogic::onStatusChanged(const int status, const uint8_t tracking_camera_direction, const bool ai_priority) {
  CTrackerLogic::onStatusChanged(status, tracking_camera_direction, ai_priority);

  switch (status) {
  case TrackingTarget_STATUS_TRACKING_LOST:
    resetTrackerCopterStatus();
    break;

  case TrackingTarget_STATUS_TRACKING_DETECTED:
    break;

  case TrackingTarget_STATUS_TRACKING_ENABLED:
    resetTrackerCopterStatus();
    break;

  case TrackingTarget_STATUS_TRACKING_STOPPED:
    resetTrackerCopterStatus();
    break;

  default:
    break;
  }
}
void CTrackerQuadLogic::readConfigParameters() {
  de::CConfigFile &cConfigFile = de::CConfigFile::getInstance();
  const Json_de &jsonConfig = cConfigFile.GetConfigJSON();

  if (jsonConfig.contains("de_pilot")) {
    const Json_de &de_pilot_root = jsonConfig["de_pilot"];

    if (de_pilot_root.contains("tracking")) {
      const Json_de &tracking_root = de_pilot_root["tracking"];

      if (tracking_root.contains("quad")) {
        const Json_de &quad_config = tracking_root["quad"];

        if (quad_config.contains("loose_altitude")) {
          m_loose_altitude = quad_config["loose_altitude"].get<bool>();
        }

        // Forward-camera tracking configuration
        if (quad_config.contains("forward")) {
          const Json_de &forward_config = quad_config["forward"];

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

        // Standing (down/up-looking camera) tracking configuration
        if (quad_config.contains("standing")) {
          const Json_de &standing_config = quad_config["standing"];

          // X-axis configuration
          if (standing_config.contains("x_axis")) {
            const Json_de &x_config = standing_config["x_axis"];

            if (x_config.contains("pid_p")) {
              m_standing_pid_p_x = x_config["pid_p"].get<double>();
            }
            if (x_config.contains("pid_i")) {
              m_standing_pid_i_x = x_config["pid_i"].get<double>();
            }
            if (x_config.contains("pid_d")) {
              m_standing_pid_d_x = x_config["pid_d"].get<double>();
            }
            if (x_config.contains("ff_scale")) {
              m_standing_ff_scale_x = x_config["ff_scale"].get<double>();
            }
            if (x_config.contains("max_accel")) {
              m_standing_max_accel_x = x_config["max_accel"].get<double>();
            }
            if (x_config.contains("max_rate")) {
              m_standing_max_rate_x = x_config["max_rate"].get<double>();
            }
            if (x_config.contains("deadband")) {
              m_standing_deadband_x = x_config["deadband"].get<double>();
            }
            if (x_config.contains("integral_limit")) {
              m_standing_integral_limit_x = x_config["integral_limit"].get<double>();
            }
            if (x_config.contains("output_limit")) {
              m_standing_output_limit_x = x_config["output_limit"].get<double>();
            }
          }

          // YZ-axis configuration
          if (standing_config.contains("yz_axis")) {
            const Json_de &yz_config = standing_config["yz_axis"];

            if (yz_config.contains("pid_p")) {
              m_standing_pid_p_yz = yz_config["pid_p"].get<double>();
            }
            if (yz_config.contains("pid_i")) {
              m_standing_pid_i_yz = yz_config["pid_i"].get<double>();
            }
            if (yz_config.contains("pid_d")) {
              m_standing_pid_d_yz = yz_config["pid_d"].get<double>();
            }
            if (yz_config.contains("ff_scale")) {
              m_standing_ff_scale_yz = yz_config["ff_scale"].get<double>();
            }
            if (yz_config.contains("max_accel")) {
              m_standing_max_accel_yz = yz_config["max_accel"].get<double>();
            }
            if (yz_config.contains("max_rate")) {
              m_standing_max_rate_yz = yz_config["max_rate"].get<double>();
            }
            if (yz_config.contains("deadband")) {
              m_standing_deadband_yz = yz_config["deadband"].get<double>();
            }
            if (yz_config.contains("integral_limit")) {
              m_standing_integral_limit_yz = yz_config["integral_limit"].get<double>();
            }
            if (yz_config.contains("output_limit")) {
              m_standing_output_limit_yz = yz_config["output_limit"].get<double>();
            }
          }

          // Read throttle control parameters
          if (standing_config.contains("throttle")) {
            const Json_de &throttle_config = standing_config["throttle"];

            if (throttle_config.contains("throttle_max")) {
              m_standing_throttle_max = throttle_config["throttle_max"].get<double>();
            }
            if (throttle_config.contains("throttle_min")) {
              m_standing_throttle_min = throttle_config["throttle_min"].get<double>();
            }
            if (throttle_config.contains("throttle_boundary_threshold")) {
              m_standing_throttle_boundary_threshold = throttle_config["throttle_boundary_threshold"].get<double>();
            }
            if (throttle_config.contains("throttle_boundary_reduction")) {
              m_standing_throttle_boundary_reduction = throttle_config["throttle_boundary_reduction"].get<double>();
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

  m_standing_pid_x.setPID(m_standing_pid_p_x, m_standing_pid_i_x, m_standing_pid_d_x);
  m_standing_pid_x.setFeedforwardGain(m_standing_ff_scale_x);
  // Delta time will be set dynamically during tracking based on actual update rate
  m_standing_pid_x.setIntegralLimit(m_standing_integral_limit_x);
  m_standing_pid_x.setOutputLimit(m_standing_output_limit_x);

  m_standing_pid_yz.setPID(m_standing_pid_p_yz, m_standing_pid_i_yz, m_standing_pid_d_yz);
  m_standing_pid_yz.setFeedforwardGain(m_standing_ff_scale_yz);
  // Delta time will be set dynamically during tracking based on actual update rate
  m_standing_pid_yz.setIntegralLimit(m_standing_integral_limit_yz);
  m_standing_pid_yz.setOutputLimit(m_standing_output_limit_yz);
}

void CTrackerQuadLogic::onTrack(const double x, const double yz) {
  if (!isTrackingActive()) {
    return;
  }

  switch(m_tracking_camera_direction)
  {
    case TRACKING_CAMERA_DIRECTION_FRONT:
    {
      // x & y .... forward camera.
      trackingDroneForward(x, yz);
    }
    break;

    case TRACKING_CAMERA_DIRECTION_UP:
    {
      m_tracking_type = TRACKING_STANDING;
      trackingStanding(x, yz);
    }
    break;

    case TRACKING_CAMERA_DIRECTION_DOWN:
    {
    }
    break;
  }
}

/**
 * Implementing logic for tracking the drone with camera
 * looking forward.
 */
void CTrackerQuadLogic::trackingDroneForward(const double x, const double yz) {
#ifdef DEBUG
  std::cout << "trackingDroneForward (Advanced PID)" << std::endl;
#endif
  const RCMAP_CHANNELS_MAP_INFO_STRUCT rc_map =
      m_fcbMain2.getRCChannelsMapInfo();

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

  // Calculate position errors (target is center: 0,0)
  const double error_x = x;
  const double error_yz = yz;

  // Apply deadband
  const double filtered_error_x = (std::abs(error_x) < m_forward_deadband_x) ? 0.0 : error_x;
  const double filtered_error_yz = (std::abs(error_yz) < m_forward_deadband_yz) ? 0.0 : error_yz;

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

  // values: [-0.5,0.5]

  // value: [0,1000] IMPORTANT: SKIP_RC_CHANNEL (-999) means channel release
  // 'R': Rudder
  // 'T': Throttle
  // 'A': Aileron
  // 'E': Elevator
  int16_t rc_channels[RC_CHANNEL_TRACKING_COUNT];
  std::fill_n(rc_channels, RC_CHANNEL_TRACKING_COUNT, SKIP_RC_CHANNEL);

  rc_channels[RC_CHANNEL_TRACKING_ROLL] =
      500; // to be aligned with default settings of Ardu

  rc_channels[RC_CHANNEL_TRACKING_YAW] = 1000 - tracking_x;

  int throttle = 500;

  if (m_loose_altitude) {
    //TODO: TO BE COMPLETED ... PARTIALLY DEVELOPED
    // Pitch: only when target is in front (yz > 0), otherwise hold
    if (isTrackerCopterStatus(TRACK_COPTER_STATUS_NONE)) {
      if (yz > 0) {
        rc_channels[RC_CHANNEL_TRACKING_PITCH] = tracking_yz; // tilt forward
      } else {
        addTrackerCopterStatus(TRACK_COPTER_STATUS_HORIZONTAL_CENTER_REACHED);
        rc_channels[RC_CHANNEL_TRACKING_PITCH] = 500; // flat
      }
    } else if (isTrackerCopterStatus(
                   TRACK_COPTER_STATUS_HORIZONTAL_CENTER_REACHED)) {
      // Start descending when target reaches middle of bottom section (yz >= 0)
      const double dead_yz = (std::abs(yz) < m_forward_deadband_yz) ? 0.0 : yz;
      if (dead_yz >= 0) {
        // Map yz in [0, 0.5] -> throttle in [500, 1000]
        // gain < 1 to keep descent balanced with pitch
        const double gain = 0.7;                         // tune this
        double norm = (dead_yz / 0.5) * gain;            // 0..gain
        throttle = 500 + static_cast<int>(norm * 500.0); // 500 .. 500+gain*500
      }

      // If target clearly above center, keep throttle neutral and let pitch
      // pull quad forward
      if (yz < -dead_yz) {
        throttle = 500;
      }
    }

    rc_channels[RC_CHANNEL_TRACKING_THROTTLE] = std::clamp(throttle, 500, 1000);

  } else {
    rc_channels[RC_CHANNEL_TRACKING_PITCH] = tracking_yz;

    if (yz > 0.3) {
      rc_channels[RC_CHANNEL_TRACKING_THROTTLE] = 0; // flying up
    } else {
      rc_channels[RC_CHANNEL_TRACKING_THROTTLE] = 500;
    }
  }

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

/**
  x,y: original data (-0.5,0.5)
*/
void CTrackerQuadLogic::trackingStanding(const double x, const double yz) {
#ifdef DEBUG
  std::cout << "trackingStanding (Advanced PID)" << std::endl;
#endif

  // Update rate tracking for feedback
  const uint64_t now = get_time_usec();
  if (m_last_rate_time > 0) {
    const double dt = (now - m_last_rate_time) / 1000000.0; // seconds
    if (dt <= 0.05) {  // If dt is too small, skip update and keep last RC values
      return;
    }
    
    // Set dynamic delta time for PID controllers based on actual update rate
    m_standing_pid_x.setDeltaTime(dt);
    m_standing_pid_yz.setDeltaTime(dt);
    
    m_current_rate_x = (x - m_last_x_for_rate) / dt;
    m_current_rate_yz = (yz - m_last_yz_for_rate) / dt;
    m_last_x_for_rate = x;
    m_last_yz_for_rate = yz;
    m_last_rate_time = now;
  } else {
    // Initialize rate tracking
    m_last_x_for_rate = x;
    m_last_yz_for_rate = yz;
    m_last_rate_time = now;
    m_current_rate_x = 0.0;
    m_current_rate_yz = 0.0;
    
    // Set initial delta time for PID controllers (use reasonable default)
    const double initial_dt = 0.01; // 10ms default
    m_standing_pid_x.setDeltaTime(initial_dt);
    m_standing_pid_yz.setDeltaTime(initial_dt);
  }

  // Calculate position errors (target is center: 0,0)
  const double error_x = x; // Negative because drone needs to move opposite to detected position
  const double error_yz = yz;

  // Apply deadband
  const double filtered_error_x = (std::abs(error_x) < m_standing_deadband_x) ? 0.0 : error_x;
  const double filtered_error_yz = (std::abs(error_yz) < m_standing_deadband_yz) ? 0.0 : error_yz;

  // Use sqrt_controller to compute desired rates
  const double desired_rate_x = de::fcb::depilot::CAdvancedPIDController::sqrt_controller(
      filtered_error_x, m_standing_pid_p_x, m_standing_max_accel_x, m_standing_max_rate_x);
  const double desired_rate_yz = de::fcb::depilot::CAdvancedPIDController::sqrt_controller(
      filtered_error_yz, m_standing_pid_p_yz, m_standing_max_accel_yz, m_standing_max_rate_yz);

  // Calculate rate errors (desired vs actual)
  const double rate_error_x = desired_rate_x - m_current_rate_x;
  const double rate_error_yz = desired_rate_yz - m_current_rate_yz;

  // Use advanced PID controllers to compute control outputs
  // Pass desired rate as feedforward for better responsiveness
  const double control_x = m_standing_pid_x.calculate(rate_error_x, desired_rate_x);
  const double control_yz = m_standing_pid_yz.calculate(rate_error_yz, desired_rate_yz);

  // Convert control outputs (+-output_limit) into RC_CHANNEL_TRACKING space
  // [0,1000] with 500 as neutral, same convention used everywhere else.
  const int tracking_x = static_cast<int>(std::clamp(500.0 + control_x, 0.0, 1000.0));
  const int tracking_yz = static_cast<int>(std::clamp(500.0 + control_yz, 0.0, 1000.0));

  // Calculate throttle based on target position
  // Use maximum absolute position to determine if target is near boundaries
  const double max_position = std::max(std::abs(x), std::abs(yz));

  double throttle_scale;
  if (max_position > m_standing_throttle_boundary_threshold) {
    // Target near edge - reduce throttle to allow centering
    throttle_scale = m_standing_throttle_boundary_reduction;
  } else {
    // Target centered - full throttle to approach
    throttle_scale = 1.0;
  }

  // Calculate throttle value (RC_CHANNEL_TRACKING space [0,1000]) and clamp
  // to configured range.
  double throttle = m_standing_throttle_min +
    (m_standing_throttle_max - m_standing_throttle_min) * throttle_scale;
  throttle = std::clamp(throttle, m_standing_throttle_min, m_standing_throttle_max);
  const int tracking_throttle = static_cast<int>(std::clamp(throttle, 0.0, 1000.0));

  // value: [0,1000] IMPORTANT: SKIP_RC_CHANNEL (-999) means channel release
  // 'R': Rudder, 'T': Throttle, 'A': Aileron, 'E': Elevator
  int16_t rc_channels[RC_CHANNEL_TRACKING_COUNT];
  std::fill_n(rc_channels, RC_CHANNEL_TRACKING_COUNT, SKIP_RC_CHANNEL);
  rc_channels[RC_CHANNEL_TRACKING_ROLL] = tracking_x;
  rc_channels[RC_CHANNEL_TRACKING_PITCH] = tracking_yz;
  rc_channels[RC_CHANNEL_TRACKING_YAW] = 500;
  rc_channels[RC_CHANNEL_TRACKING_THROTTLE] = tracking_throttle;

#ifdef DEBUG
  std::cout << _INFO_CONSOLE_BOLD_TEXT << "trackingStanding (Advanced PID) >> "  << std::endl
            << _LOG_CONSOLE_BOLD_TEXT
            << " error_x:" << _INFO_CONSOLE_BOLD_TEXT << error_x
            << _LOG_CONSOLE_BOLD_TEXT
            << " error_yz:" << _INFO_CONSOLE_BOLD_TEXT << error_yz << std::endl
            << _LOG_CONSOLE_BOLD_TEXT
            << "m_current_rate_x:" << _INFO_CONSOLE_BOLD_TEXT << m_current_rate_x 
            << _LOG_CONSOLE_BOLD_TEXT
            << " desired_rate_x:" << _INFO_CONSOLE_BOLD_TEXT << desired_rate_x << std::endl
            << _LOG_CONSOLE_BOLD_TEXT
            << " desired_rate_yz:" << _INFO_CONSOLE_BOLD_TEXT << desired_rate_yz << std::endl
            << _LOG_CONSOLE_BOLD_TEXT
            << " control_x:" << _INFO_CONSOLE_BOLD_TEXT << control_x
            << _LOG_CONSOLE_BOLD_TEXT
            << " control_yz:" << _INFO_CONSOLE_BOLD_TEXT << control_yz
            << _LOG_CONSOLE_BOLD_TEXT
            << " max_position:" << _INFO_CONSOLE_BOLD_TEXT << max_position  << std::endl
            << _LOG_CONSOLE_BOLD_TEXT
            << " throttle_scale:" << _INFO_CONSOLE_BOLD_TEXT << throttle_scale
            << _LOG_CONSOLE_BOLD_TEXT
            << " tracking_roll:" << _INFO_CONSOLE_BOLD_TEXT << tracking_x
            << _LOG_CONSOLE_BOLD_TEXT
            << " tracking_pitch:" << _INFO_CONSOLE_BOLD_TEXT << tracking_yz
            << _LOG_CONSOLE_BOLD_TEXT
            << " tracking_throttle:" << _INFO_CONSOLE_BOLD_TEXT << tracking_throttle
            << _NORMAL_CONSOLE_TEXT_ << std::endl;
#endif

  m_fcbMain2.updateTrackingControlChannels(rc_channels);
}