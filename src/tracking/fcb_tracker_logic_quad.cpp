#include "../de_common/helpers/colors.hpp"
#include "../de_common/helpers/helpers.hpp"

#include "fcb_tracker_logic_quad.hpp"
#include "fcb_tracking_manager.hpp"

#include "../de_common/de_databus/configFile.hpp"
#include "../de_common/de_databus/localConfigFile.hpp"
#include "../defines.hpp"
#include "../fcb_main.hpp"

using Json_de = nlohmann::json;
using namespace de::fcb::tracking;

static de::fcb::CFCBMain &m_fcbMain2 = de::fcb::CFCBMain::getInstance();

void CTrackerQuadLogic::readConfigParameters() {
  de::CConfigFile &cConfigFile = de::CConfigFile::getInstance();
  const Json_de &jsonConfig = cConfigFile.GetConfigJSON();
  bool expo_x_from_follow_me = false;
  bool expo_y_from_follow_me = false;

  if (jsonConfig.contains("follow_me")) {
    const Json_de &follow_me_root = jsonConfig["follow_me"];

    if (follow_me_root.contains("quad")) {
      const Json_de &follow_me = follow_me_root["quad"];

      if (follow_me.contains("loose_altitude")) {
        m_loose_altitude = follow_me["loose_altitude"].get<bool>();
      }

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
#ifdef DEBUG
        std::cout << _SUCCESS_CONSOLE_TEXT_
                  << "Apply:  " << _LOG_CONSOLE_BOLD_TEXT
                  << "PID_I_X:" << _INFO_CONSOLE_BOLD_TEXT
                  << std::to_string(m_x_PID_I) << _NORMAL_CONSOLE_TEXT_
                  << std::endl;
#endif
      }

      if (follow_me.contains("PID_I_Y")) {
        m_yz_PID_I = follow_me["PID_I_Y"].get<double>();
#ifdef DEBUG
        std::cout << _SUCCESS_CONSOLE_TEXT_
                  << "Apply:  " << _LOG_CONSOLE_BOLD_TEXT
                  << "PID_I_Y:" << _INFO_CONSOLE_BOLD_TEXT
                  << std::to_string(m_yz_PID_I) << _NORMAL_CONSOLE_TEXT_
                  << std::endl;
#endif
      }

      if (follow_me.contains("PID_D_X")) {
        m_x_PID_D = follow_me["PID_D_X"].get<double>();
#ifdef DEBUG
        std::cout << _SUCCESS_CONSOLE_TEXT_
                  << "Apply:  " << _LOG_CONSOLE_BOLD_TEXT
                  << "PID_D_X:" << _INFO_CONSOLE_BOLD_TEXT
                  << std::to_string(m_x_PID_D) << _NORMAL_CONSOLE_TEXT_
                  << std::endl;
#endif
      }

      if (follow_me.contains("PID_D_Y")) {
        m_yz_PID_D = follow_me["PID_D_Y"].get<double>();
#ifdef DEBUG
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
#ifdef DEBUG
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
#ifdef DEBUG
        std::cout << _SUCCESS_CONSOLE_TEXT_
                  << "Apply:  " << _LOG_CONSOLE_BOLD_TEXT
                  << "expo_y:" << _INFO_CONSOLE_BOLD_TEXT
                  << std::to_string(m_expo_yz) << _NORMAL_CONSOLE_TEXT_
                  << std::endl;
#endif
      }

      if (follow_me.contains("center_hold_enabled")) {
        m_center_hold_enabled = follow_me["center_hold_enabled"].get<bool>();
#ifdef DEBUG
        std::cout << _SUCCESS_CONSOLE_TEXT_
                  << "Apply:  " << _LOG_CONSOLE_BOLD_TEXT
                  << "center_hold_enabled:" << _INFO_CONSOLE_BOLD_TEXT
                  << (m_center_hold_enabled ? "true" : "false")
                  << _NORMAL_CONSOLE_TEXT_ << std::endl;
#endif
      }
      if (follow_me.contains("center_hold_y_band")) {
        m_center_hold_y_band = follow_me["center_hold_y_band"].get<double>();
#ifdef DEBUG
        std::cout << _SUCCESS_CONSOLE_TEXT_
                  << "Apply:  " << _LOG_CONSOLE_BOLD_TEXT
                  << "center_hold_y_band:" << _INFO_CONSOLE_BOLD_TEXT
                  << std::to_string(m_center_hold_y_band)
                  << _NORMAL_CONSOLE_TEXT_ << std::endl;
#endif
      }
      if (follow_me.contains("center_hold_decay")) {
        m_center_hold_decay = follow_me["center_hold_decay"].get<double>();
#ifdef DEBUG
        std::cout << _SUCCESS_CONSOLE_TEXT_
                  << "Apply:  " << _LOG_CONSOLE_BOLD_TEXT
                  << "center_hold_decay:" << _INFO_CONSOLE_BOLD_TEXT
                  << std::to_string(m_center_hold_decay)
                  << _NORMAL_CONSOLE_TEXT_ << std::endl;
#endif
      }

      if (follow_me.contains("rate_limit")) {
        m_rate_limit = follow_me["rate_limit"].get<double>();
#ifdef DEBUG
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
#ifdef DEBUG
      std::cout << _SUCCESS_CONSOLE_TEXT_
                << "Apply:  " << _LOG_CONSOLE_BOLD_TEXT
                << "deadband_x:" << _INFO_CONSOLE_BOLD_TEXT
                << std::to_string(m_deadband_x) << _NORMAL_CONSOLE_TEXT_
                << std::endl;
      std::cout << _SUCCESS_CONSOLE_TEXT_
                << "Apply:  " << _LOG_CONSOLE_BOLD_TEXT
                << "deadband_y:" << _INFO_CONSOLE_BOLD_TEXT
                << std::to_string(m_deadband_yz) << _NORMAL_CONSOLE_TEXT_
                << std::endl;
#endif
      // Kalman filter parameters
      if (follow_me.contains("kalman_enabled")) {
        m_kalman_enabled = follow_me["kalman_enabled"].get<bool>();
#ifdef DEBUG
        std::cout << _SUCCESS_CONSOLE_TEXT_
                  << "Apply:  " << _LOG_CONSOLE_BOLD_TEXT
                  << "kalman_enabled:" << _INFO_CONSOLE_BOLD_TEXT
                  << (m_kalman_enabled ? "true" : "false")
                  << _NORMAL_CONSOLE_TEXT_ << std::endl;
#endif
      }
      if (follow_me.contains("kalman_process_noise_q")) {
        m_kalman_process_noise_q =
            follow_me["kalman_process_noise_q"].get<double>();
#ifdef DEBUG
        std::cout << _SUCCESS_CONSOLE_TEXT_
                  << "Apply:  " << _LOG_CONSOLE_BOLD_TEXT
                  << "kalman_process_noise_q:" << _INFO_CONSOLE_BOLD_TEXT
                  << std::to_string(m_kalman_process_noise_q)
                  << _NORMAL_CONSOLE_TEXT_ << std::endl;
#endif
      }
      if (follow_me.contains("kalman_measurement_noise_r")) {
        m_kalman_measurement_noise_r =
            follow_me["kalman_measurement_noise_r"].get<double>();
#ifdef DEBUG
        std::cout << _SUCCESS_CONSOLE_TEXT_
                  << "Apply:  " << _LOG_CONSOLE_BOLD_TEXT
                  << "kalman_measurement_noise_r:" << _INFO_CONSOLE_BOLD_TEXT
                  << std::to_string(m_kalman_measurement_noise_r)
                  << _NORMAL_CONSOLE_TEXT_ << std::endl;
#endif
      }
    }
  }
}

void CTrackerQuadLogic::onTrack(const double x, const double yz,
                                const bool is_forward_camera) {
  CTrackerLogic::onTrack(x, yz, is_forward_camera);

  int tracking_x = static_cast<int>(m_x * 1000 + 500);
  int tracking_yz = static_cast<int>(m_yz * 1000 + 500);

  if (is_forward_camera) {
    // x & y .... forward camera.
    trackingDroneForward(x, yz, tracking_x, tracking_yz);
#ifdef DEBUG
    std::cout << "is_forward_camera: true" << std::endl;
#endif
  } else {
    // x & z .... vertical camera.
    m_tracking_type = TRACKING_STANDING;
    trackingStanding(x, yz, tracking_x, tracking_yz);
  }

#ifdef DEBUG
  std::cout << "tracking_x:" << std::to_string(tracking_x)
            << " tracking_yz:" << std::to_string(tracking_yz) << std::endl;
#endif
}

/**
 * Implementing logic for tracking the drone with camera
 * looking forward.
 */
void CTrackerQuadLogic::trackingDroneForward(const double x, const double yz,
                                             const double tracking_x,
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

  rc_channels[RC_CHANNEL_TRACKING_ROLL] =
      500; // to be aligned with default settings of Ardu

  rc_channels[RC_CHANNEL_TRACKING_YAW] = 1000 - tracking_x;

  if (m_loose_altitude) {
    if (yz > 0) {
      rc_channels[RC_CHANNEL_TRACKING_PITCH] = tracking_yz;
    } else {
      rc_channels[RC_CHANNEL_TRACKING_PITCH] = 500;
    }

    if (yz > 0.3) {
      rc_channels[RC_CHANNEL_TRACKING_THROTTLE] = 0; // flying up
    } else {
      if (abs(yz) < 0.1) {
        rc_channels[RC_CHANNEL_TRACKING_THROTTLE] = 1000; // flying down.
      } else {
        rc_channels[RC_CHANNEL_TRACKING_THROTTLE] = 500;
      }
    }
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
void CTrackerQuadLogic::trackingStanding(const double x, const double yz,
                                         const double tracking_x,
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

  rc_channels[RC_CHANNEL_TRACKING_ROLL] =
      1000 - tracking_x; // to be aligned with default settings of Ardu
  rc_channels[RC_CHANNEL_TRACKING_PITCH] = 1000 - tracking_yz;
  rc_channels[RC_CHANNEL_TRACKING_YAW] = 500;
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