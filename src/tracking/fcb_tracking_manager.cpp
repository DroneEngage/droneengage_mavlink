#include "../de_common/helpers/helpers.hpp"
#include "../de_common/helpers/colors.hpp"

#include "fcb_tracking_manager.hpp"

#include "../de_common/de_databus/localConfigFile.hpp"

#include "../fcb_main.hpp"


using namespace de::fcb::tracking;


de::fcb::CFCBMain&  m_fcbMain2 = de::fcb::CFCBMain::getInstance();


void CTrackingManager::init()
{
    de::CLocalConfigFile& cLocalConfigFile = de::CLocalConfigFile::getInstance();
    const Json_de& jsonLocalConfig = cLocalConfigFile.GetConfigJSON();
    
    if (jsonLocalConfig.contains("follow_me_PID_P_X"))
    {
        m_x_PID_P = jsonLocalConfig["follow_me_PID_P_X"].get<double>();
        std::cout << _SUCCESS_CONSOLE_TEXT_ << "Apply:  " << _LOG_CONSOLE_BOLD_TEXT << "PID_P_X:" << _INFO_CONSOLE_BOLD_TEXT << std::to_string(m_x_PID_P) << _NORMAL_CONSOLE_TEXT_ << std::endl;

    }

    if (jsonLocalConfig.contains("follow_me_PID_P_Y"))
    {
        m_yz_PID_P = jsonLocalConfig["follow_me_PID_P_Y"].get<double>();
        std::cout << _SUCCESS_CONSOLE_TEXT_ << "Apply:  " << _LOG_CONSOLE_BOLD_TEXT << "PID_P_Y:" << _INFO_CONSOLE_BOLD_TEXT << std::to_string(m_yz_PID_P) << _NORMAL_CONSOLE_TEXT_ << std::endl;

    }

    if (jsonLocalConfig.contains("follow_me_PID_I_X"))
    {
        m_x_PID_I = jsonLocalConfig["follow_me_PID_I_X"].get<double>();
        std::cout << _SUCCESS_CONSOLE_TEXT_ << "Apply:  " << _LOG_CONSOLE_BOLD_TEXT << "PID_I_X:" << _INFO_CONSOLE_BOLD_TEXT << std::to_string(m_x_PID_I) << _NORMAL_CONSOLE_TEXT_ << std::endl;

    }

    if (jsonLocalConfig.contains("follow_me_PID_I_Y"))
    {
        m_yz_PID_I = jsonLocalConfig["follow_me_PID_I_Y"].get<double>();
        std::cout << _SUCCESS_CONSOLE_TEXT_ << "Apply:  " << _LOG_CONSOLE_BOLD_TEXT << "PID_I_Y:" << _INFO_CONSOLE_BOLD_TEXT << std::to_string(m_yz_PID_I) << _NORMAL_CONSOLE_TEXT_ << std::endl;

    }

    if (jsonLocalConfig.contains("follow_me_smoothing"))
    {
        m_alpha = jsonLocalConfig["follow_me_smoothing"].get<double>();
        std::cout << _SUCCESS_CONSOLE_TEXT_ << "Apply:  " << _LOG_CONSOLE_BOLD_TEXT << "smoothing:" << _INFO_CONSOLE_BOLD_TEXT << std::to_string(m_alpha) << _NORMAL_CONSOLE_TEXT_ << std::endl;

    }

    m_PID_X.setPID(m_x_PID_P, m_x_PID_I, 0);
    m_PID_YZ.setPID(m_yz_PID_P, m_yz_PID_I, 0);
}

void CTrackingManager::onStatusChanged(const int status)
{
    std::cout << _INFO_CONSOLE_BOLD_TEXT << "onTrackStatusChanged:" << _LOG_CONSOLE_BOLD_TEXT << std::to_string(status) << _NORMAL_CONSOLE_TEXT_ << std::endl;
    
    
    switch (status)
    {
        case TrackingTarget_STATUS_TRACKING_LOST:
            m_object_detected = false;
            de::fcb::CFCBMain::getInstance().adjustRemoteJoystickByMode(RC_SUB_ACTION::RC_SUB_ACTION_RELEASED);
            break;
        
        case TrackingTarget_STATUS_TRACKING_DETECTED:
            m_object_detected = true;
            de::fcb::CFCBMain::getInstance().adjustRemoteJoystickByMode(RC_SUB_ACTION::RC_SUB_ACTION_JOYSTICK_CHANNELS);
            break;
        
        case TrackingTarget_STATUS_TRACKING_ENABLED:
            m_tracking_running = true;
            m_PID_X.setPID(m_x_PID_P, m_x_PID_I, 0);
            m_PID_YZ.setPID(m_yz_PID_P, m_yz_PID_I, 0);
            break;
        
        case TrackingTarget_STATUS_TRACKING_STOPPED:
            m_tracking_running = false;
            m_PID_X.setPID(m_x_PID_P, m_x_PID_I, 0);
            m_PID_YZ.setPID(m_yz_PID_P, m_yz_PID_I, 0);
            de::fcb::CFCBMain::getInstance().adjustRemoteJoystickByMode(RC_SUB_ACTION::RC_SUB_ACTION_RELEASED);
            break;
        
        default:
            break;
    }
}
                

void CTrackingManager::onTrack(const double x, const double yz, const bool is_xy)
{

    std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_last_message_time);

    if (duration < m_target_frame_time_ms) 
    {
        std::cout << "skip" << std::endl;
        return ;
    }
    m_last_message_time = now;

    #ifdef DDEBUG
        std::cout << _INFO_CONSOLE_BOLD_TEXT << "onTrack >> " 
        << _LOG_CONSOLE_BOLD_TEXT << "  x:" << _INFO_CONSOLE_BOLD_TEXT << x
        << _LOG_CONSOLE_BOLD_TEXT << "  yz:" << _INFO_CONSOLE_BOLD_TEXT << yz
        << _LOG_CONSOLE_BOLD_TEXT << "  is_xy:" << _INFO_CONSOLE_BOLD_TEXT << is_xy
        << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif

    const RCMAP_CHANNELS_MAP_INFO_STRUCT rc_map = m_fcbMain2.getRCChannelsMapInfo();
    if ((!rc_map.use_smart_rc) || (!rc_map.is_valid)) return ;

    // values: [-0.5,0.5]

    // value: [0,1000] IMPORTANT: SKIP_RC_CHANNEL (-999) means channel release
    // 'R': Rudder
    // 'T': Throttle
    // 'A': Aileron
    // 'E': Elevator
    int16_t rc_channels[RC_CHANNELS_MAX] = {SKIP_RC_CHANNEL};
    

    m_x = m_PID_X.calculate(x);
    m_yz = m_PID_YZ.calculate(yz);
    //m_x = m_alpha * x * m_x_PID_P + (1-m_alpha) * m_x;
    //m_yz = m_alpha * yz * m_yz_PID_P + (1-m_alpha) * m_yz;

    m_x = std::clamp(m_x, -0.5, 0.5);
    m_yz = std::clamp(m_yz, -0.5, 0.5);

    int tracking_x  = static_cast<int>(m_x  * 1000 + 500);
    int tracking_yz = static_cast<int>(m_yz * 1000 + 500);
    
    switch (de::fcb::CFCBMain::getInstance().getAndruavVehicleInfo().vehicle_type)
    {
        case ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_TRI:
        case ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_QUAD:
        break;

        case ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_BOAT:
        break;

        case ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_ROVER:
        break;
    }
    if (is_xy)
    {
        // x & y .... forward camera.
        rc_channels[rc_map.rcmap_roll]      = 500;
        rc_channels[rc_map.rcmap_pitch]     = 1000 - tracking_yz;
        rc_channels[rc_map.rcmap_yaw]       = 1000 - tracking_x; // to be aligned with default settings of Ardu
        rc_channels[rc_map.rcmap_throttle]  = 500; 
    }
    else
    {
        // x & z .... vertical camera.
        rc_channels[rc_map.rcmap_roll]      = 1000 - tracking_x; // to be aligned with default settings of Ardu
        rc_channels[rc_map.rcmap_pitch]     = 1000 - tracking_yz;
        rc_channels[rc_map.rcmap_yaw]       = 500; 
        rc_channels[rc_map.rcmap_throttle]  = 500; 
                    
    }

    
    
    #ifdef DDEBUG
        std::cout << _INFO_CONSOLE_BOLD_TEXT << "onTrack >> " 
        << _LOG_CONSOLE_BOLD_TEXT << "  rcmap_roll:" << rc_map.rcmap_roll << ":" << _INFO_CONSOLE_BOLD_TEXT << rc_channels[rc_map.rcmap_roll]
        << _LOG_CONSOLE_BOLD_TEXT << "  rcmap_pitch:" << rc_map.rcmap_pitch << ":" << _INFO_CONSOLE_BOLD_TEXT << rc_channels[rc_map.rcmap_pitch] 
       << _LOG_CONSOLE_BOLD_TEXT << "  rcmap_yaw:" << rc_map.rcmap_yaw << ":" << _INFO_CONSOLE_BOLD_TEXT << rc_channels[rc_map.rcmap_yaw]
        << _LOG_CONSOLE_BOLD_TEXT << "  rcmap_throttle:" << rc_map.rcmap_throttle << ":" << _INFO_CONSOLE_BOLD_TEXT << rc_channels[rc_map.rcmap_throttle] 
         << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif

    m_fcbMain2.updateRemoteControlChannels(rc_channels);
}

