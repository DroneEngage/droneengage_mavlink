#include "../helpers/helpers.hpp"
#include "../helpers/colors.hpp"

#include "fcb_tracking_manager.hpp"

#include "../fcb_main.hpp"


using namespace de::fcb::tracking;


de::fcb::CFCBMain&  m_fcbMain2 = de::fcb::CFCBMain::getInstance();


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
            
            break;
        
        case TrackingTarget_STATUS_TRACKING_STOPPED:
            m_tracking_running = false;
            de::fcb::CFCBMain::getInstance().adjustRemoteJoystickByMode(RC_SUB_ACTION::RC_SUB_ACTION_RELEASED);
            break;
        
        default:
            break;
    }
}
                

void CTrackingManager::onTrack(const double x, const double yz, const bool is_xy)
{

    #ifdef DEBUG
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
    
    const int tracking_x  = static_cast<int>(x  * 1000 + 500);
    const int tracking_xy = static_cast<int>(yz * 1000 + 500);
    
    if (is_xy)
    {
        // x & y .... forward camera.
        rc_channels[rc_map.rcmap_roll]      = 1000 - tracking_x; // to be aligned with default settings of Ardu
        rc_channels[rc_map.rcmap_pitch]     = 1000 - tracking_xy;
        rc_channels[rc_map.rcmap_yaw]       = 500; 
        rc_channels[rc_map.rcmap_throttle]  = 500; 
    }
    else
    {
        // x & z .... vertical camera.
        rc_channels[rc_map.rcmap_roll]      = 1000 - tracking_x; // to be aligned with default settings of Ardu
        rc_channels[rc_map.rcmap_pitch]     = 1000 - tracking_xy;
        rc_channels[rc_map.rcmap_yaw]       = 500; 
        rc_channels[rc_map.rcmap_throttle]  = 500; 
                    
    }
    
    
    #ifdef DEBUG
        std::cout << _INFO_CONSOLE_BOLD_TEXT << "onTrack >> " 
        << _LOG_CONSOLE_BOLD_TEXT << "  rcmap_roll:" << rc_map.rcmap_roll << ":" << _INFO_CONSOLE_BOLD_TEXT << rc_channels[rc_map.rcmap_roll]
        << _LOG_CONSOLE_BOLD_TEXT << "  rcmap_pitch:" << rc_map.rcmap_pitch << ":" << _INFO_CONSOLE_BOLD_TEXT << rc_channels[rc_map.rcmap_pitch] 
       << _LOG_CONSOLE_BOLD_TEXT << "  rcmap_yaw:" << rc_map.rcmap_yaw << ":" << _INFO_CONSOLE_BOLD_TEXT << rc_channels[rc_map.rcmap_yaw]
        << _LOG_CONSOLE_BOLD_TEXT << "  rcmap_throttle:" << rc_map.rcmap_throttle << ":" << _INFO_CONSOLE_BOLD_TEXT << rc_channels[rc_map.rcmap_throttle] 
         << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif

    m_fcbMain2.updateRemoteControlChannels(rc_channels);
}

