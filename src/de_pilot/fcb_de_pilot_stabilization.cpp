#include "fcb_de_pilot_stabilization.hpp"
#include "../fcb_main.hpp"
#include "../de_common/helpers/colors.hpp"
#include "../de_common/helpers/helpers.hpp"
#include "../de_common/de_databus/configFile.hpp"
#include "fcb_de_pilot_manager.hpp"
#include <mavlink_command.h>
#include <mavlink_sdk.h>
#include <vehicle.h>
#include <iostream>
#include <cmath>

using namespace de::fcb::depilot;

// Base class interface implementation
void CDEPilotStabilization::init() {
    // Initialize stabilization system
    m_phase = PHASE_IDLE;
    m_phase_start_time = get_time_usec() / 1000;
    m_last_update_time = m_phase_start_time;
    m_generic_phase = static_cast<int>(m_phase);

    startStabilization();
}

void CDEPilotStabilization::update() {
    updateStabilization();
}

void CDEPilotStabilization::uninit() {
    stopStabilization();
    m_active = false;
    m_phase = PHASE_IDLE;
    m_generic_phase = static_cast<int>(m_phase);
}

bool CDEPilotStabilization::isCompleted() {
    return true; // you can always get out of this.
}

void CDEPilotStabilization::setPhase(int phase) {
    m_generic_phase = phase;
    m_phase = static_cast<StabilizationPhase>(phase);
    m_phase_start_time = get_time_usec() / 1000;
}

int CDEPilotStabilization::getPhase() const {
    return m_generic_phase;
}

void CDEPilotStabilization::setActive(bool active) {
    m_active = active;
    if (active) {
        m_phase_start_time = get_time_usec() / 1000;
    }
}

bool CDEPilotStabilization::getActive() const {
    return m_active;
}

void CDEPilotStabilization::readConfigParameters() {
    de::CConfigFile &cConfigFile = de::CConfigFile::getInstance();
    const Json_de &jsonConfig = cConfigFile.GetConfigJSON();

    if (jsonConfig.contains("de_pilot")) {
        const Json_de &de_pilot_root = jsonConfig["de_pilot"];

        if (de_pilot_root.contains("stabilization")) {
            const Json_de &stabilization_config = de_pilot_root["stabilization"];

            if (stabilization_config.contains("stabilize_time_ms")) {
                m_stabilize_time_ms = stabilization_config["stabilize_time_ms"].get<double>();
            }
        }
    }
}

void CDEPilotStabilization::startStabilization() {
    if (m_active) {
        std::cout << _INFO_CONSOLE_BOLD_TEXT << "DEPILOT: Stabilization already in progress" 
                  << _NORMAL_CONSOLE_TEXT_ << std::endl;
        return;
    }

    m_target_altitude = mavlinksdk::CVehicle::getInstance().getMsgGlobalPositionInt().relative_alt / 1000.0;
    
    std::cout << _INFO_CONSOLE_BOLD_TEXT << "DEPILOT: Starting stabilization" 
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
    std::cout << "  - Target altitude: " << m_target_altitude << "m" << std::endl;

    m_active = true;
    m_phase = PHASE_STABILIZING;
    m_phase_start_time = get_time_usec();
    m_last_update_time = m_phase_start_time;

    de::fcb::CFCBMain &fcbMain = de::fcb::CFCBMain::getInstance();
    const ANDRUAV_VEHICLE_INFO &vehicle_info = fcbMain.getAndruavVehicleInfo();
    
    std::cout << "  - Current flight mode: " << vehicle_info.flying_mode << std::endl;
    std::cout << "  - Armed: " << (vehicle_info.is_armed ? "YES" : "NO") << std::endl;
    std::cout << "  - Flying: " << (vehicle_info.is_flying ? "YES" : "NO") << std::endl;

    de::fcb::depilot::CDEPilotManager::getInstance().setTargetAltitude(m_target_altitude);
    // Don't set operation here - it's already set by the manager when this operation is started
    
    std::cout << _SUCCESS_CONSOLE_TEXT_ << "DEPILOT: Stabilization initialized successfully" 
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
}

void CDEPilotStabilization::updateStabilization() {
    if (m_phase == PHASE_IDLE || m_phase == PHASE_COMPLETE) {
        return;
    }

    de::fcb::CFCBMain &fcbMain = de::fcb::CFCBMain::getInstance();
    const ANDRUAV_VEHICLE_INFO &vehicle_info = fcbMain.getAndruavVehicleInfo();
    const uint64_t now = get_time_usec();
    
    
    if (!vehicle_info.is_armed) {
        std::cout << _INFO_CONSOLE_BOLD_TEXT << "DEPILOT: Waiting for arm..." 
                  << _NORMAL_CONSOLE_TEXT_ << std::endl;
                // Do nothing I am not armed.
        return;
    }

    const double current_altitude = mavlinksdk::CVehicle::getInstance().getMsgGlobalPositionInt().relative_alt / 1000.0;

    switch (m_phase) {
        case PHASE_STABILIZING: {
            std::cout << _INFO_CONSOLE_BOLD_TEXT << "DEPILOT: Stabilizing altitude" 
                      << _NORMAL_CONSOLE_TEXT_ << std::endl;
            std::cout << "  - Current altitude: " << current_altitude << "m" << std::endl;
            std::cout << "  - Target altitude: " << m_target_altitude << "m" << std::endl;
            
            // In ALT-HOLD mode, send 1500 to stop climbing/descending
            if (vehicle_info.flying_mode != VEHICLE_MODE_GUIDED) {
                std::cout << "  - ALT-HOLD mode: Sending stop command (1500) to maintain altitude" << std::endl;
                
                int16_t rc_channels[RC_CHANNELS_MAX] = {SKIP_RC_CHANNEL};
                rc_channels[fcbMain.getRCChannelsMapInfo().rcmap_throttle] = 1500;  // Stop climbing
                // Force roll and pitch to neutral (1500) to prevent max PWM values
                rc_channels[fcbMain.getRCChannelsMapInfo().rcmap_roll] = 1500;
                rc_channels[fcbMain.getRCChannelsMapInfo().rcmap_pitch] = 1500;
                mavlinksdk::CMavlinkCommand::getInstance().sendRCChannels(
                    rc_channels, RC_CHANNELS_MAX);
            } else {
                std::cout << "  - GUIDED mode: Flight controller handles stabilization" << std::endl;
            }
            
            m_last_update_time = now;
            
            // Stabilization runs indefinitely - no timeout
            // It will continue sending 1500 throttle to maintain altitude
            // until manually stopped or switched to another operation
        }
        break;

        default:
            break;
    }
}

void CDEPilotStabilization::stopStabilization() {
    std::cout << _INFO_CONSOLE_BOLD_TEXT << "DEPILOT: Stopping stabilization" 
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
    
    m_active = false;
    setPhase(PHASE_COMPLETE);
}

bool CDEPilotStabilization::isStabilizationActive() const {
    return m_phase != PHASE_IDLE && m_phase != PHASE_COMPLETE;
}
