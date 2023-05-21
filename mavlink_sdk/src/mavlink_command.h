#ifndef MAVLINK_COMMAND_H_
#define MAVLINK_COMMAND_H_

#include <map>
#include <all/mavlink.h>
#include <ardupilotmega/ardupilotmega.h>

#define MAX_RC_CHANNELS     18

namespace mavlinksdk
{

/**
 * @brief This class holds methods to do actions on vehicle -mostly-.
 * So this is Vehicle.API
 * 
 */
class CMavlinkCommand
{

    public:
        //https://stackoverflow.com/questions/1008019/c-singleton-design-pattern
        static CMavlinkCommand& getInstance()
        {
            static CMavlinkCommand instance;
            
            return instance;
        };

        CMavlinkCommand(CMavlinkCommand const&)       = delete;
        void operator=(CMavlinkCommand const&)        = delete;

        
        // Note: Scott Meyers mentions in his Effective Modern
        //       C++ book, that deleted functions should generally
        //       be public as it results in better error messages
        //       due to the compilers behavior to check accessibility
        //       before deleted status

        private:

            CMavlinkCommand() 
            {
            };

    public:
        
        void requestMessageEmit(const uint32_t message_id) const ;
        void sendHeartBeatOfGCS() const;
        void sendHeartBeatOfComponent(const uint8_t component_id) const;
        void doSetMode   (const int& mode, const int& custom_mode=0, const int& custom_sub_mode=0)  const;
        void doArmDisarm (const bool& arm, const bool& force)  const;
            
        void setHome (const float& yaw, const float& latitude, const float& longitude, const float& altitude) const;
        void setROI (const float& latitude, const float& longitude, const float& altitude) const;
        void resetROI () const;
        void cmdTerminateFlight () const;
        void changeAltitude (const float& altitude) const;
        void takeOff (const float& altitude) const;
        void gotoGuidedPoint (const double& latitude, const double& longitude, const double& relative_altitude) const;
        void setYawCondition (const double& target_angle, const double& turn_rate, const bool& is_clock_wise, const bool& is_relative) const;
        void setNavigationSpeed ( const int& speed_type, const double& speed, const double& throttle, const bool& is_relative) const;
        void reloadWayPoints () const;
        void clearWayPoints () const;
        void setCurrentMission (const int& mission_number) const;
        void requestMissionList () const;
        void getWayPointByNumber (const int& mission_number) const;
        void setMissionCount (const int& mission_count, MAV_MISSION_TYPE mission_type) const;
        void writeMission (std::map <int, mavlink_mission_item_int_t> mavlink_mission) const;
        void writeMissionItem (mavlink_mission_item_int_t mavlink_mission) const;
        void writeMissionItem (mavlink_mission_item_t mavlink_mission) const;
        void sendMissionAck (const  uint8_t target_system, const uint8_t target_component, const uint8_t result) const;
        void writeParameter (const std::string& param_name, const double &value) const;
        void readParameter (const std::string& param_name) const;
        void readParameterByIndex (const uint16_t& param_index) const;

        void releaseRCChannels() const;
        void sendRCChannels(const int16_t channels[MAX_RC_CHANNELS], int channel_length) const;
        

        void ctrlGuidedVelocityInLocalFrame (const float vx, const float vy, const float vz, const float yaw_rate, MAV_FRAME mav_frame) const;

        void requestExtParametersList () const;
        void requestParametersList () const;
        void requestDataStream(MAV_DATA_STREAM stream_id) const;
        void setServo (const int& channel, const int& pwm) const;
        
        void requestHomeLocation () const;

        void sendNative(const mavlink_message_t mavlink_message) const;
    protected:
        void gotoGuidedPoint_default (const double& latitude, const double& longitude, const double& relative_altitude) const;
        void gotoGuidedPoint_px4 (const double& latitude, const double& longitude, const double& relative_altitude) const;
        void takeOff_default  (const float& altitude) const;
        void takeOff_px4  (const float& altitude) const;

        
        void sendIntCommand (const uint16_t& command,
				const uint8_t& frame,
                const float& param1,
                const float& param2,
                const float& param3,
                const float& param4,
                const int32_t& x,
                const int32_t& y,
                const float& z) const;

        void sendLongCommand (const uint16_t& command,
                const bool& confirmation = false,
                const float& param1 = 0.0f,
                const float& param2 = 0.0f,
                const float& param3 = 0.0f,
                const float& param4 = 0.0f,
                const float& param5 = 0.0f,
                const float& param6 = 0.0f,
                const float& param7 = 0.0f)  const;


};

}

#endif