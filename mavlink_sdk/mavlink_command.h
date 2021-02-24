#ifndef MAVLINK_COMMAND_H_
#define MAVLINK_COMMAND_H_

#include <map>
#include <common/mavlink.h>


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
        
        void doSetMode   (const int& mode);
        void doArmDisarm (const bool& arm, const bool& force);
            
        void setHome (const float& yaw, const float& latitude, const float& longitude, const float& altitude);
        void setROI (const float& latitude, const float& longitude, const float& altitude);
        void resetROI ();
        void cmdTerminalFlight ();
        void changeAltitude (const float& altitude);
        void takeOff (const float& altitude);
        void gotoGuidedPoint (const double& latitude, const double& longitude, const double& altitude);
        void setYawCondition (const double& target_angle, const double& turn_rate, const bool& is_clock_wise, const bool& is_relative);
        void setNavigationSpeed ( const int& speed_type, const double& speed, const double& throttle, const bool& is_relative);
        void reloadWayPoints ();
        void clearWayPoints ();
        void setCurrentMission (const int& mission_number);
        void requestMissionList ();
        void getWayPointByNumber (const int& mission_number);
        void setMissionCount (const int& mission_count, MAV_MISSION_TYPE mission_type);
        void writeMission (std::map <int, mavlink_mission_item_int_t> mavlink_mission);
        void writeMissionItem (mavlink_mission_item_int_t mavlink_mission);
    
        void sendMissionAck ();

    protected:
        void sendLongCommand (const uint16_t& command,
                const bool& confirmation = false,
                const float& param1 = 0.0f,
                const float& param2 = 0.0f,
                const float& param3 = 0.0f,
                const float& param4 = 0.0f,
                const float& param5 = 0.0f,
                const float& param6 = 0.0f,
                const float& param7 = 0.0f);


};

}

#endif