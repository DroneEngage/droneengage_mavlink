#ifndef FCB_DE_PILOT_ALTITUDE_H_
#define FCB_DE_PILOT_ALTITUDE_H_

#include <cstdint>

namespace de {
namespace fcb {
namespace depilot {

class CDEPilotAltitude {
public:
    static CDEPilotAltitude &getInstance() {
        static CDEPilotAltitude instance;
        return instance;
    }

    CDEPilotAltitude(CDEPilotAltitude const &) = delete;
    void operator=(CDEPilotAltitude const &) = delete;

private:
    CDEPilotAltitude() {}

public:
    ~CDEPilotAltitude() {}

public:
    void startAltitudeChange(double target_altitude);
    void updateAltitudeControl();
    void stopAltitudeControl();
    bool isAltitudeReached() const;
    bool isAltitudeControlActive() const;
    
    void readConfigParameters();

private:
    bool m_active = false;
    double m_target_altitude = 0.0;
    double m_start_altitude = 0.0;
    double m_last_error = 0.0;
    double m_integral = 0.0;
    uint64_t m_last_update_time = 0;
    uint64_t m_start_time = 0;
    
    // Configuration parameters
    double m_pid_p = 0.5;
    double m_pid_i = 0.1;
    double m_pid_d = 0.2;
    double m_deadband = 0.5;                    // meters
    double m_max_throttle_adjustment = 200;     // PWM
    double m_max_climb_rate = 2.5;              // m/s
    double m_rate_limit = 100;                  // PWM/s
    uint64_t m_timeout_ms = 60000;              // ms
    
    int16_t calculateThrottleAdjustment(double current_altitude);
    float calculateClimbRate(double current_altitude);
};

} // namespace depilot
} // namespace fcb
} // namespace de

#endif
