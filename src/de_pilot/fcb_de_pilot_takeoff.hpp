#ifndef FCB_DE_PILOT_TAKEOFF_H_
#define FCB_DE_PILOT_TAKEOFF_H_

#include <cstdint>

namespace de {
namespace fcb {
namespace depilot {

class CDEPilotTakeoff {
public:
    static CDEPilotTakeoff &getInstance() {
        static CDEPilotTakeoff instance;
        return instance;
    }

    CDEPilotTakeoff(CDEPilotTakeoff const &) = delete;
    void operator=(CDEPilotTakeoff const &) = delete;

private:
    CDEPilotTakeoff() {}

public:
    ~CDEPilotTakeoff() {}

public:
    void startTakeoff(double target_altitude);
    void updateTakeoff();
    void abortTakeoff();
    bool isTakeoffComplete() const;
    bool isTakeoffActive() const;
    
    void readConfigParameters();

private:
    enum TakeoffPhase {
        PHASE_IDLE,
        PHASE_ARM_CHECK,
        PHASE_CLIMBING,
        PHASE_STABILIZING,
        PHASE_COMPLETE,
        PHASE_ABORTED
    };

    bool m_active = false;
    TakeoffPhase m_phase = PHASE_IDLE;
    double m_target_altitude = 0.0;
    double m_start_altitude = 0.0;
    uint64_t m_phase_start_time = 0;
    uint64_t m_last_update_time = 0;
    
    // PID controller state
    double m_last_error = 0.0;
    double m_integral = 0.0;
    
    // Configuration parameters
    double m_max_climb_rate = 2.5;          // m/s
    double m_stabilize_time_ms = 2000;      // ms
    uint64_t m_timeout_ms = 30000;          // ms
    double m_pid_p = 0.5;
    double m_pid_i = 0.1;
    double m_pid_d = 0.2;
    double m_deadband = 0.5;                // meters
    
    int16_t calculateThrottleAdjustment(double current_altitude);
    float calculateClimbRate(double current_altitude);
};

} // namespace depilot
} // namespace fcb
} // namespace de

#endif
