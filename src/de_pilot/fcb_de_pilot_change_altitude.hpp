#ifndef FCB_DE_PILOT_CHANGE_ALTITUDE_H_
#define FCB_DE_PILOT_CHANGE_ALTITUDE_H_

#include <cstdint>
#include "fcb_de_pilot_operation_base.hpp"

namespace de {
namespace fcb {
namespace depilot {

class CDEPilotChangeAltitude : public CDEPilotOperationBase {
public:
    static CDEPilotChangeAltitude &getInstance() {
        static CDEPilotChangeAltitude instance;
        return instance;
    }

    CDEPilotChangeAltitude(CDEPilotChangeAltitude const &) = delete;
    void operator=(CDEPilotChangeAltitude const &) = delete;

private:
    CDEPilotChangeAltitude() {}

public:
    ~CDEPilotChangeAltitude() {}

    // Base class interface implementation
    void init() override;
    void update() override;
    void uninit() override;
    void readConfigParameters() override;
    void setPhase(int phase) override;
    int getPhase() const override;
    void setActive(bool active) override;
    bool getActive() const override;
    void startAltitudeChange(double target_altitude);
    bool isCompleted() override;

private:
    // Class-specific interface
    
    void updateTakeoff();
    void abortTakeoff();
    bool isTakeoffComplete() const;
    bool isTakeoffActive() const;
    bool isAltitudeReached() const;
    bool isAltitudeControlActive() const;
    void stopAltitudeControl();
    void determineAscendDescendPhase();
private:
    enum AltitudeControlPhase {
        PHASE_IDLE,
        PHASE_ARM_CHECK,
        PHASE_ARMED,
        PHASE_ASCENDING,
        PHASE_DESCENDING,
        PHASE_COMPLETE,
        PHASE_ABORTED
    };

    // Takeoff-specific member variables (base class provides m_active, m_generic_phase, m_phase_start_time, m_last_update_time)
    AltitudeControlPhase m_phase = PHASE_IDLE;
    double m_target_altitude = 0.0;
    double m_start_altitude = 0.0;
    uint64_t m_start_time = 0;
    
    // PID controller state
    double m_last_error = 0.0;
    double m_integral = 0.0;
    
    // Configuration parameters
    double m_max_climb_rate = 2.5;          // m/s
    double m_stabilize_time_ms = 2000;      // ms
    uint64_t m_timeout_ms = 10000;          // ms - smart timeout based on climb rate
    double m_pid_p = 0.5;
    double m_pid_i = 0.0;  // Disabled - use only P control
    double m_pid_d = 0.0;  // Disabled - use only P control
    double m_ff_scale = 150.0;              // PWM per m/s feedforward
    double m_deadband = 0.5;                // meters
    double m_max_accel = 2.5;               // m/s^2 - max acceleration/deceleration for sqrt_controller
    
    // Altitude control specific parameters
    double m_max_throttle_adjustment = 200; // PWM - from altitude module
    double m_rate_limit = 100;              // PWM/s - from altitude module
    uint64_t m_altitude_timeout_ms = 60000;  // ms - longer timeout for altitude changes
    
    // Climb rate tracking for smart timeout
    double m_last_altitude_for_climb_rate = 0.0;
    uint64_t m_climb_rate_check_time = 0;
    double m_current_climb_rate = 0.0;
    uint64_t m_zero_climb_rate_start_time = 0;  // Track when climb rate became zero
    
    int16_t calculateThrottleAdjustment(double current_altitude);
    float calculateClimbRate(double current_altitude);
};

} // namespace depilot
} // namespace fcb
} // namespace de

#endif
