#ifndef FCB_DE_PILOT_STABILIZATION_H_
#define FCB_DE_PILOT_STABILIZATION_H_

#include <cstdint>
#include "fcb_de_pilot_operation_base.hpp"

namespace de {
namespace fcb {
namespace depilot {

class CDEPilotStabilization : public CDEPilotOperationBase {
public:
    static CDEPilotStabilization &getInstance() {
        static CDEPilotStabilization instance;
        return instance;
    }

    CDEPilotStabilization(CDEPilotStabilization const &) = delete;
    void operator=(CDEPilotStabilization const &) = delete;

private:
    CDEPilotStabilization() {}

public:
    ~CDEPilotStabilization() {}

    // Base class interface implementation
    void init() override;
    void update() override;
    void uninit() override;
    void readConfigParameters() override;
    void setPhase(int phase) override;
    int getPhase() const override;
    void setActive(bool active) override;
    bool getActive() const override;
    bool isCompleted() override;

    // Class-specific interface
    void startStabilization();
    void updateStabilization();
    void stopStabilization();
    bool isStabilizationActive() const;

    // Yaw control interface
    void setYawTarget(double angle, double rate, bool is_clockwise, bool is_relative);
    void clearYawTarget();
    bool isYawControlActive() const;

private:
    // Helper to compute desired yaw rate with linear taper
    float calculateDesiredYawRate(double angle_error_rad) const;

private:
    enum StabilizationPhase {
        PHASE_IDLE,
        PHASE_STABILIZING,
        PHASE_COMPLETE
    };

    // Stabilization-specific member variables (base class provides m_active, m_generic_phase, m_phase_start_time, m_last_update_time)
    StabilizationPhase m_phase = PHASE_IDLE;
    double m_target_altitude = 0.0;
    
    // Yaw control member variables
    bool m_yaw_control_enabled = false;
    double m_target_yaw_angle = 0.0;
    double m_yaw_turn_rate = 0.0;
    bool m_yaw_is_clockwise = true;
    bool m_yaw_is_relative = false;
    
    // Yaw PID controller variables
    double m_yaw_error = 0.0;
    double m_yaw_error_integral = 0.0;
    double m_yaw_error_derivative = 0.0;
    double m_yaw_previous_error = 0.0;
    uint64_t m_yaw_last_time = 0;
    
    // Yaw rate tracking for rate-based control
    double m_last_heading_for_rate = 0.0;
    uint64_t m_yaw_rate_check_time = 0;
    double m_current_yaw_rate = 0.0;
    
    // Configuration parameters
    double m_stabilize_time_ms = 2000;      // ms
    double m_default_yaw_rate = 30.0;       // deg/sec
    
    // Yaw PID parameters
    double m_yaw_p = 1.0;                   // Proportional gain
    double m_yaw_i = 0.0;                   // Integral gain
    double m_yaw_d = 0.0;                   // Derivative gain
    double m_yaw_integral_limit = 100.0;    // Integral windup limit
    double m_slowdown_angle = 15.0;         // degrees - linear taper distance
};

} // namespace depilot
} // namespace fcb
} // namespace de

#endif
