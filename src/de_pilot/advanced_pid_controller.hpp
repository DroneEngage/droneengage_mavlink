#ifndef ADVANCED_PID_CONTROLLER_H_
#define ADVANCED_PID_CONTROLLER_H_

#include <cstdint>
#include <algorithm>

namespace de {
namespace fcb {
namespace depilot {

class CAdvancedPIDController {
public:
    /**
     * @brief Constructor for the CAdvancedPIDController with all features.
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     * @param dt Time step between calculations (e.g., in seconds).
     * @param integral_max The maximum absolute value for the integral sum to prevent windup.
     * @param max_min_value Maximum absolute value for the output.
     * @param feedforward_gain Feedforward gain for direct input scaling.
     * @param advanced_antiwindup Enable advanced anti-windup with saturation detection.
     * @param derivative_filter_alpha Alpha value for derivative low-pass filter (0.0-1.0).
     */
    CAdvancedPIDController(double kp, double ki, double kd, double dt, 
                          double integral_max, double max_min_value, 
                          double feedforward_gain = 0.0, 
                          bool advanced_antiwindup = false,
                          double derivative_filter_alpha = 0.2);

    /**
     * @brief Calculates the PID output (standard mode).
     * @param error The current error value.
     * @return PID output value.
     */
    double calculate(double error);

    /**
     * @brief Calculates the PID output with feedforward input.
     * @param error The current error value.
     * @param feedforward_input External feedforward input (e.g., desired rate).
     * @return PID + feedforward output value.
     */
    double calculate(double error, double feedforward_input);

    /**
     * @brief Resets the integral and previous error terms.
     * Useful when the controller is disabled or the target changes significantly.
     */
    void reset();

    /**
     * @brief ArduPilot-style sqrt controller for smooth rate profiles.
     * Blends linear P-control near target with sqrt deceleration far from target.
     * @param error Current error (can be position or angle error).
     * @param p_gain Proportional gain.
     * @param max_accel Maximum acceleration/deceleration limit.
     * @param max_rate Maximum rate limit.
     * @return Desired rate.
     */
    static double sqrt_controller(double error, double p_gain, double max_accel, double max_rate);

    // Configuration methods
    void setFeedforwardGain(double gain);
    void setDerivativeFilterAlpha(double alpha);
    void setPID(double kp, double ki, double kd);
    void setIntegralLimit(double limit);
    void setDeltaTime(double dt);
    void setOutputLimit(double limit);

private:
    // PID gains
    double m_Kp;                // Proportional gain
    double m_Ki;                // Integral gain
    double m_Kd;                // Derivative gain
    double m_delta_time;        // Time step
    
    // Limits
    double m_integral_limit;    // Maximum absolute value for the integral sum
    double m_max_min_value;    // Maximum absolute value for the output
    
    // Feedforward
    double m_feedforward_gain;  // Feedforward gain
    
    // Anti-windup
    bool m_advanced_antiwindup; // Enable advanced anti-windup
    
    // Derivative filtering
    double m_derivative_filter_alpha; // Alpha for low-pass filter
    
    // State variables
    double m_previous_error;    // Error from the previous calculation
    double m_integral_sum;      // Sum of historical errors for the integral term
    double m_filtered_derivative; // Filtered derivative value
};

} // namespace depilot
} // namespace fcb
} // namespace de

#endif // ADVANCED_PID_CONTROLLER_H_
