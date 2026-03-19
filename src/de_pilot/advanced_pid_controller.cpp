#include "advanced_pid_controller.hpp"
#include <cmath>
#include <iostream>

namespace de {
namespace fcb {
namespace depilot {

CAdvancedPIDController::CAdvancedPIDController(double kp, double ki, double kd, double dt, 
                                            double integral_max, double max_min_value, 
                                            double feedforward_gain, 
                                            bool advanced_antiwindup,
                                            double derivative_filter_alpha)
    : m_Kp(kp), m_Ki(ki), m_Kd(kd), m_delta_time(dt), 
      m_integral_limit(integral_max), m_max_min_value(max_min_value),
      m_feedforward_gain(feedforward_gain), m_advanced_antiwindup(advanced_antiwindup),
      m_derivative_filter_alpha(derivative_filter_alpha),
      m_previous_error(0.0), m_integral_sum(0.0), m_filtered_derivative(0.0)
{
    // Initialize gains and other internal states
}

double CAdvancedPIDController::calculate(double error) {
    return calculate(error, 0.0);
}

double CAdvancedPIDController::calculate(double error, double feedforward_input) {
    // 1. Proportional term
    double proportionalTerm = m_Kp * error;

    // 2. Integral term with anti-windup
    double i_term = 0.0;
    if (m_Ki != 0.0) {
        if (m_advanced_antiwindup) {
            // Advanced anti-windup: check if output would be saturated
            double pre_clamp_integral = m_integral_sum + m_Ki * error * m_delta_time;
            double tentative_output = proportionalTerm + pre_clamp_integral + m_Kd * m_filtered_derivative;
            
            // Add feedforward to check saturation
            if (m_feedforward_gain != 0.0) {
                tentative_output += feedforward_input * m_feedforward_gain;
            }
            
            // Check if adding to integral would exacerbate windup while pushing against saturation limits
            bool is_saturated_positive = (tentative_output >= m_max_min_value) && (error > 0);
            bool is_saturated_negative = (tentative_output <= -m_max_min_value) && (error < 0);
            
            if (!is_saturated_positive && !is_saturated_negative) {
                m_integral_sum += m_Ki * error * m_delta_time;
                
                // Clamp integral to absolute limit
                if (m_integral_sum > m_integral_limit) m_integral_sum = m_integral_limit;
                if (m_integral_sum < -m_integral_limit) m_integral_sum = -m_integral_limit;
            }
        } else {
            // Basic anti-windup: simple clamping
            m_integral_sum += m_Ki * error * m_delta_time;
            m_integral_sum = std::clamp(m_integral_sum, -m_integral_limit, m_integral_limit);
        }
        i_term = m_integral_sum;
    }

    // 3. Derivative term with filtering
    double derivativeTerm = 0.0;
    if (m_Kd != 0.0 && m_delta_time > 0.0) {
        double rawDerivative = (error - m_previous_error) / m_delta_time;
        // Low-pass filter for D-term
        m_filtered_derivative = m_derivative_filter_alpha * rawDerivative + 
                               (1.0 - m_derivative_filter_alpha) * m_filtered_derivative;
        derivativeTerm = m_Kd * m_filtered_derivative;
    }

    // 4. Feedforward term
    double feedforwardTerm = 0.0;
    if (m_feedforward_gain != 0.0) {
        feedforwardTerm = feedforward_input * m_feedforward_gain;
    }

    // 5. Combine terms for the total PID output
    double output = proportionalTerm + i_term + derivativeTerm + feedforwardTerm;
    output = std::clamp(output, -m_max_min_value, m_max_min_value);

    // Update previous error for the next iteration
    m_previous_error = error;

    return output;
}

void CAdvancedPIDController::reset() {
    m_previous_error = 0.0;
    m_integral_sum = 0.0;
    m_filtered_derivative = 0.0;
}

double CAdvancedPIDController::sqrt_controller(double error, double p_gain, double max_accel, double max_rate) {
    // sqrt_controller: ArduPilot-style piecewise controller
    // Blends a linear P-region near the target with a sqrt deceleration
    // curve far from the target, ensuring kinematically feasible rate profiles.

    // If no acceleration limit is defined, fallback to pure P-controller
    if (max_accel <= 0.0) {
        double rate = error * p_gain;
        if (rate > max_rate) rate = max_rate;
        if (rate < -max_rate) rate = -max_rate;
        return rate;
    }

    // If no P gain is defined, fallback to pure square root controller
    if (p_gain <= 0.0) {
        double result;
        if (error > 0.0) {
            result = std::sqrt(2.0 * max_accel * error);
        } else if (error < 0.0) {
            result = -std::sqrt(2.0 * max_accel * (-error));
        } else {
            result = 0.0;
        }
        if (result > max_rate) result = max_rate;
        if (result < -max_rate) result = -max_rate;
        return result;
    }

    // Piecewise controller: linear inside threshold, sqrt outside
    // linear_dist is the distance at which the sqrt curve's slope
    // equals the P-gain, ensuring tangent continuity at the transition.
    const double linear_dist = max_accel / (p_gain * p_gain);
    double desired_rate;

    if (error > linear_dist) {
        // Positive error beyond linear region — use sqrt branch
        // Subtract (linear_dist / 2) to align tangent with linear curve
        desired_rate = std::sqrt(2.0 * max_accel * (error - (linear_dist / 2.0)));
    } else if (error < -linear_dist) {
        // Negative error beyond linear region — use sqrt branch
        desired_rate = -std::sqrt(2.0 * max_accel * ((-error) - (linear_dist / 2.0)));
    } else {
        // Inside linear region — simple proportional control
        desired_rate = error * p_gain;
    }

    // Clamp to max rate
    if (desired_rate > max_rate) desired_rate = max_rate;
    if (desired_rate < -max_rate) desired_rate = -max_rate;

    return desired_rate;
}

void CAdvancedPIDController::setFeedforwardGain(double gain) {
    m_feedforward_gain = gain;
}

void CAdvancedPIDController::setDerivativeFilterAlpha(double alpha) {
    m_derivative_filter_alpha = std::clamp(alpha, 0.0, 1.0);
}

void CAdvancedPIDController::setPID(double kp, double ki, double kd) {
    m_Kp = kp;
    m_Ki = ki;
    m_Kd = kd;
}

void CAdvancedPIDController::setIntegralLimit(double limit) {
    m_integral_limit = limit;
}

void CAdvancedPIDController::setDeltaTime(double dt) {
    m_delta_time = dt;
}

void CAdvancedPIDController::setOutputLimit(double limit) {
    m_max_min_value = limit;
}

} // namespace depilot
} // namespace fcb
} // namespace de
