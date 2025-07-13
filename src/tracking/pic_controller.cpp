// pid_controller.cpp
#include <iostream> // For demonstration purposes, can be removed in production
#include <limits>   // Required for std::numeric_limits
#include <algorithm> // Required for std::clamp

#include "pic_controller.hpp"

using namespace de::fcb::tracking;


CPIDController::CPIDController(double kp, double ki, double kd, double dt, double integral_max, double max_min_value)
    : m_Kp(kp), m_Ki(ki), m_Kd(kd), m_delta_time(dt), m_integral_limit(integral_max), m_max_min_value (max_min_value)
      ,m_previous_error(0.0), m_integral_sum(0.0) {
    // Initialize gains and other internal states
}

double CPIDController::calculate(double setpoint, double currentValue) {
    // 1. Calculate the error
    double error = setpoint - currentValue;

    // 2. Proportional term
    double proportionalTerm = m_Kp * error;

    // 3. Integral term
    m_integral_sum += error * m_delta_time; // Accumulate error over time

    // Apply anti-windup by clamping the integral sum
    m_integral_sum = std::clamp(m_integral_sum, -m_integral_limit, m_integral_limit);
    m_integral_sum = std::clamp(m_integral_sum, -m_max_min_value, m_max_min_value);

    double integralTerm = m_Ki * m_integral_sum;

    // 4. Derivative term
    // Prevent division by zero if m_delta_time is very small or zero
    double derivativeTerm = 0.0;
    
    if ((m_delta_time > 0.0) && (m_delta_time > std::numeric_limits<double>::epsilon())) {
        derivativeTerm = m_Kd * ((error - m_previous_error) / m_delta_time);
    }

    

    // Update previous error for the next iteration
    m_previous_error = error;

    // 5. Combine terms for the total PID output
    double output = proportionalTerm + integralTerm + derivativeTerm;
    output = std::clamp(output, -m_max_min_value, m_max_min_value);

    return output;
}

void CPIDController::reset() {
    m_previous_error = 0.0;
    m_integral_sum = 0.0;
}