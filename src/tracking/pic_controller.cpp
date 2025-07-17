// pid_controller.cpp
#include <iostream> // For demonstration purposes, can be removed in production
#include <limits>   // Required for std::numeric_limits
#include <algorithm> // Required for std::clamp

#include "pic_controller.hpp"

using namespace de::fcb::tracking;


CPIDController::CPIDController(): m_Kp(1.0), m_Ki(0.0), m_Kd(0.0), m_delta_time(1), m_integral_limit(0.5), m_max_min_value (1.0)
      ,m_previous_error(0.0), m_integral_sum(0.0)
{

}

CPIDController::CPIDController(double kp, double ki, double kd, double dt, double integral_max, double max_min_value)
    : m_Kp(kp), m_Ki(ki), m_Kd(kd), m_delta_time(dt), m_integral_limit(integral_max), m_max_min_value (max_min_value)
      ,m_previous_error(0.0), m_integral_sum(0.0) {
    // Initialize gains and other internal states
}

double CPIDController::calculate(double error) {
    
    // 1. Proportional term
    double proportionalTerm = m_Kp * error;

    // 2. Integral term
    m_integral_sum += m_Ki * error ; 

    // Apply anti-windup by clamping the integral sum
    m_integral_sum = std::clamp(m_integral_sum, -m_integral_limit, m_integral_limit);
    m_integral_sum = std::clamp(m_integral_sum, -m_max_min_value, m_max_min_value);

    
    // 4. Derivative term
    const double derivativeTerm = m_Kd * (error - m_previous_error) ;
    
    // Update previous error for the next iteration
    m_previous_error = error;

    // 5. Combine terms for the total PID output
    double output = proportionalTerm + m_integral_sum + derivativeTerm;
    output = std::clamp(output, -m_max_min_value, m_max_min_value);

    return output;
}

void CPIDController::reset() {
    m_previous_error = 0.0;
    m_integral_sum = 0.0;
}