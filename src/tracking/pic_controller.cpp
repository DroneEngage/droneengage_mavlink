// pid_controller.cpp
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

    // 2. Integral term with anti-windup
    double pre_clamp_integral = m_integral_sum + m_Ki * error;
    m_integral_sum = std::clamp(pre_clamp_integral, -m_integral_limit, m_integral_limit);
    m_integral_sum = std::clamp(m_integral_sum, -m_max_min_value, m_max_min_value);

    
    // 4. Derivative term with filtering
    double derivativeTerm = 0.0;
    if (m_delta_time > 0.0)
    {
        double rawDerivative = (error - m_previous_error) / m_delta_time;
        // Simple low-pass filter for D-term (alpha = 0.2 for smoothing)
        static double filteredDerivative = 0.0;
        const double alpha = 0.2;
        filteredDerivative = alpha * rawDerivative + (1.0 - alpha) * filteredDerivative;
        derivativeTerm = m_Kd * filteredDerivative;
    }
    
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