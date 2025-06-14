// pid_controller.cpp
#include <iostream> // For demonstration purposes, can be removed in production
#include <limits>   // Required for std::numeric_limits
#include <algorithm> // Required for std::clamp

#include "pic_controller.hpp"

using namespace de::fcb::tracking;


CPIDController::CPIDController(double kp, double ki, double kd, double dt, double integralMax)
    : Kp(kp), Ki(ki), Kd(kd), deltaTime(dt), integralLimit(integralMax),
      previousError(0.0), integralSum(0.0) {
    // Initialize gains and other internal states
}

double CPIDController::calculate(double setpoint, double currentValue) {
    // 1. Calculate the error
    double error = setpoint - currentValue;

    // 2. Proportional term
    double proportionalTerm = Kp * error;

    // 3. Integral term
    integralSum += error * deltaTime; // Accumulate error over time

    // Apply anti-windup by clamping the integral sum
    integralSum = std::clamp(integralSum, -integralLimit, integralLimit);

    double integralTerm = Ki * integralSum;

    // 4. Derivative term
    // Prevent division by zero if deltaTime is very small or zero
    double derivativeTerm = 0.0;
    if (deltaTime > std::numeric_limits<double>::epsilon()) {
        derivativeTerm = Kd * ((error - previousError) / deltaTime);
    }

    // Update previous error for the next iteration
    previousError = error;

    // 5. Combine terms for the total PID output
    double output = proportionalTerm + integralTerm + derivativeTerm;

    return output;
}

void CPIDController::reset() {
    previousError = 0.0;
    integralSum = 0.0;
}