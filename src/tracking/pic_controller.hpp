#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_


namespace de
{
namespace fcb
{
namespace tracking
{

class CPIDController {
public:
    /**
     * @brief Constructor for the CPIDController.
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     * @param dt Time step between calculations (e.g., in seconds).
     * @param integralMax The maximum absolute value for the integral sum to prevent windup.
     * Set to a very large number (e.g., std::numeric_limits<double>::max())
     * or 0 if no anti-windup is desired.
     */
    CPIDController(double kp, double ki, double kd, double dt, double integralMax);

    /**
     * @brief Calculates the PID output.
     * @param setpoint The desired target value.
     * @param currentValue The current measured value.
     * @return The calculated control output.
     */
    double calculate(double setpoint, double currentValue);

    /**
     * @brief Resets the integral and previous error terms.
     * Useful when the controller is disabled or the target changes significantly.
     */
    void reset();

private:
    double Kp;           // Proportional gain
    double Ki;           // Integral gain
    double Kd;           // Derivative Derivative gain
    double deltaTime;    // Time step
    double integralLimit; // Maximum absolute value for the integral sum

    double previousError; // Error from the previous calculation
    double integralSum;   // Sum of historical errors for the integral term
};

}
}
}
#endif
