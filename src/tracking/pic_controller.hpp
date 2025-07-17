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

    CPIDController();

    /**
     * @brief Constructor for the CPIDController.
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     * @param dt Time step between calculations (e.g., in seconds).
     * @param integral_max The maximum absolute value for the integral sum to prevent windup.
     * Set to a very large number (e.g., std::numeric_limits<double>::max())
     * or 0 if no anti-windup is desired.
     */
    CPIDController(double kp, double ki, double kd, double dt, double integral_max, double max_min_value);

    /**
     * @brief Calculates the PID output.
     * @param error The current measured value.
     */
    double calculate(double error);

    /**
     * @brief Resets the integral and previous error terms.
     * Useful when the controller is disabled or the target changes significantly.
     */
    void reset();


    public:

    inline void setPID(double kp, double ki, double kd)
    {
        m_Kp = kp;
        m_Ki = ki;
        m_Kd = kd;
    }


private:
    double m_Kp;                // Proportional gain
    double m_Ki;                // Integral gain
    double m_Kd;                // Derivative Derivative gain
    double m_delta_time;           // Time step
    double m_integral_limit;       // Maximum absolute value for the integral sum
    double m_max_min_value;     // Maximum absolute value for the output

    double m_previous_error;       // Error from the previous calculation
    double m_integral_sum;         // Sum of historical errors for the integral term
};

}
}
}
#endif
