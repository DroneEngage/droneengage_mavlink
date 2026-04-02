#include <iostream>
#include <iomanip>
#include "../de_pilot/advanced_pid_controller.hpp"

using namespace de::fcb::depilot;

int main() {
    std::cout << "=== Simple Integral Parameter Test ===" << std::endl;
    std::cout << std::fixed << std::setprecision(6);
    
    // Test with only I gain to see integral clearly
    const double kp = 0.0;  // No P gain
    const double ki = 0.1;   // I gain
    const double kd = 0.0;  // No D gain
    const double dt = 0.01; // 10ms
    const double integral_max = 5.0;
    const double output_limit = 10.0;
    
    CAdvancedPIDController pid(kp, ki, kd, dt, integral_max, output_limit);
    
    std::cout << "Parameters: Kp=" << kp << ", Ki=" << ki << ", Kd=" << kd << ", dt=" << dt << std::endl;
    std::cout << "Expected integral increment per call: " << (ki * 1.0 * dt) << std::endl;
    std::cout << std::endl;
    
    std::cout << "Call#\tError\tIntegral Value\tOutput\tExpected Integral" << std::endl;
    std::cout << "-----\t-----\t-------------\t------\t-----------------" << std::endl;
    
    double error = 1.0;
    double expected_integral = 0.0;
    
    for (int i = 1; i <= 10; ++i) {
        double output = pid.calculate(error);
        expected_integral += ki * error * dt;
        double actual_integral = pid.getIntegralSum();
        
        std::cout << std::setw(5) << i << "\t" 
                  << std::setw(5) << error << "\t"
                  << std::setw(13) << actual_integral << "\t"
                  << std::setw(6) << output << "\t"
                  << std::setw(17) << expected_integral << std::endl;
        
        // Verify they match
        if (std::abs(actual_integral - expected_integral) > 0.0001) {
            std::cout << "ERROR: Integral mismatch!" << std::endl;
            return 1;
        }
    }
    
    std::cout << std::endl;
    std::cout << "✓ SUCCESS: Integral parameter is increasing exactly as expected!" << std::endl;
    std::cout << "✓ Each call adds " << (ki * error * dt) << " to the integral" << std::endl;
    std::cout << "✓ Final integral value: " << pid.getIntegralSum() << std::endl;
    
    return 0;
}
