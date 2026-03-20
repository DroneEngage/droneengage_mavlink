#include <iostream>
#include <cassert>
#include <cmath>
#include <chrono>
#include <vector>
#include "../de_pilot/advanced_pid_controller.hpp"

using namespace de::fcb::depilot;

// Simple test framework macros
#define ASSERT_TRUE(condition) \
    do { \
        if (!(condition)) { \
            std::cerr << "ASSERTION FAILED: " << #condition << " at line " << __LINE__ << std::endl; \
            return false; \
        } \
    } while(0)

#define ASSERT_FALSE(condition) ASSERT_TRUE(!(condition))

#define ASSERT_NEAR(expected, actual, tolerance) \
    do { \
        if (std::abs((expected) - (actual)) > (tolerance)) { \
            std::cerr << "ASSERTION FAILED: " << (expected) << " != " << (actual) << " (±" << (tolerance) << ") at line " << __LINE__ << std::endl; \
            return false; \
        } \
    } while(0)

#define ASSERT_GT(val1, val2) ASSERT_TRUE((val1) > (val2))
#define ASSERT_LT(val1, val2) ASSERT_TRUE((val1) < (val2))
#define ASSERT_GE(val1, val2) ASSERT_TRUE((val1) >= (val2))
#define ASSERT_LE(val1, val2) ASSERT_TRUE((val1) <= (val2))
#define ASSERT_EQ(val1, val2) ASSERT_TRUE((val1) == (val2))

#define TEST(test_name) bool test_name()

// Test parameters
const double kp = 1.0;
const double ki = 0.1;
const double kd = 0.05;
const double dt = 0.01;  // 10ms time step
const double integral_max = 10.0;
const double output_limit = 5.0;
const double feedforward_gain = 0.5;
const bool advanced_antiwindup = true;
const double derivative_filter_alpha = 0.2;

// Test basic constructor and initialization
TEST_F(AdvancedPIDControllerTest, ConstructorInitialization) {
    CAdvancedPIDController pid(kp, ki, kd, dt, integral_max, output_limit);
    
    // Test that controller is properly initialized
    EXPECT_NO_THROW(pid.calculate(1.0));
}

// Test constructor with all parameters
TEST_F(AdvancedPIDControllerTest, FullConstructorInitialization) {
    CAdvancedPIDController pid(kp, ki, kd, dt, integral_max, output_limit, 
                              feedforward_gain, advanced_antiwindup, derivative_filter_alpha);
    
    EXPECT_NO_THROW(pid.calculate(1.0, 1.0));
}

// Test basic PID calculation without feedforward
TEST_F(AdvancedPIDControllerTest, BasicPIDCalculation) {
    CAdvancedPIDController pid(kp, ki, kd, dt, integral_max, output_limit);
    
    double error = 1.0;
    double output = pid.calculate(error);
    
    // Output should be proportional to error initially
    EXPECT_GT(output, 0.0);
    EXPECT_LE(output, output_limit);
}

// Test PID calculation with feedforward
TEST_F(AdvancedPIDControllerTest, PIDWithFeedforward) {
    CAdvancedPIDController pid(kp, ki, kd, dt, integral_max, output_limit, feedforward_gain);
    
    double error = 1.0;
    double feedforward = 2.0;
    double output = pid.calculate(error, feedforward);
    
    // Output should include feedforward contribution
    EXPECT_GT(output, 0.0);
    EXPECT_LE(output, output_limit);
}

// Test integral windup prevention
TEST_F(AdvancedPIDControllerTest, IntegralWindupPrevention) {
    CAdvancedPIDController pid(kp, ki, kd, dt, integral_max, output_limit);
    
    // Apply consistent error to build up integral
    for (int i = 0; i < 100; ++i) {
        pid.calculate(10.0);  // Large error
    }
    
    // Output should be limited
    double output = pid.calculate(10.0);
    EXPECT_LE(output, output_limit);
    EXPECT_GE(output, -output_limit);
}

// Test advanced anti-windup vs basic anti-windup
TEST_F(AdvancedPIDControllerTest, AdvancedAntiWindup) {
    // Controller with advanced anti-windup
    CAdvancedPIDController pid_advanced(kp, ki, kd, dt, integral_max, output_limit, 
                                        0.0, true, derivative_filter_alpha);
    
    // Controller with basic anti-windup
    CAdvancedPIDController pid_basic(kp, ki, kd, dt, integral_max, output_limit, 
                                    0.0, false, derivative_filter_alpha);
    
    // Apply large error to both controllers
    for (int i = 0; i < 50; ++i) {
        pid_advanced.calculate(10.0);
        pid_basic.calculate(10.0);
    }
    
    // Both should limit output, but advanced should handle windup better
    double output_advanced = pid_advanced.calculate(10.0);
    double output_basic = pid_basic.calculate(10.0);
    
    EXPECT_LE(output_advanced, output_limit);
    EXPECT_LE(output_basic, output_limit);
}

// Test derivative filtering
TEST_F(AdvancedPIDControllerTest, DerivativeFiltering) {
    CAdvancedPIDController pid(kp, ki, kd, dt, integral_max, output_limit, 
                              0.0, false, derivative_filter_alpha);
    
    // Apply rapidly changing error
    double output1 = pid.calculate(1.0);
    double output2 = pid.calculate(-1.0);
    double output3 = pid.calculate(1.0);
    
    // Derivative term should be filtered (less aggressive response)
    EXPECT_GT(output1, 0.0);
    EXPECT_LT(output2, 0.0);
    EXPECT_GT(output3, 0.0);
}

// Test output limiting
TEST_F(AdvancedPIDControllerTest, OutputLimiting) {
    CAdvancedPIDController pid(kp, ki, kd, dt, integral_max, output_limit);
    
    // Apply very large error
    double output = pid.calculate(100.0);
    
    // Output should be clamped to limit
    EXPECT_LE(output, output_limit);
    EXPECT_GE(output, -output_limit);
}

// Test reset functionality
TEST_F(AdvancedPIDControllerTest, ResetFunctionality) {
    CAdvancedPIDController pid(kp, ki, kd, dt, integral_max, output_limit);
    
    // Apply some error to build up integral and previous error
    pid.calculate(1.0);
    pid.calculate(2.0);
    
    // Reset the controller
    pid.reset();
    
    // After reset, first calculation should behave like fresh start
    double output_after_reset = pid.calculate(1.0);
    EXPECT_GT(output_after_reset, 0.0);
}

// Test sqrt_controller static method
TEST_F(AdvancedPIDControllerTest, SqrtController) {
    double error = 10.0;
    double p_gain = 1.0;
    double max_accel = 5.0;
    double max_rate = 15.0;
    
    double output = CAdvancedPIDController::sqrt_controller(error, p_gain, max_accel, max_rate);
    
    // Output should be positive for positive error
    EXPECT_GT(output, 0.0);
    EXPECT_LE(output, max_rate);
    
    // Test negative error
    double output_negative = CAdvancedPIDController::sqrt_controller(-error, p_gain, max_accel, max_rate);
    EXPECT_LT(output_negative, 0.0);
    EXPECT_GE(output_negative, -max_rate);
}

// Test sqrt_controller edge cases
TEST_F(AdvancedPIDControllerTest, SqrtControllerEdgeCases) {
    // Test with zero max_accel (should fallback to P-controller)
    double output1 = CAdvancedPIDController::sqrt_controller(10.0, 1.0, 0.0, 15.0);
    EXPECT_GT(output1, 0.0);
    
    // Test with zero p_gain (should fallback to pure sqrt controller)
    double output2 = CAdvancedPIDController::sqrt_controller(10.0, 0.0, 5.0, 15.0);
    EXPECT_GT(output2, 0.0);
    
    // Test with zero error
    double output3 = CAdvancedPIDController::sqrt_controller(0.0, 1.0, 5.0, 15.0);
    EXPECT_EQ(output3, 0.0);
}

// Test configuration methods
TEST_F(AdvancedPIDControllerTest, ConfigurationMethods) {
    CAdvancedPIDController pid(kp, ki, kd, dt, integral_max, output_limit);
    
    // Test setting feedforward gain
    pid.setFeedforwardGain(1.0);
    double output1 = pid.calculate(1.0, 2.0);
    EXPECT_GT(output1, 0.0);
    
    // Test setting derivative filter alpha
    pid.setDerivativeFilterAlpha(0.5);
    double output2 = pid.calculate(1.0);
    EXPECT_GT(output2, 0.0);
    
    // Test setting PID gains
    pid.setPID(2.0, 0.2, 0.1);
    double output3 = pid.calculate(1.0);
    EXPECT_GT(output3, 0.0);
    
    // Test setting integral limit
    pid.setIntegralLimit(20.0);
    double output4 = pid.calculate(1.0);
    EXPECT_GT(output4, 0.0);
    
    // Test setting delta time
    pid.setDeltaTime(0.02);
    double output5 = pid.calculate(1.0);
    EXPECT_GT(output5, 0.0);
    
    // Test setting output limit
    pid.setOutputLimit(10.0);
    double output6 = pid.calculate(100.0);
    EXPECT_LE(output6, 10.0);
}

// Test derivative filter alpha bounds
TEST_F(AdvancedPIDControllerTest, DerivativeFilterAlphaBounds) {
    CAdvancedPIDController pid(kp, ki, kd, dt, integral_max, output_limit);
    
    // Test alpha > 1.0 (should be clamped to 1.0)
    pid.setDerivativeFilterAlpha(1.5);
    double output1 = pid.calculate(1.0);
    EXPECT_GT(output1, 0.0);
    
    // Test alpha < 0.0 (should be clamped to 0.0)
    pid.setDerivativeFilterAlpha(-0.5);
    double output2 = pid.calculate(1.0);
    EXPECT_GT(output2, 0.0);
}

// Test convergence behavior
TEST_F(AdvancedPIDControllerTest, ConvergenceBehavior) {
    CAdvancedPIDController pid(kp, ki, kd, dt, integral_max, output_limit);
    
    double setpoint = 10.0;
    double current_value = 0.0;
    int iterations = 100;
    
    // Simulate a simple step response
    for (int i = 0; i < iterations; ++i) {
        double error = setpoint - current_value;
        double output = pid.calculate(error);
        
        // Simple plant model: output moves towards setpoint
        current_value += output * dt * 0.1;  // Simple integrator plant
    }
    
    // System should have converged closer to setpoint
    double final_error = std::abs(setpoint - current_value);
    EXPECT_LT(final_error, setpoint * 0.5);  // Should reduce error by at least 50%
}

// Test zero gains behavior
TEST_F(AdvancedPIDControllerTest, ZeroGainsBehavior) {
    // Controller with only P gain
    CAdvancedPIDController pid_p(kp, 0.0, 0.0, dt, integral_max, output_limit);
    double output_p = pid_p.calculate(1.0);
    EXPECT_NEAR(output_p, kp * 1.0, 0.001);
    
    // Controller with only I gain
    CAdvancedPIDController pid_i(0.0, ki, 0.0, dt, integral_max, output_limit);
    double output_i1 = pid_i.calculate(1.0);
    double output_i2 = pid_i.calculate(1.0);  // Second call should increase integral
    EXPECT_GT(output_i2, output_i1);
    
    // Controller with only D gain
    CAdvancedPIDController pid_d(0.0, 0.0, kd, dt, integral_max, output_limit);
    pid_d.reset();
    double output_d1 = pid_d.calculate(1.0);
    double output_d2 = pid_d.calculate(2.0);  // Change in error should produce D term
    EXPECT_NE(output_d2, output_d1);
}

// Performance test
TEST_F(AdvancedPIDControllerTest, PerformanceTest) {
    CAdvancedPIDController pid(kp, ki, kd, dt, integral_max, output_limit, 
                              feedforward_gain, advanced_antiwindup, derivative_filter_alpha);
    
    auto start = std::chrono::high_resolution_clock::now();
    
    // Run 10000 iterations
    for (int i = 0; i < 10000; ++i) {
        double error = std::sin(i * 0.01);
        double feedforward = std::cos(i * 0.01);
        pid.calculate(error, feedforward);
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    
    // Should complete within reasonable time (adjust threshold as needed)
    EXPECT_LT(duration.count(), 10000);  // Less than 10ms for 10000 iterations
}

// Integration test with realistic parameters
TEST_F(AdvancedPIDControllerTest, RealisticParametersTest) {
    // Realistic drone altitude controller parameters
    double kp_alt = 2.0;
    double ki_alt = 0.5;
    double kd_alt = 0.8;
    double dt_alt = 0.02;  // 50Hz update rate
    double integral_max_alt = 5.0;
    double output_limit_alt = 10.0;  // Max climb/descent rate
    double ff_gain_alt = 0.3;
    
    CAdvancedPIDController altitude_pid(kp_alt, ki_alt, kd_alt, dt_alt, 
                                        integral_max_alt, output_limit_alt, 
                                        ff_gain_alt, true, 0.1);
    
    double target_altitude = 100.0;
    double current_altitude = 0.0;
    double vertical_speed = 0.0;
    
    // Simulate altitude control for 5 seconds
    for (int i = 0; i < 250; ++i) {  // 5 seconds at 50Hz
        double altitude_error = target_altitude - current_altitude;
        double desired_climb_rate = altitude_pid.calculate(altitude_error, 0.0);
        
        // Simple altitude dynamics
        vertical_speed += (desired_climb_rate - vertical_speed) * 0.1;  // Simple first-order response
        current_altitude += vertical_speed * dt_alt;
    }
    
    // Should have reached close to target altitude
    double final_error = std::abs(target_altitude - current_altitude);
    EXPECT_LT(final_error, target_altitude * 0.1);  // Within 10% of target
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
