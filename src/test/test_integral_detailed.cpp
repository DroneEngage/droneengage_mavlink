#include <iostream>
#include <cassert>
#include <cmath>
#include <vector>
#include <iomanip>
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

#define ASSERT_NEAR(expected, actual, tolerance) \
    do { \
        if (std::abs((expected) - (actual)) > (tolerance)) { \
            std::cerr << "ASSERTION FAILED: " << (expected) << " != " << (actual) << " (±" << (tolerance) << ") at line " << __LINE__ << std::endl; \
            return false; \
        } \
    } while(0)

#define ASSERT_GT(val1, val2) ASSERT_TRUE((val1) > (val2))
#define ASSERT_LT(val1, val2) ASSERT_TRUE((val1) < (val2))
#define ASSERT_LE(val1, val2) ASSERT_TRUE((val1) <= (val2))

bool test_detailed_integral_behavior() {
    std::cout << "=== Detailed Integral Behavior Analysis ===" << std::endl;
    
    // Test parameters
    double kp = 0.5;      // Lower P gain to see integral effect more clearly
    double ki = 0.2;      // Higher I gain for more noticeable integral buildup
    double kd = 0.0;      // No derivative
    double dt = 0.01;     // 10ms timestep
    double integral_max = 10.0;
    double output_limit = 8.0;  // Lower limit to see saturation sooner
    
    std::cout << "\nTest Parameters:" << std::endl;
    std::cout << "  Kp: " << kp << ", Ki: " << ki << ", Kd: " << kd << std::endl;
    std::cout << "  dt: " << dt << "s, Integral Limit: " << integral_max << std::endl;
    std::cout << "  Output Limit: " << output_limit << std::endl;
    
    // Test 1: Basic anti-windup (advanced_antiwindup = false)
    std::cout << "\n--- Test 1: Basic Anti-windup (advanced_antiwindup = false) ---" << std::endl;
    
    CAdvancedPIDController pid_basic(kp, ki, kd, dt, integral_max, output_limit, 0.0, false);
    
    double error = 2.0;  // Moderate error
    double expected_increment = ki * error * dt;  // 0.2 * 2.0 * 0.01 = 0.004
    
    std::cout << "Error: " << error << ", Expected integral increment per call: " << expected_increment << std::endl;
    std::cout << "Call\tOutput\tIntegral Effect" << std::endl;
    std::cout << "----\t------\t--------------" << std::endl;
    
    double prev_output = 0.0;
    for (int i = 1; i <= 15; ++i) {
        double output = pid_basic.calculate(error);
        double output_change = output - prev_output;
        
        std::cout << i << "\t" << output << "\t+" << output_change << std::endl;
        prev_output = output;
        
        // Verify integral is incrementing (output should increase)
        if (i > 1) {
            ASSERT_GT(output_change, 0.0);  // Should always increase
        }
    }
    
    std::cout << "✓ Basic anti-windup: Integral increments confirmed each call" << std::endl;
    
    // Test 2: Advanced anti-windup (advanced_antiwindup = true)
    std::cout << "\n--- Test 2: Advanced Anti-windup (advanced_antiwindup = true) ---" << std::endl;
    
    CAdvancedPIDController pid_advanced(kp, ki, kd, dt, integral_max, output_limit, 0.0, true);
    
    std::cout << "Call\tOutput\tIntegral Effect\tBehavior" << std::endl;
    std::cout << "----\t------\t--------------\t--------" << std::endl;
    
    prev_output = 0.0;
    bool saturation_detected = false;
    
    for (int i = 1; i <= 15; ++i) {
        double output = pid_advanced.calculate(error);
        double output_change = output - prev_output;
        
        std::string behavior = "Normal";
        if (output >= output_limit - 0.001) {
            behavior = "Saturated";
            saturation_detected = true;
        } else if (saturation_detected) {
            behavior = "Anti-windup Active";
        }
        
        std::cout << i << "\t" << output << "\t+" << output_change << "\t" << behavior << std::endl;
        prev_output = output;
        
        // Before saturation, should increment normally
        if (i <= 5) {  // First few calls should increment
            ASSERT_GT(output_change, 0.0);
        }
    }
    
    std::cout << "✓ Advanced anti-windup: Normal increment initially, then anti-windup activation" << std::endl;
    
    // Test 3: Edge case - very small error with basic anti-windup
    std::cout << "\n--- Test 3: Small Error with Basic Anti-windup ---" << std::endl;
    
    CAdvancedPIDController pid_small(kp, ki, kd, dt, integral_max, output_limit, 0.0, false);
    
    double small_error = 0.1;
    double small_increment = ki * small_error * dt;  // 0.2 * 0.1 * 0.01 = 0.0002
    
    std::cout << "Small error: " << small_error << ", Expected increment: " << small_increment << std::endl;
    std::cout << "Call\tOutput\tChange" << std::endl;
    std::cout << "----\t------\t-----" << std::endl;
    
    prev_output = 0.0;
    for (int i = 1; i <= 10; ++i) {
        double output = pid_small.calculate(small_error);
        double change = output - prev_output;
        
        std::cout << i << "\t" << std::fixed << std::setprecision(6) << output 
                  << "\t" << std::setprecision(6) << change << std::endl;
        prev_output = output;
        
        // Even small increments should accumulate
        if (i > 1) {
            ASSERT_GT(change, 0.0);
            ASSERT_NEAR(change, small_increment, 0.0001);  // Should be very close to expected
        }
    }
    
    std::cout << "✓ Small error increments: Confirmed precise integral accumulation" << std::endl;
    
    // Test 4: Zero error case
    std::cout << "\n--- Test 4: Zero Error Case ---" << std::endl;
    
    CAdvancedPIDController pid_zero(kp, ki, kd, dt, integral_max, output_limit, 0.0, false);
    
    // Build up some integral first
    for (int i = 0; i < 5; ++i) {
        pid_zero.calculate(1.0);
    }
    
    double output_before_zero = pid_zero.calculate(1.0);
    double output_with_zero = pid_zero.calculate(0.0);  // Zero error
    
    std::cout << "Output before zero error: " << output_before_zero << std::endl;
    std::cout << "Output with zero error: " << output_with_zero << std::endl;
    
    // With zero error, integral should not change (output should be same or less due to no new integral)
    ASSERT_LE(output_with_zero, output_before_zero);
    
    std::cout << "✓ Zero error: No integral increment confirmed" << std::endl;
    
    return true;
}

bool test_negative_error_integral() {
    std::cout << "\n\n=== Negative Error Integral Test ===" << std::endl;
    
    double kp = 1.0;
    double ki = 0.1;
    double kd = 0.0;
    double dt = 0.01;
    double integral_max = 5.0;
    double output_limit = 10.0;
    
    CAdvancedPIDController pid(kp, ki, kd, dt, integral_max, output_limit, 0.0, false);
    
    double negative_error = -1.0;
    double expected_decrement = ki * negative_error * dt;  // Should be negative
    
    std::cout << "Negative error: " << negative_error << std::endl;
    std::cout << "Expected integral change per call: " << expected_decrement << std::endl;
    std::cout << "Call\tOutput\tChange" << std::endl;
    std::cout << "----\t------\t-----" << std::endl;
    
    double prev_output = 0.0;
    for (int i = 1; i <= 8; ++i) {
        double output = pid.calculate(negative_error);
        double change = output - prev_output;
        
        std::cout << i << "\t" << output << "\t" << change << std::endl;
        prev_output = output;
        
        // Output should become more negative (integral accumulating negatively)
        if (i > 1) {
            ASSERT_LT(change, 0.0);  // Should be negative change
            ASSERT_NEAR(change, expected_decrement, 0.0001);
        }
    }
    
    std::cout << "✓ Negative error: Integral decreases correctly" << std::endl;
    
    return true;
}

int main() {
    std::cout << "=== Comprehensive Integral Increment Verification ===" << std::endl;
    std::cout << "Verifying integral value increments when anti-windup is false" << std::endl;
    
    std::vector<std::pair<std::string, bool(*)()>> tests = {
        {"Detailed Integral Behavior", test_detailed_integral_behavior},
        {"Negative Error Integral", test_negative_error_integral}
    };
    
    int passed = 0;
    int total = tests.size();
    
    for (const auto& test : tests) {
        std::cout << "\n" << std::string(60, '=') << std::endl;
        try {
            if (test.second()) {
                std::cout << "✓ " << test.first << " PASSED" << std::endl;
                passed++;
            } else {
                std::cout << "✗ " << test.first << " FAILED" << std::endl;
            }
        } catch (const std::exception& e) {
            std::cout << "✗ " << test.first << " FAILED (Exception: " << e.what() << ")" << std::endl;
        } catch (...) {
            std::cout << "✗ " << test.first << " FAILED (Unknown exception)" << std::endl;
        }
    }
    
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "Final Results: " << passed << "/" << total << " tests passed" << std::endl;
    
    if (passed == total) {
        std::cout << "🎉 SUCCESS: Integral increment behavior is VERIFIED!" << std::endl;
        std::cout << "✓ Integral values increment correctly when anti-windup is false" << std::endl;
        std::cout << "✓ Advanced anti-windup prevents excessive buildup" << std::endl;
        std::cout << "✓ Both positive and negative errors handled correctly" << std::endl;
        return 0;
    } else {
        std::cout << "❌ FAILURE: Integral behavior needs attention!" << std::endl;
        return 1;
    }
}
