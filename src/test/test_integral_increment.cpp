#include <iostream>
#include <cassert>
#include <cmath>
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

#define ASSERT_NEAR(expected, actual, tolerance) \
    do { \
        if (std::abs((expected) - (actual)) > (tolerance)) { \
            std::cerr << "ASSERTION FAILED: " << (expected) << " != " << (actual) << " (±" << (tolerance) << ") at line " << __LINE__ << std::endl; \
            return false; \
        } \
    } while(0)

#define ASSERT_GT(val1, val2) ASSERT_TRUE((val1) > (val2))
#define ASSERT_LT(val1, val2) ASSERT_TRUE((val1) < (val2))
#define ASSERT_EQ(val1, val2) ASSERT_TRUE((val1) == (val2))
#define ASSERT_LE(val1, val2) ASSERT_TRUE((val1) <= (val2))

bool test_integral_increment_basic_antiwindup() {
    std::cout << "Testing integral increment with basic anti-windup (advanced_antiwindup = false)..." << std::endl;
    
    // Create controller with basic anti-windup (advanced_antiwindup = false)
    double kp = 1.0;
    double ki = 0.1;
    double kd = 0.0;  // No derivative for simplicity
    double dt = 0.01;
    double integral_max = 5.0;
    double output_limit = 10.0;
    
    CAdvancedPIDController pid(kp, ki, kd, dt, integral_max, output_limit, 0.0, false);  // advanced_antiwindup = false
    
    double error = 1.0;
    double integral_increment = ki * error * dt;  // Expected increment: 0.1 * 1.0 * 0.01 = 0.001
    
    std::cout << "Expected integral increment per call: " << integral_increment << std::endl;
    
    // Test 1: Integral should increment normally when not saturated
    std::cout << "\nTest 1: Normal integral increment (no saturation)" << std::endl;
    double output1 = pid.calculate(error);
    std::cout << "After 1st call - Output: " << output1 << std::endl;
    
    double output2 = pid.calculate(error);
    std::cout << "After 2nd call - Output: " << output2 << std::endl;
    
    // Output should increase due to integral term
    ASSERT_GT(output2, output1);
    std::cout << "✓ Integral increment confirmed - output increased from " << output1 << " to " << output2 << std::endl;
    
    // Test 2: Continue calling to see integral buildup
    std::cout << "\nTest 2: Multiple calls to observe integral buildup" << std::endl;
    double previous_output = output2;
    
    for (int i = 3; i <= 10; ++i) {
        double current_output = pid.calculate(error);
        std::cout << "After " << i << "th call - Output: " << current_output << std::endl;
        
        // Each call should increase output (integral accumulating)
        ASSERT_GT(current_output, previous_output);
        previous_output = current_output;
    }
    std::cout << "✓ Integral buildup confirmed over multiple calls" << std::endl;
    
    // Test 3: Test with larger error to approach saturation
    std::cout << "\nTest 3: Testing with larger error (approaching saturation)" << std::endl;
    pid.reset();  // Reset to start fresh
    
    double large_error = 50.0;  // Large error to potentially saturate
    double large_increment = ki * large_error * dt;  // 0.1 * 50.0 * 0.01 = 0.05
    
    std::cout << "Large error: " << large_error << ", expected increment: " << large_increment << std::endl;
    
    double output_before_sat = pid.calculate(large_error);
    std::cout << "First call with large error - Output: " << output_before_sat << std::endl;
    
    // Continue with large error to see if integral properly increments and clamps
    for (int i = 2; i <= 20; ++i) {
        double current_output = pid.calculate(large_error);
        std::cout << "Call " << i << " - Output: " << current_output << std::endl;
        
        // Should eventually saturate at output_limit
        if (i > 5) {  // After some calls, should be near saturation
            ASSERT_LE(current_output, output_limit + 0.001);  // Allow small tolerance
        }
    }
    std::cout << "✓ Saturation behavior confirmed" << std::endl;
    
    return true;
}

bool test_integral_increment_advanced_antiwindup() {
    std::cout << "\n\nTesting integral increment with advanced anti-windup (advanced_antiwindup = true)..." << std::endl;
    
    // Create controller with advanced anti-windup
    double kp = 1.0;
    double ki = 0.1;
    double kd = 0.0;
    double dt = 0.01;
    double integral_max = 5.0;
    double output_limit = 10.0;
    
    CAdvancedPIDController pid(kp, ki, kd, dt, integral_max, output_limit, 0.0, true);  // advanced_antiwindup = true
    
    double error = 1.0;
    
    std::cout << "Test 1: Normal operation with advanced anti-windup" << std::endl;
    double output1 = pid.calculate(error);
    double output2 = pid.calculate(error);
    
    // Should increment normally when not near saturation
    ASSERT_GT(output2, output1);
    std::cout << "✓ Normal increment confirmed - output: " << output1 << " -> " << output2 << std::endl;
    
    // Test 2: Near saturation behavior
    std::cout << "\nTest 2: Near saturation with advanced anti-windup" << std::endl;
    pid.reset();
    
    double large_error = 50.0;
    double output_before = pid.calculate(large_error);
    std::cout << "First large error call - Output: " << output_before << std::endl;
    
    // Advanced anti-windup should prevent integral buildup when approaching saturation
    for (int i = 2; i <= 10; ++i) {
        double current_output = pid.calculate(large_error);
        std::cout << "Call " << i << " - Output: " << current_output << std::endl;
        
        // With advanced anti-windup, integral should stop building when near saturation
        if (i > 3) {
            // Output should not increase much once near saturation
            ASSERT_LE(std::abs(current_output - output_before), 1.0);  // Should not increase significantly
        }
    }
    std::cout << "✓ Advanced anti-windup prevented excessive integral buildup" << std::endl;
    
    return true;
}

bool test_integral_comparison() {
    std::cout << "\n\nComparing basic vs advanced anti-windup behavior..." << std::endl;
    
    double kp = 1.0;
    double ki = 0.1;
    double kd = 0.0;
    double dt = 0.01;
    double integral_max = 5.0;
    double output_limit = 10.0;
    double large_error = 50.0;
    
    // Basic anti-windup controller
    CAdvancedPIDController pid_basic(kp, ki, kd, dt, integral_max, output_limit, 0.0, false);
    
    // Advanced anti-windup controller  
    CAdvancedPIDController pid_advanced(kp, ki, kd, dt, integral_max, output_limit, 0.0, true);
    
    std::cout << "Applying large error (" << large_error << ") to both controllers..." << std::endl;
    
    // Run both controllers with same error
    for (int i = 1; i <= 10; ++i) {
        double output_basic = pid_basic.calculate(large_error);
        double output_advanced = pid_advanced.calculate(large_error);
        
        std::cout << "Call " << i << " - Basic: " << output_basic << ", Advanced: " << output_advanced << std::endl;
        
        // Advanced should be more conservative in integral buildup
        if (i > 5) {
            ASSERT_LE(output_advanced, output_basic + 0.5);  // Advanced should not exceed basic by much
        }
    }
    
    std::cout << "✓ Advanced anti-windup shows more conservative integral behavior" << std::endl;
    
    return true;
}

int main() {
    std::cout << "=== Integral Increment Test Suite ===" << std::endl;
    std::cout << "Testing integral value increments with different anti-windup settings" << std::endl;
    
    std::vector<std::pair<std::string, bool(*)()>> tests = {
        {"Basic Anti-windup Integral Increment", test_integral_increment_basic_antiwindup},
        {"Advanced Anti-windup Integral Increment", test_integral_increment_advanced_antiwindup},
        {"Anti-windup Comparison", test_integral_comparison}
    };
    
    int passed = 0;
    int total = tests.size();
    
    for (const auto& test : tests) {
        std::cout << "\n--- Running " << test.first << " ---" << std::endl;
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
    
    std::cout << "\n=== Test Results ===" << std::endl;
    std::cout << "Passed: " << passed << "/" << total << " tests" << std::endl;
    
    if (passed == total) {
        std::cout << "All tests PASSED! Integral increment behavior is correct." << std::endl;
        return 0;
    } else {
        std::cout << "Some tests FAILED! Check integral increment implementation." << std::endl;
        return 1;
    }
}
