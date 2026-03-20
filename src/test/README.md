# Advanced PID Controller Test Suite

This directory contains comprehensive tests for the `CAdvancedPIDController` class located in `../de_pilot/advanced_pid_controller.cpp`.

## Files

- `test_advanced_pid_controller_standalone.cpp` - Main test file with comprehensive test suite
- `Makefile` - Build configuration for compiling and running tests
- `CMakeLists.txt` - Alternative CMake build configuration
- `README.md` - This documentation file

## Building and Running Tests

### Using Make (Recommended)

```bash
# Build the test executable
make

# Run all tests
make test

# Or run tests directly
./test_advanced_pid_controller_standalone

# Clean build artifacts
make clean
```

### Using CMake

```bash
# Create build directory
mkdir build && cd build

# Configure and build
cmake ..
make

# Run tests
./test_advanced_pid_controller_standalone
```

## Test Coverage

The test suite includes 17 comprehensive tests covering:

### Core Functionality
- **ConstructorInitialization** - Tests basic constructor and initialization
- **FullConstructorInitialization** - Tests constructor with all parameters
- **BasicPIDCalculation** - Tests PID calculation without feedforward
- **PIDWithFeedforward** - Tests PID calculation with feedforward input

### Advanced Features
- **IntegralWindupPrevention** - Tests integral windup prevention mechanisms
- **AdvancedAntiWindup** - Tests advanced vs basic anti-windup algorithms
- **DerivativeFiltering** - Tests derivative term low-pass filtering
- **OutputLimiting** - Tests output saturation limiting
- **ResetFunctionality** - Tests controller state reset

### Static Methods
- **SqrtController** - Tests the static sqrt_controller method
- **SqrtControllerEdgeCases** - Tests edge cases for sqrt_controller

### Configuration
- **ConfigurationMethods** - Tests all configuration setter methods
- **DerivativeFilterAlphaBounds** - Tests derivative filter alpha parameter bounds

### Behavior and Performance
- **ConvergenceBehavior** - Tests controller convergence to setpoint
- **ZeroGainsBehavior** - Tests behavior with individual zero gains
- **PerformanceTest** - Tests computational performance
- **RealisticParametersTest** - Integration test with realistic drone parameters

## Test Results

All tests are designed to pass with the current implementation. The test suite validates:

- Correct PID calculations
- Proper anti-windup behavior
- Output limiting
- Derivative filtering
- Configuration parameter validation
- Performance requirements
- Convergence characteristics

## Notes

- Debug output from the PID controller is disabled during testing to keep output clean
- Tests use a custom lightweight test framework to avoid external dependencies
- Performance thresholds are set conservatively to accommodate different hardware
- Realistic parameters test uses simplified drone dynamics for validation

## Troubleshooting

If tests fail:

1. Ensure all dependencies are available (C++17 compiler)
2. Check that the source files are in the correct locations
3. Verify that include paths are correct in the build configuration
4. Make sure no modifications were made to the PID controller that break the expected behavior

## Adding New Tests

To add new tests:

1. Create a new test function using the `TEST(test_name)` macro
2. Use the provided assertion macros (`ASSERT_TRUE`, `ASSERT_EQ`, etc.)
3. Add the test to the test list in the `main()` function
4. Follow the existing naming and coding conventions
