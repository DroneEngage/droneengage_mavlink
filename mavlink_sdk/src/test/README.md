# MAVLink TLOG Test Suite

This directory contains comprehensive tests for the MAVLink TLOG functionality.

## Files

- **mavlink_tlog.cpp** - Main test suite implementation
- **makefile** - Independent build system for tests
- **README.md** - This documentation

## Quick Start

### Build and Run Tests
```bash
# Build the test suite
make

# Run all tests
make test

# Or combine both
make && make test
```

### Advanced Usage

```bash
# Clean build artifacts
make clean

# Build with debug symbols
make debug

# Build optimized release version
make release

# Run with memory checking (requires valgrind)
make test-memory

# Check dependencies
make deps

# Show all available targets
make help
```

## Test Coverage

The test suite covers the following functionality:

### 1. Constructor/Destructor Tests
- ✅ Proper initialization
- ✅ Default state verification
- ✅ Memory cleanup

### 2. File Operations Tests
- ✅ Opening files
- ✅ Closing files
- ✅ Reopening files
- ✅ Filename tracking

### 3. Message Writing Tests
- ✅ Writing to closed files (error handling)
- ✅ Single message writing
- ✅ Multiple message writing
- ✅ Message counting

### 4. Callback Functionality Tests
- ✅ Open callback
- ✅ Close callback
- ✅ Error callback
- ✅ Callback registration

### 5. File Integrity Tests
- ✅ File creation
- ✅ File content verification
- ✅ Minimum size validation

### 6. Multiple Instance Tests
- ✅ Concurrent instances
- ✅ Independent file handling
- ✅ Resource isolation

## Test Output

### Successful Run
```
========================================
   MAVLink TLOG Test Suite
========================================

=== Testing Constructor/Destructor ===
[PASS] Constructor: Initially closed
[PASS] Constructor: Zero messages written
[PASS] Constructor: Empty filename
Destructor test completed (no crash)

=== Testing File Operations ===
[PASS] Open file: Success
[PASS] Open file: Is open flag set
[PASS] Open file: Filename stored correctly
[PASS] Open file: Zero messages initially
[PASS] Close file: Is open flag cleared
[PASS] Reopen file: Success

... (more tests) ...

========================================
Test Results:
Passed: 20
Failed: 0
Total:  20
========================================

🎉 ALL TESTS PASSED! 🎉
```

### Failed Test Example
```
[FAIL] Some test description
========================================
Test Results:
Passed: 19
Failed: 1
Total:  20
========================================

❌ 1 TESTS FAILED ❌
```

## Integration with Main Build

This test suite can be built independently using its own makefile, but it's also integrated with the main SDK build system.

### Independent Build (Recommended for Development)
```bash
cd src/test
make test
```

### Main SDK Build
```bash
cd src
make  # Builds the entire SDK including tests
```

## Troubleshooting

### Common Issues

1. **Compiler Not Found**
   ```
   ❌ Compiler not found: g++
   ```
   **Solution:** Install GCC/G++:
   ```bash
   sudo apt-get install build-essential
   ```

2. **MAVLink Library Missing**
   ```
   ❌ MAVLink library not found at ../../c_library_v2
   ```
   **Solution:** Initialize git submodules:
   ```bash
   cd ../../
   git submodule update --init --recursive
   ```

3. **Permission Denied**
   ```
   ./bin/mavlink_tlog_tester: Permission denied
   ```
   **Solution:** Make executable:
   ```bash
   chmod +x bin/mavlink_tlog_tester
   ```

4. **Valgrind Not Found**
   ```
   ⚠️ Valgrind not found, running normal tests...
   ```
   **Solution:** Install Valgrind (optional):
   ```bash
   sudo apt-get install valgrind
   ```

### Debug Mode

For detailed debugging, use debug mode:
```bash
make clean debug test
```

This enables:
- Debug symbols (`-g3`)
- Debug definitions (`-DDEBUG`)
- No optimizations (`-O0`)

## Adding New Tests

To add new test cases:

1. Add a new test method to `MavlinkTlogTester` class:
```cpp
void test_new_feature()
{
    std::cout << "\n=== Testing New Feature ===" << std::endl;
    
    // Your test code here
    assert_test(condition, "New feature: Specific test case");
}
```

2. Call the new test in `run_all_tests()`:
```cpp
test_new_feature();
```

3. Rebuild and test:
```bash
make clean test
```

## Continuous Integration

This test suite is designed for CI/CD integration:

```bash
# CI-friendly command (exits with proper error codes)
make clean test && echo "CI: Tests passed"
```

The test executable returns:
- `0` - All tests passed
- `1` - One or more tests failed or crashed

## Memory Safety

For production deployments, run memory checks:
```bash
make test-memory
```

This uses Valgrind to detect:
- Memory leaks
- Invalid memory access
- Use-after-free errors
- Buffer overflows

## Performance Testing

The test suite includes basic performance metrics. For detailed performance analysis:

1. Build release version:
   ```bash
   make clean release
   ```

2. Run with time measurement:
   ```bash
   time ./bin/mavlink_tlog_tester
   ```

## Contributing

When contributing to the TLOG module:

1. Add tests for new features
2. Ensure all tests pass: `make clean test`
3. Run memory checks: `make test-memory`
4. Update documentation as needed

## License

This test suite follows the same license as the main DroneEngage project.