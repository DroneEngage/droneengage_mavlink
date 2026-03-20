cmake_minimum_required(VERSION 3.10)
project(advanced_pid_controller_test)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add the PID controller source
add_library(advanced_pid_controller STATIC
    ../de_pilot/advanced_pid_controller.cpp
)

# Include directories
target_include_directories(advanced_pid_controller PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_CURRENT_SOURCE_DIR}/../de_pilot
)

# Add the test executable
add_executable(test_advanced_pid_controller_standalone
    test_advanced_pid_controller_standalone.cpp
)

# Link the PID controller library
target_link_libraries(test_advanced_pid_controller_standalone
    advanced_pid_controller
)

# Include directories for test
target_include_directories(test_advanced_pid_controller_standalone PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_CURRENT_SOURCE_DIR}/../de_pilot
)
