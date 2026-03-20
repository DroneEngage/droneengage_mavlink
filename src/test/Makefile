# Makefile for Advanced PID Controller Test

CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -g
INCLUDES = -I.. -I../de_pilot
SRCDIR = ..
TESTDIR = .

# Source files
PID_SOURCES = $(SRCDIR)/de_pilot/advanced_pid_controller.cpp
TEST_SOURCES = test_advanced_pid_controller_standalone.cpp
INTEGRAL_TEST_SOURCES = test_integral_increment.cpp
DETAILED_TEST_SOURCES = test_integral_detailed.cpp

# Object files
PID_OBJECTS = advanced_pid_controller.o
TEST_OBJECTS = test_advanced_pid_controller_standalone.o
INTEGRAL_TEST_OBJECTS = test_integral_increment.o
DETAILED_TEST_OBJECTS = test_integral_detailed.o

# Target executables
TARGET = test_advanced_pid_controller_standalone
INTEGRAL_TARGET = test_integral_increment
DETAILED_TARGET = test_integral_detailed

.PHONY: all clean test run integral detailed

all: $(TARGET)

integral: $(INTEGRAL_TARGET)

detailed: $(DETAILED_TARGET)

$(TARGET): $(PID_OBJECTS) $(TEST_OBJECTS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $^

$(INTEGRAL_TARGET): $(PID_OBJECTS) $(INTEGRAL_TEST_OBJECTS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $^

$(DETAILED_TARGET): $(PID_OBJECTS) $(DETAILED_TEST_OBJECTS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $^

$(PID_OBJECTS): $(PID_SOURCES)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $(PID_SOURCES)

$(TEST_OBJECTS): $(TEST_SOURCES)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $(TEST_SOURCES)

$(INTEGRAL_TEST_OBJECTS): $(INTEGRAL_TEST_SOURCES)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $(INTEGRAL_TEST_SOURCES)

$(DETAILED_TEST_OBJECTS): $(DETAILED_TEST_SOURCES)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $(DETAILED_TEST_SOURCES)

test: $(TARGET)
	./$(TARGET)

run-integral: $(INTEGRAL_TARGET)
	./$(INTEGRAL_TARGET)

run-detailed: $(DETAILED_TARGET)
	./$(DETAILED_TARGET)

run: test

clean:
	rm -f $(PID_OBJECTS) $(TEST_OBJECTS) $(INTEGRAL_TEST_OBJECTS) $(DETAILED_TEST_OBJECTS) $(TARGET) $(INTEGRAL_TARGET) $(DETAILED_TARGET)

# Help target
help:
	@echo "Available targets:"
	@echo "  all          - Build the main test executable"
	@echo "  integral     - Build the integral increment test executable"
	@echo "  detailed     - Build the detailed integral test executable"
	@echo "  test         - Build and run main tests"
	@echo "  run-integral - Build and run integral increment test"
	@echo "  run-detailed - Build and run detailed integral test"
	@echo "  run          - Same as test"
	@echo "  clean        - Remove built files"
	@echo "  help         - Show this help message"
