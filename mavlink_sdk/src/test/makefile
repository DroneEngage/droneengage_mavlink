# Makefile for MAVLink TLOG Test Suite
# Allows independent compilation of test files

CXX=g++
CXXFLAGS = -std=c++11 -Wall -Wextra -g -O0
INCLUDES = -I ../../../c_library_v2 -I .. -I ../helpers
LIBS = -lpthread

# Test executable name
TEST_EXE = mavlink_tlog_tester
TEST_BIN = bin

# Source files
TEST_SRC = mavlink_tlog.cpp
TLOG_SRC = ../tlog/mavlink_tlog.cpp

# Object files
TEST_OBJ = $(TEST_BIN)/mavlink_tlog_test.o
TLOG_OBJ = $(TEST_BIN)/mavlink_tlog.o

# All objects
OBJS = $(TEST_OBJ) $(TLOG_OBJ)

# Default target
all: directories $(TEST_EXE)

# Create necessary directories
directories:
	@mkdir -p $(TEST_BIN)

# Build test executable
$(TEST_EXE): $(OBJS)
	@echo "Linking $(TEST_EXE)..."
	$(CXX) $(OBJS) $(LIBS) -o $(TEST_BIN)/$(TEST_EXE)
	@echo "✅ Test executable built: $(TEST_BIN)/$(TEST_EXE)"

# Compile test source
$(TEST_OBJ): $(TEST_SRC)
	@echo "Compiling test source..."
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $(TEST_SRC) -o $(TEST_OBJ)

# Compile tlog source
$(TLOG_OBJ): $(TLOG_SRC)
	@echo "Compiling tlog source..."
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $(TLOG_SRC) -o $(TLOG_OBJ)

# Run tests
test: $(TEST_EXE)
	@echo "🚀 Running MAVLink TLOG Test Suite..."
	@echo "========================================"
	./$(TEST_BIN)/$(TEST_EXE)

# Run tests with valgrind (if available)
test-memory: $(TEST_EXE)
	@if command -v valgrind >/dev/null 2>&1; then \
		echo "🔍 Running tests with memory check..."; \
		valgrind --leak-check=full --show-leak-kinds=all ./$(TEST_BIN)/$(TEST_EXE); \
	else \
		echo "⚠️  Valgrind not found, running normal tests..."; \
		make test; \
	fi

# Debug build
debug: CXXFLAGS += -DDEBUG -g3
debug: $(TEST_EXE)

# Release build
release: CXXFLAGS += -DNDEBUG -O2 -s
release: clean $(TEST_EXE)

# Clean build artifacts
clean:
	@echo "🧹 Cleaning test build artifacts..."
	@rm -rf $(TEST_BIN)
	@rm -f *.tlog
	@rm -f test_*.tlog
	@echo "✅ Clean completed"

# Install dependencies (if needed)
deps:
	@echo "📦 Checking dependencies..."
	@if command -v $(CXX) >/dev/null 2>&1; then \
		echo "✅ Compiler found: $(CXX)"; \
	else \
		echo "❌ Compiler not found: $(CXX)"; \
		exit 1; \
	fi
	@if [ -d "../../c_library_v2" ]; then \
		echo "✅ MAVLink library found"; \
	else \
		echo "❌ MAVLink library not found at ../../c_library_v2"; \
		echo "Please run: git submodule update --init --recursive"; \
		exit 1; \
	fi

# Help target
help:
	@echo "Available targets:"
	@echo "  all       - Build test executable (default)"
	@echo "  test      - Build and run tests"
	@echo "  test-memory- Run tests with memory check (valgrind)"
	@echo "  debug     - Build with debug symbols"
	@echo "  release   - Build optimized release version"
	@echo "  clean     - Remove build artifacts"
	@echo "  deps      - Check dependencies"
	@echo "  help      - Show this help message"
	@echo ""
	@echo "Usage examples:"
	@echo "  make              # Build tests"
	@echo "  make test         # Build and run tests"
	@echo "  make clean test   # Clean and run tests"
	@echo "  make test-memory  # Run with memory checking"

# Phony targets
.PHONY: all directories test test-memory debug release clean deps help

# Prevent make from deleting intermediate files
.PRECIOUS: $(TEST_OBJ) $(TLOG_OBJ)