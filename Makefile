# Cubs2 Development Makefile
# Convenience commands for building, testing, and formatting

.PHONY: help build clean test format-python format-cpp format lint

help:
	@echo "Cubs2 Development Commands:"
	@echo "  make build         - Build all packages"
	@echo "  make clean         - Clean build artifacts"
	@echo "  make test          - Run all tests"
	@echo "  make format        - Format all code (Python + C++)"
	@echo "  make format-python - Format Python code with black"
	@echo "  make format-cpp    - Format C++ code with clang-format"
	@echo "  make lint          - Run linters on all packages"

build:
	cd ../.. && colcon build --symlink-install

clean:
	cd ../.. && rm -rf build install log

test:
	cd ../.. && colcon test
	cd ../.. && colcon test-result --verbose

format: format-python format-cpp

format-python:
	./format_python.sh

format-cpp:
	./format_cpp.sh

lint:
	cd ../.. && colcon test --packages-select \
		cubs2_dynamics cubs2_control cubs2_simulation \
		cubs2_planning cubs2_rviz --event-handlers console_direct+
