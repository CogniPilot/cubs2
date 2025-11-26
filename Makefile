# Cubs2 Development Makefile
# Convenience commands for building and testing using standard ROS 2 tools

# Cubs2 packages to build and test
CUBS2_PACKAGES = cubs2_dynamics cubs2_control cubs2_simulation \
	cubs2_planning racecourse_description cubs2_bringup

# Optional cmake args (e.g., for CI with ccache)
CMAKE_ARGS ?=

.PHONY: help build clean test test-verbose

help:
	@echo "Cubs2 Development Commands:"
	@echo "  make build        - Build all packages"
	@echo "  make clean        - Clean build artifacts"
	@echo "  make test         - Run all tests and linting (standard ROS 2)"
	@echo "  make test-verbose - Run tests with detailed output"
	@echo ""
	@echo "Standard ROS 2 testing includes:"
	@echo "  - Unit tests (pytest, gtest)"
	@echo "  - Python linting (ament_flake8, ament_pep257)"
	@echo "  - C++ linting (ament_cpplint, ament_uncrustify)"
	@echo "  - Build linting (ament_lint_cmake, ament_xmllint)"
	@echo "  - License validation (ament_copyright)"

build:
	cd ../.. && colcon build --symlink-install --packages-up-to $(CUBS2_PACKAGES) $(if $(CMAKE_ARGS),--cmake-args $(CMAKE_ARGS))

clean:
	cd ../.. && rm -rf build install log

test:
	cd ../.. && colcon test --packages-select $(CUBS2_PACKAGES)
	cd ../.. && colcon test-result --verbose

test-verbose:
	cd ../.. && colcon test --packages-select $(CUBS2_PACKAGES) --event-handlers console_direct+
	cd ../.. && colcon test-result --verbose

