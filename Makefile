# Cubs2 Development Makefile
# Convenience commands for building and testing using standard ROS 2 tools

# Cubs2 packages to build and test
CUBS2_PACKAGES = cubs2_msgs cubs2_description cubs2_data cubs2_dynamics \
	cubs2_control cubs2_simulation cubs2_planning cubs2_rviz \
	racecourse_description cubs2_bringup

# Optional cmake args (e.g., for CI with ccache)
CMAKE_ARGS ?=

.PHONY: help build clean test test-verbose coverage

help:
	@echo "Cubs2 Development Commands:"
	@echo "  make build        - Build all packages"
	@echo "  make clean        - Clean build artifacts"
	@echo "  make test         - Run all tests and linting (standard ROS 2)"
	@echo "  make test-verbose - Run tests with detailed output"
	@echo "  make coverage     - Run Python tests with coverage report"
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

coverage:
	@echo "Building with coverage flags..."
	cd ../.. && colcon build --symlink-install \
		--packages-select cubs2_rviz \
		--cmake-args -DCMAKE_CXX_FLAGS="--coverage" -DCMAKE_BUILD_TYPE=Debug
	@echo "Running tests..."
	cd ../.. && colcon test --packages-select $(CUBS2_PACKAGES)
	@echo ""
	@echo "Generating Python coverage report..."
	cd ../.. && . install/setup.bash && \
	python3 -m pytest src/cubs2 \
		--cov=src/cubs2 \
		--cov-report=term-missing \
		--cov-report=html:coverage_html/python \
		--cov-report=xml:coverage_python.xml
	@echo ""
	@echo "Generating C++ coverage report..."
	cd ../.. && \
	mkdir -p coverage_html/cpp && \
	lcov --capture --directory build/cubs2_rviz --output-file coverage_cpp.info && \
	lcov --remove coverage_cpp.info '/usr/*' '*/test/*' --output-file coverage_cpp_filtered.info && \
	genhtml coverage_cpp_filtered.info --output-directory coverage_html/cpp
	@echo ""
	@echo "Coverage reports generated:"
	@echo "  - Python HTML: ../../coverage_html/python/index.html"
	@echo "  - Python XML: ../../coverage_python.xml"
	@echo "  - C++ HTML: ../../coverage_html/cpp/index.html"
	@echo "  - C++ Info: ../../coverage_cpp_filtered.info"

