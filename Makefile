# Cubs2 Development Makefile
# Convenience commands for building, testing, and formatting

# Cubs2 packages to build and test
CUBS2_PACKAGES = cubs2_dynamics cubs2_control cubs2_simulation \
	cubs2_planning racecourse_description cubs2_bringup

# Optional cmake args (e.g., for CI with ccache)
CMAKE_ARGS ?=

.PHONY: help build clean test coverage format-python format-cpp format format-check lint test-sim test-quick

help:
	@echo "Cubs2 Development Commands:"
	@echo "  make build         - Build all packages"
	@echo "  make clean         - Clean build artifacts"
	@echo "  make test          - Run all tests"
	@echo "  make test-quick    - Run tests without format check"
	@echo "  make test-sim      - Run only simulation tests"
	@echo "  make coverage      - Run tests with coverage report"
	@echo "  make format        - Format all code (Python + C++)"
	@echo "  make format-check  - Check code formatting without modifying"
	@echo "  make format-python - Format Python code with black"
	@echo "  make format-cpp    - Format C++ code with clang-format"
	@echo "  make lint          - Run linters on all packages"

build:
	cd ../.. && colcon build --symlink-install --packages-up-to $(CUBS2_PACKAGES) $(if $(CMAKE_ARGS),--cmake-args $(CMAKE_ARGS))

clean:
	cd ../.. && rm -rf build install log

test: format-check
	cd ../.. && colcon test --packages-select $(CUBS2_PACKAGES)
	cd ../.. && colcon test-result --verbose

test-quick:
	cd ../.. && colcon test --packages-select $(CUBS2_PACKAGES)
	cd ../.. && colcon test-result --verbose

test-sim:
	cd ../.. && colcon test --packages-select cubs2_simulation --event-handlers console_direct+
	cd ../.. && colcon test-result --verbose

coverage:
	cd ../.. && source install/setup.bash && \
	python3 -m pytest src/cubs2/cubs2_dynamics/test/ \
		--cov=src/cubs2/cubs2_dynamics/cubs2_dynamics \
		--cov=src/cubs2/cubs2_control/cubs2_control \
		--cov-config=src/cubs2/.devtools/pyproject.toml \
		--cov-report=term-missing \
		--cov-report=html:coverage_html \
		--cov-report=xml:coverage.xml
	@echo ""
	@echo "Coverage report generated:"
	@echo "  - Terminal: see above"
	@echo "  - HTML: coverage_html/index.html"
	@echo "  - XML: coverage.xml"

format: format-python format-cpp

format-check:
	@echo "Checking Python formatting with black..."
	black --config ./.devtools/pyproject.toml --check --diff .
	@echo "Checking C++ formatting with clang-format..."
	find . -name "*.cpp" -o -name "*.hpp" | xargs clang-format --dry-run -Werror -style=file:./.devtools/.clang-format
	@echo "✅ All formatting checks passed!"

format-python:
	./.devtools/format_python.sh

format-cpp:
	./.devtools/format_cpp.sh

lint:
	cd ../.. && colcon test --packages-select \
		cubs2_dynamics cubs2_control cubs2_simulation \
		cubs2_planning cubs2_rviz --event-handlers console_direct+
