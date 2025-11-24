#!/bin/bash
# Test CI workflow locally using Docker
# This replicates the GitHub Actions CI environment

set -e

echo "================================================"
echo "Testing CI locally with ROS Jazzy Docker image"
echo "================================================"

# Run the ROS Jazzy container and execute CI steps
docker run --rm -v "$(pwd):/workspace" -w /workspace ros:jazzy bash -c '
set -e

echo ">>> Installing system dependencies..."
apt-get update
apt-get install -y software-properties-common ccache curl git

# Enable universe and multiverse repositories
add-apt-repository -y universe
add-apt-repository -y multiverse
apt-get update

echo ">>> Initializing rosdep..."
rosdep init || echo "rosdep already initialized"
rosdep update

echo ">>> Cloning cyecca dependency..."
cd /workspace/..
if [ ! -d "cyecca" ]; then
    git clone https://github.com/CogniPilot/cyecca.git
fi
cd /workspace

echo ">>> Installing project dependencies..."
rosdep install --from-paths . --ignore-src -r -y

echo ">>> Installing pytest-cov
apt-get install -y python3-pytest-cov

echo ">>> Downloading and installing CasADi..."
if [ ! -f "/tmp/casadi.deb" ]; then
    curl -L -o /tmp/casadi.deb https://github.com/CogniPilot/helmet/raw/main/install/resources/casadi-3.6.3-328.0d8030d49-Linux.deb
fi
apt-get install -y /tmp/casadi.deb

echo ">>> Building packages..."
. /opt/ros/jazzy/setup.sh
cd ..
colcon build --symlink-install --base-paths /workspace --cmake-args -DCMAKE_BUILD_TYPE=Debug

echo ">>> Running Python tests with coverage..."
. /opt/ros/jazzy/setup.sh
. install/setup.sh
cd cubs2
mkdir -p test_results
python3 -m pytest cubs2_dynamics/test/ \
    --cov=cubs2_dynamics \
    --cov=cubs2_control \
    --cov-config=.devtools/pyproject.toml \
    --cov-report=term-missing \
    --cov-report=html:test_results/python_coverage \
    --cov-report=xml:test_results/coverage.xml

echo ">>> Running ROS tests..."
cd ..
. /opt/ros/jazzy/setup.sh
. install/setup.sh
colcon test --event-handlers console_direct+ --base-paths /workspace

echo ">>> Checking test results..."
colcon test-result --verbose

echo ""
echo "================================================"
echo "✅ Local CI test completed successfully!"
echo "================================================"
'
