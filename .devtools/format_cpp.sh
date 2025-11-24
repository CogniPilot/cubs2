#!/bin/bash
# Format C++ code using clang-format

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$SCRIPT_DIR/.."
cd "$REPO_ROOT"

echo "Installing/upgrading clang-format..."
sudo apt-get update && sudo apt-get install -y clang-format

echo ""
echo "Running clang-format on all C++ files..."
find cubs2_rviz -name '*.cpp' -o -name '*.hpp' | xargs clang-format -i -style=file:.devtools/.clang-format

echo ""
echo "C++ formatting complete!"
