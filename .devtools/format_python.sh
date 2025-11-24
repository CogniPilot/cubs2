#!/bin/bash
# Autoformat all Python code in the Cubs2 workspace using black

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$SCRIPT_DIR/.."
cd "$REPO_ROOT"

# Check if required tools are installed
if ! command -v black &> /dev/null; then
    echo "Error: black is not installed. Install it with: sudo apt install python3-black"
    exit 1
fi

# if ! command -v isort &> /dev/null; then
#     echo "Error: isort is not installed. Install it with: sudo apt install isort"
#     exit 1
# fi

# if ! command -v flake8 &> /dev/null; then
#     echo "Error: flake8 is not installed. Install it with: sudo apt install flake8"
#     exit 1
# fi

echo ""
echo "Running black formatter on all Python files..."
black --config .devtools/pyproject.toml \
  cubs2_dynamics \
  cubs2_control \
  cubs2_simulation \
  cubs2_planning \
  racecourse_description \
  cubs2_description

# echo ""
# echo "Running isort on all Python files..."
# isort --settings-path .devtools/pyproject.toml \
#   cubs2_dynamics \
#   cubs2_control \
#   cubs2_simulation \
#   cubs2_planning \
#   racecourse_description

# echo ""
# echo "Checking with flake8..."
# flake8 \
#   cubs2_dynamics \
#   cubs2_control \
#   cubs2_simulation \
#   cubs2_planning \
#   racecourse_description || true

echo ""
echo "Python formatting complete!"
