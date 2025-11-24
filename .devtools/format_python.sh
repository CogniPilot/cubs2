#!/bin/bash
# Autoformat all Python code in the Cubs2 workspace using black

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$SCRIPT_DIR/.."
cd "$REPO_ROOT"

echo "Installing/upgrading formatting tools..."
pip3 install --user --upgrade black isort autopep8 flake8

echo ""
echo "Running black formatter on all Python files..."
black --config .devtools/pyproject.toml \
  cubs2_dynamics \
  cubs2_control \
  cubs2_simulation \
  cubs2_planning \
  racecourse_description

echo ""
echo "Running isort on all Python files..."
isort --settings-path .devtools/pyproject.toml \
  cubs2_dynamics \
  cubs2_control \
  cubs2_simulation \
  cubs2_planning \
  racecourse_description

echo ""
echo "Checking with flake8..."
flake8 \
  cubs2_dynamics \
  cubs2_control \
  cubs2_simulation \
  cubs2_planning \
  racecourse_description || true

echo ""
echo "Python formatting complete!"
