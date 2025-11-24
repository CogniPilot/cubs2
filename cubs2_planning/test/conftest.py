"""Pytest configuration for cubs2_planning tests."""

import os

# Disable simulation time for all tests to prevent timeout waiting for sim clock
os.environ["ROS_DOMAIN_ID"] = "42"  # Use isolated domain for tests
