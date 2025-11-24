"""Unit tests for DubinsGatePlannerNode."""

import pytest
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor

from cubs2_planning.nodes.planner_dubins import DubinsGatePlannerNode


@pytest.fixture
def ros_context():
    """Initialize and cleanup ROS context for each test."""
    rclpy.init()
    yield
    rclpy.shutdown()


class TestDubinsGatePlannerNode:
    """Unit tests for the DubinsGatePlannerNode class."""

    def test_node_initialization(self, ros_context):
        """Test that DubinsGatePlannerNode initializes correctly with default parameters."""
        # Note: This test may fail if racecourse YAML is not found
        # We'll use a try-except to handle missing racecourse file gracefully
        try:
            node = DubinsGatePlannerNode()

            # Verify node name
            assert node.get_name() == "dubins_gate_planner"

            # Verify parameters were declared
            assert node.has_parameter("racecourse_yaml")
            assert node.has_parameter("turn_radius")
            assert node.has_parameter("sample_points")
            assert node.has_parameter("altitude")
            assert node.has_parameter("velocity")

            # Verify planning functions were created
            assert hasattr(node, "plan_fn")
            assert hasattr(node, "eval_fn")

            # Verify segments were planned
            assert hasattr(node, "segments")

            node.destroy_node()
        except FileNotFoundError as e:
            # If racecourse YAML not found, skip this test
            pytest.skip(f"Racecourse YAML not found: {e}")

    def test_parameters(self, ros_context):
        """Test that node parameters are accessible."""
        try:
            node = DubinsGatePlannerNode()

            # Get parameter values
            turn_radius = node.get_parameter("turn_radius").value
            sample_points = node.get_parameter("sample_points").value
            altitude = node.get_parameter("altitude").value
            velocity = node.get_parameter("velocity").value

            # Verify reasonable defaults
            assert turn_radius > 0.0
            assert sample_points > 0
            assert altitude >= 0.0
            assert velocity > 0.0

            node.destroy_node()
        except FileNotFoundError:
            pytest.skip("Racecourse YAML not found")

    def test_dubins_functions_exist(self, ros_context):
        """Test that Dubins planning functions were created."""
        try:
            node = DubinsGatePlannerNode()

            # Verify functions are callable
            assert callable(node.plan_fn)
            assert callable(node.eval_fn)

            node.destroy_node()
        except FileNotFoundError:
            pytest.skip("Racecourse YAML not found")

    def test_trajectory_planning(self, ros_context):
        """Test that trajectory segments are generated."""
        try:
            node = DubinsGatePlannerNode()

            # Verify segments were created
            assert node.segments is not None
            assert isinstance(node.segments, list)

            # If gates exist, should have segments
            if hasattr(node, "racecourse") and len(node.racecourse.gates) > 0:
                assert len(node.segments) > 0

            node.destroy_node()
        except FileNotFoundError:
            pytest.skip("Racecourse YAML not found")

    def test_gate_sequence_parameter(self, ros_context):
        """Test that gate sequence parameter is accessible."""
        try:
            node = DubinsGatePlannerNode()

            gate_sequence = node.get_parameter("gate_sequence").value
            assert isinstance(gate_sequence, list)

            node.destroy_node()
        except FileNotFoundError:
            pytest.skip("Racecourse YAML not found")

    def test_reference_frame_parameter(self, ros_context):
        """Test that reference frame ID is set correctly."""
        try:
            node = DubinsGatePlannerNode()

            ref_frame = node.get_parameter("reference_frame_id").value
            assert isinstance(ref_frame, str)
            assert len(ref_frame) > 0

            node.destroy_node()
        except FileNotFoundError:
            pytest.skip("Racecourse YAML not found")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
