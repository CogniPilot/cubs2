# Copyright 2025 CogniPilot Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Unit tests for DubinsGatePlannerNode."""
from cubs2_planning.planner_dubins import DubinsGatePlannerNode
import pytest
import rclpy


@pytest.fixture
def ros_context():
    """Initialize and cleanup ROS context for each test."""
    # Initialize ROS without use_sim_time
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
            assert node.get_name() == 'dubins_gate_planner'

            # Verify parameters were declared
            assert node.has_parameter('racecourse_yaml')
            assert node.has_parameter('turn_radius')
            assert node.has_parameter('sample_points')
            assert node.has_parameter('planner.altitude')
            assert node.has_parameter('planner.velocity')

            # Verify planning functions were created
            assert hasattr(node, 'plan_fn')
            assert hasattr(node, 'eval_fn')

            # Verify segments were planned
            assert hasattr(node, 'segments')

        except FileNotFoundError as e:
            # If racecourse YAML not found, skip this test
            pytest.skip(f'Racecourse YAML not found: {e}')
        finally:
            # Ensure node is destroyed even if test fails
            if 'node' in locals():
                node.destroy_node()

    def test_parameters(self, ros_context):
        """Test that node parameters are accessible."""
        try:
            node = DubinsGatePlannerNode()

            # Get parameter values
            turn_radius = node.get_parameter('turn_radius').value
            sample_points = node.get_parameter('sample_points').value
            altitude = node.get_parameter('planner.altitude').value
            velocity = node.get_parameter('planner.velocity').value

            # Verify reasonable defaults
            assert turn_radius > 0.0
            assert sample_points > 0
            assert altitude >= 0.0
            assert velocity > 0.0

        except FileNotFoundError:
            pytest.skip('Racecourse YAML not found')
        finally:
            if 'node' in locals():
                node.destroy_node()

    def test_dubins_functions_exist(self, ros_context):
        """Test that Dubins planning functions were created."""
        try:
            node = DubinsGatePlannerNode()

            # Verify functions are callable
            assert callable(node.plan_fn)
            assert callable(node.eval_fn)

        except FileNotFoundError:
            pytest.skip('Racecourse YAML not found')
        finally:
            if 'node' in locals():
                node.destroy_node()

    def test_trajectory_planning(self, ros_context):
        """Test that trajectory segments are generated."""
        try:
            node = DubinsGatePlannerNode()

            # Verify segments were created
            assert node.segments is not None
            assert isinstance(node.segments, list)

            # If gates exist, should have segments
            if hasattr(node, 'racecourse') and len(node.racecourse.gates) > 0:
                assert len(node.segments) > 0

        except FileNotFoundError:
            pytest.skip('Racecourse YAML not found')
        finally:
            if 'node' in locals():
                node.destroy_node()

    def test_gate_sequence_parameter(self, ros_context):
        """Test that gate sequence parameter is accessible."""
        try:
            node = DubinsGatePlannerNode()

            gate_sequence = node.get_parameter('planner.gate_sequence').value
            assert isinstance(gate_sequence, list)

        except FileNotFoundError:
            pytest.skip('Racecourse YAML not found')
        finally:
            if 'node' in locals():
                node.destroy_node()

    def test_reference_frame_parameter(self, ros_context):
        """Test that reference frame ID is set correctly."""
        try:
            node = DubinsGatePlannerNode()

            ref_frame = node.get_parameter('reference_frame_id').value
            assert isinstance(ref_frame, str)
            assert len(ref_frame) > 0

        except FileNotFoundError:
            pytest.skip('Racecourse YAML not found')
        finally:
            if 'node' in locals():
                node.destroy_node()


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
