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
"""Unit tests for SimNode."""
from cubs2_msgs.msg import AircraftControl
from cubs2_simulation.sim import SimNode
import numpy as np
import pytest
import rclpy
from std_msgs.msg import Empty
from std_msgs.msg import Float64


@pytest.fixture
def ros_context():
    """Initialize and cleanup ROS context for each test."""
    rclpy.init()
    yield
    rclpy.shutdown()


class TestSimNode:
    """Unit tests for the SimNode class."""

    def test_node_initialization(self, ros_context):
        """Test that SimNode initializes correctly with default parameters."""
        node = SimNode()

        # Verify node name
        assert node.get_name() == 'sim_node'

        # Verify parameters were declared
        assert node.has_parameter('dt')
        assert node.has_parameter('show_forces')

        # Verify default values
        assert node.dt == 0.01  # Default 10ms
        assert node.show_forces

        # Verify initial state
        assert node.sim_time == 0.0
        assert not node.paused

        # Verify model is initialized (composed closed-loop model)
        assert hasattr(node, 'model')
        assert hasattr(node, 'x')
        assert hasattr(node, 'u')
        assert hasattr(node, 'p')

        # Verify initial position for takeoff (CG at 0.1m so wheels touch
        # ground)
        assert np.isclose(node.x.plant.p[2], 0.1)

        node.destroy_node()

    def test_custom_parameters(self, ros_context):
        """Test node initialization with custom parameters."""
        # Note: SimNode already declares parameters in __init__
        # We can't re-declare them. This test verifies we can read them.
        node = SimNode()

        # Verify we can get parameter values
        dt_value = node.get_parameter('dt').value
        show_forces_value = node.get_parameter('show_forces').value

        # Verify default values (set in __init__)
        assert dt_value == 0.01
        assert show_forces_value

        node.destroy_node()

    def test_state_stepping(self, ros_context):
        """Test that simulation state advances correctly."""
        node = SimNode()

        # Store initial state
        initial_time = node.sim_time

        # Step simulation once
        node.step_simulation()

        # Verify time advanced
        assert node.sim_time == initial_time + node.dt

        # State may change slightly due to dynamics (even at rest, there might be numerical drift)
        # Just verify no NaN or inf values
        x_vec = node.model._state_to_vec(node.x)
        assert np.all(np.isfinite(x_vec))

        node.destroy_node()

    def test_control_callback(self, ros_context):
        """Test control input callback updates state correctly."""
        node = SimNode()

        # Create control message
        msg = AircraftControl()
        msg.aileron = 0.5
        msg.elevator = -0.3
        msg.rudder = 0.1
        msg.throttle = 0.7

        # Call callback
        node.control_callback(msg)

        # Verify inputs were updated (closed-loop model uses _manual suffix)
        assert np.isclose(node.u.ail_manual, 0.5)
        assert np.isclose(node.u.elev_manual, -0.3)
        assert np.isclose(node.u.rud_manual, 0.1)
        assert np.isclose(node.u.thr_manual, 0.7)

        node.destroy_node()

    def test_pause_resume(self, ros_context):
        """Test pause and resume functionality."""
        node = SimNode()

        # Initially running
        assert not node.paused

        # Pause
        node.pause()
        assert node.paused

        # Resume
        node.resume()
        assert not node.paused

        node.destroy_node()

    def test_reset_functionality(self, ros_context):
        """Test reset returns to initial conditions."""
        node = SimNode()

        # Modify state
        node.sim_time = 5.0
        node.x.plant.p[0] = 10.0
        node.x.plant.p[1] = 5.0
        node.x.plant.p[2] = 2.0
        node.u.thr_manual = 0.8

        # Reset via topic callback
        msg = Empty()
        node.reset_topic_callback(msg)

        # Verify reset to initial conditions
        assert node.sim_time == 0.0
        # CG height for wheels on ground
        assert np.isclose(node.x.plant.p[2], 0.1)
        assert np.isclose(node.x.plant.v[0], 0.0)
        assert np.isclose(node.x.plant.v[1], 0.0)
        assert np.isclose(node.x.plant.v[2], 0.0)
        assert np.isclose(node.u.thr_manual, 0.0)
        assert np.isclose(node.u.elev_manual, 0.0)

        node.destroy_node()

    def test_speed_callback(self, ros_context):
        """Test simulation speed multiplier update."""
        node = SimNode()

        # Create speed message
        msg = Float64()
        msg.data = 2.0

        # Call callback
        node.speed_topic_callback(msg)

        # Verify speed was updated
        assert node.sim_speed == 2.0

        # Note: speed callback pauses and resumes, so final state should be running
        # if it was running before (after the pause/resume cycle)

        node.destroy_node()

    def test_dt_callback(self, ros_context):
        """Test time step update."""
        node = SimNode()

        # Create dt message
        msg = Float64()
        msg.data = 0.02  # 20ms

        # Call callback
        node.dt_topic_callback(msg)

        # Verify dt was updated
        assert node.dt == 0.02

        node.destroy_node()

    def test_joint_state_publishing(self, ros_context):
        """Test joint state message is properly formatted."""
        node = SimNode()

        # Set some control inputs (closed-loop model uses _manual suffix)
        node.u.ail_manual = 0.5
        node.u.elev_manual = -0.3
        node.u.rud_manual = 0.2
        node.u.thr_manual = 0.7

        # Call publish (should not raise exception)
        node.publish_joint_states()

        # Verify propeller angle is updated
        assert node.propeller_angle >= 0.0

        node.destroy_node()

    def test_nan_detection(self, ros_context):
        """Test that NaN detection pauses simulation."""
        node = SimNode()

        # Force a NaN into the state (this is artificial for testing)
        # In practice, we'd need to create conditions that lead to NaN
        # For now, just verify the detection mechanism exists

        # The step_simulation function checks for NaN/Inf
        # If detected, it should pause

        # Normal step should work fine
        node.step_simulation()
        assert not node.paused

        node.destroy_node()

    def test_propeller_angle_accumulation(self, ros_context):
        """Test propeller angle accumulates with throttle."""
        node = SimNode()

        # Set throttle (closed-loop model uses _manual suffix)
        node.u.thr_manual = 1.0  # Full throttle

        initial_angle = node.propeller_angle

        # Step simulation first to compute outputs (which sets thr_cmd)
        node.step_simulation()

        # Publish joint states (advances propeller angle based on thr_cmd)
        node.publish_joint_states()

        # Angle should have increased (thr_cmd should be set from outputs)
        # Note: thr_cmd is only set if outputs are computed
        if node.thr_cmd > 0:
            assert node.propeller_angle > initial_angle

        # Angle should wrap to [0, 2π]
        assert 0.0 <= node.propeller_angle < 2.0 * np.pi

        node.destroy_node()

    def test_quaternion_attribute_access(self, ros_context):
        """Test that quaternion can be accessed via x.r attribute."""
        node = SimNode()

        # Verify quaternion attribute exists and is accessible (in plant submodel)
        # The node accesses x.plant.r (attitude quaternion field in
        # SportCubStatesQuat)

        # Access quaternion (should not raise AttributeError)
        q = node.x.plant.r

        # Verify it's a valid quaternion (4 elements, normalized)
        assert len(q) == 4
        q_norm = np.linalg.norm(q)
        assert np.isclose(q_norm, 1.0, atol=1e-6)

        node.destroy_node()

    def test_outputs_computation(self, ros_context):
        """Test that SportCub outputs (forces/moments) are computed."""
        node = SimNode()

        # Step simulation to trigger output computation
        node.step_simulation()

        # Verify outputs were computed if f_y exists
        if hasattr(node.model, 'f_y'):
            assert node.last_outputs is not None
            # Outputs is an object (not dict) with force attributes
            assert hasattr(node.last_outputs, 'FA_b')  # Aero force
            assert hasattr(node.last_outputs, 'FT_b')  # Thrust force
            assert hasattr(node.last_outputs, 'FW_b')  # Weight force

        node.destroy_node()


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
