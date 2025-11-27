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
"""Tests for cubs2_control controllers."""

from cubs2_control.autolevel_controller import autolevel_controller
from cubs2_control.pid_controller import pid_controller


class TestControllers:
    """Test controller model creation."""

    def test_autolevel_controller_creation(self):
        """Test that autolevel controller can be created."""
        model = autolevel_controller()
        assert model is not None
        assert model.name == 'AutolevelController'
        assert 'i_p' in model.state_names
        assert 'i_q' in model.state_names

    def test_pid_controller_creation(self):
        """Test that PID controller can be created."""
        model = pid_controller()
        assert model is not None
        assert model.name == 'PIDController'
        assert 'i_roll' in model.state_names
        assert 'i_pitch' in model.state_names
        assert 'i_yaw' in model.state_names
        assert 'i_speed' in model.state_names
