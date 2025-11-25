"""Unit tests for RacecourseNode."""

import os
import tempfile

import pytest
import rclpy
from racecourse_description.racecourse_markers import RacecourseNode
from rclpy.executors import SingleThreadedExecutor


@pytest.fixture
def ros_context():
    """Initialize and cleanup ROS context for each test."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def temp_racecourse_yaml():
    """Create a temporary racecourse YAML file for testing."""
    yaml_content = """
frame_id: "map"

meshes:
  gate: "package://fixed_wing_purt/resources/meshes/gate.glb"

generic:
  - name: "start_finish_line"
    mesh: "package://fixed_wing_purt/resources/meshes/gate.glb"
    position: [0.0, 0.0, 0.0]
    rpy: [0.0, 0.0, 0.0]
    scale: [1.0, 1.0, 1.0]
    color: [1.0, 1.0, 1.0]
    alpha: 1.0

gates:
  - name: "gate_0"
    center: [10.0, 0.0, 2.0]
    yaw: 0.0
    width: 5.0
"""

    # Create temporary file
    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as f:
        f.write(yaml_content)
        temp_path = f.name

    yield temp_path

    # Cleanup
    if os.path.exists(temp_path):
        os.remove(temp_path)


class TestRacecourseNode:
    """Unit tests for the RacecourseNode class."""

    def test_node_initialization_with_valid_yaml(self, ros_context):
        """Test that RacecourseNode initializes correctly with valid YAML."""
        # Use the actual racecourse.yaml from resources/config
        import os
        import pathlib

        from racecourse_description import MarkerFactory, RacecourseLoader

        # Get the package root directory (test is in test/nodes/)
        test_dir = pathlib.Path(__file__).parent
        package_root = test_dir.parent.parent
        yaml_path = package_root / "config" / "racecourse.yaml"

        # Verify the file exists
        assert yaml_path.exists(), f"racecourse.yaml not found at {yaml_path}"

        # Change to package root so the default "racecourse.yaml" can be resolved
        # by creating a symlink in the current directory
        original_cwd = os.getcwd()
        try:
            os.chdir(package_root)

            # Create a temporary symlink to the actual file
            symlink_path = package_root / "racecourse.yaml"
            # Remove existing file/symlink if present
            if symlink_path.exists() or symlink_path.is_symlink():
                symlink_path.unlink()
            symlink_path.symlink_to(yaml_path)

            try:
                # Now create the node - it should find racecourse.yaml
                node = RacecourseNode()

                # Verify node was created correctly
                assert node.get_name() == "racecourse_markers_pub"
                assert node.loader is not None
                assert node.factory is not None

                node.destroy_node()
            finally:
                # Clean up symlink
                if symlink_path.exists() or symlink_path.is_symlink():
                    symlink_path.unlink()
        finally:
            os.chdir(original_cwd)

    def test_node_initialization_missing_yaml(self, ros_context):
        """Test that RacecourseNode raises error with missing YAML."""
        # This should raise FileNotFoundError
        with pytest.raises(FileNotFoundError):
            node = RacecourseNode()
            # Force parameter to non-existent file
            node.declare_parameter("course_yaml", "/nonexistent/file.yaml")
            # Try to access the parameter (would trigger load in __init__)
            yaml_path = node.get_parameter("course_yaml").value

            # In actual __init__, this would fail during loader creation
            # For this test, we verify the exception is raised

    def test_loader_initialization(self, ros_context, temp_racecourse_yaml):
        """Test that RacecourseLoader is initialized correctly."""
        # We can't easily test the full node without modifying how parameters work
        # But we can test the loader directly
        from racecourse_description import RacecourseLoader

        loader = RacecourseLoader(temp_racecourse_yaml)

        # Verify loader loaded the data
        assert hasattr(loader, "frame_id")
        assert loader.frame_id == "map"
        assert hasattr(loader, "gates")
        assert len(loader.gates) > 0
        assert hasattr(loader, "generic")

    def test_marker_factory_initialization(self, ros_context, temp_racecourse_yaml):
        """Test that MarkerFactory is initialized correctly."""
        from racecourse_description import MarkerFactory, RacecourseLoader

        loader = RacecourseLoader(temp_racecourse_yaml)
        factory = MarkerFactory(loader.frame_id)

        # Verify factory was created
        assert factory is not None

    def test_publish_method_exists(self, ros_context, temp_racecourse_yaml):
        """Test that publish method exists and can be called."""
        # Since we can't easily override parameters in __init__,
        # we'll create a modified test that doesn't instantiate the node
        # but verifies the publish logic would work

        from racecourse_description import MarkerFactory, RacecourseLoader
        from visualization_msgs.msg import MarkerArray

        loader = RacecourseLoader(temp_racecourse_yaml)
        factory = MarkerFactory(loader.frame_id)

        # Simulate what publish() does
        msg = MarkerArray()
        id_counter = 0

        # Add generic markers
        for obj in loader.generic:
            markers = obj.markers(factory, id_counter)
            assert isinstance(markers, list)
            for m in markers:
                msg.markers.append(m)
            id_counter += 10

        # Add gate markers
        for g in loader.gates:
            markers = g.markers(factory, id_counter)
            assert isinstance(markers, list)
            for m in markers:
                msg.markers.append(m)
            id_counter += 10

        # Verify we created markers
        assert len(msg.markers) > 0

    def test_parameter_declaration(self, ros_context):
        """Test that course_yaml parameter can be declared."""
        from rclpy.node import Node

        # Create a simple node to test parameter declaration
        node = Node("test_node")
        node.declare_parameter("course_yaml", "test.yaml")

        param_value = node.get_parameter("course_yaml").value
        assert param_value == "test.yaml"

        node.destroy_node()


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
