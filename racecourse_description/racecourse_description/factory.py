# fixed_wing_purt/racecourse/factory.py

import math
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, Quaternion
from std_msgs.msg import ColorRGBA, Header
from builtin_interfaces.msg import Duration
from tf_transformations import quaternion_from_euler


class MarkerFactory:
    def __init__(self, frame_id: str):
        self.frame_id = frame_id

    def header(self, stamp=None):
        """Create a Header with this factory's frame_id."""
        h = Header()
        h.frame_id = self.frame_id
        if stamp is not None:
            h.stamp = stamp
        return h

    def pose(self, position, heading=0.0):
        """Create a Pose from position [x,y,z] and heading (yaw in radians)."""
        p = Pose()
        p.position.x, p.position.y, p.position.z = position
        q = Quaternion()
        q.z = np.sin(heading / 2)
        q.w = np.cos(heading / 2)
        p.orientation = q
        return p

    @staticmethod
    def color(r, g, b, a=1.0):
        """Create a ColorRGBA."""
        c = ColorRGBA()
        c.r, c.g, c.b, c.a = r, g, b, a
        return c

    def mesh(self, marker_id, mesh, pos, rpy, scale, color, alpha):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.id = marker_id
        m.ns = "racecourse"
        m.type = Marker.MESH_RESOURCE
        m.action = Marker.ADD
        m.lifetime = Duration(sec=0)

        # --- Position ---
        x, y, z = [float(v) for v in pos]

        # --- Orientation (RPY) ---
        roll, pitch, yaw = [float(v) for v in rpy]
        q = quaternion_from_euler(roll, pitch, yaw)

        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z

        m.pose.orientation.x = q[0]
        m.pose.orientation.y = q[1]
        m.pose.orientation.z = q[2]
        m.pose.orientation.w = q[3]

        # --- Mesh ---
        m.mesh_resource = mesh
        m.mesh_use_embedded_materials = True

        # --- Scale ---
        sx, sy, sz = [float(v) for v in scale]
        m.scale.x = sx
        m.scale.y = sy
        m.scale.z = sz

        # --- Color ---
        r, g, b = [float(v) for v in color]
        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = float(alpha)

        return m

    def direction_arrow(self, marker_id, origin, yaw, length=2.0, height=2.0):
        x, y, z = origin

        dx = math.cos(yaw) * length
        dy = math.sin(yaw) * length

        m = Marker()
        m.header.frame_id = self.frame_id
        m.id = marker_id
        m.ns = "gate_direction"
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.lifetime = Duration(sec=0)

        m.points = [
            Point(x=float(x), y=float(y), z=float(z + height)),
            Point(x=float(x + dx), y=float(y + dy), z=float(z + height)),
        ]

        m.scale.x = 0.3
        m.scale.y = 0.7
        m.scale.z = 0.7

        m.color.r = 0.0
        m.color.g = 0.4
        m.color.b = 1.0
        m.color.a = 1.0

        return m

    def line_strip(
        self, marker_id, points, stamp=None, ns="line_strip", line_width=0.1, color=None
    ):
        """Create a LINE_STRIP marker from a list of points.

        Args:
            marker_id: Marker ID
            points: List of Point objects or list of [x, y, z] coordinates
            stamp: Optional timestamp for header
            ns: Namespace for the marker
            line_width: Width of the line (scale.x)
            color: ColorRGBA object or None for default (white)
        """
        m = Marker()
        m.header = self.header(stamp)
        m.id = marker_id
        m.ns = ns
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = line_width

        if color is None:
            m.color = self.color(1.0, 1.0, 1.0, 1.0)
        else:
            m.color = color

        # Handle both Point objects and coordinate lists
        if points and isinstance(points[0], Point):
            m.points = points
        else:
            m.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in points]

        return m

    def arrow_pose(
        self,
        marker_id,
        position,
        heading,
        stamp=None,
        ns="arrow",
        length=1.0,
        width=0.15,
        color=None,
    ):
        """Create an ARROW marker using pose (good for heading visualization).

        Args:
            marker_id: Marker ID
            position: [x, y, z] position
            heading: Yaw angle in radians
            stamp: Optional timestamp for header
            ns: Namespace for the marker
            length: Arrow length (scale.x)
            width: Arrow width (scale.y and scale.z)
            color: ColorRGBA object or None for default (green)
        """
        m = Marker()
        m.header = self.header(stamp)
        m.id = marker_id
        m.ns = ns
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose = self.pose(position, heading)
        m.scale.x = length
        m.scale.y = width
        m.scale.z = width

        if color is None:
            m.color = self.color(0.0, 1.0, 0.0, 0.8)
        else:
            m.color = color

        return m

    def circle(
        self,
        marker_id,
        center,
        radius,
        z=0.0,
        stamp=None,
        ns="circle",
        line_width=0.1,
        color=None,
        num_points=51,
    ):
        """Create a circular LINE_STRIP marker.

        Args:
            marker_id: Marker ID
            center: [x, y] center position
            radius: Circle radius
            z: Height of the circle
            stamp: Optional timestamp for header
            ns: Namespace for the marker
            line_width: Width of the line (scale.x)
            color: ColorRGBA object or None for default (gray)
            num_points: Number of points to approximate the circle
        """
        angles = np.linspace(0, 2 * np.pi, num_points)
        points = [
            Point(x=center[0] + radius * np.cos(a), y=center[1] + radius * np.sin(a), z=z)
            for a in angles
        ]

        if color is None:
            color = self.color(0.5, 0.5, 0.5, 1.0)

        return self.line_strip(
            marker_id, points, stamp=stamp, ns=ns, line_width=line_width, color=color
        )
