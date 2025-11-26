#!/usr/bin/env python3
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
"""Visualization marker utilities for force and moment vectors."""

from geometry_msgs.msg import Point
import numpy as np
import rclpy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


def create_force_arrow(
    marker_id: int,
    name: str,
    vec: np.ndarray,
    color: ColorRGBA,
    scale: float = 1.0,
    offset: tuple[float, float, float] = (0.0, 0.0, 0.0),
    frame_id: str = 'vehicle',
) -> Marker | None:
    """
    Create arrow marker for force vector.

    Parameters
    ----------
    marker_id : int
        Unique marker ID
    name : str
        Marker name (not currently used but kept for consistency)
    vec : np.ndarray
        Force vector (3D) in Newtons
    color : ColorRGBA
        Marker color
    scale : float, optional
        Scale factor (meters per Newton), by default 1.0
    offset : tuple[float, float, float], optional
        Offset from origin, by default (0.0, 0.0, 0.0)
    frame_id : str, optional
        Reference frame, by default 'vehicle'

    Returns
    -------
    Marker | None
        Arrow marker, or None if magnitude too small

    """
    magnitude = np.linalg.norm(vec)

    # Skip markers for very small forces to avoid rendering issues
    if magnitude < 1e-6:
        return None

    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rclpy.time.Time().to_msg()  # Frame locking enabled
    marker.ns = 'forces'
    marker.id = marker_id
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.frame_locked = True  # Enable frame locking in RViz2

    # Arrow from origin to scaled endpoint
    start = Point(x=offset[0], y=offset[1], z=offset[2])
    end = Point(
        x=offset[0] + vec[0] * scale,
        y=offset[1] + vec[1] * scale,
        z=offset[2] + vec[2] * scale,
    )
    marker.points = [start, end]

    # Arrow shaft and head dimensions
    marker.scale.x = 0.03  # Shaft diameter (3cm)
    marker.scale.y = 0.06  # Head diameter (6cm)
    marker.scale.z = 0.0  # Not used for ARROW type with points

    marker.color = color
    marker.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()

    return marker


def create_moment_arc(
    marker_id: int,
    name: str,
    vec: np.ndarray,
    color: ColorRGBA,
    scale: float = 1.0,
    frame_id: str = 'vehicle',
) -> Marker | None:
    """
    Create circular arc marker for moment vector using LINE_STRIP.

    The arc is drawn perpendicular to the moment vector (right-hand rule).
    Radius proportional to moment magnitude.

    Parameters
    ----------
    marker_id : int
        Unique marker ID
    name : str
        Marker name (not currently used but kept for consistency)
    vec : np.ndarray
        Moment vector (3D) in N·m
    color : ColorRGBA
        Marker color
    scale : float, optional
        Scale factor (meters per N·m), by default 1.0
    frame_id : str, optional
        Reference frame, by default 'vehicle'

    Returns
    -------
    Marker | None
        Arc marker, or None if magnitude too small

    """
    magnitude = np.linalg.norm(vec)

    # Skip markers for very small moments
    if magnitude < 1e-6:
        return None

    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rclpy.time.Time().to_msg()
    marker.ns = 'moments'
    marker.id = marker_id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.frame_locked = True

    # Normalize moment vector to get rotation axis
    axis = vec / magnitude

    # Find two perpendicular vectors to create the arc plane
    # Choose an arbitrary vector not parallel to axis
    if abs(axis[0]) < 0.9:
        arbitrary = np.array([1.0, 0.0, 0.0])
    else:
        arbitrary = np.array([0.0, 1.0, 0.0])

    # Create orthonormal basis in the plane perpendicular to axis
    u = np.cross(axis, arbitrary)
    u = u / np.linalg.norm(u)
    v = np.cross(axis, u)

    # Arc radius proportional to moment magnitude
    radius = magnitude * scale

    # Create arc points (270 degrees, showing rotation direction)
    num_points = 30
    angle_range = 3.0 * np.pi / 2.0  # 270 degrees

    points = []
    for i in range(num_points):
        angle = (i / (num_points - 1)) * angle_range
        # Point on circle in the perpendicular plane
        point = radius * (np.cos(angle) * u + np.sin(angle) * v)
        points.append(
            Point(x=float(point[0]), y=float(point[1]), z=float(point[2]))
        )

    # Add arrowhead at the end by creating small segments
    last_point = points[-1]
    # Direction tangent to arc at end point
    tangent = -np.sin(angle_range) * u + np.cos(angle_range) * v
    tangent = tangent / np.linalg.norm(tangent)

    # Arrow tip perpendicular to tangent
    arrow_left = np.cross(axis, tangent) * radius * 0.1
    arrow_right = -arrow_left

    # Add arrow tip lines
    tip_center = np.array([last_point.x, last_point.y, last_point.z])
    points.append(
        Point(
            x=float(tip_center[0] + arrow_left[0]),
            y=float(tip_center[1] + arrow_left[1]),
            z=float(tip_center[2] + arrow_left[2]),
        )
    )
    points.append(last_point)
    points.append(
        Point(
            x=float(tip_center[0] + arrow_right[0]),
            y=float(tip_center[1] + arrow_right[1]),
            z=float(tip_center[2] + arrow_right[2]),
        )
    )

    marker.points = points

    # Line width - make thicker for visibility
    marker.scale.x = 0.03  # Line width (3cm)

    marker.color = color
    marker.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()

    return marker
