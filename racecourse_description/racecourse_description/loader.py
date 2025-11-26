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
# fixed_wing_purt/racecourse/loader.py

import math
import os
from typing import List

from ament_index_python.packages import get_package_share_directory
import yaml

from .elements import Gate
from .elements import GenericModel


def resolve_package_path(path: str) -> str:
    """Resolve package:// URIs to filesystem paths."""
    if path.startswith('package://'):
        parts = path[len('package://'):].split('/', 1)
        pkg_name = parts[0]
        resource_path = parts[1] if len(parts) > 1 else ''
        pkg_share = get_package_share_directory(pkg_name)
        return os.path.join(pkg_share, resource_path)
    return path


class RacecourseLoader:
    def __init__(self, yaml_path: str):
        resolved_path = resolve_package_path(yaml_path)
        with open(resolved_path, 'r') as f:
            y = yaml.safe_load(f)

        self.frame_id = y.get('frame_id', 'map')
        meshes = y.get('meshes', {})

        # ---------------------------
        # Load Generic Models
        # ---------------------------
        self.generic: List[GenericModel] = []

        for obj in y.get('generic', []):
            # ensure 3D
            pos = obj['position']
            if len(pos) == 2:
                pos = (pos[0], pos[1], 0.0)

            # Convert rpy from degrees to radians
            rpy_deg = obj.get('rpy', (0.0, 0.0, 0.0))
            rpy_rad = tuple(math.radians(float(x)) for x in rpy_deg)

            self.generic.append(
                GenericModel(
                    name=obj['name'],
                    mesh=obj['mesh'],
                    position=tuple(float(x) for x in pos),
                    rpy=rpy_rad,
                    scale=tuple(float(x)
                                for x in obj.get('scale', (1.0, 1.0, 1.0))),
                    color=tuple(float(x)
                                for x in obj.get('color', (1.0, 1.0, 1.0))),
                    alpha=float(obj.get('alpha', 1.0)),
                )
            )

        # ---------------------------
        # Load Gates (semantic)
        # ---------------------------
        self.gates: List[Gate] = []

        for g in y.get('gates', []):
            center = g['center']
            if len(center) == 2:
                center = (center[0], center[1], 0.0)

            # Convert yaw from degrees to radians
            yaw_rad = math.radians(float(g['yaw']))

            self.gates.append(
                Gate(
                    name=g['name'],
                    mesh=meshes['gate'],
                    center=tuple(float(x) for x in center),
                    yaw=yaw_rad,
                    width=float(g.get('width', 2.0)),
                    scale=tuple(float(x)
                                for x in g.get('scale', (1.0, 1.0, 1.0))),
                )
            )
