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
# fixed_wing_purt/racecourse/elements.py

from dataclasses import dataclass
from typing import Tuple

from .factory import MarkerFactory


@dataclass
class GenericModel:
    name: str
    mesh: str
    position: Tuple[float, float, float]
    rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    scale: Tuple[float, float, float] = (1.0, 1.0, 1.0)
    color: Tuple[float, float, float] = (1.0, 1.0, 1.0)
    alpha: float = 1.0

    def markers(self, factory: MarkerFactory, base_id: int):
        return [
            factory.mesh(
                marker_id=base_id,
                mesh=self.mesh,
                pos=self.position,
                rpy=self.rpy,
                scale=self.scale,
                color=self.color,
                alpha=self.alpha,
            )
        ]


@dataclass
class Gate:
    name: str
    mesh: str
    center: Tuple[float, float, float]
    yaw: float
    width: float = 2.0
    scale: Tuple[float, float, float] = (1.0, 1.0, 1.0)

    def markers(self, factory: MarkerFactory, base_id: int):
        gate_marker = factory.mesh(
            marker_id=base_id,
            mesh=self.mesh,
            pos=self.center,
            rpy=(0.0, 0.0, self.yaw),
            scale=self.scale,
            color=(1.0, 1.0, 1.0),
            alpha=1.0,
        )

        arrow_marker = factory.direction_arrow(
            marker_id=base_id + 1000, origin=self.center, yaw=self.yaw
        )

        return [gate_marker, arrow_marker]
