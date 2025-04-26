import numpy as np
from dataclasses import dataclass, asdict
from typing import List

@dataclass
class Cone:
    center: np.ndarray  # shape (3,)
    extent_x: float
    extent_y: float
    aspect_ratio: float
    area: float
    count: int = 1

    def update(self, new_center, extent_x, extent_y, aspect_ratio, area):
        # Update with weighted average
        self.center = (self.center * self.count + new_center) / (self.count + 1)
        self.extent_x = (self.extent_x * self.count + extent_x) / (self.count + 1)
        self.extent_y = (self.extent_y * self.count + extent_y) / (self.count + 1)
        self.aspect_ratio = (self.aspect_ratio * self.count + aspect_ratio) / (self.count + 1)
        self.area = (self.area * self.count + area) / (self.count + 1)
        self.count += 1


class ConeMap:
    def __init__(self, position_tolerance=0.5):
        self.position_tolerance = position_tolerance
        self.cones: List[Cone] = []

    def process_cone(self, center, extent_x, extent_y, aspect_ratio, area):
        for cone in self.cones:
            if np.linalg.norm(cone.center - center) <= self.position_tolerance:
                cone.update(center, extent_x, extent_y, aspect_ratio, area)
                return

        # If no existing cone is close enough, add a new one
        new_cone = Cone(center=np.array(center), extent_x=extent_x, extent_y=extent_y,
                        aspect_ratio=aspect_ratio, area=area)
        self.cones.append(new_cone)

    def get_cones_summary(self):
        # Return a list of dicts with all cone data
        return [asdict(cone) for cone in self.cones]
