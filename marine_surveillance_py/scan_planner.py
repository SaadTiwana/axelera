"""
Scan Area Planning and Division Algorithm
"""
import math
from typing import List, Tuple, Iterator
from config import ConfigManager, ScanAreaConfig


class ScanPosition:
    """Represents a scan position with azimuth and elevation"""

    def __init__(self, azimuth: float, elevation: float):
        self.azimuth = azimuth
        self.elevation = elevation

    def __repr__(self) -> str:
        try:
            if (isinstance(self.azimuth, (int, float)) and self.azimuth is not None and
                isinstance(self.elevation, (int, float)) and self.elevation is not None):
                return f"ScanPosition(az={self.azimuth:.2f} deg, el={self.elevation:.2f} deg)"
            else:
                return f"ScanPosition(az={self.azimuth}, el={self.elevation})"
        except (ValueError, TypeError):
            return f"ScanPosition(invalid)"

    def to_tuple(self) -> Tuple[float, float]:
        return (self.azimuth, self.elevation)


class ScanPlanner:
    """Plans and manages scan positions for maritime surveillance"""

    def __init__(self, config: ConfigManager):
        self.config = config
        self.scan_config = config.scan_area

    def generate_scan_positions(self) -> List[ScanPosition]:
        """Generate all scan positions based on configuration"""
        positions = []

        # Get camera FOV from config
        camera_config = self.config.camera
        fov_azimuth, fov_elevation = camera_config.fov_degrees

        # Calculate effective step sizes based on FOV and overlap
        azimuth_step = fov_azimuth * (1.0 - self.scan_config.min_overlap_percentage)
        elevation_step = fov_elevation * (1.0 - self.scan_config.min_overlap_percentage)

        # Calculate number of steps in each direction
        azimuth_range_size = self.scan_config.azimuth_range[1] - self.scan_config.azimuth_range[0]
        elevation_range_size = self.scan_config.elevation_range[1] - self.scan_config.elevation_range[0]

        azimuth_steps = max(1, int(azimuth_range_size / azimuth_step) + 1)
        elevation_steps = max(1, int(elevation_range_size / elevation_step) + 1)

        # Generate positions in vertical serpentine pattern (top-left to bottom-right)
        # Start from top-left, go down, then right, then up, then right, etc.
        for az_step in range(azimuth_steps):
            azimuth = self.scan_config.azimuth_range[0] + az_step * azimuth_step

            # For even columns, scan top to bottom; for odd columns, scan bottom to top
            if az_step % 2 == 0:
                # Even column: top to bottom (high elevation to low elevation)
                el_range = range(elevation_steps - 1, -1, -1)  # Start from highest elevation
            else:
                # Odd column: bottom to top (low elevation to high elevation)
                el_range = range(elevation_steps)  # Start from lowest elevation

            for el_step in el_range:
                elevation = self.scan_config.elevation_range[0] + el_step * elevation_step

                # Ensure we don't exceed the configured ranges
                azimuth = min(max(azimuth, self.scan_config.azimuth_range[0]), self.scan_config.azimuth_range[1])
                elevation = min(max(elevation, self.scan_config.elevation_range[0]), self.scan_config.elevation_range[1])

                positions.append(ScanPosition(azimuth, elevation))

        print(f"Generated {len(positions)} scan positions using FOV-based calculation")
        print(f"   Camera FOV: {fov_azimuth:.1f}° x {fov_elevation:.1f}°")
        print(f"   Min overlap: {self.scan_config.min_overlap_percentage * 100:.1f}%")
        print(f"   Calculated steps: azimuth={azimuth_step:.1f}°, elevation={elevation_step:.1f}°")
        print(f"   Azimuth range: {self.scan_config.azimuth_range[0]}° to {self.scan_config.azimuth_range[1]}°")
        print(f"   Elevation range: {self.scan_config.elevation_range[0]}° to {self.scan_config.elevation_range[1]}°")
        print(f"   Vertical serpentine grid: {azimuth_steps} columns x {elevation_steps} rows")
        print(f"   Pattern: Start top-left, alternate down/up per column")

        return positions

    def get_spiral_scan_positions(self) -> List[ScanPosition]:
        """Generate scan positions in a spiral pattern (more efficient for surveillance)"""
        positions = []
        center_az = (self.scan_config.azimuth_range[0] + self.scan_config.azimuth_range[1]) / 2
        center_el = (self.scan_config.elevation_range[0] + self.scan_config.elevation_range[1]) / 2

        # Calculate spiral parameters
        max_radius_az = (self.scan_config.azimuth_range[1] - self.scan_config.azimuth_range[0]) / 2
        max_radius_el = (self.scan_config.elevation_range[1] - self.scan_config.elevation_range[0]) / 2
        max_radius = min(max_radius_az, max_radius_el)

        # Generate spiral positions
        num_turns = 3
        points_per_turn = 8
        total_points = num_turns * points_per_turn

        for i in range(total_points):
            # Calculate angle and radius for this point
            angle = (i / points_per_turn) * 2 * math.pi
            radius_factor = i / (total_points - 1)  # 0 to 1
            radius = radius_factor * max_radius

            # Convert polar to cartesian
            azimuth = center_az + radius * math.cos(angle)
            elevation = center_el + radius * math.sin(angle)

            # Ensure within bounds
            azimuth = max(self.scan_config.azimuth_range[0],
                         min(self.scan_config.azimuth_range[1], azimuth))
            elevation = max(self.scan_config.elevation_range[0],
                           min(self.scan_config.elevation_range[1], elevation))

            positions.append(ScanPosition(azimuth, elevation))

        print(f"🌀 Generated {len(positions)} spiral scan positions")
        return positions

    def optimize_scan_order(self, positions: List[ScanPosition]) -> List[ScanPosition]:
        """Optimize scan order to minimize gimbal movement"""
        if not positions:
            return positions

        # Start from center position
        center_az = (self.scan_config.azimuth_range[0] + self.scan_config.azimuth_range[1]) / 2
        center_el = (self.scan_config.elevation_range[0] + self.scan_config.elevation_range[1]) / 2

        # Find closest position to center
        start_idx = min(range(len(positions)),
                       key=lambda i: self._distance(positions[i], ScanPosition(center_az, center_el)))

        # Use nearest neighbor algorithm for optimization
        optimized = [positions[start_idx]]
        remaining = positions[:start_idx] + positions[start_idx+1:]

        while remaining:
            # Find closest remaining position to last optimized position
            last_pos = optimized[-1]
            closest_idx = min(range(len(remaining)),
                            key=lambda i: self._distance(last_pos, remaining[i]))

            optimized.append(remaining[closest_idx])
            remaining.pop(closest_idx)

        print("🔄 Optimized scan order for minimal gimbal movement")
        return optimized

    def _distance(self, pos1: ScanPosition, pos2: ScanPosition) -> float:
        """Calculate angular distance between two positions"""
        az_diff = abs(pos1.azimuth - pos2.azimuth)
        el_diff = abs(pos1.elevation - pos2.elevation)

        # Use Euclidean distance in angular space
        return math.sqrt(az_diff**2 + el_diff**2)

    def filter_restricted_areas(self, positions: List[ScanPosition]) -> List[ScanPosition]:
        """Filter out positions that are in restricted areas"""
        filtered = []
        restricted_areas = self.config.restricted_areas

        for pos in positions:
            in_restricted = False
            for area in restricted_areas:
                if self._point_in_polygon(pos.to_tuple(), area.polygon):
                    in_restricted = True
                    break

            if not in_restricted:
                filtered.append(pos)

        if len(filtered) < len(positions):
            print(f"🚫 Filtered out {len(positions) - len(filtered)} positions in restricted areas")

        return filtered

    def _point_in_polygon(self, point: Tuple[float, float], polygon: List[Tuple[float, float]]) -> bool:
        """Check if point is inside polygon using ray casting algorithm"""
        x, y = point
        n = len(polygon)
        inside = False

        p1x, p1y = polygon[0]
        for i in range(1, n + 1):
            p2x, p2y = polygon[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y

        return inside

    def get_scan_statistics(self, positions: List[ScanPosition]) -> dict:
        """Get statistics about the scan plan"""
        if not positions:
            return {}

        azimuths = [p.azimuth for p in positions]
        elevations = [p.elevation for p in positions]

        # Calculate total angular movement
        total_movement = 0
        for i in range(1, len(positions)):
            total_movement += self._distance(positions[i-1], positions[i])

        return {
            'total_positions': len(positions),
            'azimuth_range': (min(azimuths), max(azimuths)),
            'elevation_range': (min(elevations), max(elevations)),
            'total_movement': total_movement,
            'avg_movement_per_step': total_movement / max(1, len(positions) - 1)
        }