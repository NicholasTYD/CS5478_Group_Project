import numpy as np
import heapq
from typing import List, Tuple, Optional

class GridMap:
    """
    Represents the warehouse as a 2D occupancy grid.
    """
    def __init__(self, rows, cols, cell_size=1.0, offset=(0, 0)):
        """
        Args:
            rows: Number of grid rows
            cols: Number of grid columns
            cell_size: Size of each grid cell in meters (default 1.0m)
            offset: (x_offset, y_offset) to align grid with world coordinates
        """
        self.rows = rows
        self.cols = cols
        self.cell_size = cell_size
        self.offset = offset  # (x_offset, y_offset)

        # 0 = free, 1 = occupied
        self.grid = np.zeros((rows, cols), dtype=np.uint8)

    def set_occupied(self, grid_x, grid_y):
        """Mark a grid cell as occupied."""
        if 0 <= grid_x < self.cols and 0 <= grid_y < self.rows:
            self.grid[grid_y, grid_x] = 1

    def set_unoccupied(self, grid_x, grid_y):
        """Mark a grid cell as occupied."""
        if 0 <= grid_x < self.cols and 0 <= grid_y < self.rows:
            self.grid[grid_y, grid_x] = 0

    def is_occupied(self, grid_x, grid_y):
        """Check if a grid cell is occupied."""
        if 0 <= grid_x < self.cols and 0 <= grid_y < self.rows:
            return self.grid[grid_y, grid_x] == 1
        return True  # Out of bounds is considered occupied

    def world_to_grid(self, world_x, world_y):
        """
        Convert world coordinates to grid indices.

        Args:
            world_x, world_y: Position in world coordinates (meters)

        Returns:
            (grid_x, grid_y): Grid cell indices
        """
        # The grid is centered, so we need to adjust
        # Grid coordinates go from -cols/2 to +cols/2 in world space
        # The -0.5 offset accounts for the +0.5 offset used in URDF generation
        grid_x = int(np.round((world_x - self.offset[0]) / self.cell_size + self.cols / 2 - 0.5))
        grid_y = int(np.round((world_y - self.offset[1]) / self.cell_size + self.rows / 2 - 0.5))
        return grid_x, grid_y

    def grid_to_world(self, grid_x, grid_y):
        """
        Convert grid indices to world coordinates (center of cell).

        Args:
            grid_x, grid_y: Grid cell indices

        Returns:
            (world_x, world_y): Position in world coordinates (meters)
        """
        # The +0.5 offset aligns grid cells with URDF box positions
        world_x = (grid_x - self.cols / 2 + 0.5) * self.cell_size + self.offset[0]
        world_y = (grid_y - self.rows / 2 + 0.5) * self.cell_size + self.offset[1]
        return world_x, world_y

    def add_obstacles_from_positions(self, obstacle_positions, inflation_radius=0):
        """
        Mark grid cells as occupied based on world coordinate positions.

        Args:
            obstacle_positions: List/array of [x, y, z] positions
            inflation_radius: Inflate obstacles by this many cells (for robot radius)
        """
        for pos in obstacle_positions:
            grid_x, grid_y = self.world_to_grid(pos[0], pos[1])
            self.set_occupied(grid_x, grid_y)

            # Inflate obstacle by radius (for robot collision avoidance)
            if inflation_radius > 0:
                for dx in range(-inflation_radius, inflation_radius + 1):
                    for dy in range(-inflation_radius, inflation_radius + 1):
                        # Use circular inflation (Euclidean distance)
                        if dx*dx + dy*dy <= inflation_radius*inflation_radius:
                            self.set_occupied(grid_x + dx, grid_y + dy)

    def print_grid(self):
        """Print ASCII representation of the grid."""
        for y in range(self.rows):
            row = ""
            for x in range(self.cols):
                row += "#" if self.grid[y, x] == 1 else "."
            print(row)


class AStarPlanner:
    """
    A* pathfinding algorithm for grid-based navigation.
    """

    def __init__(self, grid_map: GridMap):
        """
        Args:
            grid_map: GridMap object representing the environment
        """
        self.grid_map = grid_map

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """
        Manhattan distance heuristic (good for 4-connected grid).
        For 8-connected grid, could use Euclidean or Chebyshev distance.

        Args:
            a, b: Grid coordinates (x, y)

        Returns:
            Estimated distance between a and b
        """
        # Euclidean distance for smoother paths with 8-connected movement
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def get_neighbors(self, node: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        Get valid neighboring cells (8-connected grid).

        Args:
            node: Current grid cell (x, y)

        Returns:
            List of valid neighbor cells
        """
        x, y = node
        neighbors = []

        # 8-connected grid: N, S, E, W, NE, NW, SE, SW
        directions = [
            (0, 1),   # North
            (0, -1),  # South
            (1, 0),   # East
            (-1, 0),  # West
            (1, 1),   # NE
            (-1, 1),  # NW
            (1, -1),  # SE
            (-1, -1), # SW
        ]

        for dx, dy in directions:
            nx, ny = x + dx, y + dy

            # Check bounds and occupancy
            if (0 <= nx < self.grid_map.cols and
                0 <= ny < self.grid_map.rows and
                not self.grid_map.is_occupied(nx, ny)):
                neighbors.append((nx, ny))

        return neighbors

    def plan(self, start_world: Tuple[float, float],
             goal_world: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        """
        Plan a path from start to goal using A*.

        Args:
            start_world: Start position in world coordinates (x, y)
            goal_world: Goal position in world coordinates (x, y)

        Returns:
            List of waypoints in world coordinates, or None if no path found
        """
        # Convert to grid coordinates
        start_grid = self.grid_map.world_to_grid(start_world[0], start_world[1])
        goal_grid = self.grid_map.world_to_grid(goal_world[0], goal_world[1])

        # Check if start or goal is occupied
        if self.grid_map.is_occupied(*start_grid):
            print(f"Warning: Start position {start_grid} is occupied!")
            # Try to find nearest free cell
            start_grid = self._find_nearest_free_cell(start_grid)
            if start_grid is None:
                return None

        if self.grid_map.is_occupied(*goal_grid):
            print(f"Warning: Goal position {goal_grid} is occupied!")
            goal_grid = self._find_nearest_free_cell(goal_grid)
            if goal_grid is None:
                return None

        # A* algorithm
        open_set = []
        heapq.heappush(open_set, (0, start_grid))

        came_from = {}
        g_score = {start_grid: 0}
        f_score = {start_grid: self.heuristic(start_grid, goal_grid)}

        while open_set:
            _, current = heapq.heappop(open_set)

            # Goal reached
            if current == goal_grid:
                return self._reconstruct_path(came_from, current)

            for neighbor in self.get_neighbors(current):
                # Cost for diagonal movement is sqrt(2), orthogonal is 1
                dx = abs(neighbor[0] - current[0])
                dy = abs(neighbor[1] - current[1])
                move_cost = 1.414 if (dx + dy) == 2 else 1.0

                tentative_g_score = g_score[current] + move_cost

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal_grid)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        # No path found
        print(f"No path found from {start_grid} to {goal_grid}")
        return None

    def _find_nearest_free_cell(self, grid_pos: Tuple[int, int]) -> Optional[Tuple[int, int]]:
        """Find the nearest free cell to a given position."""
        x, y = grid_pos
        max_search_radius = 5

        for radius in range(1, max_search_radius + 1):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if abs(dx) == radius or abs(dy) == radius:  # Only check perimeter
                        nx, ny = x + dx, y + dy
                        if (0 <= nx < self.grid_map.cols and
                            0 <= ny < self.grid_map.rows and
                            not self.grid_map.is_occupied(nx, ny)):
                            return (nx, ny)
        return None

    def _reconstruct_path(self, came_from: dict, current: Tuple[int, int]) -> List[Tuple[float, float]]:
        """
        Reconstruct path from A* came_from dict and convert to world coordinates.

        Args:
            came_from: Dictionary mapping each node to its parent
            current: Goal node

        Returns:
            List of waypoints in world coordinates
        """
        path_grid = [current]
        while current in came_from:
            current = came_from[current]
            path_grid.append(current)

        path_grid.reverse()

        # Convert to world coordinates
        path_world = []
        for grid_x, grid_y in path_grid:
            world_x, world_y = self.grid_map.grid_to_world(grid_x, grid_y)
            path_world.append((world_x, world_y))

        return path_world


def create_warehouse_grid(rows, cols, wall_pos, shelves_pos, cell_size=1.0, offset=(0, 0), robot_radius=0.3,
                          endpoints_pos=None, work_stns_pos=None):
    """
    Create a GridMap from warehouse structure positions.

    Args:
        rows, cols: Grid dimensions
        wall_pos: Array of wall positions
        shelves_pos: Array of shelf positions
        cell_size: Size of each grid cell (default 1.0m)
        offset: (x_offset, y_offset) tuple
        robot_radius: Ignored (kept for backward compatibility)
        endpoints_pos: Ignored (kept for backward compatibility)
        work_stns_pos: Ignored (kept for backward compatibility)

    Returns:
        GridMap object with only actual obstacle cells marked (no inflation)
    """
    # Add padding for walls
    grid_map = GridMap(rows + 2, cols + 2, cell_size, offset)

    # SIMPLE APPROACH: No inflation, only actual obstacle cells are blocked
    # Robot radius and collision detection handled by PyBullet physics
    # Grid is just for high-level pathfinding
    grid_map.add_obstacles_from_positions(wall_pos, inflation_radius=0)
    grid_map.add_obstacles_from_positions(shelves_pos, inflation_radius=0)
    grid_map.add_obstacles_from_positions(work_stns_pos, inflation_radius=0)

    return grid_map
