"""Vanilla Conflict-Based Search (CBS) planner for the warehouse grid."""

from __future__ import annotations

import heapq
from collections import defaultdict
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple


GridPos = Tuple[int, int]


@dataclass(frozen=True)
class AgentSpec:
    """Defines a single agent's planning problem on the grid."""

    agent_id: int
    start: GridPos
    goal: GridPos
    start_time: int = 0


@dataclass(frozen=True)
class Constraint:
    """Represents a vertex or edge constraint for an agent."""

    agent_id: Optional[int]
    time: int
    position: GridPos
    next_position: Optional[GridPos] = None  # Only used for edge constraints
    constraint_type: str = "vertex"  # Either "vertex" or "edge"


class CBSPlanner:
    """Simple implementation of the standard CBS algorithm."""

    def __init__(self, grid_map, max_time: int = 200):
        self.grid_map = grid_map
        self.max_time = max_time

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def plan_paths(self, agent_specs: List[AgentSpec]) -> Dict[int, List[GridPos]]:
        """Plan conflict-free paths for the provided set of agents.

        Args:
            agent_specs: Planning request for each agent.

        Returns:
            Mapping from agent_id to a list of grid cells (per discrete timestep).
        """

        agent_map = {spec.agent_id: spec for spec in agent_specs}
        root_constraints: List[Constraint] = []
        root_paths: Dict[int, List[GridPos]] = {}

        for spec in agent_specs:
            path = self._low_level_search(spec, root_constraints)
            if path is None:
                raise RuntimeError(f"No path found for agent {spec.agent_id} in root planning")
            root_paths[spec.agent_id] = path

        root_node = {
            "constraints": root_constraints,
            "paths": root_paths,
            "cost": self._compute_cost(root_paths),
        }

        open_list: List[Tuple[float, int, Dict]] = []
        counter = 0
        heapq.heappush(open_list, (root_node["cost"], counter, root_node))

        while open_list:
            _, _, node = heapq.heappop(open_list)
            conflict = self._find_conflict(node["paths"])
            if not conflict:
                return node["paths"]

            agent_a, agent_b = conflict["agents"]
            time = conflict["time"]
            pos = conflict["position"]

            for agent in (agent_a, agent_b):
                new_constraints = list(node["constraints"])
                if conflict["type"] == "vertex":
                    new_constraints.append(
                        Constraint(agent_id=agent, time=time, position=pos, constraint_type="vertex")
                    )
                else:  # edge conflict
                    frm = self._get_position_at_time(node["paths"][agent], time - 1)
                    to = self._get_position_at_time(node["paths"][agent], time)
                    new_constraints.append(
                        Constraint(
                            agent_id=agent,
                            time=time,
                            position=frm,
                            next_position=to,
                            constraint_type="edge",
                        )
                    )

                new_paths = dict(node["paths"])
                spec = agent_map[agent]
                replanned = self._low_level_search(spec, new_constraints)
                if replanned is None:
                    continue
                new_paths[agent] = replanned
                counter += 1
                heapq.heappush(
                    open_list,
                    (
                        self._compute_cost(new_paths),
                        counter,
                        {
                            "constraints": new_constraints,
                            "paths": new_paths,
                            "cost": self._compute_cost(new_paths),
                        },
                    ),
                )

        raise RuntimeError("CBS failed to find a conflict-free solution")

    def grid_path_to_world(self, path: List[GridPos]) -> List[Tuple[float, float]]:
        """Helper to convert grid indices back into world coordinates."""

        world_path: List[Tuple[float, float]] = []
        for cell in path:
            world_path.append(self.grid_map.grid_to_world(cell[0], cell[1]))
        return world_path

    # ------------------------------------------------------------------
    # CBS helpers
    # ------------------------------------------------------------------
    def _low_level_search(self, spec: AgentSpec, constraints: List[Constraint]) -> Optional[List[GridPos]]:
        """Run constrained A* for a single agent."""

        constraint_table = self._build_constraint_table(constraints, spec.agent_id)

        start_state = (spec.start[0], spec.start[1], spec.start_time)
        goal = spec.goal

        open_list: List[Tuple[float, int, Tuple[int, int, int]]] = []
        count = 0

        g_scores = {start_state: 0}
        f_score = self._heuristic(spec.start, goal) + spec.start_time
        heapq.heappush(open_list, (f_score, count, start_state))
        came_from: Dict[Tuple[int, int, int], Tuple[int, int, int]] = {}

        directions = [
            (1, 0),
            (-1, 0),
            (0, 1),
            (0, -1),
            (0, 0),  # wait in place
        ]

        while open_list:
            _, _, current = heapq.heappop(open_list)
            x, y, t = current

            if t > self.max_time:
                continue

            if (x, y) == goal and not self._violates_goal_constraint(goal, t, constraint_table):
                path = self._reconstruct_path(came_from, current)
                if spec.start_time > 0:
                    padding = [spec.start] * spec.start_time
                    path = padding + path
                return path

            for dx, dy in directions:
                nx, ny = x + dx, y + dy
                nt = t + 1

                if nt > self.max_time:
                    continue

                if not (0 <= nx < self.grid_map.cols and 0 <= ny < self.grid_map.rows):
                    continue

                if (dx, dy) != (0, 0) and self.grid_map.is_occupied(nx, ny):
                    continue

                if self._violates_constraints((x, y), (nx, ny), nt, constraint_table):
                    continue

                neighbor = (nx, ny, nt)
                tentative_g = g_scores[current] + 1

                if neighbor not in g_scores or tentative_g < g_scores[neighbor]:
                    g_scores[neighbor] = tentative_g
                    came_from[neighbor] = current
                    count += 1
                    priority = tentative_g + self._heuristic((nx, ny), goal)
                    heapq.heappush(open_list, (priority, count, neighbor))

        return None

    def _build_constraint_table(self, constraints: List[Constraint], agent_id: int):
        vertex = defaultdict(set)
        edge = defaultdict(set)

        for constraint in constraints:
            if constraint.agent_id is not None and constraint.agent_id != agent_id:
                continue
            if constraint.constraint_type == "vertex":
                vertex[constraint.position].add(constraint.time)
            else:
                key = (constraint.position, constraint.next_position)
                edge[key].add(constraint.time)

        return {"vertex": vertex, "edge": edge}

    def _violates_goal_constraint(self, goal: GridPos, time: int, table) -> bool:
        forbidden_times = table["vertex"].get(goal, set())
        return time in forbidden_times

    def _violates_constraints(self, current: GridPos, nxt: GridPos, new_time: int, table) -> bool:
        # Vertex constraint
        if new_time in table["vertex"].get(nxt, set()):
            return True

        # Edge constraint
        edge_key = (current, nxt)
        if new_time in table["edge"].get(edge_key, set()):
            return True

        return False

    def _reconstruct_path(self, came_from, current_state) -> List[GridPos]:
        path: List[GridPos] = []
        current = current_state
        while current in came_from:
            path.append((current[0], current[1]))
            current = came_from[current]
        path.append((current[0], current[1]))
        path.reverse()
        return path

    def _find_conflict(self, paths: Dict[int, List[GridPos]]):
        if not paths:
            return None

        max_len = max(len(p) for p in paths.values())

        agents = list(paths.keys())
        for i in range(len(agents)):
            for j in range(i + 1, len(agents)):
                a_id = agents[i]
                b_id = agents[j]
                path_a = paths[a_id]
                path_b = paths[b_id]

                for t in range(max_len):
                    pos_a = self._get_position_at_time(path_a, t)
                    pos_b = self._get_position_at_time(path_b, t)

                    if pos_a == pos_b:
                        return {"type": "vertex", "agents": (a_id, b_id), "time": t, "position": pos_a}

                    if t > 0:
                        prev_a = self._get_position_at_time(path_a, t - 1)
                        prev_b = self._get_position_at_time(path_b, t - 1)
                        if prev_a == pos_b and pos_a == prev_b:
                            return {
                                "type": "edge",
                                "agents": (a_id, b_id),
                                "time": t,
                                "position": pos_a,
                            }

        return None

    @staticmethod
    def _get_position_at_time(path: List[GridPos], time_step: int) -> GridPos:
        if time_step < 0:
            time_step = 0
        if time_step < len(path):
            return path[time_step]
        return path[-1]

    def _compute_cost(self, paths: Dict[int, List[GridPos]]) -> float:
        return sum(len(path) for path in paths.values())

    def _heuristic(self, node: GridPos, goal: GridPos) -> float:
        return abs(node[0] - goal[0]) + abs(node[1] - goal[1])
