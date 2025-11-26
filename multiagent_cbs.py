"""Vanilla Conflict-Based Search (CBS) planner for the warehouse grid."""

from __future__ import annotations

import heapq
from collections import defaultdict
from dataclasses import dataclass
from pprint import pprint
from typing import Dict, List, Optional, Tuple

from pathfinding import GridMap


GridPos = Tuple[int, int]


@dataclass(frozen=True)
class AgentSpec:
    """Defines a single agent's planning problem on the grid."""

    agent_id: int
    start: GridPos
    goal: GridPos
    grid_map: GridMap
    start_time: int = 0


@dataclass(frozen=True)
class Constraint:
    """Represents a vertex or edge constraint for an agent."""

    agent_id: Optional[int]
    time: int
    position: GridPos
    next_position: Optional[GridPos] = None  # Only used for edge constraints
    constraint_type: str = "vertex"  # Either "vertex" or "edge"

    def __eq__(self, other):
        # Equality comparison (for checking duplicates)
        if not isinstance(other, Constraint):
            return False
        return (self.agent_id == other.agent_id and
                self.time == other.time and
                self.position == other.position and
                self.constraint_type == other.constraint_type and
                self.next_position == other.next_position)
    
    def __lt__(self, other):
        # Less-than comparison for ordering in a heap or sorted list
        if not isinstance(other, Constraint):
            return NotImplemented
        # Example comparison based on (agent_id, time, position)
        if self.time != other.time:
            return self.time < other.time
        if self.agent_id != other.agent_id:
            return self.agent_id < other.agent_id
        if self.position != other.position:
            return self.position < other.position
        return False  # For cases where the constraints are otherwise identical


class CBSPlanner:
    """Simple implementation of the standard CBS algorithm."""

    def __init__(self, algo, all_work_stn_grid_pos:set[GridPos], max_time: int = 200, allow_backtrack=True):
        '''
        Algo is either 'astar', 'cbs' or 'shy_cbs'
        '''

        valid_algos = ('astar', 'cbs', 'shy_cbs')
        assert algo in valid_algos, f'CBSPlanner received wrong argument for algo. Needs to be one of {valid_algos}, got {algo}'

        self.all_work_stn_grid_pos = all_work_stn_grid_pos

        self.max_time = max_time
        self.total_conflicts_resolved = 0
        self.total_nodes_expanded = 0

        self.allow_backtrack = allow_backtrack

        self.algo = algo
        self.is_shy_cbs = algo == 'shy_cbs'

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def plan_paths(self, agent_specs: List[AgentSpec]) -> Dict[int, List[GridPos]]:
        """Plan conflict-free paths for the provided set of agents.

        Args:
            agent_specs: Planning request for each agent.



            DISABLED FEATURED BECAUSE BUGGY (If a node is close to a goal it might falsefully terminate):
            max_path_multiplier_cost: Highest path cost to accomodate based on the agent's shortest possible cost.
            E.g. If agent's shortest path is 10 and multiplier is 2, the algo will never return a path of above
            20 for that agent.

        Returns:
            Mapping from agent_id to a list of grid cells (per discrete timestep).
        """

        agent_map = {spec.agent_id: spec for spec in agent_specs}
        root_constraints: set[Constraint] = set()
        root_paths: Dict[int, List[GridPos]] = {}

        shortest_agent_costs: dict[int, int] = {}
        for spec in agent_specs:
            path = self._low_level_search(spec, root_constraints, spec.grid_map, 
                                          allow_backtrack=self.allow_backtrack,
                                          discount_starting_idles=self.is_shy_cbs)
            if path is None:
                raise RuntimeError(f"No path found for agent {spec.agent_id} in root planning")
            root_paths[spec.agent_id] = path
            shortest_agent_costs[spec.agent_id] = self._compute_path_cost(path)

        # Normal a*star doesn't take into account collisions
        if self.algo == 'astar':
            return root_paths

        root_node = {
            "constraints": root_constraints,
            "paths": root_paths,
            "cost": self._compute_cost(root_paths),
        }

        # Count conflicts in independent planning (root node)
        self.total_conflicts_resolved = self._count_all_conflicts(root_paths)

        open_list: List[Tuple[float, int, Dict]] = []
        counter = 0
        heapq.heappush(open_list, (root_node["cost"], counter, root_node))

        nodes_expanded = 0
        while open_list:
            _, _, node = heapq.heappop(open_list)
            nodes_expanded += 1

            if nodes_expanded % 10000 == 0:
                print(f'This iteration of CBS is taking a while... Current path combinations considered: {nodes_expanded}')

            conflict = self._find_conflict(node["paths"])
            if not conflict:
                self.total_nodes_expanded += nodes_expanded
                return node["paths"]
            agent_a, agent_b = conflict["agents"]
            time = conflict["time"]
            pos = conflict["position"]

            for agent in (agent_a, agent_b):
                # Only add constraints into the constraint set if it doesn't already exist.
                # So we use a set
                new_constraints = set(node["constraints"])

                if conflict["type"] == "vertex":
                    # Since get_position_at_time
                    final_agent_time_step = len(node["paths"][agent]) - 1

                    for time_steps in range(final_agent_time_step, time + 1):
                        # Basically tell the shorter conflicting path to replan a path that takes as much steps 
                        # as the longer conflicting path to reach the goal
                        # 
                        # Had to do this because the conflict happens in the future; The shorter conflicting path
                        # is assumed to idle at the goal position after reaching there.
                        extra_constraint = Constraint(
                            agent_id=agent, 
                            time=time_steps,
                            position=pos,
                            constraint_type="vertex"
                        )
                        new_constraints.add(extra_constraint)
                        # break

                    constraint = Constraint(agent_id=agent, time=time, position=pos, constraint_type="vertex")
                    new_constraints.add(constraint)
                else:  # edge conflict
                    frm = self._get_position_at_time(node["paths"][agent], time - 1)
                    to = self._get_position_at_time(node["paths"][agent], time)
                    constraint = Constraint(
                        agent_id=agent,
                        time=time,
                        position=frm,
                        next_position=to,
                        constraint_type="edge",
                    )
                    new_constraints.add(constraint)

                new_paths = dict(node["paths"])
                spec = agent_map[agent]
                replanned = self._low_level_search(spec, new_constraints, spec.grid_map, 
                                                   allow_backtrack=self.allow_backtrack,
                                                   discount_starting_idles=self.is_shy_cbs)

                if replanned is None:
                    continue

                new_paths[agent] = replanned
                counter += 1
                new_cost = self._compute_cost(new_paths)
                new_node = (
                    new_cost,
                    counter,
                    {
                        "constraints": new_constraints,
                        "paths": new_paths,
                        "cost": new_cost,
                    },
                )
                if new_node not in open_list:
                    heapq.heappush(open_list, new_node)
                else:
                    pass

        raise RuntimeError("CBS failed to find a conflict-free solution")

    def grid_path_to_world(self, path: List[GridPos], grid_map:GridMap) -> List[Tuple[float, float]]:
        """Helper to convert grid indices back into world coordinates."""
        world_path: List[Tuple[float, float]] = []
        for cell in path:
            world_path.append(grid_map.grid_to_world(cell[0], cell[1]))
        return world_path

    # ------------------------------------------------------------------
    # CBS helpers
    # ------------------------------------------------------------------
    def _low_level_search(self, spec: AgentSpec, constraints: set[Constraint], grid_map:GridMap, 
                          allow_backtrack, discount_starting_idles) -> Optional[List[GridPos]]:
        """
        Run constrained A* for a single agent.
        
        allow_backtrack: Allows the robot to go back to where it belongs.
        allow_idle_after_movement: If a agent starts moving, it cannot idle anymore.
        """

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
                # If argument is set:
                # Don't allow idling aka (0,0) after agent makes a move that is not (0, 0)
                # if (not allow_idle_after_movement) and (dx, dy) == (0, 0) and (current in came_from):
                #     continue

                nx, ny = x + dx, y + dy
                nt = t + 1

                if nt > self.max_time:
                    continue

                if not (0 <= nx < grid_map.cols and 0 <= ny < grid_map.rows):
                    continue

                if (dx, dy) != (0, 0) and grid_map.is_occupied(nx, ny):
                    continue

                if self._violates_constraints((x, y), (nx, ny), nt, constraint_table):
                    continue

                # When preventing backtracking backtracking: disallow moving into any cell this agent has previously visited
                # (except waiting in place). Reconstruct the path to the current state and check visited positions.
                # current position is allowed to be waited on, but moving into any previously visited
                # *different* cell is forbidden.
                if not allow_backtrack and (dx, dy) != (0, 0):
                    # This code should be optimized if need be, its quite inefficient,
                    # but we usually get backtrack to true so thais wouldn't matter right now
                    visited_positions = set(self._reconstruct_path(came_from, current))
                    if (nx, ny) in visited_positions:
                        continue
                    
                neighbor = (nx, ny, nt)
                tentative_g = g_scores[current] + 1
                
                if discount_starting_idles and (dx, dy) == (0, 0) and ((x, y) in self.all_work_stn_grid_pos):
                    DISCOUNT = 0.0001
                    tentative_g -= DISCOUNT


                if neighbor not in g_scores or tentative_g < g_scores[neighbor]:
                    g_scores[neighbor] = tentative_g
                    came_from[neighbor] = current
                    count += 1
                    priority = tentative_g + self._heuristic((nx, ny), goal)

                    heapq.heappush(open_list, (priority, count, neighbor))

        return None

    def _build_constraint_table(self, constraints: set[Constraint], agent_id: int):
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
        """Find the first conflict in the given paths (used by CBS search)."""
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

                    if t >= len(path_a) + 1 or t >= len(path_b) + 1:
                        continue
 
                    if pos_a == pos_b and (pos_a is not None) and (pos_b is not None):
                        return {"type": "vertex", "agents": (a_id, b_id), "time": t, "position": pos_a}
                    
                    if t > 0:
                        prev_a = self._get_position_at_time(path_a, t - 1)
                        prev_b = self._get_position_at_time(path_b, t - 1)
                        if prev_a == pos_b and pos_a == prev_b and (pos_a is not None) and (pos_b is not None):
                            return {
                                "type": "edge",
                                "agents": (a_id, b_id),
                                "time": t,
                                "position": pos_a,
                            }

        return None

    def _count_all_conflicts(self, paths: Dict[int, List[GridPos]]) -> int:
        """Count total number of conflicts in the given paths."""
        if not paths:
            return 0

        conflict_count = 0
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

                    # Vertex conflict
                    if pos_a == pos_b:
                        conflict_count += 1

                    # Edge conflict
                    if t > 0:
                        prev_a = self._get_position_at_time(path_a, t - 1)
                        prev_b = self._get_position_at_time(path_b, t - 1)
                        if prev_a == pos_b and pos_a == prev_b:
                            conflict_count += 1

        return conflict_count

    @staticmethod
    def _get_position_at_time(path: List[GridPos], time_step: int) -> GridPos:
        if time_step < 0:
            time_step = 0
        if time_step < len(path):
            return path[time_step]
        
        return path[-1]

    def _compute_cost(self, paths: Dict[int, List[GridPos]]) -> float:
        total = 0
        for path in paths.values():
            total += self._compute_path_cost(path)
        return total
    
    def _compute_path_cost(self, path:List[GridPos]) -> float:
            return len(path)

    def _heuristic(self, node: GridPos, goal: GridPos) -> float:
        return abs(node[0] - goal[0]) + abs(node[1] - goal[1])
    
    def _count_consecutive_idles_at_start(self, lst):
        if not lst:
            return 0

        first_element = lst[0]
        for i, element in enumerate(lst):
            if first_element != element:
                return i - 1
            
        return len(lst) - 1