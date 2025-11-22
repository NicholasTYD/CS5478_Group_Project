"""Standalone CBS demo with a handful of robots in the warehouse environment."""
# python main_cbs_demo.py --robots 12 --step-duration 0.1

from __future__ import annotations

import argparse
from copy import deepcopy
import json
import logging
import math
import time
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np
import pybullet as p

import utils
from multiagent_cbs import AgentSpec, CBSPlanner
from pathfinding import GridMap, create_warehouse_grid
from collisions import CBSCollisionsTracker

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)

rng = np.random.default_rng(seed=7)

class Endpoint:
    def __init__(self, position:Tuple[float, float, float], state='hidden'):
        self.obj_id = self._create_endpoint_marker(position)
        self.update_visual(state=state)        
        self.world_pos = position

    def _create_endpoint_marker(self, position: Tuple[float, float, float]) -> int:
        """Create a visual marker for an assigned endpoint (order pickup location)."""
        marker_visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[0.4, 0.4, 0.15],
        )
        marker_id = p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=marker_visual,
            basePosition=[position[0], position[1], 0.3],
            baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
        )

        return marker_id
    
    def get_world_pos_2d(self):
        return self.world_pos[:2]
    
    def update_visual(self, state='hidden'):
        '''
        State is either 'hidden', 'uncollected', 'collected', or 'completed'
        '''

        assert state in ('hidden', 'uncollected', 'collected', 'completed'), f"State {state} not valid!"
        if state == 'hidden':
            # Make the visual transparent
            p.changeVisualShape(self.obj_id, -1, rgbaColor=[0, 1, 0, 0])
        elif state == 'uncollected':
            # Orange
            p.changeVisualShape(self.obj_id, -1, rgbaColor=[1, 0.6, 0.0, 0.9])
        elif state == 'collected':
            # Yellow
            p.changeVisualShape(self.obj_id, -1, rgbaColor=[1, 1, 0, 0.9])
        elif state == 'completed':
            # Green
            p.changeVisualShape(self.obj_id, -1, rgbaColor=[0, 1, 0, 0.9])
        else:
            raise Exception(f"State {state} not valid!")
        
    def destroy(self):
        p.remove_body(self.obj_id)

class DeliveryTask:
    def __init__(self, order_id: int, endpoint:Endpoint, workstation:Tuple[float, float], curr_sim_step:int):
        self.order_id = order_id

        self.creation_sim_step:int = curr_sim_step
        self.pickup_sim_step:int | None = None
        self.delivered_sim_step: int | None = None

        self.endpoint: Endpoint = endpoint
        self.workstation: Tuple[float, float] = workstation

    def record_pickup(self, curr_sim_step:int):
        self.pickup_sim_step = curr_sim_step

    def record_delivery(self, curr_sim_step:int):
        self.delivered_sim_step = curr_sim_step

    def order_fufilled(self):
        return self.delivered_sim_step is not None

@dataclass
class CBSDemoBot:
    """Controls a single PyBullet robot body using a CBS schedule."""

    body_id: int
    steps_per_grid: float
    workstation: Tuple[float, float]
    grid_map: GridMap

    def __post_init__(self):
        # self.total_duration = max((len(self.schedule) - 1) * self.step_duration, 0.0)
        self.goal_pos: Tuple[float, float] | None = None
    
        self.schedule: List[Tuple[float, float]] | None = None
        self.schedule_step_start:int = None

        self.delivery_task: DeliveryTask | None = None

        self._set_pose(self.workstation)

    def allocate_new_task(self, delivery_task:DeliveryTask):
        self.delivery_task = delivery_task

        self.goal_pos = delivery_task.endpoint.get_world_pos_2d()
        delivery_task.endpoint.update_visual('uncollected')

    def set_schedule(self, schedule):
        self.schedule = schedule
        self.schedule_step_start = None

    def requires_pathfinding_schedule(self):
        '''
        Returns True if this demo bot requires the central CBS algo to generate a pathfinding algorithm for it.
        '''
        return (self.goal_pos is not None) and (self.schedule is None)
    
    def has_goal(self):
        return self.goal_pos is not None
    
    def check_goal_reached(self, sim_time:float, sim_step:int):
        if np.array_equal(self.goal_pos, self.delivery_task.endpoint.get_world_pos_2d()) and self._goal_reached():
            self.schedule = None
            logger.info(
                "COLLECTED (CBS): order %s picked up by at %s (t=%.2fs), sim step=%i",
                self.delivery_task.order_id,
                np.round(self.delivery_task.endpoint.get_world_pos_2d(), 2),
                sim_time,
                sim_step,
            )
            self.delivery_task.record_pickup(sim_step)
            #Change endpoint marker to yellow
            self.delivery_task.endpoint.update_visual(state='collected')
            self._pickup_visual_updated = True
            self.goal_pos = self.workstation

        if np.array_equal(self.goal_pos, self.workstation) and self._goal_reached():
            self.schedule = None
            logger.info(
                "DELIVERED (CBS): order %s returned to workstation at t=%.2fs, sim step=%i",
                self.delivery_task.order_id,
                sim_time,
                sim_step
            )
            self.delivery_task.record_delivery(sim_step)
            # Change endpoint marker to green
            self.delivery_task.endpoint.update_visual(state='completed')
            self.goal_pos = None
        
    def act(self, sim_step:int):
        # Don't do anything if there's no schedule
        if self.schedule is None:
            return
        
        if self.schedule_step_start is None:
            self.schedule_step_start = sim_step
        sim_steps_since_schedule_start = sim_step - self.schedule_step_start 
        
        segment = sim_steps_since_schedule_start // self.steps_per_grid
        alpha = (sim_steps_since_schedule_start % self.steps_per_grid) / self.steps_per_grid

        start = np.array(self.schedule[segment])
        next_segment = min(segment + 1, len(self.schedule) - 1) # Prevent index out of bounds
        end = np.array(self.schedule[next_segment])
        interp = start + alpha * (end - start)

        self._set_pose(tuple(interp))

    def _set_pose(self, xy_pos: Tuple[float, float]):
        p.resetBasePositionAndOrientation(
            self.body_id,
            [xy_pos[0], xy_pos[1], 0],
            p.getQuaternionFromEuler([0, 0, 0]),
        )

    def get_goal_pos_2d(self):
        return self.goal_pos
    
    def get_world_pos_2d(self):
        pos_3d, ori = p.getBasePositionAndOrientation(self.body_id)
        return pos_3d[:2]
    
    def _goal_reached(self):
        # Target reached if robot is within 0.01 cell (1cm) of goal (Same as default course project)
        # This is for dealing with rounding errors when comparing grid pos and robot world pos.
        # Technically can pass in the grid map but 
        GOAL_TOLERANCE = 0.01


        robot_xy_pos = np.array(self.get_world_pos_2d())
        goal_xy_pos = self.goal_pos[0], self.goal_pos[1]
    
        diff = robot_xy_pos - goal_xy_pos
        dist_diff_squared = np.dot(diff, diff)
        return dist_diff_squared < (GOAL_TOLERANCE * GOAL_TOLERANCE) 


class CBSDemo:
    """Creates a lightweight simulator showcasing CBS collision avoidance."""

    def __init__(self, num_robots: int = 10, step_duration: float = 0.2, steps_per_grid: int = 10,
                metrics_file: str | None = None):
        if num_robots < 2:
            raise ValueError("CBS demo needs at least two robots to illustrate coordination")

        self.physics_client = p.connect(p.GUI)
        p.setGravity(0, 0, -10)

        (self.rows,
         self.cols,
         self.workstations_map,
         self.shelves_map,
         self.endpoints_map) = utils.get_warehouse_params(layout='default')

        (self.wall_pos,
         self.work_stn_pos,
         self.shelves_pos,
         self.all_endpoints_pos,
         self.wall_struct_id,
         self.shelves_struct_id) = self._load_map()

        x_offset = 0 if self.cols % 2 == 0 else 0.5
        y_offset = 0 if self.rows % 2 == 0 else 0.5
        self.default_grid_map = create_warehouse_grid(
            self.rows,
            self.cols,
            self.wall_pos,
            self.shelves_pos,
            cell_size=1.0,
            offset=(x_offset, y_offset),
            robot_radius=0.2,
            endpoints_pos=self.all_endpoints_pos,
            work_stns_pos=self.work_stn_pos,
        )

        self.step_duration = step_duration
        self.steps_per_grid = steps_per_grid
        self.metrics_file = metrics_file
        self.num_active = min(num_robots, len(self.work_stn_pos))
        # Number of collisions avoided had robots are allowed to execute their low level search directly without
        # any collision avoidance check. Accumulated everytime CBS is called.
        self.collisions_avoided = 0

        logger.info("Initializing CBS demo with %s robots", self.num_active)

        self.active_robot_indices = list(range(self.num_active))
        self.body_ids = self._spawn_bodies()
        self.demo_bots: List[CBSDemoBot] = []
        
        self.collision_checker = CBSCollisionsTracker()
        for body_id in self.body_ids:
            self.collision_checker.add_robot_to_track(body_id)
        self.collision_checker.add_obstacle_to_track(self.wall_struct_id)
        self.collision_checker.add_obstacle_to_track(self.shelves_struct_id)

        self.tasks_created: List[DeliveryTask] = []

        logger.info("Initialization Done, running pathfinding...")
        self._init_cbs_demobots()
        self._allocate_tasks()
        self._plan_and_assign_paths()
        self._load_camera()

    # ------------------------------------------------------------------
    def _load_map(self):
        x_offset = 0 if self.cols % 2 == 0 else 0.5
        y_offset = 0 if self.rows % 2 == 0 else 0.5
        floor_base_pos = [x_offset, y_offset, 0]
        p.loadURDF("assets/plane/plane.urdf", basePosition=floor_base_pos)

        whouse_map = np.zeros([self.rows + 2, self.cols + 2])
        whouse_map[0, :] = 1
        whouse_map[-1, :] = 1
        whouse_map[:, 0] = 1
        whouse_map[:, -1] = 1

        wall_pos = utils.create_struct_urdf(
            whouse_map,
            "assets/warehouse/wall.urdf",
            grid_z=3,
            box_color=(0.1, 0.1, 0.1, 1),
        )
        work_stn_pos = utils.create_struct_urdf(
            self.workstations_map,
            "assets/warehouse/workstations.urdf",
            grid_z=1.5,
            box_color=(1, 0.2, 0.6, 0.6),
            has_collison=False,
        )
        shelves_pos = utils.create_struct_urdf(
            self.shelves_map,
            "assets/warehouse/shelves.urdf",
            grid_z=1,
            box_color=(0.3, 0.3, 0.3, 0.9),
            collision_scale=0.7,
        )

        # We'll also highlight endpoints individually for assigned orders later on
        endpoints_pos = utils.create_struct_urdf(
            self.endpoints_map,
            "assets/warehouse/endpoints.urdf",
            grid_z=0.3,
            box_color=(0, 0, 1, 0.1),
            has_collison=False,
        )

        wall_struct_id = p.loadURDF("assets/warehouse/wall.urdf", useFixedBase=1, flags=p.URDF_MERGE_FIXED_LINKS)
        p.loadURDF("assets/warehouse/workstations.urdf", useFixedBase=1, flags=p.URDF_MERGE_FIXED_LINKS)
        shelves_struct_id = p.loadURDF("assets/warehouse/shelves.urdf", useFixedBase=1, flags=p.URDF_MERGE_FIXED_LINKS)
        # Load all endpoints visually but at a greatly reduced transparency
        p.loadURDF("assets/warehouse/endpoints.urdf", useFixedBase=1, flags=p.URDF_MERGE_FIXED_LINKS)

        return wall_pos, work_stn_pos, shelves_pos, endpoints_pos, wall_struct_id, shelves_struct_id

    def _spawn_bodies(self):
        body_ids = []
        robot_radius = 0.3
        robot_height = 0.2
        colors = [
            [0.2, 0.2, 1.0, 1],
            [1.0, 0.2, 0.2, 1],
            [0.2, 1.0, 0.2, 1],
            [1.0, 1.0, 0.2, 1],
            [1.0, 0.5, 0.0, 1],
            [0.5, 0.0, 1.0, 1],
        ]

        for idx in self.active_robot_indices:
            robot_collision = p.createCollisionShape(p.GEOM_CYLINDER, radius=robot_radius, height=robot_height)
            robot_visual = p.createVisualShape(
                p.GEOM_CYLINDER,
                radius=robot_radius,
                length=robot_height,
                rgbaColor=colors[idx % len(colors)],
            )
            pos = self.work_stn_pos[idx].copy()
            pos[2] = 0
            body_id = p.createMultiBody(
                baseMass=10,
                baseCollisionShapeIndex=robot_collision,
                baseVisualShapeIndex=robot_visual,
                basePosition=pos,
                baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
            )
            body_ids.append(body_id)

        return body_ids
    
    def _init_cbs_demobots(self):
        for idx, body_id in enumerate(self.body_ids):
            assigned_work_stn = self.work_stn_pos[idx]

            # This code segments creates a new grid map spec ific to the demobot that allows exclusive access
            # to its own workstation (Other robots can't enter it)
            work_stn_grid_pos = self.default_grid_map.world_to_grid(assigned_work_stn[0], assigned_work_stn[1])
            bot_grid_map = deepcopy(self.default_grid_map)
            bot_grid_map.set_unoccupied(*work_stn_grid_pos)
            
            bot = CBSDemoBot(
                body_id=body_id,
                steps_per_grid=self.steps_per_grid,
                workstation=(assigned_work_stn[0], assigned_work_stn[1]),
                grid_map=bot_grid_map,
            )
            self.demo_bots.append(bot)

    def _allocate_tasks(self):
        chosen_endpoints = rng.choice(len(self.all_endpoints_pos), size=self.num_active, replace=False)
        endpoint_list:list[Endpoint] = [Endpoint(self.all_endpoints_pos[i]) for i in chosen_endpoints]

        for i, bot in enumerate(self.demo_bots):
            delivery_task = DeliveryTask(
                order_id=i, endpoint=endpoint_list[i], workstation=bot.workstation, curr_sim_step=0
            )
            bot.allocate_new_task(delivery_task)
            self.tasks_created.append(delivery_task)

    def _plan_and_assign_paths(self):
        planner = CBSPlanner(max_time=400)

        pathfinding_needed = any(bot.requires_pathfinding_schedule() for bot in self.demo_bots)
        if not pathfinding_needed:
            return
        
        agent_specs = []
        for bot in self.demo_bots:
            # We RECOMPUTE CBS for all bots that currently have a goal
            # (even if they have an active schedule, because we will override it) 
            if not bot.has_goal(): 
                continue

            curr_pos = bot.get_world_pos_2d()
            goal_pos = bot.get_goal_pos_2d()

            start_grid = bot.grid_map.world_to_grid(curr_pos[0], curr_pos[1])
            pickup_grid = bot.grid_map.world_to_grid(goal_pos[0], goal_pos[1])

            assert all([int(coord) == coord for coord in start_grid + pickup_grid]) \
                , f'Bot {bot.body_id} coordinates must be integer, but got curr_pos={start_grid}, goal_pos={pickup_grid}'

            agent_specs.append(AgentSpec(agent_id=bot.body_id, start=start_grid, goal=pickup_grid, grid_map=bot.grid_map))


        planned_paths = planner.plan_paths(agent_specs)
        path_conflicts = planner.conflicts_resolved
        self.collisions_avoided += path_conflicts

        idx_ptr = 0
        for bot in self.demo_bots:
            if not bot.has_goal(): # Bot doesn't need pathfinding
                continue

            bot_id = bot.body_id
            pick_path = planned_paths[bot_id]

            path_world = planner.grid_path_to_world(pick_path, agent_specs[idx_ptr].grid_map)

            schedule = [(x, y) for (x, y) in path_world]
            bot.set_schedule(schedule)

            idx_ptr += 1

            # print(f" Planned - Robot {bot.body_id}: {schedule[0]} -> {schedule[-1]}")

        logger.info("CBS plans generated successfully for %s robots", len(self.demo_bots))
        logger.info("Collisions avoided by CBS: %s", self.collisions_avoided)

    def _load_camera(self):
        p.resetDebugVisualizerCamera(
            cameraDistance=35,
            cameraYaw=180,
            cameraPitch=-89,
            cameraTargetPosition=[0, 0, 0],
        )

    def _pairwise_clearance(self) -> float:
        if len(self.body_ids) < 2:
            return math.inf
        positions = [p.getBasePositionAndOrientation(body_id)[0] for body_id in self.body_ids]
        best = math.inf
        for i in range(len(positions)):
            for j in range(i + 1, len(positions)):
                ax, ay = positions[i][0], positions[i][1]
                bx, by = positions[j][0], positions[j][1]
                dist = math.hypot(ax - bx, ay - by)
                if dist < best:
                    best = dist
        return best
    
    def from_sim_step_to_time(self, sim_step:int, step_duration:float):
        return sim_step * step_duration

    def _export_metrics(self, total_duration: float, min_clearance: float):
        if not self.metrics_file:
            return

        pickups = [task.pickup_sim_step for task in self.tasks_created if task.pickup_sim_step is not None]
        deliveries = [task.delivered_sim_step for task in self.tasks_created if task.delivered_sim_step is not None]
        total_orders = len(self.tasks_created)
        completed = len(deliveries)
        pending = total_orders - completed

        rr_collisions = self.collision_checker.get_rr_collision_records(as_list_of_dicts=True)
        ro_collisions = self.collision_checker.get_ro_collision_records(as_list_of_dicts=True)

        # Build per-order details
        order_details = []
        for task in self.tasks_created:
            order_details.append({
                "order_id": task.order_id,

                "creation_sim_step": task.creation_sim_step,

                "pickup_sim_step": task.pickup_sim_step if task.pickup_sim_step is not None else None,

                "delivery_sim_step": task.delivered_sim_step if task.delivered_sim_step is not None else None,

                "total_delivery_sim_step": task.delivered_sim_step - task.creation_sim_step if task.delivered_sim_step is not None else None,

                "endpoint": list(np.round(task.endpoint.get_world_pos_2d(), 2)),
                "workstation": list(np.round(task.workstation, 2)),
                "status": "completed" if task.delivered_sim_step is not None else "pending"
            })

        json_metrics = {
            "summary": {
                "total_orders": total_orders,
                "orders_completed": completed,
                "orders_pending": pending,
                "collisions_avoided": self.collisions_avoided,
                "total_sim_steps_for_all_orders": round(total_duration, 4),
                "avg_pickup_sim_steps": round(sum(pickups) / len(pickups), 4) if pickups else None,
                "avg_delivery_sim_steps": round(sum(deliveries) / len(deliveries), 4) if deliveries else None,
                "min_clearance": round(min_clearance, 4) if not math.isinf(min_clearance) else None,
                "num_robots": self.num_active,
                "step_duration": self.step_duration,
                "num_robot_robot_collisions": len(rr_collisions),
                "num_robot_obstacle_collisions": len(ro_collisions),
            },
            "orders": order_details,
            "robot_robot_collisions": rr_collisions,
            "robot_obstacle_collisions": ro_collisions,

        }

        with open(self.metrics_file, "w") as jsonfile:
            json.dump(json_metrics, jsonfile, indent=2)
        print(f"Metrics written to {self.metrics_file}")

    # ------------------------------------------------------------------
    def run(self):
        # total_duration = max(bot.total_duration for bot in self.demo_bots) + 5.0
        max_sim_duration = 2000
        sim_time = 0.0
        sim_step: int = 0

        print("\n" + "🏭 " * 10)
        print("CBS DEMO - SMALL ROBOT FLEET")
        print("🏭 " * 10 + "\n")
        print(f"Robots participating: {self.num_active}")
        print(f"CBS step duration: {self.step_duration:.2f}s")
        print("Orders:")
        print("\nWatching collision-free trajectories...\n")

        min_clearance = math.inf
        while sim_time < max_sim_duration:
            self.collision_checker.check_robot_collisions(sim_step)

            for bot in self.demo_bots:
                bot.check_goal_reached(sim_time, sim_step)

            if sim_step % self.steps_per_grid == 0:
                self._plan_and_assign_paths()

            for bot in self.demo_bots:
                bot.act(sim_step)
            p.stepSimulation()
            time.sleep(self.step_duration)
            sim_time += self.step_duration
            sim_step += 1

            current_clearance = self._pairwise_clearance()
            min_clearance = min(min_clearance, current_clearance)

            if all(task.order_fufilled() for task in self.tasks_created):
                print("\n✓ All CBS orders delivered. Ending demo early.")
                self._export_metrics(sim_time, min_clearance)
                return

        print("\n✓ CBS demonstration finished. Close the PyBullet window to exit.")
        self._export_metrics(sim_time, min_clearance)


def _parse_args():
    parser = argparse.ArgumentParser(description="Run the CBS warehouse demo")
    parser.add_argument(
        "--robots",
        type=int,
        default=12,
        help="Number of robots to include in the demo (default: 6)",
    )
    parser.add_argument(
        "--steps-per-grid",
        type=int,
        default=60,
        help="Number of time steps needed to travel 1 CBS Grid (Smaller = Faster)",
    )
    parser.add_argument(
        "--step-duration",
        type=float,
        default=0.0005,
        help="How long each time step lasts (Smaller = faster)",
    )
    parser.add_argument(
        "--metrics-file",
        type=str,
        default="results.json",
        help="JSON path to dump pickup/delivery timings and summary stats (default: results.json)",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    demo = CBSDemo(
        num_robots=args.robots,
        steps_per_grid=args.steps_per_grid,
        step_duration=args.step_duration,
        metrics_file=args.metrics_file,
    )
    demo.run()
