"""Standalone CBS demo with a handful of robots in the warehouse environment."""
# python main_cbs_demo.py --robots 12 --step-duration 0.1

from __future__ import annotations

import argparse
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
from pathfinding import create_warehouse_grid
from collisions import CBSCollisionsTracker

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)

class Endpoint:
    def __init__(self, position:Tuple[float, float]):
        self.obj_id = self._create_endpoint_marker(position)
        self.world_pos = position

    def _create_endpoint_marker(self, position: Tuple[float, float]) -> int:
        """Create a visual marker for an assigned endpoint (order pickup location)."""
        marker_visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[0.4, 0.4, 0.15],
            rgbaColor=[1, 0.6, 0.0, 0.0],  # Transparent initially
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
        State is either 'hidden', 'pending', 'collected', or 'completed'
        '''

        assert state in ('hidden', 'pending', 'collected', 'completed'), f"State {state} not valid!"
        if state == 'hidden':
            # Make the visual transparent
            p.changeVisualShape(self.obj_id, -1, rgbaColor=[0, 1, 0, 0])
        elif state == 'pending':
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

@dataclass
class CBSDemoBot:
    """Controls a single PyBullet robot body using a CBS schedule."""

    body_id: int
    schedule: List[Tuple[float, float]]
    step_duration: float
    pickup_index: int
    order_id: int
    endpoint: Endpoint
    workstation: Tuple[float, float]

    def __post_init__(self):
        self.total_duration = max((len(self.schedule) - 1) * self.step_duration, 0.0)
        self._pickup_logged = False
        self._delivery_logged = False
        self._pickup_visual_updated = False
        self._delivery_visual_updated = False
        self.pickup_time: float | None = None
        self.delivery_time: float | None = None

        self._set_pose(self.workstation)

    def update_task(self, order_id=None, workstation=None, endpoint=None, endpoint_marker_id=None):
        if order_id is not None:
            self.order_id = order_id
        if workstation is not None:
            self.workstation = workstation
        if endpoint is not None:
            self.endpoint = endpoint
        self._pickup_logged = False

    def update(self, sim_time: float):
        if len(self.schedule) == 0:
            return

        if sim_time >= self.total_duration:
            self._set_pose(self.schedule[-1])
            if not self._delivery_logged:
                logger.info(
                    "DELIVERED (CBS): order %s returned to workstation at t=%.2fs",
                    self.order_id,
                    sim_time,
                )
                self._delivery_logged = True
                if self.delivery_time is None:
                    self.delivery_time = sim_time
                # Change endpoint marker to green
                if self.endpoint is not None and not self._delivery_visual_updated:
                    self.endpoint.update_visual(state='completed')
                    self._delivery_visual_updated = True
            return

        segment = min(int(sim_time / self.step_duration), len(self.schedule) - 2)
        t0 = segment * self.step_duration
        alpha = (sim_time - t0) / self.step_duration

        start = np.array(self.schedule[segment])
        end = np.array(self.schedule[segment + 1])
        interp = start + alpha * (end - start)

        self._set_pose(tuple(interp))

        pickup_time = self.pickup_index * self.step_duration
        if (not self._pickup_logged) and sim_time >= pickup_time:
            logger.info(
                "COLLECTED (CBS): order %s picked up at %s (t=%.2fs)",
                self.order_id,
                np.round(self.endpoint.get_world_pos_2d(), 2),
                sim_time,
            )
            self._pickup_logged = True
            if self.pickup_time is None:
                self.pickup_time = sim_time
            # Change endpoint marker to yellow
            if self.endpoint is not None and not self._pickup_visual_updated:
                self.endpoint.update_visual(state='collected')
                self._pickup_visual_updated = True

    def _set_pose(self, xy_pos: Tuple[float, float]):
        p.resetBasePositionAndOrientation(
            self.body_id,
            [xy_pos[0], xy_pos[1], 0],
            p.getQuaternionFromEuler([0, 0, 0]),
        )


class CBSDemo:
    """Creates a lightweight simulator showcasing CBS collision avoidance."""

    def __init__(self, num_robots: int = 10, step_duration: float = 0.2, metrics_file: str | None = None):
        if num_robots < 2:
            raise ValueError("CBS demo needs at least two robots to illustrate coordination")

        self.physics_client = p.connect(p.GUI)
        p.setGravity(0, 0, -10)

        (self.rows,
         self.cols,
         self.workstations,
         self.shelves,
         self.endpoints) = utils.get_warehouse_params(layout='default')

        (self.wall_pos,
         self.work_stn_pos,
         self.shelves_pos,
         self.all_endpoints_pos,
         self.wall_struct_id,
         self.shelves_struct_id) = self._load_map()

        x_offset = 0 if self.cols % 2 == 0 else 0.5
        y_offset = 0 if self.rows % 2 == 0 else 0.5
        self.grid_map = create_warehouse_grid(
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
        self.time_step = 1 / 720.0  # faster visual updates keep interpolation smooth
        self.metrics_file = metrics_file
        self.num_active = min(num_robots, len(self.work_stn_pos))
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
            self.workstations,
            "assets/warehouse/workstations.urdf",
            grid_z=1.5,
            box_color=(1, 0.2, 0.6, 0.6),
            has_collison=False,
        )
        shelves_pos = utils.create_struct_urdf(
            self.shelves,
            "assets/warehouse/shelves.urdf",
            grid_z=1,
            box_color=(0.3, 0.3, 0.3, 0.9),
            collision_scale=0.7,
        )

        # We'll also highlight endpoints individually for assigned orders later on
        endpoints_pos = utils.create_struct_urdf(
            self.endpoints,
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

    def _plan_and_assign_paths(self):
        rng = np.random.default_rng(seed=7)
        planner = CBSPlanner(self.grid_map, max_time=400)

        chosen_endpoints = rng.choice(len(self.all_endpoints_pos), size=self.num_active, replace=False)
        # endpoint_list = [self.all_endpoints_pos[i] for i in chosen_endpoints]
        endpoint_list:list[Endpoint] = [Endpoint(self.all_endpoints_pos[i]) for i in chosen_endpoints]

        to_pick_specs = []
        to_drop_specs = []
        self.order_metadata = {}

        for idx, robot_idx in enumerate(self.active_robot_indices):
            work_pos = self.work_stn_pos[robot_idx]
            endpoint: Endpoint = endpoint_list[idx]
            endpoint.update_visual('pending')

            start_grid = self.grid_map.world_to_grid(work_pos[0], work_pos[1])
            pickup_grid = self.grid_map.world_to_grid(*endpoint.get_world_pos_2d())

            to_pick_specs.append(AgentSpec(agent_id=idx, start=start_grid, goal=pickup_grid))
            to_drop_specs.append(AgentSpec(agent_id=idx, start=pickup_grid, goal=start_grid))
            self.order_metadata[idx] = {
                "endpoint": endpoint,
                "workstation": (work_pos[0], work_pos[1]),
            }

        pickup_paths = planner.plan_paths(to_pick_specs)
        pickup_conflicts = planner.conflicts_resolved
        drop_paths = planner.plan_paths(to_drop_specs)
        drop_conflicts = planner.conflicts_resolved
        self.collisions_avoided = pickup_conflicts + drop_conflicts

        for idx, body_id in enumerate(self.body_ids):
            pick_path = pickup_paths[idx]
            drop_path = drop_paths[idx]

            path_grid = pick_path + drop_path[1:]
            path_world = planner.grid_path_to_world(path_grid)

            schedule = [(x, y) for (x, y) in path_world]
            pickup_index = len(pick_path) - 1

            metadata = self.order_metadata[idx]

            # Create endpoint marker for this assigned order
            # endpoint_marker_id = self._create_endpoint_marker(metadata["endpoint"])

            bot = CBSDemoBot(
                body_id=body_id,
                schedule=schedule,
                step_duration=self.step_duration,
                pickup_index=pickup_index,
                order_id=idx,
                endpoint=metadata["endpoint"],
                workstation=metadata["workstation"],
                # endpoint_marker_id=endpoint_marker_id,
            )
            self.demo_bots.append(bot)

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

    def _export_metrics(self, total_duration: float, min_clearance: float):
        if not self.metrics_file:
            return

        pickups = [bot.pickup_time for bot in self.demo_bots if bot.pickup_time is not None]
        deliveries = [bot.delivery_time for bot in self.demo_bots if bot.delivery_time is not None]
        total_orders = len(self.demo_bots)
        completed = len(deliveries)
        pending = total_orders - completed

        rr_collisions = self.collision_checker.get_rr_collision_records(as_list_of_dicts=True)
        ro_collisions = self.collision_checker.get_ro_collision_records(as_list_of_dicts=True)

        # Build per-order details
        order_details = []
        for bot in self.demo_bots:
            order_details.append({
                "order_id": bot.order_id,
                "pickup_time": round(bot.pickup_time, 4) if bot.pickup_time is not None else None,
                "delivery_time": round(bot.delivery_time, 4) if bot.delivery_time is not None else None,
                "total_time": round(bot.delivery_time, 4) if bot.delivery_time is not None else None,
                "endpoint": list(np.round(bot.endpoint.get_world_pos_2d(), 2)),
                "workstation": list(np.round(bot.workstation, 2)),
                "status": "completed" if bot._delivery_logged else "pending"
            })

        json_metrics = {
            "summary": {
                "total_orders": total_orders,
                "orders_completed": completed,
                "orders_pending": pending,
                "collisions_avoided": self.collisions_avoided,
                "total_time_for_all_orders": round(total_duration, 4),
                "avg_pickup_time": round(sum(pickups) / len(pickups), 4) if pickups else None,
                "avg_delivery_time": round(sum(deliveries) / len(deliveries), 4) if deliveries else None,
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
        total_duration = max(bot.total_duration for bot in self.demo_bots) + 5.0
        sim_time = 0.0

        print("\n" + "🏭 " * 10)
        print("CBS DEMO - SMALL ROBOT FLEET")
        print("🏭 " * 10 + "\n")
        print(f"Robots participating: {self.num_active}")
        print(f"CBS step duration: {self.step_duration:.2f}s")
        print("Orders:")
        for idx, meta in self.order_metadata.items():
            endpoint = np.round(meta["endpoint"].get_world_pos_2d(), 2)
            workstation = np.round(meta["workstation"], 2)
            print(f"  - Robot {idx}: workstation {workstation} -> endpoint {endpoint} -> workstation")
        print("\nWatching collision-free trajectories...\n")

        min_clearance = math.inf
        while sim_time < total_duration:
            self.collision_checker.check_robot_collisions(sim_time)

            for bot in self.demo_bots:
                bot.update(sim_time)
            p.stepSimulation()
            time.sleep(self.time_step)
            sim_time += self.time_step

            current_clearance = self._pairwise_clearance()
            min_clearance = min(min_clearance, current_clearance)

            if all(bot._delivery_logged for bot in self.demo_bots):
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
        default=10,
        help="Number of robots to include in the demo (default: 6)",
    )
    parser.add_argument(
        "--step-duration",
        type=float,
        default=0.08,
        help="Seconds allocated per CBS grid step (smaller = faster)",
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
        step_duration=args.step_duration,
        metrics_file=args.metrics_file,
    )
    demo.run()
