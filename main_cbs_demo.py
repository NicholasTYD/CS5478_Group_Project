"""Standalone CBS demo with a handful of robots in the warehouse environment."""

from __future__ import annotations

import argparse
import csv
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

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)


@dataclass
class CBSDemoBot:
    """Controls a single PyBullet robot body using a CBS schedule."""

    body_id: int
    schedule: List[Tuple[float, float]]
    step_duration: float
    pickup_index: int
    order_id: int
    endpoint: Tuple[float, float]
    workstation: Tuple[float, float]

    def __post_init__(self):
        self.total_duration = max((len(self.schedule) - 1) * self.step_duration, 0.0)
        self._pickup_logged = False
        self._delivery_logged = False
        self.pickup_time: float | None = None
        self.delivery_time: float | None = None

    def update(self, sim_time: float):
        if len(self.schedule) == 1:
            self._set_pose(self.schedule[0])
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
                np.round(self.endpoint, 2),
                sim_time,
            )
            self._pickup_logged = True
            if self.pickup_time is None:
                self.pickup_time = sim_time

    def _set_pose(self, xy_pos: Tuple[float, float]):
        p.resetBasePositionAndOrientation(
            self.body_id,
            [xy_pos[0], xy_pos[1], 0],
            p.getQuaternionFromEuler([0, 0, 0]),
        )


class CBSDemo:
    """Creates a lightweight simulator showcasing CBS collision avoidance."""

    def __init__(self, num_robots: int = 6, step_duration: float = 0.2, metrics_file: str | None = None):
        if num_robots < 2:
            raise ValueError("CBS demo needs at least two robots to illustrate coordination")

        self.physics_client = p.connect(p.GUI)
        p.setGravity(0, 0, -10)

        (self.rows,
         self.cols,
         self.workstations,
         self.shelves,
         self.endpoints) = utils.get_default_warehouse_params()

        (self.wall_pos,
         self.work_stn_pos,
         self.shelves_pos,
         self.endpoints_pos) = self._load_map()

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
            endpoints_pos=self.endpoints_pos,
            work_stns_pos=self.work_stn_pos,
        )

        self.step_duration = step_duration
        self.time_step = 1 / 720.0  # faster visual updates keep interpolation smooth
        self.metrics_file = metrics_file
        self.near_collision_events = 0
        self.num_active = min(num_robots, len(self.work_stn_pos))

        logger.info("Initializing CBS demo with %s robots", self.num_active)

        self.active_robot_indices = list(range(self.num_active))
        self.body_ids = self._spawn_bodies()
        self.demo_bots: List[CBSDemoBot] = []

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
        endpoints_pos = utils.create_struct_urdf(
            self.endpoints,
            "assets/warehouse/endpoints.urdf",
            grid_z=0.3,
            box_color=(1, 0.6, 0.0, 0.9),
            has_collison=False,
        )

        p.loadURDF("assets/warehouse/wall.urdf", useFixedBase=1, flags=p.URDF_MERGE_FIXED_LINKS)
        p.loadURDF("assets/warehouse/workstations.urdf", useFixedBase=1, flags=p.URDF_MERGE_FIXED_LINKS)
        p.loadURDF("assets/warehouse/shelves.urdf", useFixedBase=1, flags=p.URDF_MERGE_FIXED_LINKS)
        p.loadURDF("assets/warehouse/endpoints.urdf", useFixedBase=1, flags=p.URDF_MERGE_FIXED_LINKS)

        return wall_pos, work_stn_pos, shelves_pos, endpoints_pos

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

        chosen_endpoints = rng.choice(len(self.endpoints_pos), size=self.num_active, replace=False)
        endpoint_list = [self.endpoints_pos[i] for i in chosen_endpoints]

        to_pick_specs = []
        to_drop_specs = []
        self.order_metadata = {}

        for idx, robot_idx in enumerate(self.active_robot_indices):
            work_pos = self.work_stn_pos[robot_idx]
            endpoint_pos = endpoint_list[idx]

            start_grid = self.grid_map.world_to_grid(work_pos[0], work_pos[1])
            pickup_grid = self.grid_map.world_to_grid(endpoint_pos[0], endpoint_pos[1])

            to_pick_specs.append(AgentSpec(agent_id=idx, start=start_grid, goal=pickup_grid))
            to_drop_specs.append(AgentSpec(agent_id=idx, start=pickup_grid, goal=start_grid))
            self.order_metadata[idx] = {
                "endpoint": (endpoint_pos[0], endpoint_pos[1]),
                "workstation": (work_pos[0], work_pos[1]),
            }

        pickup_paths = planner.plan_paths(to_pick_specs)
        drop_paths = planner.plan_paths(to_drop_specs)

        for idx, body_id in enumerate(self.body_ids):
            pick_path = pickup_paths[idx]
            drop_path = drop_paths[idx]

            path_grid = pick_path + drop_path[1:]
            path_world = planner.grid_path_to_world(path_grid)

            schedule = [(x, y) for (x, y) in path_world]
            pickup_index = len(pick_path) - 1

            metadata = self.order_metadata[idx]
            bot = CBSDemoBot(
                body_id=body_id,
                schedule=schedule,
                step_duration=self.step_duration,
                pickup_index=pickup_index,
                order_id=idx,
                endpoint=metadata["endpoint"],
                workstation=metadata["workstation"],
            )
            self.demo_bots.append(bot)

        logger.info("CBS plans generated successfully for %s robots", len(self.demo_bots))

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

        summary_clearance = "NA" if math.isinf(min_clearance) else round(min_clearance, 4)
        pickups = [bot.pickup_time for bot in self.demo_bots if bot.pickup_time is not None]
        deliveries = [bot.delivery_time for bot in self.demo_bots if bot.delivery_time is not None]
        total_orders = len(self.demo_bots)
        completed = len(deliveries)
        pending = total_orders - completed
        avg_pickup = round(sum(pickups) / len(pickups), 4) if pickups else ""
        avg_delivery = round(sum(deliveries) / len(deliveries), 4) if deliveries else ""

        fieldnames = [
            "total_orders",
            "orders_delivered",
            "orders_pending",
            "avg_pickup_time",
            "avg_delivery_time",
            "total_sim_duration",
            "min_clearance",
            "near_collision_events",
        ]

        with open(self.metrics_file, "w", newline="") as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerow({
                "total_orders": total_orders,
                "orders_delivered": completed,
                "orders_pending": pending,
                "avg_pickup_time": avg_pickup,
                "avg_delivery_time": avg_delivery,
                "total_sim_duration": round(total_duration, 4),
                "min_clearance": summary_clearance,
                "near_collision_events": self.near_collision_events,
            })
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
            endpoint = np.round(meta["endpoint"], 2)
            workstation = np.round(meta["workstation"], 2)
            print(f"  - Robot {idx}: workstation {workstation} -> endpoint {endpoint} -> workstation")
        print("\nWatching collision-free trajectories...\n")

        min_clearance = math.inf
        while sim_time < total_duration:
            for bot in self.demo_bots:
                bot.update(sim_time)
            p.stepSimulation()
            time.sleep(self.time_step)
            sim_time += self.time_step

            current_clearance = self._pairwise_clearance()
            min_clearance = min(min_clearance, current_clearance)
            if current_clearance < 0.35:
                self.near_collision_events += 1

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
        default=6,
        help="Number of robots to include in the demo (default: 6)",
    )
    parser.add_argument(
        "--step-duration",
        type=float,
        default=0.2,
        help="Seconds allocated per CBS grid step (smaller = faster)",
    )
    parser.add_argument(
        "--metrics-file",
        type=str,
        default=None,
        help="Optional CSV path to dump pickup/delivery timings and summary stats",
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
