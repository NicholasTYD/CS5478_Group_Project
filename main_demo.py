"""
Enhanced demo version of the warehouse simulation with better visualization.
This version has:
- More visible endpoints (green instead of transparent blue)
- Multiple orders (10 orders in first 100 steps)
- Colored robots for easier tracking
- Debug info printed to console
"""

import pybullet as p
import time
from collisions import CollisionChecker
from robot import Robot, RobotTask
import utils
import numpy as np
import logging
from metrics import MetricsCollector

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)
rng = np.random.default_rng(seed=42)

# Import pathfinding
from pathfinding import create_warehouse_grid, AStarPlanner

class Simulator:
    def __init__(self):
        # 'connecting' to the physics simulation
        self.physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
        p.setGravity(0,0,-10)

        rows, cols, work_stns, shelves, endpoints = utils.get_default_warehouse_params()
        self.wall_pos, self.work_stns_pos, self.shelves_pos, self.endpoints_pos, \
            walls_struct_id, shelves_struct_id  = self._load_map(rows, cols, work_stns, shelves, endpoints)

        # Create occupancy grid for pathfinding
        x_offset = 0 if cols % 2 == 0 else 0.5
        y_offset = 0 if rows % 2 == 0 else 0.5
        # Use 0.2m buffer instead of full 0.3m radius to avoid blocking endpoints near walls
        # This provides clearance while keeping paths accessible
        robot_radius = 0.2  # Lighter inflation than actual robot radius (0.3m)
        self.grid_map = create_warehouse_grid(rows, cols, self.wall_pos, self.shelves_pos,
                                               cell_size=1.0, offset=(x_offset, y_offset),
                                               robot_radius=robot_radius,
                                               endpoints_pos=self.endpoints_pos,
                                               work_stns_pos=self.work_stns_pos)
        self.planner = AStarPlanner(self.grid_map)
        print("\n" + "="*60)
        print("WAREHOUSE SIMULATION INITIALIZED")
        print("="*60)
        print(f"Grid size: {rows} x {cols}")
        print(f"Workstations: {len(self.work_stns_pos)}")
        print(f"Shelves (obstacles): {len(self.shelves_pos)}")
        print(f"Endpoints (pickup dots): {len(self.endpoints_pos)} scattered points")
        print("Occupancy grid created for A* pathfinding")
        print("="*60 + "\n")

        self.max_sim_steps = 9999
        self.curr_sim_step = 0

        # DEMO: Create 20 orders spread across first 200 steps for better visualization
        # This ensures all robots get at least one task
        self.order_creation_times = sorted(rng.choice(range(1, 200), 20, replace=False))
        print(f"📦 Orders scheduled at steps: {self.order_creation_times}\n")
        self.order_idx = 0 # Just a pointer for delivery_creation_times arr (A helper)

        self.robots = []
        self.work_stn_robot_dict = {}

        # Initialize metrics collector
        num_robots = len(self.work_stns_pos)
        self.metrics = MetricsCollector(num_robots=num_robots, time_step=1/240.0)
        print(f"📊 Metrics collector initialized for {num_robots} robots\n")

        # Robot colors for visual distinction
        robot_colors = [
            [0.2, 0.2, 1.0, 1],   # Blue
            [1.0, 0.2, 0.2, 1],   # Red
            [0.2, 1.0, 0.2, 1],   # Green
            [1.0, 1.0, 0.2, 1],   # Yellow
            [1.0, 0.5, 0.0, 1],   # Orange
            [0.5, 0.0, 1.0, 1],   # Purple
            [0.0, 1.0, 1.0, 1],   # Cyan
            [1.0, 0.0, 1.0, 1],   # Magenta
        ]

        for idx, stn_pos in enumerate(self.work_stns_pos):
            robot_pos = stn_pos.copy()
            robot_pos[2] = 0
            robot_color = robot_colors[idx % len(robot_colors)]
            robot_obj = self._load_robot(robot_pos, self.planner, robot_color)
            self.robots.append(robot_obj)
            # Convert np_array into a hashable type
            self.work_stn_robot_dict[tuple(stn_pos.tolist())] = robot_obj

        self.collision_checker = CollisionChecker(self.robots, [walls_struct_id, shelves_struct_id])

        self._load_camera()

        print("🤖 All robots loaded and ready!\n")

    def run(self):
        print("▶️  Starting simulation...\n")
        try:
            while self.curr_sim_step < self.max_sim_steps:
                sim.step()
                time.sleep(1./240.)
                self.curr_sim_step += 1

                # Check if all orders are completed (optional early stopping)
                if (self.order_idx >= len(self.order_creation_times) and
                    self.metrics.orders_completed >= len(self.order_creation_times)):
                    print(f"\n✓ All {self.metrics.orders_completed} orders completed at step {self.curr_sim_step}!")
                    # Give some buffer time to ensure all metrics are recorded
                    buffer_steps = 100
                    for _ in range(buffer_steps):
                        p.stepSimulation()
                        self.curr_sim_step += 1
                        queue_lengths = [robot.task_queue.qsize() for robot in self.robots]
                        self.metrics.step(queue_lengths)
                    break

        except KeyboardInterrupt:
            print("\n⚠️  Simulation interrupted by user")

        # Print and export metrics
        self.finalize_metrics()

    def finalize_metrics(self):
        """Print summary and export metrics after simulation."""
        print("\n" + "="*70)
        print("SIMULATION COMPLETE - GENERATING METRICS")
        print("="*70 + "\n")

        # Print summary to console
        self.metrics.print_summary()

        # Export metrics to files
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.metrics.export_to_csv(f"metrics_summary_{timestamp}.csv")
        self.metrics.export_to_json(f"metrics_full_{timestamp}.json", include_events=True, include_step_data=True)
        self.metrics.export_timeseries_csv(f"metrics_timeseries_{timestamp}.csv")

        print(f"\n✓ Metrics exported with timestamp: {timestamp}")

    def step(self):
        robot_robot_collision_ids, robot_obstacle_collison_ids = self.collision_checker.check_robot_collisions()
        for object_ids, contact_points in robot_robot_collision_ids.items():
            body_a_id, body_b_id = object_ids
            print(f"WARNING: Robot-Robot Collision between {body_a_id} and {body_b_id}: {len(contact_points)} points")
            self.metrics.robot_robot_collided(body_a_id, body_b_id, contact_points)

        for object_ids, contact_points in robot_obstacle_collison_ids.items():
            body_a_id, body_b_id = object_ids
            print(f"WARNING: Robot-Obstacle Collision between {body_a_id} and {body_b_id}: {len(contact_points)} points")
            self.metrics.robot_obstacle_collided(body_a_id, body_b_id, contact_points)

        # Create a delivery order if scheduled in this sim step.
        while self.order_idx < len(self.order_creation_times):
            next_order_time = self.order_creation_times[self.order_idx]
            if next_order_time == self.curr_sim_step:
                self.create_delivery_order(self.order_idx)
                self.order_idx += 1
            else:
                break

        p.stepSimulation()
        for robot in self.robots:
            robot.act()

        # Record queue lengths and update metrics for this step
        queue_lengths = [robot.task_queue.qsize() for robot in self.robots]
        self.metrics.step(queue_lengths)

    def create_delivery_order(self, id):
        # This is where we implement the scheduling system for the robots.
        # For now we implement a basic 'deterministic' algorithm where it doesn't depend on how fast a robot completes
        # a delivery order to ensure that we can accurate measure benchmark algos.
        # Can consider improving this scheduling algorithm if we have spare time in the future.

        # Log order creation to metrics
        self.metrics.order_created(id)

        # Randomly allocate an endpoint for a robot to collect an item from
        chosen_endpoint_pos = rng.choice(self.endpoints_pos)
        # That means we can't schedule so that orders prioritize idling robots for example
        # So, use round robin for deterministic choice of robot and work_stns
        chosen_work_stn_pos = self.work_stns_pos[self.order_idx % len(self.work_stns_pos)]
        allocated_robot = self.work_stn_robot_dict[tuple(chosen_work_stn_pos.tolist())]
        task = RobotTask(id, chosen_endpoint_pos, chosen_work_stn_pos)
        allocated_robot.add_task(task)


    def _load_map(self, rows, cols, work_stn_arr, shelves_arr, endpoints_arr):
        # We offset the floor to align with the local coordinates instead of the global coordinates
        x_offset = 0 if cols % 2 == 0 else 0.5
        y_offset = 0 if rows % 2 == 0 else 0.5
        floor_base_pos = [x_offset, y_offset, 0]
        planeId = p.loadURDF("assets/plane/plane.urdf", basePosition=floor_base_pos)

        whouse_map = np.zeros([rows + 2, cols + 2])

        # Create border walls
        whouse_map[0,:] = 1
        whouse_map[-1,:] = 1
        whouse_map[:,0] = 1
        whouse_map[:,-1] = 1

        wall_pos = utils.create_struct_urdf(whouse_map, "assets/warehouse/wall.urdf", grid_z=3, box_color=(0.1, 0.1, 0.1, 1))
        # Workstations (DELIVERY POINTS) - Bright pink/magenta, more opaque for visibility
        work_stns_pos = utils.create_struct_urdf(work_stn_arr, "assets/warehouse/workstations.urdf", grid_z=1.5, box_color=(1, 0.2, 0.6, 0.5), has_collison=False)
        # Shelves with smaller collision boxes (0.7m) for robot clearance
        shelves_pos = utils.create_struct_urdf(shelves_arr, "assets/warehouse/shelves.urdf", grid_z=1, box_color=(0.3, 0.3, 0.3, 0.9), collision_scale=0.7)

        # Endpoints (PICKUP POINTS) - Small visible dots (red/orange for easy visibility)
        endpoints_pos = utils.create_struct_urdf(endpoints_arr, "assets/warehouse/endpoints.urdf", grid_z=0.3, box_color=(1, 0.5, 0, 0.3), has_collison=False)

        wall_struct_id = p.loadURDF("assets/warehouse/wall.urdf", useFixedBase=1, flags=p.URDF_MERGE_FIXED_LINKS)
        workstn_struct_id = p.loadURDF("assets/warehouse/workstations.urdf", useFixedBase=1, flags=p.URDF_MERGE_FIXED_LINKS)
        shelves_struct_id = p.loadURDF("assets/warehouse/shelves.urdf", useFixedBase=1, flags=p.URDF_MERGE_FIXED_LINKS)
        endpoints_struct_id = p.loadURDF("assets/warehouse/endpoints.urdf", useFixedBase=1, flags=p.URDF_MERGE_FIXED_LINKS)

        return wall_pos, work_stns_pos, shelves_pos, endpoints_pos, wall_struct_id, shelves_struct_id

    def _load_robot(self, pos, planner, color=[0.2, 0.2, 1, 1]):
        try:
            robot_radius = 0.3
            robot_height = 0.2

            # Create collision and visual shapes
            robot_collision = p.createCollisionShape(p.GEOM_CYLINDER, radius=robot_radius, height=robot_height)
            robot_visual = p.createVisualShape(p.GEOM_CYLINDER, radius=robot_radius, length=robot_height,
                                              rgbaColor=color)

            # Create the robot body
            robot_id = p.createMultiBody(
                baseMass=10,
                baseCollisionShapeIndex=robot_collision,
                baseVisualShapeIndex=robot_visual,
                basePosition=pos,
                baseOrientation=p.getQuaternionFromEuler([0, 0, 0])
            )

            return Robot(robot_id, planner, self.metrics)
        except Exception as e:
            print(f"Error creating robot: {e}")
            return None

    def _load_camera(self):
        # Init Camera - better overhead view
        p.resetDebugVisualizerCamera(
            cameraDistance=35,      # slightly further for full view
            cameraYaw=180,          # left-right rotation (degrees)
            cameraPitch=-89,        # almost top-down view
            cameraTargetPosition=[0, 0, 0]  # warehouse center
        )

if __name__ == "__main__":
    print("\n" + "🏭 " * 20)
    print("WAREHOUSE ROBOT SIMULATION - DEMO MODE")
    print("🏭 " * 20 + "\n")
    print("CONTROLS:")
    print("  Left Mouse Drag  - Rotate view")
    print("  Right Mouse Drag - Pan view")
    print("  Mouse Wheel      - Zoom in/out")
    print("\nCOLOR LEGEND:")
    print("  🟦 Blue/Red/etc cylinders - Robots (different colors)")
    print("  ⬛ Dark gray boxes       - Walls (boundaries)")
    print("  🟫 Gray boxes (1m tall)  - Shelves (OBSTACLES - robots avoid these)")
    print("  🟪 Pink boxes (1.5m)     - Workstations (DELIVERY POINTS on edges)")
    print("  🟠 Orange dots (0.3m)    - Endpoints (PICKUP POINTS)")
    print("\n⚠️  No physical cargo boxes - pickup/delivery is tracked by console logs")
    print("\nROBOT TASK FLOW:")
    print("  1. Robot starts at 🟪 Pink workstation")
    print("  2. Navigates to 🟠 Orange pickup point → Console shows 'COLLECTED'")
    print("  3. Returns to 🟪 Pink workstation → Console shows 'DELIVERED'")
    print("\n💡 Watch robots navigate around 🟫 gray shelves to reach 🟠 orange dots!")
    print("\n" + "-" * 60 + "\n")

    sim = Simulator()
    sim.run()
