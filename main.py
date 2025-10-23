import pybullet as p
import time
from robot import Robot, RobotTask
import utils
import numpy as np
import logging

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)
rng = np.random.default_rng(seed=42)

class Simulator:
    def __init__(self):
        # 'connecting' to the physics simulation
        self.physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
        p.setGravity(0,0,-10)

        rows, cols, work_stns, shelves, endpoints = utils.get_default_warehouse_params()
        self.wall_pos, self.work_stns_pos, self.shelves_pos, self.endpoints_pos = self._load_map(rows, cols, work_stns, shelves, endpoints)

        self.max_sim_steps = 3000
        self.curr_sim_step = 0

        # Create a list of time steps where orders should be created. Should be in ascending order.
        self.order_creation_times = sorted(rng.choice(range(self.max_sim_steps), 150))
        print(self.order_creation_times)
        self.order_idx = 0 # Just a pointer for delivery_creation_times arr (A helper)
        
        self.robots = []
        self.work_stn_robot_dict = {}
        for stn_pos in self.work_stns_pos:
            robot_pos = stn_pos.copy()
            robot_pos[2] = 0
            robot_obj = self._load_robot(robot_pos)
            self.robots.append(robot_obj)
            # Convert np_array into a hashable type
            self.work_stn_robot_dict[tuple(stn_pos.tolist())] = robot_obj
        
        self._load_camera()

    def run(self):
        while self.curr_sim_step < self.max_sim_steps:
            sim.step()
            time.sleep(1./240.)
            self.curr_sim_step += 1

    def step(self):
        # Create a delivery order if scheduled in this sim step.
        while self.order_idx < len(self.order_creation_times):
            next_order_time = self.order_creation_times[self.order_idx]
            if next_order_time == self.curr_sim_step:
                self.create_delivery_order()
                self.order_idx += 1
            else:
                break

        p.stepSimulation()
        for robot in self.robots:
            robot.act()

    def create_delivery_order(self):
        # This is where we implement the scheduling system for the robots.
        # For now we implement a basic 'deterministic' algorithm where it doesn't depend on how fast a robot completes
        # a delivery order to ensure that we can accurate measure benchmark algos.
        # Can consider improving this scheduling algorithm if we have spare time in the future.

        # Randomly allocate an endpoint for a robot to collect an item from
        chosen_endpoint_pos = rng.choice(self.endpoints_pos)
        # That means we can't schedule so that orders prioritize idling robots for example
        # So, use round robin for deterministic choice of robot and work_stns
        chosen_work_stn_pos = self.work_stns_pos[self.order_idx % len(self.work_stns_pos)]
        allocated_robot = self.work_stn_robot_dict[tuple(chosen_work_stn_pos.tolist())]
        task = RobotTask(chosen_endpoint_pos, chosen_work_stn_pos)
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
        work_stns_pos = utils.create_struct_urdf(work_stn_arr, "assets/warehouse/workstations.urdf", grid_z=1.25, box_color=(1, 0, 0.5, 0.5), has_collison=False)
        shelves_pos = utils.create_struct_urdf(shelves_arr, "assets/warehouse/shelves.urdf", grid_z=1, box_color=(0.3, 0.3, 0.3, 0.9))
        endpoints_pos = utils.create_struct_urdf(endpoints_arr, "assets/warehouse/endpoints.urdf", grid_z=0.1, box_color=(0, 0, 1, 0.1), has_collison=False)

        walls = p.loadURDF("assets/warehouse/wall.urdf", useFixedBase=1, flags=p.URDF_MERGE_FIXED_LINKS)
        work_stns = p.loadURDF("assets/warehouse/workstations.urdf", useFixedBase=1, flags=p.URDF_MERGE_FIXED_LINKS)
        shelves = p.loadURDF("assets/warehouse/shelves.urdf", useFixedBase=1, flags=p.URDF_MERGE_FIXED_LINKS)
        endpoints = p.loadURDF("assets/warehouse/endpoints.urdf", useFixedBase=1, flags=p.URDF_MERGE_FIXED_LINKS)

        return wall_pos, work_stns_pos, shelves_pos, endpoints_pos
    
    def _load_robot(self, pos):
        try:
            robot_radius = 0.3
            robot_height = 0.2
            
            # Create collision and visual shapes
            robot_collision = p.createCollisionShape(p.GEOM_CYLINDER, radius=robot_radius, height=robot_height)
            robot_visual = p.createVisualShape(p.GEOM_CYLINDER, radius=robot_radius, length=robot_height, 
                                              rgbaColor=[0.2, 0.2, 1, 1])
            
            # Create the robot body
            robot_id = p.createMultiBody(
                baseMass=10,
                baseCollisionShapeIndex=robot_collision,
                baseVisualShapeIndex=robot_visual,
                basePosition=pos,
                baseOrientation=p.getQuaternionFromEuler([0, 0, 0])
            )
            
            return Robot(robot_id)
        except Exception as e:
            print(f"Error creating robot: {e}")
            return None
        
        # base_ori = p.getQuaternionFromEuler([0, 0, 0])
        # robot_id = p.loadURDF(
        #     "assets/headless_fetch/fetch.urdf",
        #     pos,
        #     base_ori,
        #     # useFixedBase=True,
        # )
        # robot_collison = p.createCollisionShape(p.GEOM_CYLINDER, radius=robot_radius, height=robot_height)
        # robot_visual = p.createVisualShape(p.GEOM_CYLINDER, radius=robot_radius, length=robot_height, 
        #                                    rgbaColor=[0.2, 0.2, 1, 1])
        # robot_id = p.createMultiBody(baseMass=1,
        #                         baseCollisionShapeIndex=robot_collison,
        #                         baseVisualShapeIndex=robot_visual,
        #                         basePosition=pos.tolist(),
        #                         baseOrientation=p.getQuaternionFromEuler([0,0,0]))
        # return Robot(robot_id)

    def _load_camera(self):
        # Init Camera
        p.resetDebugVisualizerCamera(
            cameraDistance=30,      # how far away the camera is
            cameraYaw=180,          # left-right rotation (degrees)
            cameraPitch=265,       # up-down rotation (degrees)
            cameraTargetPosition=[0, 0, 0]  # what the camera looks at
        )

sim = Simulator()
# startPos = [5, 2 ,0]
# startOrientation = p.getQuaternionFromEuler([0,0,0])
# boxId = p.loadURDF("assets/box/box.urdf",startPos, startOrientation)
sim.run()
