import pybullet as p
import time
from robot import Robot
import utils
import numpy as np

class Simulator:
    def __init__(self, seed=0):
        # 'connecting' to the physics simulation
        self.physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
        p.setGravity(0,0,-10)

        rows, cols, work_stns, shelves = utils.get_default_warehouse_params()
        self.wall_pos, self.work_stns_pos, self.shelves = self._load_map(rows, cols, work_stns, shelves)

        self.robots = []
        for stn_pos in self.work_stns_pos:
            robot_pos = stn_pos.copy()
            robot_pos[2] = 0
            self.robots.append(self._load_robot(robot_pos))
        self._load_camera()

    def step(self):
        p.stepSimulation()
        time.sleep(1./240.)

    def _load_map(self, rows, cols, work_stn_arr, shelves_arr):
        # We offset the floor to align with the local coordinates instead of the global coordinates
        floor_base_pos = [(rows%2+1)/2, (cols%2+1)/2, 0]
        planeId = p.loadURDF("assets/plane/plane.urdf", basePosition=floor_base_pos)

        whouse_map = np.zeros([rows + 2, cols + 2])

        # Create border walls
        whouse_map[0,:] = 1
        whouse_map[-1,:] = 1
        whouse_map[:,0] = 1
        whouse_map[:,-1] = 1

        wall_pos = utils.create_struct_urdf(whouse_map, "assets/warehouse/wall.urdf", grid_z=3, box_color=(0.1, 0.1, 0.1, 1))
        work_stns_pos = utils.create_struct_urdf(work_stn_arr, "assets/warehouse/endpoints.urdf", grid_z=1.25, box_color=(1, 0, 0.5, 0.5), has_collison=False)
        shelves_pos = utils.create_struct_urdf(shelves_arr, "assets/warehouse/shelves.urdf", grid_z=1, box_color=(0.3, 0.3, 0.3, 0.9))

        wh = p.loadURDF("assets/warehouse/wall.urdf", useFixedBase=1, flags=p.URDF_MERGE_FIXED_LINKS)
        endpoints = p.loadURDF("assets/warehouse/endpoints.urdf", useFixedBase=1, flags=p.URDF_MERGE_FIXED_LINKS)
        shelves = p.loadURDF("assets/warehouse/shelves.urdf", useFixedBase=1, flags=p.URDF_MERGE_FIXED_LINKS)

        return wall_pos, work_stns_pos, shelves_pos
    
    def _load_robot(self, pos):
        robot_radius = 0.4
        robot_height = 0.25

        robot_collison = p.createCollisionShape(p.GEOM_CYLINDER, radius=robot_radius, height=robot_height)
        robot_visual = p.createVisualShape(p.GEOM_CYLINDER, radius=robot_radius, length=robot_height, 
                                           rgbaColor=[0.2, 0.2, 1, 1])

        robot_id = p.createMultiBody(baseMass=1,
                                baseCollisionShapeIndex=robot_collison,
                                baseVisualShapeIndex=robot_visual,
                                basePosition=pos.tolist(),
                                baseOrientation=p.getQuaternionFromEuler([0,0,0]))
        
        return Robot(robot_id, robot_height, robot_radius)

    def _load_camera(self):
        # Init Camera
        p.resetDebugVisualizerCamera(
            cameraDistance=30,      # how far away the camera is
            cameraYaw=0,          # left-right rotation (degrees)
            cameraPitch=269,       # up-down rotation (degrees)
            cameraTargetPosition=[0, 0, 0]  # what the camera looks at
        )

sim = Simulator()
# startPos = [0, 0 ,0]
# startOrientation = p.getQuaternionFromEuler([0,0,0])
# boxId = p.loadURDF("assets/box/box.urdf",startPos, startOrientation)
while True:
    # cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
    # print(cubePos,cubeOrn)
    sim.step()


