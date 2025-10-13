import pybullet as p
import time
import utils
import numpy as np

# 'connecting' to the physics simulation
physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setGravity(0,0,-10)

# load the scene
# Layout obtained from here
# https://ojs.aaai.org/index.php/SOCS/article/view/31593/33753
rows, cols, work_stns, shelves = utils.get_default_warehouse_params()
cube_pos, work_stns_pos, shelves_pos = utils.init_scene(rows, cols, work_stns, shelves)

# Init the robots
work_stns_pos += [0, 0, 1] # TODO: Remove the 0,0,1 later
robot_ids = []
for stn in work_stns_pos:
    robot_radius = 0.4
    robot_height = 0.25
    
    robot_collison = p.createCollisionShape(p.GEOM_CYLINDER, radius=robot_radius, height=robot_height)
    robot_visual = p.createVisualShape(p.GEOM_CYLINDER, radius=robot_radius, length=robot_height, 
                                       rgbaColor=[0.2, 0.2, 1, 1])
    
    bodyId = p.createMultiBody(baseMass=1,
                            baseCollisionShapeIndex=robot_collison,
                            baseVisualShapeIndex=robot_visual,
                            basePosition=stn.tolist(),
                            baseOrientation=p.getQuaternionFromEuler([0,0,0]))
    robot_ids.append(bodyId)

# Init Camera
p.resetDebugVisualizerCamera(
    cameraDistance=30,      # how far away the camera is
    cameraYaw=0,          # left-right rotation (degrees)
    cameraPitch=269,       # up-down rotation (degrees)
    cameraTargetPosition=[0, 0, 0]  # what the camera looks at
)

startPos = [0, 0 ,4]
startOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("assets/box/box.urdf",startPos, startOrientation)

# run the simulation
while True:
    p.stepSimulation()
    # cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
    # print(cubePos,cubeOrn)
    time.sleep(1./240.)
    
p.disconnect()