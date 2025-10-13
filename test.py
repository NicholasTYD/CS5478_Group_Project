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
rows, cols = 36, 33
work_stns = np.zeros([rows, cols])
work_stns[-1, 
          1:-1:3] = 1
work_stns[0,
          1:-1:3] = 1

shelves = np.zeros([rows, cols])
shelves[2: -1,
        2:-2:4] = 1

utils.init_scene(rows, cols, work_stns, shelves)

startPos = [0,0,4]
startOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("assets/box/box.urdf",startPos, startOrientation)

# run the simulation
for i in range (10000):
    p.stepSimulation()
    cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
    # print(cubePos,cubeOrn)
    time.sleep(1./240.)
    
p.disconnect()