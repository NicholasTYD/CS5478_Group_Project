import pybullet as p

class Robot:
    def __init__(self, id):
        # self.robot_radius = 0.4
        # self.robot_height = 0.25
        self.id = id

        self.pos, self.orn = [], []

    def act(self):
        self.pos, self.orn = p.getBasePositionAndOrientation(self.id)
        print(self.pos)
        # self.id.getBasePositionAndOrientation()

        # print(p.getNumJoints(self.id))        
        # p.setJointMotorControlArray(
        #     bodyUniqueId=self.id,
        #     jointIndices=[],
        #     controlMode=p.POSITION_CONTROL,

        #     # odyUniqueId=self.robot_id,
        #     
        #     # 
        #     # controlMode=p.PD_CONTROL,
        #   targetPositions=target.tolist(),
        #     # targetVelocities=[0.0] * len(self.joint_indices),
        #     # forces=[self.max_force] * len(self.joint_indices),
        #     # positionGains=[0.3] * len(self.joint_indices),
        #     # positionGains=[2, 2, 2],
        #     # veloci
        # )
        pass
