import pybullet as p
import numpy as np

class Robot:
    def __init__(self, id):
        # self.robot_radius = 0.4
        # self.robot_height = 0.25
        self.id = id
        self.pos, self.orn = [], []

        n_joints = p.getNumJoints(self.id)
        indices = []
        lowers, uppers, ranges, rest = [], [], [], []

        
        # Code below from default project
        self.max_force = 2000000

        # Collect joint info (skip fixed)
        for j in range(n_joints):
            info = p.getJointInfo(self.id, j)
            joint_type = info[2]
            if joint_type in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
                indices.append(j)
                lowers.append(info[8])
                uppers.append(info[9])
                ranges.append(info[9] - info[8])
                rest.append(info[10])  # joint damping? (PyBullet packs different things; we keep a placeholder)

                p.setJointMotorControl2(
                    self.id, j, p.VELOCITY_CONTROL, force=0.0,
                )

        self.joint_indices = indices
        self.joint_lower = np.array(lowers, dtype=np.float32)
        self.joint_upper = np.array(uppers, dtype=np.float32)
        self.joint_ranges = np.array(ranges, dtype=np.float32)
        self.rest_poses = np.zeros_like(self.joint_lower, dtype=np.float32)
        
        # self.joint_lower[self.joint_upper==-1] = -np.inf
        # self.joint_upper[self.joint_upper==-1] = np.inf
        # self.joint_ranges[self.joint_upper==np.inf] = np.inf
        
        # print("Controllable joints:", len(self.joint_indices))
        # print("Joint Indices:", self.joint_indices)
        # print("Joint Lower:", self.joint_lower)
        # print("Joint Upper:", self.joint_upper)

    def act(self):
        self.pos, self.orn = p.getBasePositionAndOrientation(self.id)
        # print(self.pos, self.orn)
        
        # Read the setJointMotorControlArray pybullet quickstart for how this works
        # But TLDR there's 2 modes, one that moves the robot using velocity p.VELOCITY_CONTROL,
        # and another based on moving the robot to a set positon p.POSITION_CONTROL
        #
        # targetVelocities/targetPositions: The actual velocity/positon you want
        # velocityGains/positionGains: Something to smooth out the motion
        p.setJointMotorControlArray(
            bodyUniqueId=self.id,
            jointIndices=self.joint_indices,
            forces=[self.max_force] * len(self.joint_indices), # 

            controlMode=p.VELOCITY_CONTROL,
            targetVelocities=[2, 0, 0],
            # velocityGains=[123213, 12321 ,31321],

            # controlMode=p.POSITION_CONTROL,
            # targetPositions=[0, 5, 0],
            # positionGains=[0.003] * len(self.joint_indices),
        )
        pass
