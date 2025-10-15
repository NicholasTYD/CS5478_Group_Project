class Robot:
    def __init__(self, id, height, radius):
        self.robot_radius = radius
        self.robot_height = height
        self.pos = id

        self.target_pos = []
        self.curr_pos = []