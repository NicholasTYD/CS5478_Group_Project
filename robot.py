import pybullet as p
import numpy as np
import time
from queue import Queue
import logging

logger = logging.getLogger(__name__)

class RobotTask:
    def __init__(self, endpoint_pos, work_stn_pos):
        self.endpoint_pos = endpoint_pos
        self.work_stn_pos = work_stn_pos

class Robot:
    def __init__(self, id):
        self.id = id
        self.pos, self.orn = [], []

        self.task_queue = Queue()
        self.curr_task = None
        
        # Simple robot - no joints, just base movement
        self.start_time = time.time()
        self.velocity = [0, 0, 0]  # [vx, vy, vz]
        
        print(f"Simple robot loaded with ID: {self.id}")

    def act(self):
        # Get its own self position and orientiaton
        self.pos, self.orn = p.getBasePositionAndOrientation(self.id)

        # Check if it is idle and there are pending tasks
        if self.curr_task is None and not self.task_queue.empty():
            self.curr_task = self.task_queue.get()
            logging.info(f'Robot {self.id} received task with endpoint at:'
                         f'{self.curr_task.endpoint_pos} and workstation at: {self.curr_task.work_stn_pos}')

        try:
            # Check if robot still exists
            if not p.getNumBodies() or self.id >= p.getNumBodies():
                print(f"Robot {self.id} no longer exists, skipping...")
                return
                
            self.pos, self.orn = p.getBasePositionAndOrientation(self.id)
        except Exception as e:
            print(f"Error getting robot {self.id} position: {e}")
            return
        
        # Enhanced boundary checking to prevent robots from going out of bounds
        x, y, z = self.pos
        warehouse_bounds = 12  # Reduced bounds to avoid walls
        
        # Calculate movement - robots should actually navigate the warehouse
        import math
        current_time = time.time()
        time_factor = (current_time - self.start_time) * 0.5  # Faster time factor
        
        # Create a wandering behavior - robots move in different directions
        robot_id = (self.id - 1) % 3  # Different behavior for each robot (IDs 1,2,3 -> 0,1,2)
        
        if robot_id == 0:
            # Robot 0: Move in a large circle
            desired_vx = 1.5 * math.sin(time_factor)
            desired_vy = 1.5 * math.cos(time_factor)
        elif robot_id == 1:
            # Robot 1: Move in a figure-8 pattern
            desired_vx = 1.2 * math.sin(time_factor)
            desired_vy = 1.2 * math.sin(time_factor * 2)
        else:
            # Robot 2: Move in a square pattern
            phase = (time_factor * 0.5) % 4
            if phase < 1:
                desired_vx, desired_vy = 1.0, 0  # Move right
            elif phase < 2:
                desired_vx, desired_vy = 0, 1.0  # Move up
            elif phase < 3:
                desired_vx, desired_vy = -1.0, 0  # Move left
            else:
                desired_vx, desired_vy = 0, -1.0  # Move down
        
        # Boundary checking - reverse direction if hitting walls
        if x < -warehouse_bounds:
            desired_vx = 1.0  # Move right
        elif x > warehouse_bounds:
            desired_vx = -1.0  # Move left
            
        if y < -warehouse_bounds:
            desired_vy = 1.0  # Move forward
        elif y > warehouse_bounds:
            desired_vy = -1.0  # Move backward
        
        # Apply velocity directly to the robot base
        try:
            p.resetBaseVelocity(self.id, [desired_vx, desired_vy, 0], [0, 0, 0])
            # Debug: print movement every 100 steps
            if hasattr(self, 'debug_counter'):
                self.debug_counter += 1
            else:
                self.debug_counter = 0
            if self.debug_counter % 100 == 0:
                print(f"Robot {self.id}: pos=({x:.1f},{y:.1f}) vel=({desired_vx:.1f},{desired_vy:.1f})")
        except Exception as e:
            print(f"Error setting robot {self.id} velocity: {e}")

    def add_task(self, task: RobotTask):
        self.task_queue.put(task)

    def check_tgt_pos_reached(self):
        robot_xy_pos = self.pos[:2]
        tgt_xy_pos = self.curr_tgt_pos[:2]

        dist_diff = np.linalg.norm(robot_xy_pos, tgt_xy_pos)
        # If distance between robot and tgt differs by this amt (disgarding z axis),
        # we consider target reached
        return dist_diff < 0.1 
    
