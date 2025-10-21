import pybullet as p
import time
import math
import numpy as np
from robot import Robot
import utils

class Simulator:
    def __init__(self):
        # Connect to PyBullet
        self.physicsClient = p.connect(p.GUI)
        p.setGravity(0, 0, -10)
        
        # Load warehouse environment with obstacles
        self._load_warehouse_environment()
        
        # Create robots with safe positions (avoiding obstacles)
        self.robots = []
        safe_positions = [
            [0, 0, 0.5],      # Center
            [8, 8, 0.5],      # Far corner
            [-8, -8, 0.5]     # Opposite corner
        ]
        
        print("Creating robots...")
        for i, pos in enumerate(safe_positions):
            try:
                robot = self._load_robot(pos)
                if robot:
                    self.robots.append(robot)
                    print(f"Robot {i} created successfully at {pos}")
                else:
                    print(f"Failed to create robot {i}")
            except Exception as e:
                print(f"Error creating robot {i}: {e}")
        
        print(f"Total robots created: {len(self.robots)}")
        
        # Set camera
        p.resetDebugVisualizerCamera(
            cameraDistance=20,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=[0, 0, 0]
        )

    def _load_warehouse_environment(self):
        """Load warehouse with walls, shelves, and obstacles"""
        # Load floor
        p.loadURDF("assets/plane/plane.urdf", basePosition=[0, 0, 0])
        
        # Create simple warehouse layout with obstacles
        self._create_walls()
        self._create_shelves()
        self._create_boxes()

    def _create_walls(self):
        """Create boundary walls"""
        wall_height = 2.0
        wall_thickness = 0.2
        
        # Create walls around the perimeter
        wall_positions = [
            # North wall
            [0, 15, wall_height/2],
            # South wall  
            [0, -15, wall_height/2],
            # East wall
            [15, 0, wall_height/2],
            # West wall
            [-15, 0, wall_height/2]
        ]
        
        wall_dimensions = [
            [30, wall_thickness, wall_height],  # North/South walls
            [30, wall_thickness, wall_height],  # North/South walls
            [wall_thickness, 30, wall_height],  # East/West walls
            [wall_thickness, 30, wall_height]   # East/West walls
        ]
        
        for i, (pos, dim) in enumerate(zip(wall_positions, wall_dimensions)):
            wall_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=dim)
            wall_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=dim, 
                                            rgbaColor=[0.3, 0.3, 0.3, 1])
            p.createMultiBody(
                baseMass=0,  # Static
                baseCollisionShapeIndex=wall_collision,
                baseVisualShapeIndex=wall_visual,
                basePosition=pos
            )
            print(f"Wall {i+1} created at {pos}")

    def _create_shelves(self):
        """Create shelf obstacles"""
        shelf_positions = [
            [5, 5, 1], [5, -5, 1], [-5, 5, 1], [-5, -5, 1],
            [10, 0, 1], [-10, 0, 1], [0, 10, 1], [0, -10, 1]
        ]
        
        for i, pos in enumerate(shelf_positions):
            shelf_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[1, 1, 1])
            shelf_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[1, 1, 1], 
                                              rgbaColor=[0.5, 0.3, 0.1, 1])
            p.createMultiBody(
                baseMass=0,  # Static
                baseCollisionShapeIndex=shelf_collision,
                baseVisualShapeIndex=shelf_visual,
                basePosition=pos
            )
            print(f"Shelf {i+1} created at {pos}")

    def _create_boxes(self):
        """Create movable box obstacles"""
        box_positions = [
            [2, 2, 0.5], [-2, -2, 0.5], [7, -7, 0.5], [-7, 7, 0.5]
        ]
        
        for i, pos in enumerate(box_positions):
            box_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, 0.5])
            box_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, 0.5], 
                                           rgbaColor=[0.8, 0.4, 0.2, 1])
            p.createMultiBody(
                baseMass=5,  # Movable
                baseCollisionShapeIndex=box_collision,
                baseVisualShapeIndex=box_visual,
                basePosition=pos
            )
            print(f"Box {i+1} created at {pos}")

    def _load_robot(self, pos):
        """Create a simple robot using basic shapes"""
        try:
            robot_radius = 0.3
            robot_height = 0.5
            
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

    def step(self):
        """Step the simulation"""
        try:
            for robot in self.robots:
                robot.act()
            # CRITICAL: Step the physics simulation
            p.stepSimulation()
        except Exception as e:
            print(f"Error in simulation step: {e}")

def main():
    sim = Simulator()
    
    print("Simulation started! Press 'q' to quit, 'r' to reset")
    print("Controls: Mouse to rotate view, scroll to zoom")
    print("Robots will navigate around obstacles!")
    
    step_count = 0
    max_steps = 10000
    
    try:
        while step_count < max_steps:
            # Check for keyboard input
            keys = p.getKeyboardEvents()
            if ord('q') in keys and keys[ord('q')] & p.KEY_WAS_TRIGGERED:
                print("Quitting simulation...")
                break
            elif ord('r') in keys and keys[ord('r')] & p.KEY_WAS_TRIGGERED:
                print("Resetting simulation...")
                p.resetSimulation()
                sim = Simulator()
                step_count = 0
            
            # Step simulation
            sim.step()
            step_count += 1
            
            # Print status every 1000 steps
            if step_count % 1000 == 0:
                print(f"Step {step_count}: {len(sim.robots)} robots active")
                for i, robot in enumerate(sim.robots):
                    try:
                        pos, _ = p.getBasePositionAndOrientation(robot.id)
                        print(f"  Robot {i}: Position {[round(x, 2) for x in pos]}")
                    except:
                        print(f"  Robot {i}: Position unknown")
            
            time.sleep(1./240.)
    
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user")
    
    finally:
        p.disconnect()
        print("Simulation ended")

if __name__ == "__main__":
    main()
