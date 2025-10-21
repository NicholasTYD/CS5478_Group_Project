import pybullet as p
import time
import math
from robot import Robot

class Simulator:
    def __init__(self):
        # Connect to PyBullet
        self.physicsClient = p.connect(p.GUI)
        p.setGravity(0, 0, -10)
        
        # Load a simple plane instead of complex warehouse
        p.loadURDF("assets/plane/plane.urdf", basePosition=[0, 0, 0])
        
        # Create robots with safe positions
        self.robots = []
        safe_positions = [
            [0, 0, 0.5],
            [5, 5, 0.5], 
            [-5, -5, 0.5]
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
            cameraDistance=15,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=[0, 0, 0]
        )

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
