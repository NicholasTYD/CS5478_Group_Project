import pybullet as p
import time
import math

# Connect to PyBullet
physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -10)

# Load a simple plane
p.loadURDF("assets/plane/plane.urdf", basePosition=[0, 0, 0])

# Create a simple robot
robot_collision = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.3, height=0.5)
robot_visual = p.createVisualShape(p.GEOM_CYLINDER, radius=0.3, length=0.5, 
                                  rgbaColor=[0.2, 0.2, 1, 1])
robot_id = p.createMultiBody(
    baseMass=10,
    baseCollisionShapeIndex=robot_collision,
    baseVisualShapeIndex=robot_visual,
    basePosition=[0, 0, 0.5]
)

print(f"Robot created with ID: {robot_id}")

# Set camera
p.resetDebugVisualizerCamera(
    cameraDistance=10,
    cameraYaw=45,
    cameraPitch=-30,
    cameraTargetPosition=[0, 0, 0]
)

start_time = time.time()
step_count = 0

print("Robot should be moving in a circle...")

try:
    while step_count < 2000:  # Run for 2000 steps
        # Get robot position
        pos, orn = p.getBasePositionAndOrientation(robot_id)
        x, y, z = pos
        
        # Calculate movement
        current_time = time.time()
        time_factor = (current_time - start_time) * 0.5
        
        # Robot moves in a circle
        desired_vx = 2.0 * math.sin(time_factor)
        desired_vy = 2.0 * math.cos(time_factor)
        
        # Apply velocity
        p.resetBaseVelocity(robot_id, [desired_vx, desired_vy, 0], [0, 0, 0])
        
        # Step simulation
        p.stepSimulation()
        step_count += 1
        
        # Print position every 200 steps
        if step_count % 200 == 0:
            print(f"Step {step_count}: Robot at X={x:.2f}, Y={y:.2f}, Z={z:.2f}")
            print(f"  Velocity: X={desired_vx:.2f}, Y={desired_vy:.2f}")
        
        time.sleep(1./240.)
        
except KeyboardInterrupt:
    print("Demo interrupted")

finally:
    p.disconnect()
    print("Debug movement completed")
