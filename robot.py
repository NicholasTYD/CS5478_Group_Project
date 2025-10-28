import pybullet as p
import numpy as np
import time
from queue import Queue
import logging

logger = logging.getLogger(__name__)

class RobotTask:
    def __init__(self, id, endpoint_pos, work_stn_pos):
        self.id = id
        self.endpoint_pos = endpoint_pos
        self.work_stn_pos = work_stn_pos

class Robot:
    def __init__(self, id, planner=None, metrics_collector=None):
        self.id = id
        self.pos, self.orn = [], []

        self.task_queue: Queue = Queue()
        self.curr_task: RobotTask = None
        self.curr_tgt_pos = None
        self.curr_tgt_type = None # Either 'ENDPOINT', or 'WORKSTN'

        # A* pathfinding
        self.planner = planner
        self.current_path = []  # List of (x, y) waypoints in world coordinates
        self.waypoint_index = 0  # Current waypoint we're heading to
        self.waypoint_tolerance = 0.1  # Distance to consider waypoint reached (meters)

        # Simple robot - no joints, just base movement
        self.start_time = time.time()
        self.velocity = [0, 0, 0]  # [vx, vy, vz]
        self.max_speed = 5.0  # Maximum speed in m/s (increased for faster movement)

        # Metrics tracking
        self.metrics_collector = metrics_collector
        self.last_position = None  # Track position for distance calculation

        # Debug tracking
        self._debug_counter = 0

        print(f"Simple robot loaded with ID: {self.id}")

    def update_task_status(self):
        '''
        Does housekeeping regarding the tasks.
        Checks if the target coords is reached, and also updates the current task with the next one if available.
        '''
        if self.curr_tgt_pos is not None:
            has_tgt_reached = self.check_tgt_pos_reached()
            if has_tgt_reached:
                if self.curr_tgt_type == 'ENDPOINT':
                    logging.info(f'COLLECTED: Robot {self.id} collected order {self.curr_task.id} at {self.curr_task.endpoint_pos}')
                    # Log item collection to metrics
                    if self.metrics_collector:
                        self.metrics_collector.item_collected(
                            self.curr_task.id,
                            self.id,
                            tuple(self.curr_task.endpoint_pos)
                        )
                    self.curr_tgt_pos = self.curr_task.work_stn_pos
                    self.curr_tgt_type = 'WORKSTN'
                    # Plan new path to workstation
                    self.plan_path_to_target()
                else:
                    logging.info(f'DELIVERED: Robot {self.id} delivered order {self.curr_task.id} at {self.curr_task.work_stn_pos}')
                    # Log order delivery to metrics
                    if self.metrics_collector:
                        self.metrics_collector.order_delivered(
                            self.curr_task.id,
                            self.id,
                            tuple(self.curr_task.work_stn_pos)
                        )
                    self.curr_task = None
                    self.curr_tgt_pos = None
                    self.current_path = []  # Clear path when task completed

        # Check if it is idle and there are pending tasks
        if self.curr_task is None and not self.task_queue.empty():
            self.curr_task = self.task_queue.get()
            self.curr_tgt_pos = self.curr_task.endpoint_pos
            self.curr_tgt_type = 'ENDPOINT'
            logging.info(f'RECEIVED: Robot {self.id} received order {self.curr_task.id} with endpoint at:'
                         f'{self.curr_task.endpoint_pos} and workstation at: {self.curr_task.work_stn_pos}')
            # Log order assignment to metrics
            if self.metrics_collector:
                self.metrics_collector.order_assigned(self.curr_task.id, self.id)
            # Plan path to endpoint
            self.plan_path_to_target()

    def act(self):
        try:
            # Check if robot still exists
            if not p.getNumBodies() or self.id >= p.getNumBodies():
                print(f"Robot {self.id} no longer exists, skipping...")
                return

            # Get current position and orientation
            self.pos, self.orn = p.getBasePositionAndOrientation(self.id)
        except Exception as e:
            print(f"Error getting robot {self.id} position: {e}")
            return

        # Update task status (this may trigger path planning)
        self.update_task_status()

        # Update robot state for metrics (idle if no current task)
        if self.metrics_collector:
            is_idle = self.curr_task is None
            self.metrics_collector.update_robot_state(self.id, is_idle)

        # Increment debug counter
        self._debug_counter += 1

        # Follow the path if we have one
        if len(self.current_path) > 0:
            self.follow_path()
        else:
            # No path, stay still
            if self._debug_counter % 1000 == 0 and self.curr_task is not None:
                logging.warning(f"DEBUG Robot {self.id}: Has task but NO PATH! Task type: {self.curr_tgt_type}")
            try:
                p.resetBaseVelocity(self.id, [0, 0, 0], [0, 0, 0])
            except Exception as e:
                print(f"Error stopping robot {self.id}: {e}")

    def add_task(self, task: RobotTask):
        self.task_queue.put(task)

    def plan_path_to_target(self):
        """Use A* to plan a path from current position to target position."""
        if self.planner is None or self.curr_tgt_pos is None:
            logging.warning(f"Robot {self.id}: Cannot plan path (planner={self.planner}, target={self.curr_tgt_pos})")
            return

        start = (self.pos[0], self.pos[1])
        goal = (self.curr_tgt_pos[0], self.curr_tgt_pos[1])

        logging.info(f"Robot {self.id}: Planning path from {start} to {goal}")

        # Plan path using A*
        path = self.planner.plan(start, goal)

        if path is not None and len(path) > 0:
            # IMPORTANT: A* returns path to grid cell centers, not exact goal position
            # Append the exact goal as final waypoint to ensure robot reaches it
            if path[-1] != goal:
                path.append(goal)
            self.current_path = path
            self.waypoint_index = 0
            logging.info(f"Robot {self.id}: Path planned with {len(path)} waypoints (includes exact goal)")
        else:
            logging.error(f"Robot {self.id}: Failed to find path from {start} to {goal}")
            self.current_path = []

    def follow_path(self):
        """Follow the current path by moving towards the next waypoint."""
        if self.waypoint_index >= len(self.current_path):
            # Reached end of path
            logging.info(f"DEBUG Robot {self.id}: Finished path! Now at distance {np.linalg.norm(np.array(self.pos[:2]) - np.array(self.curr_tgt_pos[:2])):.3f}m from target")
            self.current_path = []
            self.waypoint_index = 0
            return

        # Get current waypoint
        waypoint = self.current_path[self.waypoint_index]
        wx, wy = waypoint

        # Calculate direction to waypoint
        x, y = self.pos[0], self.pos[1]
        dx = wx - x
        dy = wy - y
        distance = np.sqrt(dx**2 + dy**2)

        # Track distance traveled for metrics
        if self.metrics_collector and self.last_position is not None:
            last_x, last_y = self.last_position[0], self.last_position[1]
            distance_traveled = np.sqrt((x - last_x)**2 + (y - last_y)**2)
            self.metrics_collector.robot_moved(self.id, distance_traveled)

        # Update last position for next step
        self.last_position = self.pos

        # Check if waypoint reached
        # SIMPLE: Use consistent tolerance for all waypoints
        tolerance = self.waypoint_tolerance

        if distance < tolerance:
            self.waypoint_index += 1
            if self.waypoint_index >= len(self.current_path):
                # Path complete
                logging.info(f"DEBUG Robot {self.id}: Finished path! Final distance: {distance:.3f}m")
                self.current_path = []
                self.waypoint_index = 0
                try:
                    p.resetBaseVelocity(self.id, [0, 0, 0], [0, 0, 0])
                except Exception as e:
                    print(f"Error stopping robot {self.id}: {e}")
            return

        # Calculate velocity towards waypoint
        if distance > 0:
            # Normalize direction and scale by max speed
            vx = (dx / distance) * self.max_speed
            vy = (dy / distance) * self.max_speed
        else:
            vx, vy = 0, 0

        # Apply velocity
        try:
            p.resetBaseVelocity(self.id, [vx, vy, 0], [0, 0, 0])
        except Exception as e:
            print(f"Error setting robot {self.id} velocity: {e}")

    def check_tgt_pos_reached(self):
        robot_xy_pos = self.pos[:2]
        tgt_xy_pos = self.curr_tgt_pos[:2]

        diff = robot_xy_pos - tgt_xy_pos
        dist_diff_squared = np.dot(diff, diff)
        actual_dist = np.sqrt(dist_diff_squared)

        # SIMPLE: Target reached if robot is within 0.1 cell (10cm) of target (Same as default course project)
        # This aligns with our simple grid-based pathfinding
        dist_threshold = 0.1

        # DEBUG: Print distance every 1000 steps
        if self.curr_tgt_type is not None and hasattr(self, '_debug_counter') and self._debug_counter % 1000 == 0:
            logging.info(f'DEBUG Robot {self.id}: Distance to {self.curr_tgt_type}: {actual_dist:.3f}m (threshold: {dist_threshold}m)')

        return dist_diff_squared < (dist_threshold * dist_threshold) 
    
