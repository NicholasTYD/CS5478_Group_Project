from collections import defaultdict
from dataclasses import dataclass
from typing import Dict
import pybullet as p
from robot import Robot
import time

class CollisionChecker:
    def __init__(self, robot_objs:list[Robot], obstacle_ids:list[int]):
        '''
        robot_obj: A list of Robot objects
        obstacle_ids: Any obstacle. Takes in Pybullet's assigned id for them when they are loaded in.
        '''
        self.robot_ids = [robot_obj.id for robot_obj in robot_objs]
        self.obstacle_ids = obstacle_ids

    def check_robot_collisions(self):
        '''
        Check robot-robot collisions and robot-obstacle collisions.

        Returns a 2-tuple, of (robot-robot collisions, robot-obstacle collisions).

        robot-robot collisions and robot-obstacle collisions are both dicts,
        with key = (bodyA_id, bodyB_id), containing the id of the robot/obstacle involved
        and value = list containing all point of contacts in the form of [(x1, y1, z1), (x2, y2, z2), ...]

        Note: Sometimes it will record more than 1 contact point in almost the same place. This likely occurs due to
        them intersecting it other or something like that.
        '''
        contacts = p.getContactPoints()

        curr_step_rr_collisions = defaultdict(list)
        curr_step_rd_collisions = defaultdict(list)
        for c in contacts:
            # c[5] and c[6] measure contact position at A and B, but we assume the contact pos for both is the same,
            # so we only track c[5]
            bodyA, bodyB, contact_point = c[1], c[2], c[5] 

            # Sort in ascending order to avoid double counting A collided with B and B collide with A
            if bodyA > bodyB:
                bodyA, bodyB = bodyB, bodyA
            
            # Check robot-robot collisions
            if bodyA in self.robot_ids and bodyB in self.robot_ids:
                curr_step_rr_collisions[(bodyA, bodyB)].append(contact_point)

            # Check robot-obstacle collisions
            if (bodyA in self.robot_ids and bodyB in self.obstacle_ids) \
                    or (bodyA in self.obstacle_ids and bodyB in self.robot_ids):
                curr_step_rd_collisions[(bodyA, bodyB)].append(contact_point)

        return curr_step_rr_collisions, curr_step_rd_collisions


@dataclass(frozen=True)
class CollisionRecord:
    """Defines a recorded collision."""

    body_a_id: int
    body_b_id: int
    contact_point: list[float, float, float]
    time: float

    def to_dict(self):
        return {
            'body_a_id': self.body_a_id,
            'body_b_id': self.body_b_id,
            'contact_point': self.contact_point,
            'time': self.time
        }

class CBSCollisionsTracker:
    def __init__(self):
        '''
        robot_obj: A list of CBSDemoBot objects
        obstacle_ids: Any obstacle. Takes in Pybullet's assigned id for them when they are loaded in.
        '''
        self.robot_ids: list[int] = []
        self.obstacle_ids: list[int] = []

        self.rr_collision_records: list[CollisionRecord] = []
        self.ro_collision_records: list[CollisionRecord] = []

    def add_robot_to_track(self, robot_id):
        self.robot_ids.append(robot_id)

    def add_obstacle_to_track(self, obstacle_id):
        self.obstacle_ids.append(obstacle_id)

    def check_robot_collisions(self, sim_time):
        '''
        Check robot-robot collisions and robot-obstacle collisions.

        Note: Sometimes it will record more than 1 contact point in almost the same place. This likely occurs due to
        them intersecting it other or something like that.
        '''
        contacts = p.getContactPoints()

        for c in contacts:
            # c[5] and c[6] measure contact position at A and B, but we assume the contact pos for both is the same,
            # so we only track c[5]
            bodyA, bodyB, contact_point = c[1], c[2], c[5] 

            # Sort in ascending order to avoid double counting A collided with B and B collide with A
            if bodyA > bodyB:
                bodyA, bodyB = bodyB, bodyA
            
            # Check robot-robot collisions
            if bodyA in self.robot_ids and bodyB in self.robot_ids:
                self.rr_collision_records.append(CollisionRecord(bodyA, bodyB, contact_point, sim_time))
                print(f"ROBOT-ROBOT COLLISION DETECTED: Between ids {bodyA} and {bodyB} at time {sim_time:.3f}")

            # Check robot-obstacle collisions
            if (bodyA in self.robot_ids and bodyB in self.obstacle_ids) \
                    or (bodyA in self.obstacle_ids and bodyB in self.robot_ids):
                self.ro_collision_records.append(CollisionRecord(bodyA, bodyB, contact_point, sim_time))
                print(f"ROBOT-OBSTACLE COLLISION DETECTED: Between ids {bodyA} and {bodyB} at time {sim_time:.3f}")

    def get_rr_collision_records(self, as_list_of_dicts=False) -> list[CollisionRecord|dict]:
        if as_list_of_dicts:
            return list(map(lambda x: x.to_dict(), self.rr_collision_records))
        else:
            return self.rr_collision_records
    
    def get_ro_collision_records(self, as_list_of_dicts=False) -> list[CollisionRecord|dict]:
        if as_list_of_dicts:
            return list(map(lambda x: x.to_dict(), self.ro_collision_records))
        else:
            return self.ro_collision_records