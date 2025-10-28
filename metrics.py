"""
Performance Metrics Collection System for Warehouse Robot Simulation

Tracks:
- Orders completed per time unit (throughput)
- Average delivery time
- Robot utilization rates
- Total distance traveled
- Queue lengths over time
- Makespan (time to complete all orders)
"""

import time
import json
import csv
from collections import defaultdict
from typing import Dict, List, Optional
import numpy as np


class MetricsCollector:
    """Collects and analyzes performance metrics for warehouse robot simulation."""

    def __init__(self, num_robots: int, time_step: float = 1/240.0):
        """
        Initialize metrics collector.

        Args:
            num_robots: Total number of robots in simulation
            time_step: Simulation time step in seconds (default 240 Hz)
        """
        self.num_robots = num_robots
        self.time_step = time_step
        self.start_time = time.time()

        # Order tracking
        self.orders_created = 0
        self.orders_completed = 0
        self.order_start_times: Dict[int, float] = {}  # order_id -> start_time
        self.order_completion_times: Dict[int, float] = {}  # order_id -> completion_time
        self.order_delivery_times: List[float] = []  # List of delivery times (seconds)

        # Robot tracking
        self.robot_distances: Dict[int, float] = defaultdict(float)  # robot_id -> total_distance
        self.robot_task_counts: Dict[int, int] = defaultdict(int)  # robot_id -> completed_tasks
        self.robot_states: Dict[int, str] = {}  # robot_id -> current_state ('idle'/'working')
        self.robot_idle_times: Dict[int, float] = defaultdict(float)  # robot_id -> idle_time
        self.robot_working_times: Dict[int, float] = defaultdict(float)  # robot_id -> working_time

        # Per-step metrics (for time series analysis)
        self.step_metrics: List[Dict] = []
        self.current_step = 0

        # Queue length tracking
        self.queue_lengths_per_step: List[List[int]] = []  # [step][robot_queue_lengths]

        # Event log
        self.events: List[Dict] = []

        # Initialize robot states
        for robot_id in range(num_robots):
            self.robot_states[robot_id] = 'idle'

    def log_event(self, event_type: str, robot_id: Optional[int] = None, order_id: Optional[int] = None,
                  position: Optional[tuple] = None, details: Optional[Dict] = None):
        """
        Log a simulation event.

        Args:
            event_type: Type of event ('order_created', 'order_assigned', 'item_collected', 'order_delivered')
            robot_id: ID of robot involved (if applicable)
            order_id: ID of order involved (if applicable)
            position: Position where event occurred (if applicable)
            details: Additional event details
        """
        event = {
            'step': self.current_step,
            'sim_time': self.current_step * self.time_step,
            'wall_time': time.time() - self.start_time,
            'type': event_type,
            'robot_id': robot_id,
            'order_id': order_id,
            'position': position,
            'details': details or {}
        }
        self.events.append(event)

    def order_created(self, order_id: int):
        """Record when a new order is created."""
        self.orders_created += 1
        self.order_start_times[order_id] = self.current_step * self.time_step
        self.log_event('order_created', order_id=order_id)

    def order_assigned(self, order_id: int, robot_id: int):
        """Record when an order is assigned to a robot."""
        self.log_event('order_assigned', robot_id=robot_id, order_id=order_id)
        if self.robot_states[robot_id] == 'idle':
            self.robot_states[robot_id] = 'working'

    def item_collected(self, order_id: int, robot_id: int, position: tuple):
        """Record when a robot collects an item from an endpoint."""
        self.log_event('item_collected', robot_id=robot_id, order_id=order_id, position=position)

    def order_delivered(self, order_id: int, robot_id: int, position: tuple):
        """Record when an order is completed (delivered to workstation)."""
        self.orders_completed += 1
        completion_time = self.current_step * self.time_step
        self.order_completion_times[order_id] = completion_time

        # Calculate delivery time
        if order_id in self.order_start_times:
            delivery_time = completion_time - self.order_start_times[order_id]
            self.order_delivery_times.append(delivery_time)

        # Update robot task count
        self.robot_task_counts[robot_id] += 1

        self.log_event('order_delivered', robot_id=robot_id, order_id=order_id, position=position)

    def robot_moved(self, robot_id: int, distance: float):
        """
        Record robot movement distance.

        Args:
            robot_id: ID of the robot
            distance: Distance moved in this step (meters)
        """
        self.robot_distances[robot_id] += distance

    def update_robot_state(self, robot_id: int, is_idle: bool):
        """
        Update robot state and track idle/working time.

        Args:
            robot_id: ID of the robot
            is_idle: True if robot is currently idle
        """
        current_state = 'idle' if is_idle else 'working'

        # Track time in each state
        if current_state == 'idle':
            self.robot_idle_times[robot_id] += self.time_step
        else:
            self.robot_working_times[robot_id] += self.time_step

        self.robot_states[robot_id] = current_state

    def record_queue_lengths(self, queue_lengths: List[int]):
        """
        Record queue lengths for all robots at current step.

        Args:
            queue_lengths: List of queue lengths for each robot
        """
        self.queue_lengths_per_step.append(queue_lengths.copy())

    def step(self, queue_lengths: Optional[List[int]] = None):
        """
        Called at each simulation step to record per-step metrics.

        Args:
            queue_lengths: Optional list of current queue lengths for all robots
        """
        self.current_step += 1

        # Record snapshot metrics
        step_data = {
            'step': self.current_step,
            'sim_time': self.current_step * self.time_step,
            'orders_completed': self.orders_completed,
            'orders_pending': self.orders_created - self.orders_completed,
            'active_robots': sum(1 for state in self.robot_states.values() if state == 'working'),
            'idle_robots': sum(1 for state in self.robot_states.values() if state == 'idle'),
        }

        if queue_lengths:
            step_data['avg_queue_length'] = np.mean(queue_lengths)
            step_data['max_queue_length'] = max(queue_lengths)
            self.record_queue_lengths(queue_lengths)

        self.step_metrics.append(step_data)

    def get_summary(self) -> Dict:
        """
        Get summary statistics of simulation performance.

        Returns:
            Dictionary containing all performance metrics
        """
        sim_time = self.current_step * self.time_step

        # Throughput (orders per second)
        throughput = self.orders_completed / sim_time if sim_time > 0 else 0

        # Average delivery time
        avg_delivery_time = np.mean(self.order_delivery_times) if self.order_delivery_times else 0
        std_delivery_time = np.std(self.order_delivery_times) if self.order_delivery_times else 0

        # Total distance traveled
        total_distance = sum(self.robot_distances.values())
        avg_distance_per_robot = total_distance / self.num_robots if self.num_robots > 0 else 0

        # Robot utilization (percentage of time working vs idle)
        robot_utilizations = {}
        for robot_id in range(self.num_robots):
            total_time = self.robot_idle_times[robot_id] + self.robot_working_times[robot_id]
            utilization = (self.robot_working_times[robot_id] / total_time * 100) if total_time > 0 else 0
            robot_utilizations[robot_id] = utilization

        avg_utilization = np.mean(list(robot_utilizations.values())) if robot_utilizations else 0

        # Makespan (time to complete all orders)
        makespan = sim_time if self.orders_completed == self.orders_created else None

        # Queue statistics
        if self.queue_lengths_per_step:
            avg_queue_lengths = [np.mean([step[i] for step in self.queue_lengths_per_step])
                                for i in range(len(self.queue_lengths_per_step[0]))]
            max_queue_length = max(max(step) for step in self.queue_lengths_per_step)
            avg_avg_queue_length = np.mean([np.mean(step) for step in self.queue_lengths_per_step])
        else:
            avg_queue_lengths = []
            max_queue_length = 0
            avg_avg_queue_length = 0

        summary = {
            # Simulation info
            'total_steps': self.current_step,
            'sim_time_seconds': sim_time,
            'num_robots': self.num_robots,

            # Order metrics
            'orders_created': self.orders_created,
            'orders_completed': self.orders_completed,
            'orders_pending': self.orders_created - self.orders_completed,
            'throughput_orders_per_sec': throughput,
            'throughput_orders_per_min': throughput * 60,

            # Delivery time metrics
            'avg_delivery_time_sec': avg_delivery_time,
            'std_delivery_time_sec': std_delivery_time,
            'min_delivery_time_sec': min(self.order_delivery_times) if self.order_delivery_times else 0,
            'max_delivery_time_sec': max(self.order_delivery_times) if self.order_delivery_times else 0,

            # Distance metrics
            'total_distance_traveled_m': total_distance,
            'avg_distance_per_robot_m': avg_distance_per_robot,
            'avg_distance_per_order_m': total_distance / self.orders_completed if self.orders_completed > 0 else 0,

            # Robot utilization
            'avg_robot_utilization_percent': avg_utilization,
            'robot_utilizations': robot_utilizations,
            'robot_task_counts': dict(self.robot_task_counts),

            # Queue metrics
            'max_queue_length': max_queue_length,
            'avg_queue_length': avg_avg_queue_length,
            'avg_queue_per_robot': avg_queue_lengths,

            # Makespan
            'makespan_seconds': makespan,
            'all_orders_completed': self.orders_completed == self.orders_created,
        }

        return summary

    def print_summary(self):
        """Print a formatted summary of metrics."""
        summary = self.get_summary()

        print("\n" + "="*70)
        print("SIMULATION PERFORMANCE SUMMARY")
        print("="*70)

        print(f"\n📊 SIMULATION INFO:")
        print(f"  Total Steps: {summary['total_steps']}")
        print(f"  Sim Time: {summary['sim_time_seconds']:.2f} seconds")
        print(f"  Num Robots: {summary['num_robots']}")

        print(f"\n📦 ORDER METRICS:")
        print(f"  Orders Created: {summary['orders_created']}")
        print(f"  Orders Completed: {summary['orders_completed']}")
        print(f"  Orders Pending: {summary['orders_pending']}")
        print(f"  Throughput: {summary['throughput_orders_per_min']:.2f} orders/min")
        print(f"  All Completed: {'✓ Yes' if summary['all_orders_completed'] else '✗ No'}")

        print(f"\n⏱️  DELIVERY TIME:")
        print(f"  Average: {summary['avg_delivery_time_sec']:.2f} ± {summary['std_delivery_time_sec']:.2f} sec")
        print(f"  Min: {summary['min_delivery_time_sec']:.2f} sec")
        print(f"  Max: {summary['max_delivery_time_sec']:.2f} sec")

        print(f"\n🚗 DISTANCE TRAVELED:")
        print(f"  Total: {summary['total_distance_traveled_m']:.2f} m")
        print(f"  Avg per Robot: {summary['avg_distance_per_robot_m']:.2f} m")
        print(f"  Avg per Order: {summary['avg_distance_per_order_m']:.2f} m")

        print(f"\n🤖 ROBOT UTILIZATION:")
        print(f"  Average: {summary['avg_robot_utilization_percent']:.1f}%")
        for robot_id, util in summary['robot_utilizations'].items():
            tasks = summary['robot_task_counts'].get(robot_id, 0)
            print(f"  Robot {robot_id}: {util:.1f}% utilization, {tasks} tasks completed")

        print(f"\n📋 QUEUE METRICS:")
        print(f"  Max Queue Length: {summary['max_queue_length']}")
        print(f"  Avg Queue Length: {summary['avg_queue_length']:.2f}")

        if summary['makespan_seconds'] is not None:
            print(f"\n⏰ MAKESPAN: {summary['makespan_seconds']:.2f} seconds")

        print("="*70 + "\n")

    def export_to_csv(self, filepath: str):
        """
        Export summary metrics to CSV file.

        Args:
            filepath: Path to output CSV file
        """
        summary = self.get_summary()

        # Flatten nested dictionaries for CSV
        flat_summary = {}
        for key, value in summary.items():
            if isinstance(value, dict):
                for sub_key, sub_value in value.items():
                    flat_summary[f"{key}_{sub_key}"] = sub_value
            elif isinstance(value, list):
                flat_summary[key] = str(value)
            else:
                flat_summary[key] = value

        with open(filepath, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=flat_summary.keys())
            writer.writeheader()
            writer.writerow(flat_summary)

        print(f"✓ Metrics exported to {filepath}")

    def export_to_json(self, filepath: str, include_events: bool = False, include_step_data: bool = False):
        """
        Export all metrics to JSON file.

        Args:
            filepath: Path to output JSON file
            include_events: Include detailed event log
            include_step_data: Include per-step metrics
        """
        summary = self.get_summary()

        output = {
            'summary': summary,
        }

        if include_events:
            output['events'] = self.events

        if include_step_data:
            output['step_metrics'] = self.step_metrics

        with open(filepath, 'w') as f:
            json.dump(output, f, indent=2)

        print(f"✓ Metrics exported to {filepath}")

    def export_timeseries_csv(self, filepath: str):
        """
        Export per-step time series data to CSV.

        Args:
            filepath: Path to output CSV file
        """
        if not self.step_metrics:
            print("No step metrics to export")
            return

        with open(filepath, 'w', newline='') as f:
            fieldnames = self.step_metrics[0].keys()
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.step_metrics)

        print(f"✓ Time series data exported to {filepath}")
