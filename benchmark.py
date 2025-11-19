"""
Benchmarking framework for warehouse robot simulation.

Automates running multiple simulations with different configurations,
collects metrics, and generates comparison reports.
"""

import pybullet as p
import time
import numpy as np
import logging
from pathlib import Path
import csv
import json
from typing import List, Dict, Optional
from datetime import datetime

# Import simulation components
from robot import Robot, RobotTask
from pathfinding import create_warehouse_grid, AStarPlanner
from metrics import MetricsCollector
from config import SimConfig, get_benchmark_configs
import utils


class BenchmarkSimulator:
    """
    Simulator specifically designed for benchmarking.
    Runs headless (no GUI) for faster execution.
    """

    def __init__(self, config: SimConfig, run_id: str = ""):
        """
        Initialize simulator with configuration.

        Args:
            config: SimConfig object with all parameters
            run_id: Unique identifier for this run
        """
        self.config = config
        self.run_id = run_id or f"run_{int(time.time())}"

        # Setup logging
        log_level = getattr(logging, config.simulation.log_level)
        logging.basicConfig(level=log_level, format='%(levelname)s: %(message)s')
        self.logger = logging.getLogger(__name__)

        # Connect to physics (GUI or DIRECT mode)
        if config.simulation.enable_gui:
            self.physicsClient = p.connect(p.GUI)
        else:
            self.physicsClient = p.connect(p.DIRECT)  # Headless for benchmarking

        p.setGravity(0, 0, config.simulation.gravity)

        # Load warehouse
        rows, cols = config.warehouse.rows, config.warehouse.cols
        work_stns, shelves, endpoints = self._generate_warehouse_layout(rows, cols)
        self.wall_pos, self.work_stns_pos, self.shelves_pos, self.endpoints_pos = \
            self._load_map(rows, cols, work_stns, shelves, endpoints)

        # Create pathfinding grid
        x_offset = 0 if cols % 2 == 0 else 0.5
        y_offset = 0 if rows % 2 == 0 else 0.5
        self.grid_map = create_warehouse_grid(rows, cols, self.wall_pos, self.shelves_pos,
                                              cell_size=config.warehouse.cell_size,
                                              offset=(x_offset, y_offset),
                                              endpoints_pos=self.endpoints_pos,
                                              work_stns_pos=self.work_stns_pos)
        self.planner = AStarPlanner(self.grid_map)

        # Initialize metrics
        num_robots = len(self.work_stns_pos)
        self.metrics = MetricsCollector(num_robots=num_robots, time_step=config.simulation.time_step)

        # Create robots
        self.robots = []
        self.work_stn_robot_dict = {}
        for idx, stn_pos in enumerate(self.work_stns_pos):
            robot_pos = stn_pos.copy()
            robot_pos[2] = 0
            robot_obj = self._load_robot(robot_pos)
            self.robots.append(robot_obj)
            self.work_stn_robot_dict[tuple(stn_pos.tolist())] = robot_obj

        # Order generation
        self.curr_sim_step = 0
        self.order_idx = 0
        self.rng = np.random.default_rng(seed=config.orders.random_seed)
        self.order_creation_times = self._generate_order_schedule()

        self.logger.info(f"Benchmark simulator initialized: {config.name}")
        self.logger.info(f"  Warehouse: {rows}x{cols}, Robots: {num_robots}")
        self.logger.info(f"  Orders: {len(self.order_creation_times)}")

    def _generate_warehouse_layout(self, rows, cols):
        """Generate warehouse layout based on configuration."""
        # Use default layout scaled to the requested size
        default_rows, default_cols, work_stns, shelves, endpoints = utils.get_warehouse_params()

        # If dimensions match default, use as-is
        if rows == default_rows and cols == default_cols:
            return work_stns, shelves, endpoints

        # Otherwise, create new arrays with the same pattern but different size
        # Setup workstations (on left and right edges)
        work_stns = np.zeros([rows, cols])
        work_stns[1:-1:3, -1] = 1
        work_stns[1:-1:3, 0] = 1

        # Setup shelves (obstacles in the middle)
        shelves = np.zeros([rows, cols])
        shelves[2:-2:4, 1:-2] = 1
        shelves[3:-2:4, 1:-2] = 1
        # Create gaps for navigation
        shelves[2:-2:4, 1:-2:11] = 0
        shelves[3:-2:4, 1:-2:11] = 0

        # Setup endpoints (sparse pickup locations)
        endpoints = np.zeros([rows, cols])
        endpoints[1:-1:4, 2:-2:6] = 1
        endpoints[4:-1:4, 5:-2:6] = 1

        return work_stns, shelves, endpoints

    def _load_map(self, rows, cols, work_stn_arr, shelves_arr, endpoints_arr):
        """Load warehouse map into physics simulation."""
        x_offset = 0 if cols % 2 == 0 else 0.5
        y_offset = 0 if rows % 2 == 0 else 0.5
        floor_base_pos = [x_offset, y_offset, 0]
        p.loadURDF("assets/plane/plane.urdf", basePosition=floor_base_pos)

        whouse_map = np.zeros([rows + 2, cols + 2])
        whouse_map[0,:] = 1
        whouse_map[-1,:] = 1
        whouse_map[:,0] = 1
        whouse_map[:,-1] = 1

        wall_pos = utils.create_struct_urdf(whouse_map, "assets/warehouse/wall.urdf",
                                           grid_z=3, box_color=(0.1, 0.1, 0.1, 1))
        work_stns_pos = utils.create_struct_urdf(work_stn_arr, "assets/warehouse/workstations.urdf",
                                                 grid_z=1.5, box_color=(1, 0.2, 0.6, 0.8),
                                                 has_collison=False)
        # Shelves with smaller collision boxes (0.7m) for robot clearance
        shelves_pos = utils.create_struct_urdf(shelves_arr, "assets/warehouse/shelves.urdf",
                                               grid_z=1, box_color=(0.3, 0.3, 0.3, 0.9), collision_scale=0.7)
        endpoints_pos = utils.create_struct_urdf(endpoints_arr, "assets/warehouse/endpoints.urdf",
                                                 grid_z=0.3, box_color=(1, 0.5, 0, 1),
                                                 has_collison=False)

        p.loadURDF("assets/warehouse/wall.urdf", useFixedBase=1, flags=p.URDF_MERGE_FIXED_LINKS)
        p.loadURDF("assets/warehouse/workstations.urdf", useFixedBase=1, flags=p.URDF_MERGE_FIXED_LINKS)
        p.loadURDF("assets/warehouse/shelves.urdf", useFixedBase=1, flags=p.URDF_MERGE_FIXED_LINKS)
        p.loadURDF("assets/warehouse/endpoints.urdf", useFixedBase=1, flags=p.URDF_MERGE_FIXED_LINKS)

        return wall_pos, work_stns_pos, shelves_pos, endpoints_pos

    def _load_robot(self, pos):
        """Load robot into simulation."""
        cfg = self.config.robot
        robot_collision = p.createCollisionShape(p.GEOM_CYLINDER, radius=cfg.radius, height=cfg.height)
        robot_visual = p.createVisualShape(p.GEOM_CYLINDER, radius=cfg.radius, length=cfg.height,
                                          rgbaColor=[0.2, 0.2, 1, 1])

        robot_id = p.createMultiBody(
            baseMass=cfg.mass,
            baseCollisionShapeIndex=robot_collision,
            baseVisualShapeIndex=robot_visual,
            basePosition=pos,
            baseOrientation=p.getQuaternionFromEuler([0, 0, 0])
        )

        return Robot(robot_id, self.planner, self.metrics)

    def _generate_order_schedule(self):
        """Generate order creation schedule based on configuration."""
        cfg = self.config.orders

        if cfg.order_generation_mode == "scheduled":
            # Random distribution across time window
            return sorted(self.rng.choice(
                range(cfg.schedule_start_step, cfg.schedule_end_step),
                cfg.num_orders,
                replace=False
            ))
        elif cfg.order_generation_mode == "batch":
            # Batch at specific intervals
            schedule = []
            for batch_size, start_step in zip(cfg.batch_sizes, cfg.batch_intervals):
                schedule.extend([start_step] * batch_size)
            return sorted(schedule)
        else:
            # For continuous mode, generate enough orders
            # Convert rate to steps between orders
            steps_per_order = int(1.0 / (cfg.continuous_rate * self.config.simulation.time_step))
            return list(range(1, cfg.num_orders * steps_per_order, steps_per_order))

    def create_delivery_order(self, order_id):
        """Create and assign a delivery order."""
        self.metrics.order_created(order_id)

        chosen_endpoint_pos = self.rng.choice(self.endpoints_pos)

        # Scheduling algorithm selection
        if self.config.scheduling.algorithm == "round_robin":
            # Round-robin: cycle through robots deterministically
            chosen_work_stn_pos = self.work_stns_pos[self.order_idx % len(self.work_stns_pos)]

        elif self.config.scheduling.algorithm == "nearest_robot":
            # Nearest robot: find robot closest to endpoint
            chosen_work_stn_pos = self._find_nearest_workstation(chosen_endpoint_pos)

        elif self.config.scheduling.algorithm == "load_balanced":
            # Load balanced: assign to robot with smallest queue
            chosen_work_stn_pos = self._find_least_loaded_workstation()

        elif self.config.scheduling.algorithm == "random":
            # Random assignment
            chosen_work_stn_pos = self.rng.choice(self.work_stns_pos)

        else:
            # Default to round-robin
            self.logger.warning(f"Unknown scheduling algorithm '{self.config.scheduling.algorithm}', using round_robin")
            chosen_work_stn_pos = self.work_stns_pos[self.order_idx % len(self.work_stns_pos)]

        allocated_robot = self.work_stn_robot_dict[tuple(chosen_work_stn_pos.tolist())]
        task = RobotTask(order_id, chosen_endpoint_pos, chosen_work_stn_pos)
        allocated_robot.add_task(task)

    def _find_nearest_workstation(self, endpoint_pos):
        """Find the workstation closest to the given endpoint."""
        min_dist = float('inf')
        nearest_stn = self.work_stns_pos[0]

        for stn_pos in self.work_stns_pos:
            dist = np.linalg.norm(endpoint_pos[:2] - stn_pos[:2])
            if dist < min_dist:
                min_dist = dist
                nearest_stn = stn_pos

        return nearest_stn

    def _find_least_loaded_workstation(self):
        """Find the workstation whose robot has the smallest queue."""
        min_queue_size = float('inf')
        best_stn = self.work_stns_pos[0]

        for stn_pos in self.work_stns_pos:
            robot = self.work_stn_robot_dict[tuple(stn_pos.tolist())]
            queue_size = robot.task_queue.qsize()
            if queue_size < min_queue_size:
                min_queue_size = queue_size
                best_stn = stn_pos

        return best_stn

    def step(self):
        """Execute one simulation step."""
        # Create orders if scheduled
        while self.order_idx < len(self.order_creation_times):
            next_order_time = self.order_creation_times[self.order_idx]
            if next_order_time == self.curr_sim_step:
                self.create_delivery_order(self.order_idx)
                self.order_idx += 1
            else:
                break

        # Step physics
        p.stepSimulation()

        # Update all robots
        for robot in self.robots:
            robot.act()

        # Update metrics
        queue_lengths = [robot.task_queue.qsize() for robot in self.robots]
        self.metrics.step(queue_lengths)

    def run(self):
        """Run the simulation."""
        start_time = time.time()
        self.logger.info(f"Starting simulation run: {self.run_id}")

        max_steps = self.config.simulation.max_steps
        early_stop = self.config.simulation.early_stop_on_completion

        for step in range(max_steps):
            self.step()
            self.curr_sim_step += 1

            # Check early stopping
            if early_stop and self.order_idx >= len(self.order_creation_times):
                if self.metrics.orders_completed >= len(self.order_creation_times):
                    self.logger.info(f"All orders completed at step {self.curr_sim_step}")
                    # Buffer steps
                    for _ in range(self.config.simulation.buffer_steps_after_completion):
                        self.step()
                        self.curr_sim_step += 1
                    break

        elapsed_time = time.time() - start_time
        self.logger.info(f"Simulation completed in {elapsed_time:.2f}s (real time)")

        # Cleanup
        p.disconnect()

        return self.metrics

    def get_results(self) -> Dict:
        """Get simulation results as dictionary."""
        summary = self.metrics.get_summary()
        summary['config'] = self.config.to_dict()
        summary['run_id'] = self.run_id
        return summary


class BenchmarkRunner:
    """Runs multiple benchmark simulations and compares results."""

    def __init__(self, output_dir: str = "benchmark_results"):
        """
        Initialize benchmark runner.

        Args:
            output_dir: Directory to save benchmark results
        """
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        self.results = []
        self.logger = logging.getLogger(__name__)

    def run_benchmark(self, configs: List[SimConfig], num_runs_per_config: int = 1):
        """
        Run benchmarks for multiple configurations.

        Args:
            configs: List of SimConfig objects to benchmark
            num_runs_per_config: Number of runs per configuration (for averaging)
        """
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.logger.info(f"Starting benchmark suite with {len(configs)} configurations")
        self.logger.info(f"Runs per config: {num_runs_per_config}")

        for config_idx, config in enumerate(configs):
            self.logger.info(f"\n{'='*60}")
            self.logger.info(f"Configuration {config_idx + 1}/{len(configs)}: {config.name}")
            self.logger.info(f"{'='*60}")

            config_results = []

            for run_idx in range(num_runs_per_config):
                run_id = f"{config.name}_run{run_idx}_{timestamp}"
                self.logger.info(f"  Run {run_idx + 1}/{num_runs_per_config}")

                # Run simulation
                sim = BenchmarkSimulator(config, run_id=run_id)
                metrics = sim.run()

                # Get results
                results = sim.get_results()
                config_results.append(results)
                self.results.append(results)

                # Export individual run results
                self._export_run_results(results, run_id)

            # Compute average metrics for this configuration
            if num_runs_per_config > 1:
                avg_results = self._average_results(config_results)
                self._export_averaged_results(avg_results, f"{config.name}_averaged_{timestamp}")

        # Generate comparison report
        self._generate_comparison_report(timestamp)

        self.logger.info(f"\n{'='*60}")
        self.logger.info(f"Benchmark suite completed!")
        self.logger.info(f"Results saved to: {self.output_dir}")
        self.logger.info(f"{'='*60}\n")

    def _export_run_results(self, results: Dict, run_id: str):
        """Export results for a single run."""
        # Export JSON with full details
        json_path = self.output_dir / f"{run_id}.json"
        with open(json_path, 'w') as f:
            json.dump(results, f, indent=2)

    def _export_averaged_results(self, results: Dict, filename: str):
        """Export averaged results."""
        json_path = self.output_dir / f"{filename}.json"
        with open(json_path, 'w') as f:
            json.dump(results, f, indent=2)

    def _average_results(self, results_list: List[Dict]) -> Dict:
        """Average metrics across multiple runs."""
        if not results_list:
            return {}

        # Extract numeric metrics
        numeric_keys = [
            'throughput_orders_per_sec', 'avg_delivery_time_sec', 'total_distance_traveled_m',
            'avg_robot_utilization_percent', 'avg_queue_length', 'makespan_seconds'
        ]

        averaged = results_list[0].copy()
        for key in numeric_keys:
            values = [r.get(key, 0) for r in results_list if r.get(key) is not None]
            if values:
                averaged[key] = np.mean(values)
                averaged[f"{key}_std"] = np.std(values)

        return averaged

    def _generate_comparison_report(self, timestamp: str):
        """Generate comparison report across all configurations."""
        if not self.results:
            return

        # Create CSV with key metrics for easy comparison
        csv_path = self.output_dir / f"comparison_{timestamp}.csv"

        fieldnames = [
            'config_name', 'orders_completed', 'throughput_orders_per_min',
            'avg_delivery_time_sec', 'total_distance_traveled_m',
            'avg_robot_utilization_percent', 'makespan_seconds'
        ]

        with open(csv_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()

            for result in self.results:
                row = {key: result.get(key, 'N/A') for key in fieldnames}
                row['config_name'] = result['config']['name']
                writer.writerow(row)

        self.logger.info(f"Comparison report saved to: {csv_path}")


def main():
    """Run benchmarks with predefined configurations."""
    import argparse

    parser = argparse.ArgumentParser(description="Warehouse Robot Simulation - Benchmarking Framework")
    parser.add_argument(
        "--suite",
        choices=["default", "quick", "scheduling", "warehouse_size", "comprehensive"],
        default="default",
        help="Benchmark suite to run (default: default)"
    )
    parser.add_argument(
        "--runs",
        type=int,
        default=1,
        help="Number of runs per configuration for averaging (default: 1)"
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="benchmark_results",
        help="Output directory for results (default: benchmark_results)"
    )

    args = parser.parse_args()

    print("\n" + "="*70)
    print("WAREHOUSE ROBOT SIMULATION - BENCHMARKING FRAMEWORK")
    print("="*70 + "\n")

    # Get benchmark configurations based on suite
    if args.suite == "default":
        from config import get_benchmark_configs
        configs = get_benchmark_configs()
    elif args.suite == "quick":
        from config import get_quick_test_config
        configs = [get_quick_test_config()]
    elif args.suite == "scheduling":
        from config import get_scheduling_comparison_configs
        configs = get_scheduling_comparison_configs()
    elif args.suite == "warehouse_size":
        from config import get_warehouse_size_comparison_configs
        configs = get_warehouse_size_comparison_configs()
    elif args.suite == "comprehensive":
        from config import get_comprehensive_benchmark_configs
        configs = get_comprehensive_benchmark_configs()

    print(f"Benchmark Suite: {args.suite}")
    print(f"Runs per config: {args.runs}")
    print(f"Output directory: {args.output_dir}")
    print(f"\nRunning {len(configs)} benchmark configuration(s):\n")

    for i, config in enumerate(configs, 1):
        print(f"  {i}. {config.name}: {config.description}")

    print("\n" + "-"*70 + "\n")

    # Create runner and execute benchmarks
    runner = BenchmarkRunner(output_dir=args.output_dir)
    runner.run_benchmark(configs, num_runs_per_config=args.runs)

    print("\n" + "="*70)
    print("✓ BENCHMARKING COMPLETE!")
    print(f"Results saved to: {args.output_dir}/")
    print("="*70 + "\n")


if __name__ == "__main__":
    main()
