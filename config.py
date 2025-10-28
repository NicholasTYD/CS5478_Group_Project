"""
Configuration management for warehouse robot simulation.

Provides configurable parameters for:
- Warehouse dimensions
- Robot settings
- Order generation
- Simulation runtime
- Scheduling algorithms
"""

from dataclasses import dataclass, field
from typing import Optional, List
import json


@dataclass
class WarehouseConfig:
    """Warehouse layout configuration."""
    rows: int = 30
    cols: int = 36
    cell_size: float = 1.0  # meters


@dataclass
class RobotConfig:
    """Robot physical and behavioral parameters."""
    radius: float = 0.3  # meters
    height: float = 0.2  # meters
    mass: float = 10.0  # kg
    max_speed: float = 5.0  # m/s
    waypoint_tolerance: float = 0.5  # meters
    target_threshold: float = 0.1  # meters (for goal reaching)


@dataclass
class OrderConfig:
    """Order generation configuration."""
    num_orders: int = 20
    order_generation_mode: str = "scheduled"  # "scheduled", "continuous", "batch"

    # For scheduled mode
    schedule_start_step: int = 1
    schedule_end_step: int = 200

    # For continuous mode
    continuous_rate: float = 0.1  # orders per second

    # For batch mode
    batch_sizes: List[int] = field(default_factory=lambda: [5, 5, 5, 5])
    batch_intervals: List[int] = field(default_factory=lambda: [1, 50, 100, 150])

    random_seed: int = 42


@dataclass
class SchedulingConfig:
    """Task scheduling algorithm configuration."""
    algorithm: str = "round_robin"  # "round_robin", "nearest_robot", "load_balanced", "random"

    # Algorithm-specific parameters
    priority_enabled: bool = False
    dynamic_reassignment: bool = False


@dataclass
class SimulationConfig:
    """Simulation runtime and physics configuration."""
    max_steps: int = 9999
    time_step: float = 1/240.0  # seconds (240 Hz)
    enable_gui: bool = True
    gravity: float = -10.0  # m/s^2
    early_stop_on_completion: bool = True
    buffer_steps_after_completion: int = 100

    # Logging
    log_level: str = "INFO"  # "DEBUG", "INFO", "WARNING", "ERROR"


@dataclass
class MetricsConfig:
    """Metrics collection configuration."""
    enable_metrics: bool = True
    export_csv: bool = True
    export_json: bool = True
    export_timeseries: bool = True
    include_events: bool = True
    include_step_data: bool = True
    output_prefix: str = "metrics"


@dataclass
class SimConfig:
    """Complete simulation configuration."""
    name: str = "default_sim"
    description: str = "Default warehouse simulation configuration"

    warehouse: WarehouseConfig = field(default_factory=WarehouseConfig)
    robot: RobotConfig = field(default_factory=RobotConfig)
    orders: OrderConfig = field(default_factory=OrderConfig)
    scheduling: SchedulingConfig = field(default_factory=SchedulingConfig)
    simulation: SimulationConfig = field(default_factory=SimulationConfig)
    metrics: MetricsConfig = field(default_factory=MetricsConfig)

    def to_dict(self):
        """Convert config to dictionary."""
        return {
            'name': self.name,
            'description': self.description,
            'warehouse': self.warehouse.__dict__,
            'robot': self.robot.__dict__,
            'orders': self.orders.__dict__,
            'scheduling': self.scheduling.__dict__,
            'simulation': self.simulation.__dict__,
            'metrics': self.metrics.__dict__,
        }

    def to_json(self, filepath: str):
        """Export configuration to JSON file."""
        with open(filepath, 'w') as f:
            json.dump(self.to_dict(), f, indent=2)
        print(f"Configuration exported to {filepath}")

    @classmethod
    def from_dict(cls, config_dict: dict):
        """Create config from dictionary."""
        return cls(
            name=config_dict.get('name', 'default_sim'),
            description=config_dict.get('description', ''),
            warehouse=WarehouseConfig(**config_dict.get('warehouse', {})),
            robot=RobotConfig(**config_dict.get('robot', {})),
            orders=OrderConfig(**config_dict.get('orders', {})),
            scheduling=SchedulingConfig(**config_dict.get('scheduling', {})),
            simulation=SimulationConfig(**config_dict.get('simulation', {})),
            metrics=MetricsConfig(**config_dict.get('metrics', {})),
        )

    @classmethod
    def from_json(cls, filepath: str):
        """Load configuration from JSON file."""
        with open(filepath, 'r') as f:
            config_dict = json.load(f)
        return cls.from_dict(config_dict)


# Predefined configurations for common scenarios

def get_default_config() -> SimConfig:
    """Get default configuration (same as current main_demo.py)."""
    return SimConfig(
        name="default",
        description="Default configuration with 20 orders, round-robin scheduling",
    )


def get_quick_test_config() -> SimConfig:
    """Get quick test configuration (fewer orders, faster completion)."""
    config = SimConfig(
        name="quick_test",
        description="Quick test with 5 orders for rapid testing",
    )
    config.orders.num_orders = 5
    config.orders.schedule_end_step = 50
    config.simulation.max_steps = 5000  # Increased to allow time for completion
    config.simulation.enable_gui = False  # Headless for faster execution
    return config


def get_stress_test_config() -> SimConfig:
    """Get stress test configuration (many orders, high load)."""
    config = SimConfig(
        name="stress_test",
        description="Stress test with 100 orders to evaluate scalability",
    )
    config.orders.num_orders = 100
    config.orders.schedule_end_step = 500
    config.simulation.max_steps = 20000
    return config


def get_continuous_load_config() -> SimConfig:
    """Get continuous order generation configuration."""
    config = SimConfig(
        name="continuous_load",
        description="Continuous order generation at steady rate",
    )
    config.orders.order_generation_mode = "continuous"
    config.orders.continuous_rate = 0.1  # 0.1 orders per second
    config.simulation.max_steps = 10000
    config.simulation.early_stop_on_completion = False
    return config


def get_small_warehouse_config() -> SimConfig:
    """Get small warehouse configuration for faster testing."""
    config = SimConfig(
        name="small_warehouse",
        description="Smaller warehouse (15x18) for quick experiments",
    )
    config.warehouse.rows = 15
    config.warehouse.cols = 18
    config.orders.num_orders = 10
    return config


def get_large_warehouse_config() -> SimConfig:
    """Get large warehouse configuration for scalability testing."""
    config = SimConfig(
        name="large_warehouse",
        description="Large warehouse (45x54) for scalability analysis",
    )
    config.warehouse.rows = 45
    config.warehouse.cols = 54
    config.orders.num_orders = 50
    config.orders.schedule_end_step = 400
    config.simulation.max_steps = 30000
    return config


def get_scheduling_comparison_configs() -> List[SimConfig]:
    """Get configurations to compare different scheduling algorithms."""
    configs = []

    # Test each scheduling algorithm with same parameters
    for algo in ["round_robin", "nearest_robot", "load_balanced", "random"]:
        config = SimConfig(
            name=f"scheduling_{algo}",
            description=f"Benchmark {algo} scheduling algorithm",
        )
        config.scheduling.algorithm = algo
        config.orders.num_orders = 30
        config.orders.schedule_end_step = 300
        config.simulation.max_steps = 30000  # Increased from 5000 to allow orders to complete
        config.simulation.enable_gui = False  # Headless for faster benchmarking
        configs.append(config)

    return configs


def get_warehouse_size_comparison_configs() -> List[SimConfig]:
    """Get configurations to compare different warehouse sizes."""
    sizes = [
        (15, 18, "small"),
        (30, 36, "medium"),
        (45, 54, "large"),
    ]

    configs = []
    for rows, cols, size_name in sizes:
        config = SimConfig(
            name=f"warehouse_{size_name}",
            description=f"{size_name.capitalize()} warehouse ({rows}x{cols})",
        )
        config.warehouse.rows = rows
        config.warehouse.cols = cols
        config.orders.num_orders = rows  # Scale orders with size
        config.orders.schedule_end_step = rows * 10
        config.simulation.max_steps = rows * 300
        config.simulation.enable_gui = False
        configs.append(config)

    return configs


def get_benchmark_configs() -> List[SimConfig]:
    """Get a list of configurations for benchmarking different scenarios."""
    return [
        get_default_config(),
        get_quick_test_config(),
        get_stress_test_config(),
    ]


def get_comprehensive_benchmark_configs() -> List[SimConfig]:
    """Get comprehensive list of all benchmark configurations."""
    configs = []

    # Basic scenarios
    configs.extend(get_benchmark_configs())

    # Scheduling algorithm comparison
    configs.extend(get_scheduling_comparison_configs())

    # Warehouse size comparison
    configs.extend(get_warehouse_size_comparison_configs())

    return configs


if __name__ == "__main__":
    # Example: Generate and save default configuration
    config = get_default_config()
    config.to_json("config_default.json")

    print("\nAvailable predefined configurations:")
    print("- get_default_config(): Default 20 orders, round-robin")
    print("- get_quick_test_config(): 5 orders for quick testing")
    print("- get_stress_test_config(): 100 orders for stress testing")
    print("- get_continuous_load_config(): Continuous order generation")
    print("- get_small_warehouse_config(): 15x18 warehouse")
    print("- get_large_warehouse_config(): 45x54 warehouse")
    print("\nExample JSON config saved to: config_default.json")
