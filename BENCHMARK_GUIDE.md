# Scheduling Algorithm Benchmark - Simple Guide

## 🚀 Run the Experiment

```bash
python benchmark.py --suite scheduling --output-dir results
```

**Time**: ~8 minutes (4 algorithms × 2 minutes each)

---

## 📋 What Gets Tested

**4 Scheduling Algorithms:**
1. Round Robin - Fair rotation through robots
2. Nearest Robot - Assign to closest robot
3. Load Balanced - Assign to robot with smallest queue
4. Random - Random assignment (baseline)

**Configuration:**
- 30 orders per algorithm
- 8 robots in 30×36 warehouse
- A* pathfinding (same for all)
- 30,000 steps = ~125 seconds simulation time

---

## 📊 View Results

```bash
# Quick summary
cat results/comparison_*.csv

# Detailed view
python -c "
import json, glob
for f in sorted(glob.glob('results/*run0*.json')):
    data = json.load(open(f))
    name = data['config']['name'].split('_')[1]
    print(f'{name:15s} | Orders: {data[\"orders_completed\"]:2d} | '
          f'Throughput: {data[\"throughput_orders_per_min\"]:5.1f} ord/min | '
          f'Delivery: {data[\"avg_delivery_time_sec\"]:5.1f}s | '
          f'Distance: {data[\"total_distance_traveled_m\"]:6.1f}m | '
          f'Utilization: {data[\"avg_robot_utilization_percent\"]:4.1f}%')
"
```

---

## 🎯 Understanding the Metrics

### throughput_orders_per_min
- **Higher is better**
- How many orders completed per minute
- **Expected best**: Nearest Robot (shortest paths = fastest)

### avg_delivery_time_sec
- **Lower is better**
- Time from order creation to delivery
- **Expected best**: Nearest Robot (closest robot = quick delivery)

### total_distance_traveled_m
- **Lower is better**
- Total meters all robots traveled (energy efficiency)
- **Expected best**: Nearest Robot (assigns closest robot)

### avg_robot_utilization_percent
- **Higher is better**
- % of time robots working vs idle
- **Expected best**: Load Balanced (keeps all robots busy)

---

## 🔍 Expected Results

| Algorithm | Throughput | Distance | Utilization |
|-----------|------------|----------|-------------|
| **Nearest Robot** | **BEST** ✓ | **BEST** ✓ | Worst ✗ |
| **Load Balanced** | Good | Worst ✗ | **BEST** ✓ |
| **Round Robin** | Medium | Medium | Medium |
| **Random** | **Worst** ✗ | Bad | Bad |

**Key Trade-off:**
- Nearest Robot: Fast & efficient BUT some robots idle (low utilization)
- Load Balanced: All robots busy BUT longer distances (not optimized)

---

## ❌ Troubleshooting

**Problem**: orders_completed = 0
```bash
# Edit config.py line 233, increase max_steps:
config.simulation.max_steps = 50000
```

**Problem**: Too slow
```bash
# Reduce orders in config.py line 231:
config.orders.num_orders = 20
```

---

## 📈 Quick Plot (Optional)

```python
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('results/comparison_20251027_*.csv')

fig, axes = plt.subplots(2, 2, figsize=(12, 8))

axes[0,0].bar(df['config_name'], df['throughput_orders_per_min'])
axes[0,0].set_title('Throughput (higher=better)')

axes[0,1].bar(df['config_name'], df['avg_delivery_time_sec'])
axes[0,1].set_title('Delivery Time (lower=better)')

axes[1,0].bar(df['config_name'], df['total_distance_traveled_m'])
axes[1,0].set_title('Distance (lower=better)')

axes[1,1].bar(df['config_name'], df['avg_robot_utilization_percent'])
axes[1,1].set_title('Utilization (higher=better)')

for ax in axes.flat:
    ax.tick_params(axis='x', rotation=45)

plt.tight_layout()
plt.savefig('comparison.png', dpi=300)
print("Saved to comparison.png")
```

---

## ✅ That's It!

Run experiment → Check CSV → Analyze trade-offs → Done! 🎓
