
# Create dynamic scheduled deliveries
# TODO: Fix CBS pathfinding looping for some reason
# TODO Add mechanism to discourage or even ban backtracking cuz it might loop forever?

# CBS Algorithm Analysis & Improvement Opportunities

## 1. What's Going Well: CBS vs Plain A*

### Computational Tractability
- **Plain A* on joint state space**: Complexity O(b^(d*n)) where n = number of agents
  - **Exponential in agent count** - becomes intractable with many agents
  - 6 robots would require A* on 12-dimensional state space
- **CBS low-level search**: O(b^d) per agent in 2D space
  - **Linear in agent count** for low-level planning
  - 6 separate 2D searches instead of one massive joint search

### Optimality with Efficiency
- CBS finds the optimal solution (minimum sum of costs)
- **Lazy conflict resolution**: Only adds constraints when conflicts actually occur
- Many agents never conflict, so no wasted computation coordinating them
- Exploits independence between non-conflicting agents

### Scalability in Sparse Environments
- In warehouse environments (lines 127-137 in main_cbs_demo.py), most paths don't conflict
- CBS exploits this sparsity effectively
- If robots never get close, they plan independently with zero coordination overhead
- High-level search only activates when necessary

---

## 2. Main Cons of CBS

### Exponential Worst-Case in Constraint Tree
- Each conflict branches into 2 children (multiagent_cbs.py:85-122)
- With k conflicts, can get up to 2^k nodes in the constraint tree
- In dense environments or with many agents, this explodes
- **Current implementation has no pruning or early termination strategies**

### Memory Intensive
- Each CT node stores complete paths for ALL agents (line 66-69)
- With 6 agents and 400 timestep paths, each node stores ~2400 positions
- Large constraint trees = massive memory usage
- Redundant path storage across similar nodes

### No Bounded Suboptimality
- CBS finds optimal solution or nothing
- Cannot trade solution quality for speed (like weighted A*)
- Line 124: Just gives up if no solution found
- No anytime behavior or iterative deepening

---

## 3. Optimization Opportunities

### High-Level Search Issues (multiagent_cbs.py:75-122)

#### Conflict Selection Strategy
**Current**: Line 77 returns first conflict found
**Problem**: All conflicts treated equally
**Improvements**:
- **Cardinal vs semi-cardinal vs non-cardinal conflicts**: Which conflicts block more paths?
- **Early conflicts first**: Conflicts earlier in time are often cheaper to resolve
- **Most constrained agent**: Prioritize conflicts involving heavily constrained agents

#### Symmetry Breaking
**Current**: Creates 2 children per conflict (one constraint per agent)
**Problem**: May explore symmetric solutions
**Improvements**:
- Detect when one agent MUST wait (e.g., at bottleneck)
- Use reasoning to prune obviously inferior branches
- Meta-agent reasoning for persistent conflicts

#### Node Pruning
**Current**: No dominated node detection
**Improvements**:
- Detect nodes with same constraints but higher cost
- Use A* pruning techniques at CT level
- Implement closed list for constraint sets

---

### Low-Level Search Issues (lines 137-201)

#### Incremental Search
**Current**: Full A* replan on each constraint addition
**Problem**: Wasteful when path is mostly similar
**Improvements**:
- Use incremental search (e.g., D* Lite, LPA*)
- Reuse portions of previous path
- Only replan affected segments

#### Constraint Table Management
**Current**: Rebuilds constraint table each search (line 203)
**Problem**: Redundant computation
**Improvements**:
- Cache constraint tables
- Incremental constraint table updates
- Better data structures for constraint lookup

#### Heuristic Quality
**Current**: Manhattan distance (line 289-290)
**Problem**: Can be inadmissible with true distances
**Improvements**:
- Precompute true distance heuristics
- Use differential heuristics
- Better informed estimates for complex environments

---

### Conflict Detection Issues (lines 244-314)

#### Time Complexity
**Current**: O(n² × t) checking all pairs at all timesteps
**Problem**: Very expensive with many agents or long horizons
**Improvements**:
- **Spatial hashing**: Only check nearby agents
- **Temporal windowing**: Don't check conflicts far in future
- **Early termination**: Stop after finding first conflict

#### Redundant Computation
**Current**: `_count_all_conflicts()` (line 283) iterates entire time horizon
**Problem**: Called for metrics, very expensive
**Improvements**:
- Cache conflict counts
- Incremental conflict counting
- Only compute when needed, not for every CT node

---

### Implementation-Specific Issues

#### Root Planning (line 59-65)
**Current**: Sequential planning of agents
**Problem**: Could be parallelized
**Improvement**: Plan all agents independently in parallel

#### Cost Function (line 68)
**Current**: `_compute_cost()` sums path lengths only
**Problem**: Ignores makespan (longest path time)
**Improvement**:
- Configurable objective: sum-of-costs vs makespan
- Multi-objective optimization options

#### No Search Limits
**Current**: Unbounded CT search
**Problem**: Can run indefinitely
**Improvements**:
- Timeout or iteration limit on CT search
- Anytime behavior with incumbent solution
- Progressive widening

#### Priority Queue Tie-Breaking
**Current**: Basic counter for FIFO tie-breaking
**Problem**: Misses opportunities for better node ordering
**Improvements**:
- Secondary heuristics (number of conflicts, makespan)
- Depth-first vs breadth-first bias
- Learned tie-breaking strategies

---

## Modern CBS Variants (Not Implemented)

### ECBS (Explicit Estimation CBS)
- **Bounded suboptimal**: Trades optimality for speed
- Uses focal search with suboptimality bound w
- Much faster in practice, guarantees solution within w × optimal

### MA-CBS (Meta-Agent CBS)
- Merges conflicting agents into meta-agents
- Plans merged agents jointly in higher-dimensional space
- Prevents repeated conflicts between same agent pairs

### CBS with Highways
- Reserves common corridors for high-traffic areas
- Reduces conflicts in bottlenecks
- Particularly useful in warehouse environments

### Disjoint Splitting
- Splits on both agents simultaneously when safe
- Reduces branching factor from 2 to 1 in some cases
- Provably reduces CT size

### ICBS (Improved CBS)
- Better conflict selection heuristics
- Bypass and rectangle reasoning
- Corridor reasoning for narrow passages

---

## Recommendations for This Implementation

### High Priority (Biggest Impact)
1. **Add conflict selection heuristics** - Easy to implement, significant speedup
2. **Implement search timeout** - Prevents hanging on infeasible problems
3. **Optimize conflict detection** - Currently O(n² × t), can be much better

### Medium Priority (Quality of Life)
4. **Incremental constraint tables** - Reduce low-level search overhead
5. **Better cost metrics** - Add makespan tracking
6. **Parallel root planning** - Minor speedup, cleaner code

### Low Priority (Advanced)
7. **ECBS implementation** - Requires significant refactoring
8. **Meta-agent merging** - Complex but powerful for persistent conflicts
9. **Learned heuristics** - Machine learning for conflict selection

---

## Conclusion

The current implementation is **clean vanilla CBS** - excellent for understanding fundamentals. The main strengths (optimality, sparse environment performance) are preserved, but several optimizations could improve:
- **Performance**: 2-10x speedup with better conflict selection
- **Scalability**: Handle 20+ agents with better conflict detection
- **Robustness**: Timeouts and anytime behavior for difficult problems

For the warehouse demo with 6-12 robots, the current implementation is **sufficient**. Optimizations become critical when scaling to 20+ agents or very dense environments.
