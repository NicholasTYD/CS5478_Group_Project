# CS5478_Group_Project

A simulation of a autonomous warehouse with multiple robot agents and continuously streaming deliveries, to simulate the Multi-Agent Pickup and Delivery (MAPD) problem.

This simulation implements Conflict-Based Search (CBS) and a customized variant of CBS (Shy-CBS), offering collision avoidance that work in a MAPD setting.

Shy-CBS is our custom implementation of CBS. It functions mostly identically to CBS, with the minor modification to the low-level A* search algorithm, to provide a extremely small discount to path cost if an agent chooses to idle in a workstation. This discount is so small that it virtually makes no difference to the outcome of the pathfinding algorithm for majority of cases, except during the case when the low-level algorithm is presented with two paths that would otherwise have the same path cost, where it'll preferentially pick the path that idles more often in a workstation. 

The rationale behind Shy-CBS is that by allowing agents to remain idle at their assigned workstations more often, overall congestion can be reduced. Since each workstation is a protected cell accessible only to its designated agent, waiting there effectively removes the agent from the shared navigation space. This leaves more room for other agents to successfully propose shorter paths with potentially less conflicts, which would hopefully improve the overall efficiency of the system.

# Pathfinding in action:

Successful collision avoidance in chokepoints, even when many agents are present:

https://github.com/user-attachments/assets/1bc50e76-50d7-424f-b59a-7ca348a96edb

Both algorithms works for long, narrow corridors as well:

https://github.com/user-attachments/assets/312b803b-10f8-41bb-8ce5-6def446c0c6a


# Setup

First install all dependencies:

```
pip install -r /path/to/requirements.txt
```

After installation, switch to the source directory and run the follow command to start the simulation:

```
python main_cbs_demo.py
```

The simulation allows for certain input arguments, such as specifying the number of agents or environment layout.
You can run the following command to view all the available arguments:
```
python main_cbs_demo.py --help   
```
# Simulation Specifications

This section contains a list of specifications of the simulation in detail.

## General Specifications

The simulation replicates a simplified version of a warehouse. Making up the warehouse environment are four types of structures: exterior *walls* (black tiles) that bound the simulation perimeters, *shelves* (gray tiles) that are functionally identical to *walls* but exist within the simulation bounds, *endpoints* (blue tiles) that function as intermediate path-finding goals for *agents* (circular discs) to 'pick up' a package, and *workstations* (pink tiles), the starting and ending point for a robot executing a delivery order.

The entire simulation is discretized into tiles. This includes the environmental obstacles, and also the pathfinding algorithms implemented. This is to simplify pathfinding algorithms. Furthermore, since the simulation is modeled in the context of a warehouse environment which usually has a orderly structure, performing such a simplification is practical for real life applications as well.

The main goal for agents in the simulation is to fufill *delivery tasks*, requiring them to path find from its own dedicated *workstation* to a specified *endpoint*, then back to the same *workstation*, while avoiding any collisions with any obstacles or other agents.

The simulation terminates after all *delivery tasks* are fulfilled. A summary of the results of the simulation is then written into a `.json` file.

## Environmental Structures

As previously mentioned, the entire simulation is discretized into *tiles*. This includes the environmental obstacles, and also the pathfinding algorithms implemented.

### Walls and Shelves

*Walls* and *shelves* act as rigid, unmoving obstacles that *agents* must avoid. They are functionally identical, and serve no other additional functions (*Agents* do not interact with shelves, see~Section \ref{ap:spec_endpoints}). Running into said obstacles will result in a collision.

### Endpoints

*Endpoints* act as intermediate goals for *agents* during the process of fulfilling a *delivery task*. They are represented as collision-free environmental structures that simulate the action of agent 'picking up' a package from a shelf in real-life. As such, the positions of endpoints in the simulation are mostly placed adjacent to *shelves*, and *agents* must enter the *endpoint* tile to pick up an order.

### Workstations

*Workstations* act as the starting and final positions for an *agent* executing a *delivery task*. *Workstations* are collision-free environmental structures that simulate the action of *agents* handing over picked objects to a human handler for further processing in the warehouse. *Agents* are only permitted to enter the *workstation* that it is assigned to, and must physically enter the tile to fufill the task.

### Tiles

Grids within the simulation bounds that are not *walls*, *shelves*, *endpoints*, or *workstations* are automatically considered as *tiles*, which are valid spaces that any *agents* can traverse to.

## Agents

*Agents* (or *robots*) are mobile entities that move around the simulation to accomplish *delivery tasks* assigned to them. They are modeled as circular discs of roughly 0.3 *tiles* in radius to simplify collision checking, as the bounding box of circles are unchanged during rotation. They are assigned to a unique *workstation* in which only they themselves and no other *agents* have access to it. When an *agent* has no active assigned *tasks*, they idle in their *workstation*.

The amount of *agents* in the simulation can be specified as an input argument in the simulation. By default, the amount of *agents* present will be the same as the number of *workstations*.

## Delivery Task

*Delivery Tasks* (or simply *tasks*) are assigned to *agents*, and completing them are the main focus of the simulation. They are only assigned when an *agent* has no other active *tasks* and are idling in their *workstation*.

A *delivery task* consists of two phases: The pickup phase, where an *agent* pathfind from their *workstation* to a randomly chosen *endpoint*, and the return phase, where the *agent* then pathfinds back to the same *workstation* they are assigned to. This simulation assumes that *agents* do not idle upon reaching their *endpoint* (i.e. We do not allocate any time spent to simulate picking up an item from the shelf).

The total amount of *delivery tasks* created across the entire duration of the simulation can be specified as an input argument by the user. By default, the amount of *tasks* present will be the same as the number of *agents*. *Tasks* are immediately assigned to all *agents* when they are idle. Once all *tasks* are successfully completed, the simulation automatically terminates.

The simulation will constantly and immediately assign new *delivery tasks* to idling agents, up until the total amount of *tasks* have been created.

The randomly chosen *endpoints* for *delivery tasks* are determined by a set seed for consistency, which can be passed in as a input argument. Given the same warehouse layout and seed, the sequence of chosen endpoints for all the delivery tasks will remain the same across all runs.

To aid with tracking deliveries during a simulation, a visual indicator is provided for keeping track of a *delivery task*. When a *task* is created, an orange indicator will appear at the target *endpoint* tile. Once the *agent* has successfully reached the *endpoint*, the indicator will turn yellow. It will turn green after the *agent* returns to its assigned *workstation* and fulfills the *delivery task*. This visual indicator will disappear once the *agent* receives another *task*.

### Extra Pathfinding Algorithm Details

All algorithms have backtracking enabled by default, though this can be disabled as part of an input argument to the simulation (`--disable-backtrack`). Note that disabling backtrack may cause the pathfinding algorithm to find no solution and terminate the simulation early.

The centralized pathfinding algorithm is triggered whenever an *agent* receives a new *delivery task* or pickups a delivery at the assigned *endpoint*. When this triggered, **all** *agents* that have an active *task* are assigned new paths, replacing their existing paths if present. *Agents* without a *delivery task* continue to idle and do not take part in the replannning.

The path that the algorithm allocates to an *agent* is either the path from its current position to its target *endpoint*, or from its current position to its *workstation*, depending on if the *agent*'s delivery state. While it is possible to plan a combined route that does the pickup and return phases at once, we decided against it because doing so increases the search space even more by generating even longer paths. Additionally, although not accounted for in our simulation, real-world warehouse scenarios require additional time for *agents* to physically pick up an item. Since the time required for this process is unpredictable and varies, an actual warehouse running MAPD algorithms would need to separately replan the return path to accommodate this uncertainty.

While robot movement and collision checking is executed in every frame of the simulation, the pathfinding algorithm is only triggered every "grid time", which is the time taken for an *agent* to travel 1 tile in the simulation. This decouples the *agent's* actual position in the world and the discretized grid used for the pathfinding algorithm, ensuring that all *agents* are in sync with their grid positions as specified in the pathfinding algorithm.

On the case where the simulation cannot find a valid path configuration for all the *agents*, the simulation will raise an error and forcibly terminate. In practice, this will not occur unless an impossible warehouse layout is provided.

Note that however, as the pathfinding search space (possible paths combinations for all *agents*) explodes with increasing *agent* count and reachable *tiles*, the simulation might occasionally lag or hang when trying to generate a valid path for more complex pathfinding setups for CBS or Shy-CBS. Situations where this is more likely to happen include having large number of *agents* in the simulation, pathfinding through congested areas such as narrow corridors, or if *agent* goal tiles are situated in close proximity of each other such as if many *agents* are trying to pathfind to the same *endpoint*.

It is expected that the simulation might lag or hang occasionally. However, in the case that the simulation hangs for and extended period of time during the pathfinding process, we suggest manually terminating the simulation, as traversing the entire search space is computationally infeasible especially since this simulation does not offer GPU support. A good rule of thumb on when to manually terminate is if the pathfinding algorithm has considered over 50000 pathfinding combinations (as printed in the console log) without a successful path resolution. Note that it also takes more time to evaluate longer or more complex path combinations.

If it consistently hangs, the simulation is probably too complex. Consider switching to a simpler layout, specifying less *agents*, or trying a different seed.
