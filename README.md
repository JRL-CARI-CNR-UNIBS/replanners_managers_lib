# replanners_managers_lib

The `replanners_managers_lib` library is a core component of the [`OpenMORE`](https://github.com/JRL-CARI-CNR-UNIBS/OpenMORE.git) project. It is specifically designed to handle path replanning tasks in dynamic environments, providing the architecture necessary to manage and coordinate the execution of path replanning algorithms during robot trajectory execution.

This library integrates is based on the other `OpenMORE` components:
- [`replanners_lib`](https://github.com/JRL-CARI-CNR-UNIBS/replanners_lib): Defines sampling-based path replanning algorithms.
- [`trajectories_processors_lib`](https://github.com/JRL-CARI-CNR-UNIBS/trajectories_processors_lib): Handles time-parameterization of paths.

It also leverages [MoveIt](https://moveit.ros.org/) for planning scene tracking and collision checking.

## ROS Compatibility

`replanners_managers_lib` is the only `OpenMORE` package that depends on ROS. Currently, it is tested on ROS Noetic, with ongoing work to support ROS 2 Humble. Follow the [`OpenMORE`](https://github.com/JRL-CARI-CNR-UNIBS/OpenMORE) installation guide for setup instructions.

## Concepts

As the core of `OpenMORE`, `replanners_managers_lib` provides the framework to execute path replanning algorithms in real-time during robot trajectory execution. The distinction between a *replanner* and a *replanner_manager* is crucial:
- A **replanner** implements the algorithm responsible for path replanning.
- A **replanner_manager** orchestrates trajectory execution, scene updates, and replanning algorithm execution.

<p align="center">
  <img src="documentation/overview.png?raw=true" alt="Fig.1: Architecture overview" width="60%" style="display: block; margin: auto;">
</p>
<p align="center"><b>Fig.1:</b> Architecture overview.</p>

Fig.1 is a simplified overview of the architecture. It is composed of three threads:

- **Trajectory Execution Thread**: Ensures smooth execution of the robot's trajectory by interpolating it and providing the commands to be sent to the robot's controller at a high rate.
- **Collision Check Thread**: Updates environmental information and checks for collisions along the robot's current path. By offloading collision checking, the replanning thread focuses entirely on computing new paths, improving responsiveness.
- **Replanning Thread**: Executes the path replanning algorithm. Depending on the specific algorithm, replanning is triggered when the current path is obstructed or can be optimized. Once a new path is computed, a time law is generated for the path using the `trajectory_processors_lib`, and the updated trajectory is shared with the other threads.

The threads share crucial information, including the current path, trajectory, scene data, and robot configuration. To ensure a smooth transition to a new trajectory, replanning considers a configuration slightly ahead on the current trajectory (at time *t + time_shift*) rather than the robot's current position (at time *t*). After replanning, the actual robot configuration is reconnected to the new path, for instance, by merging the segment of the current path from the robot's actual position to the planned configuration and appending the new path.

There are also additional features:
- **Spawn Objects Thread**: Dynamically generates virtual objects during robot motion to invalidate its current path. This feature is crucial for testing and simulating replanning algorithms. It relies on the [`cnr_scene_manager`](https://github.com/JRL-CARI-CNR-UNIBS/cnr_scene_manager) for spawning and manipulating obstacles in the MoveIt planning scene.
- **Display Thread**: Visualizes the initial and current paths on RViz.
- **Benchmark Thread**: Collects performance metrics, such as replanning times and path quality, for benchmarking and analysis.

## Tutorials

Visit [this tutorial](documentation/tuutorial/tutorial.md) for step-by-step instructions on using a replanner manager or integrating your custom algorithm with `replanners_managers_lib` and `replanners_lib`. 

