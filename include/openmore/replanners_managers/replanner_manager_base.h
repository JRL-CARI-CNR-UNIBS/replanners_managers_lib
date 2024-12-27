/*
Copyright (c) 2024

JRL-CARI CNR-STIIMA/UNIBS
Cesare Tonola, c.tonola001@unibs.it
Manuel Beschi, manuel.beschi@unibs.it

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.

   3. Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include <openmore/replanners_managers/utils.h>

namespace openmore
{
class ReplannerManagerBase;
typedef std::shared_ptr<ReplannerManagerBase> ReplannerManagerBasePtr;

/**
 * @brief Abstract base class for managing path replanning in holonomic robots.
 *
 * This class serves as a foundation for running path replanning during trajectory execution in holonomic robots.
 * It manages critical tasks such as simultaneous trajectory execution, collision detection, and dynamic replanning.
 * Additionally, it includes threads for benchmarking the replanning algorithms and for spawning virtual obstacles to simulate dynamic environments.
 * The class is deeply integrated with ROS, utilizing ROS topics and services, and leverages MoveIt for managing the planning scene.
 */
class ReplannerManagerBase: public std::enable_shared_from_this<ReplannerManagerBase>
{
  /*
   * The replanning configuration is adjusted relative to the robot's current configuration
   * by a factor equal to the maximum replanning time multiplied by K_OFFSET.
   */
#define K_OFFSET 1.1

protected:

  /* To be assigned by the constructor */

  /**
   * @brief Frequency of the trajectory execution thread. Default: 500 Hz.
   */
  unsigned int trj_exec_thread_frequency_;

  /**
   * @brief Frequency of the collision checker thread. Default: 100 Hz.
   */
  unsigned int collision_checker_thread_frequency_;

  /**
   * @brief Maximum time interval for replanning. Default: 0.100 s.
   */
  double dt_replan_;

  /**
   * @brief The current path being followed.
   */
  PathPtr current_path_;

  /**
   * @brief Shared pointer to the current path, used for synchronization across threads.
   */
  PathPtr current_path_shared_;

  /**
   * @brief Name of the group used for planning and execution. No default value.
   */
  std::string group_name_;

  /**
   * @brief Pointer to the trajectory processor, which is in charge of computing timing-laws for the new paths found.
   */
  TrajectoryPtr trajectory_processor_;

  /**
   * @brief Pointer to a solver, required by the path replanning algorithm.
   */
  TreeSolverPtr solver_;

  /**
   * @brief Namespace for accessing parameters using the cnr_param library.
   */
  std::string param_ns_;

  /**
   * @brief Pointer to the cnr_logger logger for tracing and debugging.
   */
  TraceLoggerPtr logger_;

  /* Global variables */

  /**
   * @brief Flag to signal when the entire process should stop.
   */
  bool stop_;

  /**
   * @brief Flag indicating whether benchmarking thread should be enabled. Default: false.
   */
  bool benchmark_;

  /**
   * @brief Flag indicating whether the thread for spawning virtual obstacles should be enabled. Default: false.
   */
  bool spawn_objs_;

  /**
   * @brief Flag indicating whether the goal has been reached.
   */
  bool goal_reached_;

  /**
   * @brief Flag indicating whether the path cost has been updated.
   *
   * This flag is set to false when a new path is computed by the replanner, signaling that
   * the path cost needs to be recalculated. Once the collision check thread synchronizes
   * with the new path, computes its cost, and uploads the updated cost, this flag is set to true.
   */
  bool cost_updated_;

  /**
   * @brief Flag indicating whether to read the safe scaling factor from topic. Default: false.
   */
  bool read_safe_scaling_;

  /**
   * @brief Flag to enable or disable the replanning thread. Default: true.
   */
  bool replanning_enabled_;

  /**
   * @brief Flag to control verbosity of the replanner. Default: false.
   */
  bool replanner_verbosity_;

  /**
   * @brief Flag indicating whether to use graph::collision_check::ParallelMoveitCollisionChecker
   *  or graph::collision_check::MoveitCollisionChecker.  Default: true.
   */
  bool use_parallel_checker_;

  /**
   * @brief Flag to display the configuration used during replanning.
   *
   * This flag controls whether the configuration used for replanning is displayed.
   * The configuration is obtained by projecting the trajectory point considered for replanning, `pnt_replan_`,
   * onto the current path.  Default: true.
   */
  bool display_replan_config_;

  /**
   * @brief Flag to display the current configuration.
   *
   * This flag controls whether the current configuration is displayed.
   * The configuration is obtained by projecting the current trajectory point considered for replanning, `pnt_`,
   * onto the current path.  Default: true.
   */
  bool display_current_config_;

  /**
   * @brief Flag to enable the display of warnings when threads exceed the specified loop rate.  Default: false.
   */
  bool display_timing_warning_;

  /**
   * @brief Flag to display the trajectory point considered for replanning, 'pnt_replan_'.  Default: false.
   */
  bool display_replan_trj_point_;

  /**
   * @brief Flag to display the current trajectory point, 'pnt_'.  Default: true.
   */
  bool display_current_trj_point_;

  /**
   * @brief Flag indicating whether the current path needs to be synchronized by the collision check thread.
   *
   * This flag is set to true by the replanning thread when it updates the shared current path with a new solution.
   * The collision check thread will then update its internal copy of the shared path and reset this flag to false.
   */
  bool current_path_sync_needed_;

  /**
   * @brief Flag to display the success of the replanning process.  Default: false.
   */
  bool display_replanning_success_;

  /**
   * @brief Frequency of the replanning thread.
   */
  unsigned int replanning_thread_frequency_;

  /**
   * @brief Number of threads for the parallel collision checker.  Default: 4.
   */
  unsigned int parallel_checker_n_threads_;

  /**
   * @brief Counter to track the number of direction changes for spawned objects. Used by the spawnObjectsThread().
   *  Default: std::numeric_limits<unsigned int>::max().
   */
  unsigned int direction_change_;

  /**
   * @brief Current time along the followed trajectory.
   *
   * This variable represents the elapsed time along the current trajectory and it is used to sample the trajectory
   * and compute the current trajectory point 'pnt_'.
   * It is reset to zero each time a new trajectory is computed.
   */
  double t_;

  /**
   * @brief Time step for trajectory execution.
   *
   * This variable defines the time increment for each step during trajectory execution.
   */
  double dt_;

  /**
   * @brief Tracks the real-time elapsed since the start of the entire process.
   *
   * This variable represents the total elapsed time since the start of the entire process.
   * It is used by spawnObjectsThread, to manage timing-related tasks.
   */
  double real_time_;

  /**
   * @brief Represents the maximum size of the spawned objects.
   *
   * This variable is used by the benchmarkingThread to detect potential collisions without performing an actual collision check,
   * which improves processing speed. A collision is considered to have occurred when the distance between the current configuration
   * and the object's center is less than `obj_max_size_`. Default: 0.05.
   */
  double obj_max_size_;

  /**
   * @brief Time shift applied to 't_' to determine 't_replan_'.
   *
   * This time shift is used to compute 't_replan_', which is then used to sample the current trajectory
   * in order to calculate 'pnt_replan_' and, subsequently, the current replanning configuration.
   */
  double time_shift_;

  /**
   * @brief Time along the current trajectory considered for replanning.
   *
   * This value represents a point in the future along the current trajectory,
   * used to sample the trajectory and determine the configuration from which replanning should begin.
   */
  double t_replan_;

  /**
   * @brief Time taken for the replanning process.
   */
  double replanning_time_;

  /**
   * @brief Reads the scaling factor from parameters.
   *
   * This scaling factor is set at the beginning and remains constant throughout the process.
   * It is combined with the scaling factor obtained from subscribed topics to adjust the trajectory.  Default: 1.0.
   */
  double scaling_from_param_;

  /**
   * @brief Resolution of the collision checker. Default: 0.05.
   */
  double checker_resolution_;

  /**
   * @brief Tolerance allowed for reaching the goal.  Default: 1e-06.
   */
  double goal_tol_;

  /**
   * @brief Target scaling factor to be applied to the trajectory.
   */
  double target_scaling_;

  /**
   * @brief Actual scaling factor computed during the trajectory interpolation and used to follow the trajectory.
   */
  double updated_scaling_;

  /**
   * @brief Global override read by the topics specified by 'scaling_topics_names_'.
   */
  double global_override_;

  /**
   * @brief Velocity of spawned objects.  Default: 0.0.
   */
  double obj_vel_;

  /**
   * @brief Time interval between each movements of the spawned objects. Default: std::numeric_limits<double>::infinity().
   */
  double dt_move_;

  /**
   * @brief ROS node handle used for managing ROS communication.
   */
  ros::NodeHandle nh_;

  /**
   * @brief Timestamp when the replanning thread reads the current configuration.
   *
   * This variable stores the time when the `replanningThread` captures the current configuration
   * before adapting the replanned path to it using `startReplannedPathFromNewCurrentConf`.
   */
  graph_time_point tic_current_conf_;

  /**
   * @brief Pointer to the current trajectory point.
   */
  TrjPointPtr pnt_;

  /**
   * @brief Pointer to the current unscaled trajectory point.
   */
  TrjPointPtr pnt_unscaled_;

  /**
   * @brief Pointer to the replanned trajectory point.
   */
  TrjPointPtr pnt_replan_;

  /**
   * @brief Pointer to the replanner instance.
   */
  ReplannerBasePtr replanner_;

  /**
   * @brief Current configuration of the robot.
   */
  Eigen::VectorXd current_configuration_;

  /**
   * @brief Configuration used for replanning.
   */
  Eigen::VectorXd configuration_replan_;

  /**
   * @brief Pointer to the collision checker used by the collision check thread.
   */
  MoveitCollisionCheckerPtr checker_cc_;

  /**
   * @brief Pointer to the collision checker used by the replanner.
   */
  MoveitCollisionCheckerPtr checker_replanning_;

  /**
   * @brief Pointer to the starting node of the path.
   */
  NodePtr path_start_;

  /**
   * @brief Pointer to the planning scene used by the collision check thread.
   */
  planning_scene::PlanningScenePtr planning_scn_cc_;

  /**
   * @brief Pointer to the planning scene used by the replanning thread.
   */
  planning_scene::PlanningScenePtr planning_scn_replanning_;

  /**
   * @brief Unscaled joint state messages.
   */
  sensor_msgs::JointState new_joint_state_unscaled_;

  /**
   * @brief Joint state message.
   */
  sensor_msgs::JointState new_joint_state_;

  /**
   * @brief Message representing the updated current planning scene get from MoveIt service.
   */
  moveit_msgs::PlanningScene planning_scene_msg_;

  /**
   * @brief Message representing the updated differential planning scene get from MoveIt service.
   */
  moveit_msgs::PlanningScene planning_scene_diff_msg_;

  /**
   * @brief Message representing the planning scene used by the benchmarking thread.
   */
  moveit_msgs::PlanningScene planning_scene_msg_benchmark_;

  /**
   * @brief Type of objects to be spawned. Deault: "sphere".
   */
  std::string obj_type_;

  /**
   * @brief Instants at which objects should be spawned. Default: {0.5}.
   */
  std::vector<double> spawn_instants_;

  /**
   * @brief List of object IDs for spawned objects.
   */
  std::vector<std::string> obj_ids_;

  /**
   * @brief Positions of spawned objects.
   */
  std::vector<Eigen::VectorXd> obj_pos_;

  /**
   * @brief Thread for displaying the current trajectory and related information.
   */
  std::thread display_thread_;

  /**
   * @brief Thread for executing the trajectory.
   */
  std::thread trj_exec_thread_;

  /**
   * @brief Thread for checking collisions during trajectory execution.
   */
  std::thread col_check_thread_;

  /**
   * @brief Thread for spawning objects in the environment.
   */
  std::thread spawn_obj_thread_;

  /**
   * @brief Thread for running benchmarking operations.
   */
  std::thread benchmark_thread_;

  /**
   * @brief Thread for managing the replanning process.
   */
  std::thread replanning_thread_;

  /**
   * @brief Mutex for synchronizing access to the trajectory data.
   */
  std::mutex trj_mtx_;

  /**
   * @brief Mutex for synchronizing access to the path data.
   */
  std::mutex paths_mtx_;

  /**
   * @brief Mutex for synchronizing access to the planning scene.
   */
  std::mutex scene_mtx_;

  /**
   * @brief Mutex for synchronizing access to the replanner data.
   */
  std::mutex replanner_mtx_;

  /**
   * @brief Mutex for synchronizing access to override data.
   */
  std::mutex ovr_mtx_;

  /**
   * @brief Mutex for synchronizing access to benchmarking data.
   */
  std::mutex bench_mtx_;

  /**
   * @brief List of topic names used for scaling overrides. Default: "/speed_ovr","/safe_ovr_1","/safe_ovr_2
   */
  std::vector<std::string> scaling_topics_names_;

  /**
   * @brief Vector of subscription notifiers for scaling topics.
   */
  std::vector<std::shared_ptr<ros_helper::SubscriptionNotifier<std_msgs::Int64>>> scaling_topics_vector_;

  /**
   * @brief Map of scaling overrides by topic name.
   */
  std::map<std::string, double> overrides_;

  /**
   * @brief ROS publisher for joint target states.
   */
  ros::Publisher target_pub_;

  /**
   * @brief ROS publisher for object poses.
   */
  ros::Publisher obj_pose_pub_;

  /**
   * @brief ROS publisher for text overlays in RViz. Subscribes topic "/rviz_text_overlay_replanner_bench".
   */
  ros::Publisher text_overlay_pub_;

  /**
   * @brief ROS publisher for unscaled joint target states.
   */
  ros::Publisher unscaled_target_pub_;

  /**
   * @brief Topic name for obstacle poses. Default: "/poses".
   */
  std::string obs_pose_topic_;

  /**
   * @brief Topic name for joint target states. Default: "/joint_target_replanning".
   */
  std::string joint_target_topic_;

  /**
   * @brief Topic name for unscaled joint target states. Default "/unscaled_joint_target_replanning".
   */
  std::string unscaled_joint_target_topic_;

  /**
   * @brief Name of the link to be displayed in the path visualization. Default: "end_effector".
   */
  std::string which_link_display_path_;

  /**
   * @brief ROS service client for adding objects to the scene.
   */
  ros::ServiceClient add_obj_;

  /**
   * @brief ROS service client for moving objects in the scene.
   */
  ros::ServiceClient move_obj_;

  /**
   * @brief ROS service client for removing objects from the scene.
   */
  ros::ServiceClient remove_obj_;

  /**
   * @brief ROS service client for getting the planning scene upates from MoveIt.
   */
  ros::ServiceClient plannning_scene_client_;

  /**
   * @brief Subscribes to necessary ROS topics and services.
   *
   * This method sets up subscriptions to ROS topics and services required for replanning,
   * such as topics for speed overrides and services for managing the planning scene and objects in the environment.
   */
  virtual void subscribeTopicsAndServices();

  /**
   * @brief Initiates the replanning process.
   *
   * This virtual method triggers the replanning algorithm, attempting to find an improved or alternate path
   * based on the current situation. Derived classes can override this method to implement specific replanning logic.
   *
   * @return True if the replanning process successfully generates a new path, otherwise false.
   */
  virtual bool replan();

  /**
   * @brief Reads and initializes parameters using the cnr_param library.
   *
   * This method retrieves various parameters required by the replanner through the cnr_param library,
   * setting up internal variables based on these values. Parameters are read from the namespace specified by 'param_ns_'.
   */
  virtual void fromParam();

  /**
   * @brief Synchronizes the path cost from the shared path to the current path.
   *
   * This method updates the costs of connections in the current path to match those in the shared path,
   * ensuring consistency between them. The shared path's costs are periodically updated by the collision
   * check thread using the `uploadPathCost` function. Before the replanning routine begins, this method
   * is used to download the latest costs, maintaining alignment between the two paths.
   */
  virtual void downloadPathCost();

  /**
   * @brief Updates the shared path with the latest replanning solution.
   *
   * This method updates `current_path_shared_` with the new path solution found by the `replanningThread`.
   * It ensures that the shared path is synchronized with the latest replanning outcome, allowing other
   * threads to access and work with the most current path.
   */
  virtual void updateSharedPath();

  /**
   * @brief Uploads and updates the path cost in the shared path.
   *
   * This method updates the connection costs in the shared path based on a provided updated path,
   * ensuring that the shared path reflects the latest cost information. This is done only if the shared path
   * has not been modified in the meantime, and the provided updated path is still an accurate copy of the shared path.
   *
   * @param current_path_updated_copy A copy of the updated path from which to upload the connection costs.
   * @return True if the path costs were successfully updated, otherwise false.
   */
  virtual bool uploadPathCost(const PathPtr& current_path_updated_copy);

  /**
   * @brief Initializes key attributes and configurations for the ReplannerManagerBase.
   *
   * This method is responsible for setting up and initializing various class attributes,
   * configuring the planning scene, and ensuring that the trajectory processor and collision
   * checker objects are correctly initialized. This function handles the general initialization
   * tasks required for the replanning manager to function properly.
   */
  virtual void attributeInitialization();

  /**
   * @brief Replanning thread responsible for managing replanning operations.
   *
   * This thread periodically checks if replanning is needed, performs replanning,
   * and updates the trajectory accordingly.
   */
  virtual void replanningThread();

  /**
   * @brief Collision checking thread responsible for checking collisions.
   *
   * This thread continuously checks for collisions along the current path and updates
   * the planning scene if any changes are detected.
   */
  virtual void collisionCheckThread();

  /**
   * @brief Visualizes the current trajectory and related path information.
   *
   * This method is responsible for displaying the initial path, the current executing path,
   * waypoints, and the current and replanned trajectory points, including their projections
   * on the current path. The specific elements to be displayed can be configured through parameters.
   */
  virtual void displayThread();

  /**
   * @brief Benchmarking thread for evaluating the replanner.
   *
   * This thread benchmarks the performance of the replanner by tracking collisions,
   * path length, and other relevant metrics during trajectory execution.
   */
  virtual void benchmarkThread();

  /**
   * @brief Spawns objects in the environment for testing.
   *
   * This thread manages the spawning of objects in the environment, updating their
   * positions periodically to simulate dynamic obstacles.
   */
  virtual void spawnObjectsThread();

  /**
   * @brief Thread responsible for executing the robot's trajectory.
   *
   * This thread manages the execution of the planned trajectory, continuously updating
   * the current configuration and publishing joint states to the relevant ROS topics
   * It also reads the latest published speed overrides to appropriately scale the robot's
   * trajectory during execution. The scaled trajectory point is published on the topic defined by
   * 'joint_target_topic_', while the unscaled one on the topic 'unscaled_joint_target_topic_'.
   */
  virtual void trajectoryExecutionThread();

  /**
   * @brief Reads the current scaling factor from the scaling topics defined by 'scaling_topics_names_'.
   *
   * This method retrieves the global override scaling factor that has been set based on
   * messages received from the subscribed scaling topics.
   *
   * @return The current global override scaling factor.
   */
  virtual double readScalingTopics();

  /**
   * @brief Callback function for updating the scaling factor from published messages.
   *
   * This method updates internal scaling overrides based on messages received from specified topics.
   * The overall scaling factor is calculated as the product of the overrides published on the topics listed in 'scaling_topics_names_'.
   * This scaling factor is used to adjust the robot's trajectory if 'read_safe_scaling_' is enabled.
   *
   * @param msg The message received from the topic containing the scaling value.
   * @param override_name The name of the specific override parameter being updated.
   */
  virtual void overrideCallback(const std_msgs::Int64ConstPtr& msg, const std::string& override_name);

  /**
   * @brief Performs forward kinematics to calculate the position (x,y,z) of the 'last_link'.
   *
   * This method calculates the end effector position based on the given robot configuration.
   *
   * @param conf The configuration to be used.
   * @param last_link The name of the last link, for which the position should be computed.
   * @param planning_scene The planning scene to be used.
   * @param pose The calculated pose of the 'last_link'.
   * @return The calculated position.
   */
  Eigen::Vector3d forwardIk(const Eigen::VectorXd& conf, const std::string& last_link, const planning_scene::PlanningScenePtr& planning_scene);
  Eigen::Vector3d forwardIk(const Eigen::VectorXd& conf, const std::string& last_link, const planning_scene::PlanningScenePtr& planning_scene, geometry_msgs::Pose &pose);

  /**
   * @brief Pure virtual function to initialize the replanner.
   *
   * This method is responsible for setting up the replanner object, which will be used
   * to handle dynamic replanning based on the current configuration and path.
   * Derived classes must implement this method to define their specific replanner initialization.
   */
  virtual void initReplanner()=0;

  /**
   * @brief Pure virtual function to determine if replanning is required.
   *
   * This method evaluates whether the current path necessitates replanning based on specific conditions.
   * Derived classes must implement this function to define the criteria for triggering the replanning process.
   *
   * For algorithms that always replan to seek an improved solution, this function can be implemented by invoking `alwaysReplan()`.
   * Alternatively, if replanning is only needed when the current path is obstructed, the implementation should call `replanIfObstructed()`.
   *
   * @param path_obstructed A boolean indicating whether the current path is blocked or obstructed.
   * @return True if replanning is necessary, otherwise false.
   */
  virtual bool haveToReplan(const bool path_obstructed)=0;

  /**
   * @brief Always triggers replanning.
   *
   * This inline method returns true, indicating that replanning should always be performed.
   * It can be used as a default behavior in situations where continuous replanning is desired.
   *
   * @return True, indicating that replanning is always required.
   */
  inline bool alwaysReplan()
  {
    return true;
  }

  /**
   * @brief Triggers replanning only if the path is obstructed.
   *
   * This inline method checks whether the path is obstructed and returns true if replanning
   * is needed. It is useful in scenarios where replanning should occur only when necessary.
   *
   * @param path_obstructed A boolean indicating whether the current path is obstructed.
   * @return True if the path is obstructed and replanning is required, otherwise false.
   */
  inline bool replanIfObstructed(const bool path_obstructed)
  {
    return path_obstructed;
  }

  /**
   * @brief Preprocesses a trajectory path before computing the timing-law.
   *
   * This function handles the preprocessing of the path for which the trajectory is computed.
   * Typically, it simplifies the path to produce smoother trajectories. The basic implementation
   * removes duplicate waypoints and resamples nodes to ensure better tracking during trajectory execution.
   *
   * @param path The path to be preprocessed. (Note: The original path should not be modified; use a clone instead.)
   * @return The preprocessed path with improvements applied.
   */
  virtual PathPtr preprocessTrajectoryPath(const PathPtr& path);

  /**
   * @brief Joins all threads.
   *
   * This method waits for all threads to finish execution and joins them.
   *
   * @return True if all threads were successfully joined.
   */
  virtual bool joinThreads();

  /**
   * @brief Runs the replanning manager.
   *
   * This method initializes attributes and starts all necessary threads for the replanning
   * process.
   *
   * @return True if the manager was successfully started.
   */
  virtual bool run();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Initializes the ReplannerManagerBase with given parameters.
   *
   * This constructor initializes the manager with the current path, trajectory processor,
   * solver, parameter namespace, and logger.
   * It reads the required parameters under the provided namespace and sets up necessary ROS topics and services.
   *
   * @param current_path The current robot path to be used.
   * @param trajectory_processor The trajectory processor to be used, e.g the algorithm to compute the timing-law given a path.
   * @param solver The solver to be used, usually required by the path replanning algorithm.
   * @param param_ns The namespace for parameters.
   * @param logger The logger to be used.
   */
  ReplannerManagerBase(const PathPtr &current_path,
                       const TrajectoryPtr& trajectory_processor,
                       const TreeSolverPtr &solver,
                       const std::string &param_ns,
                       const TraceLoggerPtr& logger);

  /**
   * @brief Destructor for ReplannerManagerBase.
   *
   * The destructor handles any necessary cleanup when the manager is destroyed.
   */
  ~ReplannerManagerBase();

  /**
   * @brief Sets the name of the group for planning.
   *
   * This method assigns a group name that will be used for planning and execution.
   *
   * @param group_name The name of the group to be used.
   */
  void setGroupName(const std::string& group_name)
  {
    group_name_ = group_name;
  }

  /**
   * @brief Sets the goal tolerance.
   *
   * This method defines the tolerance allowed for reaching the goal during path execution.
   *
   * @param toll The tolerance value to be set.
   */
  void setGoalToll(const double& toll)
  {
    goal_tol_ = toll;
  }

  /**
   * @brief Retrieves the current joint target.
   *
   * This method returns the latest joint target sampled from the trajectory, which includes the state and the time
   * from the start of the trajectory.
   *
   * @return The current joint target as a TrjPoint.
   */
  TrjPoint getJointTarget()
  {
    TrjPoint pnt;
    trj_mtx_.lock();
    *pnt.state_ = *pnt_->state_;
    pnt.time_from_start_ = pnt_->time_from_start_;
    trj_mtx_.unlock();

    return pnt;
  }

  /**
   * @brief Retrieves the current unscaled joint target.
   *
   * This method returns the latest unscaled joint target sampled from the trajectory, which includes the state and the time
   * from the start of the trajectory.
   *
   * @return The current unscaled joint target as a TrjPoint.
   */
  TrjPoint getUnscaledJointTarget()
  {
    TrjPoint pnt_unscaled;
    trj_mtx_.lock();
    *pnt_unscaled.state_ = *pnt_unscaled_->state_;
    pnt_unscaled.time_from_start_ = pnt_unscaled_->time_from_start_;
    trj_mtx_.unlock();

    return pnt_unscaled;
  }

  /**
   * @brief Sets the current path.
   *
   * This method assigns a new current path to be followed by the planner.
   *
   * @param current_path The new path to be set as the current path.
   */
  virtual void setCurrentPath(const PathPtr& current_path)
  {
    current_path_ = current_path;
  }

  /**
   * @brief Retrieves the current replanner instance.
   *
   * This method returns the current replanner object being used for path replanning.
   *
   * @return A shared pointer to the current replanner instance.
   */
  ReplannerBasePtr getReplanner()
  {
    return replanner_;
  }

  /**
   * @brief Enables or disables replanning.
   *
   * This method enables or disables the replanning process based on the provided flag.
   * If replanning is not enabled, the replanning thread will not be executed.
   *
   * @param enable Set to true to enable replanning, false to disable.
   */
  void enableReplanning(const bool enable)
  {
    replanning_enabled_ = enable;
  }

  /**
   * @brief Checks if the goal has been reached.
   *
   * This method returns true if the goal has been reached, otherwise false.
   *
   * @return True if the goal is reached, false otherwise.
   */
  bool goalReached()
  {
    return goal_reached_;
  }

  /**
   * @brief Stops the replanning process.
   *
   * This method stops the replanning process and joins all threads.
   *
   * @return True if the process was successfully stopped.
   */
  virtual bool stop();

  /**
   * @brief Starts the replanning manager and waits for all threads to finish.
   *
   * This method runs the replanning manager and blocks until all threads are finished.
   *
   * @return True if the manager was successfully started and completed.
   */
  virtual bool start();

  /**
   * @brief Pure virtual function to adjust the start of the replanned path to a new current configuration.
   *
   * This method allows the derived class to modify the replanned path so that it begins from a new current configuration.
   * It is typically called after a successful replanning operation.
   *
   * This function is crucial because the replanning algorithm often considers a robot configuration that is slightly
   * ahead of the current one along the path. This adjustment accounts for the non-zero time required by the replanning
   * process and helps ensure smoother transitions by starting from a configuration further along the path.
   *
   * @param configuration The new configuration from which the replanned path should commence execution.
   */
  virtual void startReplannedPathFromNewCurrentConf(const Eigen::VectorXd& configuration) = 0;
};

}
