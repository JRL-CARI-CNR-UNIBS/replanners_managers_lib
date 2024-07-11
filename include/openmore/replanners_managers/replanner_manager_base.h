#pragma once

#include <openmore/replanners_managers/utils.h>

namespace openmore
{
class ReplannerManagerBase;
typedef std::shared_ptr<ReplannerManagerBase> ReplannerManagerBasePtr;

class ReplannerManagerBase: public std::enable_shared_from_this<ReplannerManagerBase>
{
#define K_OFFSET 1.1

protected:

  /* To be assigned by the constructor */
  unsigned int   trj_exec_thread_frequency_         ;
  unsigned int   collision_checker_thread_frequency_;
  double         dt_replan_                         ;
  PathPtr        current_path_                      ;
  PathPtr        current_path_shared_               ;
  std::string    group_name_                        ;
  TrajectoryPtr  trajectory_processor_              ;
  TreeSolverPtr  solver_                            ;
  std::string    param_ns_                          ;
  TraceLoggerPtr logger_                            ;

  /* Global variables */
  bool stop_                      ;
  bool benchmark_                 ;
  bool goal_reached_              ;
  bool spawn_objs_                ;
  bool read_safe_scaling_         ;
  bool replanning_enabled_        ;
  bool download_scene_info_       ;
  bool replanner_verbosity_       ;
  bool display_replan_config_     ;
  bool display_current_config_    ;
  bool display_timing_warning_    ;
  bool display_replan_trj_point_  ;
  bool current_path_sync_needed_  ;
  bool display_current_trj_point_ ;
  bool display_replanning_success_;

  unsigned int replanning_thread_frequency_;
  unsigned int parallel_checker_n_threads_ ;
  unsigned int direction_change_           ;

  double t_                          ;
  double dt_                         ;
  double real_time_                  ;
  double obj_max_size_               ;
  double time_shift_                 ;
  double t_replan_                   ;
  double replanning_time_            ;
  double scaling_from_param_         ;
  double checker_resolution_         ;
  double goal_tol_                   ;
  double scaling_                    ;
  double global_override_            ;
  double obj_vel_                    ;
  double dt_move_                    ;

  ros::NodeHandle nh_   ;
  ros::WallTime tic_trj_;

  TrjPointPtr                      pnt_                         ;
  TrjPointPtr                      pnt_unscaled_                ;
  TrjPointPtr                      pnt_replan_                  ;
  ReplannerBasePtr                 replanner_                   ;
  Eigen::VectorXd                  current_configuration_       ;
  Eigen::VectorXd                  configuration_replan_        ;
  MoveitCollisionCheckerPtr        checker_cc_                  ;
  MoveitCollisionCheckerPtr        checker_replanning_          ;
  NodePtr                          path_start_                  ;
  planning_scene::PlanningScenePtr planning_scn_cc_             ;
  planning_scene::PlanningScenePtr planning_scn_replanning_     ;
  sensor_msgs::JointState          new_joint_state_unscaled_    ;
  sensor_msgs::JointState          new_joint_state_             ;
  moveit_msgs::PlanningScene       planning_scene_msg_          ;
  moveit_msgs::PlanningScene       planning_scene_diff_msg_     ;
  moveit_msgs::PlanningScene       planning_scene_msg_benchmark_;

  std::string obj_type_                ;
  std::vector<double> spawn_instants_  ;
  std::vector<std::string> obj_ids_    ;
  std::vector<Eigen::VectorXd> obj_pos_;

  std::thread display_thread_   ;
  std::thread trj_exec_thread_  ;
  std::thread col_check_thread_ ;
  std::thread spawn_obj_thread_ ;
  std::thread benchmark_thread_ ;
  std::thread replanning_thread_;

  std::mutex trj_mtx_         ;
  std::mutex paths_mtx_       ;
  std::mutex scene_mtx_       ;
  std::mutex replanner_mtx_   ;
  std::mutex ovr_mtx_         ;
  std::mutex bench_mtx_       ;

  std::vector<std::string>                                                        scaling_topics_names_ ;
  std::vector<std::shared_ptr<ros_helper::SubscriptionNotifier<std_msgs::Int64>>> scaling_topics_vector_;
  std::map<std::string,double> overrides_;

  ros::Publisher target_pub_         ;
  ros::Publisher obj_pose_pub_       ;
  ros::Publisher text_overlay_pub_   ;
  ros::Publisher unscaled_target_pub_;

  std::string obs_pose_topic_             ;
  std::string joint_target_topic_         ;
  std::string unscaled_joint_target_topic_;
  std::string which_link_display_path_    ;

  ros::ServiceClient add_obj_               ;
  ros::ServiceClient move_obj_              ;
  ros::ServiceClient remove_obj_            ;
  ros::ServiceClient plannning_scene_client_;

  virtual void overrideCallback(const std_msgs::Int64ConstPtr& msg, const std::string& override_name);
  virtual void subscribeTopicsAndServices();
  virtual bool replan();
  virtual void fromParam();
  virtual void downloadPathCost();
  virtual void updateSharedPath();
  virtual bool uploadPathCost(const PathPtr& current_path_updated_copy);
  virtual void attributeInitialization();
  virtual void replanningThread();
  virtual void collisionCheckThread();
  virtual void displayThread();
  virtual void benchmarkThread();
  virtual void spawnObjectsThread();
  virtual void trajectoryExecutionThread();
  virtual double readScalingTopics();

  Eigen::Vector3d forwardIk(const Eigen::VectorXd& conf, const std::string& last_link, const planning_scene::PlanningScenePtr& planning_scene);
  Eigen::Vector3d forwardIk(const Eigen::VectorXd& conf, const std::string& last_link, const planning_scene::PlanningScenePtr& planning_scene, geometry_msgs::Pose &pose);

  virtual void initReplanner()=0;
  virtual bool haveToReplan(const bool path_obstructed)=0;

  inline bool alwaysReplan()
  {
    return true;
  }

  inline bool replanIfObstructed(const bool path_obstructed)
  {
    return path_obstructed;
  }

  void displayTrj(const DisplayPtr& disp);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ReplannerManagerBase(const PathPtr &current_path,
                       const TrajectoryPtr& trajectory_processor,
                       const TreeSolverPtr &solver,
                       const std::string &param_ns,
                       const TraceLoggerPtr& logger);
  ~ReplannerManagerBase();

  void setGroupName(const std::string& group_name)
  {
    group_name_ = group_name;
  }

  void setGoalToll(const double& toll)
  {
    goal_tol_ = toll;
  }

  TrjPoint getJointTarget()
  {
    TrjPoint pnt;
    trj_mtx_.lock();
    *pnt.state_ = *pnt_->state_;
    pnt.time_from_start_ = pnt_->time_from_start_;
    trj_mtx_.unlock();

    return pnt;
  }

  TrjPoint getUnscaledJointTarget()
  {
    TrjPoint pnt_unscaled;
    trj_mtx_.lock();
    *pnt_unscaled.state_ = *pnt_unscaled_->state_;
    pnt_unscaled.time_from_start_ = pnt_unscaled_->time_from_start_;
    trj_mtx_.unlock();

    return pnt_unscaled;
  }

  virtual void setCurrentPath(const PathPtr& current_path)
  {
    current_path_ = current_path;
  }

  ReplannerBasePtr getReplanner()
  {
    return replanner_;
  }

  void enableReplanning(const bool enable)
  {
    replanning_enabled_ = enable;
  }

  bool goalReached()
  {
    return goal_reached_;
  }

  virtual bool joinThreads();
  virtual bool stop();
  virtual bool run();
  virtual bool start();

  virtual void startReplannedPathFromNewCurrentConf(const Eigen::VectorXd& configuration) = 0;
};

}
