#include <openmore/replanners_managers/replanner_manager_base.h>

namespace openmore
{

ReplannerManagerBase::ReplannerManagerBase(const PathPtr &current_path,
                                           const TrajectoryPtr& trajectory_processor,
                                           const TreeSolverPtr &solver,
                                           const std::string& param_ns,
                                           const TraceLoggerPtr &logger):
  current_path_(current_path),trajectory_processor_(trajectory_processor),solver_(solver),param_ns_(param_ns),logger_(logger)
{
  nh_ = ros::NodeHandle("~");
  replanning_enabled_ = true;

  fromParam();
  subscribeTopicsAndServices();
}

ReplannerManagerBase::~ReplannerManagerBase()
{
}

void ReplannerManagerBase::fromParam()
{
  unsigned int trj_freq_default = 500;
  unsigned int cc_freq_default = 30;
  unsigned int cc_n_threads_default = std::thread::hardware_concurrency();

  get_param(logger_,param_ns_,"trj_execution_thread_frequency",trj_exec_thread_frequency_,trj_freq_default);
  get_param(logger_,param_ns_,"collision_checker_thread_frequency",collision_checker_thread_frequency_,cc_freq_default);
  get_param(logger_,param_ns_,"dt_replan",dt_replan_,0.100);
  get_param(logger_,param_ns_,"checker_resolution",checker_resolution_,0.05);
  get_param(logger_,param_ns_,"parallel_checker_n_threads",parallel_checker_n_threads_,cc_n_threads_default);
  get_param(logger_,param_ns_,"read_safe_scaling",read_safe_scaling_,false);

  if(read_safe_scaling_)
    get_param(logger_,param_ns_,"overrides",scaling_topics_names_,{"/speed_ovr","/safe_ovr_1","/safe_ovr_2"});

  if(get_param(logger_,param_ns_,"goal_tol",goal_tol_,1.0e-06))
  {
    if(goal_tol_<graph::core::TOLERANCE)
    {
      goal_tol_ = graph::core::TOLERANCE;
      CNR_WARN(logger_,"goal_tol set equal to TOLERANCE (%f), it can't be less than that value", graph::core::TOLERANCE);
    }
  }

  if(not get_param(logger_,param_ns_,"group_name",group_name_))
    CNR_WARN(logger_,"group_name not set, maybe set later with setGroupName(..)?");

  get_param(logger_,param_ns_,"joint_target_topic",joint_target_topic_,std::string("/joint_target_replanning"));
  get_param(logger_,param_ns_,"unscaled_joint_target_topic",unscaled_joint_target_topic_,std::string("/unscaled_joint_target_replanning"));
  get_param(logger_,param_ns_,"scaling",scaling_from_param_,1.0);
  get_param(logger_,param_ns_,"display_timing_warning",display_timing_warning_,false);
  get_param(logger_,param_ns_,"display_replanning_success",display_replanning_success_,false);
  get_param(logger_,param_ns_,"replanner_verbosity",replanner_verbosity_,false);
  get_param(logger_,param_ns_,"display_replan_trj_point",display_replan_trj_point_,false);
  get_param(logger_,param_ns_,"display_replan_config",display_replan_config_,true);
  get_param(logger_,param_ns_,"display_current_trj_point",display_current_trj_point_,true);
  get_param(logger_,param_ns_,"display_current_config",display_current_config_,true);
  get_param(logger_,param_ns_,"which_link_display_path",which_link_display_path_,std::string(""));
  get_param(logger_,param_ns_,"benchmark",benchmark_,false);

  if(get_param(logger_,param_ns_,"virtual_obj/spawn_objs",spawn_objs_,false))
  {
    spawn_instants_.clear();
    get_param(logger_,param_ns_,"virtual_obj/spawn_instants",spawn_instants_,{0.5});
    get_param(logger_,param_ns_,"virtual_obj/obj_type",obj_type_,std::string("sphere"));
    get_param(logger_,param_ns_,"virtual_obj/obj_max_size",obj_max_size_,0.05);
    get_param(logger_,param_ns_,"virtual_obj/obj_vel",obj_vel_,0.0);
    get_param(logger_,param_ns_,"virtual_obj/dt_move",dt_move_,std::numeric_limits<double>::infinity());
    get_param(logger_,param_ns_,"virtual_obj/moves_before_change_direction",direction_change_,std::numeric_limits<unsigned int>::max());
    get_param(logger_,param_ns_,"virtual_obj/obs_pose_topic",obs_pose_topic_,std::string("/poses"));
  }
}

void ReplannerManagerBase::attributeInitialization()
{
  stop_                        = false;
  goal_reached_                = false;
  download_scene_info_         = true ;
  current_path_sync_needed_    = false;
  replanning_time_             = 0.0  ;
  scaling_                     = 1.0  ;
  real_time_                   = 0.0  ;
  t_                           = 0.0  ;
  dt_                          = 1.0/double(trj_exec_thread_frequency_);
  time_shift_                  = dt_replan_*K_OFFSET           ;
  t_replan_                    = t_+time_shift_                ;
  replanning_thread_frequency_ = 100                           ;
  global_override_             = 0.0                           ;

  if(group_name_.empty())
    throw std::invalid_argument("group name not set");

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

  moveit_msgs::GetPlanningScene ps_srv;
  if(not plannning_scene_client_.call(ps_srv))
    throw std::runtime_error("call to planning scene srv not ok");

  planning_scn_cc_ = std::make_shared<planning_scene::PlanningScene>(kinematic_model);
  if (not planning_scn_cc_->setPlanningSceneMsg(ps_srv.response.scene))
    throw std::runtime_error("unable to update planning scene");
  planning_scn_replanning_ = planning_scn_cc_->diff();

  planning_scene_msg_              = ps_srv.response.scene;
  planning_scene_diff_msg_.is_diff = true;
  planning_scene_diff_msg_.world   = ps_srv.response.scene.world;

  robot_state::RobotState state(planning_scn_cc_->getCurrentState());
  const robot_state::JointModelGroup* joint_model_group = state.getJointModelGroup(group_name_);
  std::vector<std::string> joint_names = joint_model_group->getActiveJointModelNames();

  current_path_shared_ = current_path_->clone();

  checker_cc_         = std::make_shared<ParallelMoveitCollisionChecker>(planning_scn_cc_,        group_name_,logger_,parallel_checker_n_threads_,checker_resolution_);
  checker_replanning_ = std::make_shared<ParallelMoveitCollisionChecker>(planning_scn_replanning_,group_name_,logger_,parallel_checker_n_threads_,checker_resolution_);

  current_path_shared_->setChecker(checker_cc_        );
  current_path_       ->setChecker(checker_replanning_);
  solver_             ->setChecker(checker_replanning_);

  double scaling = scaling_from_param_;
  if(read_safe_scaling_)
    scaling = scaling*readScalingTopics();

  // Check if trajectory_processor_ has already computed a trajectory for current_path_
  if(trajectory_processor_->getPath().empty())
    trajectory_processor_->setPath(current_path_->getWaypoints()); //trj is now an empty vector
  else
  { //check if the path in the trajectory_processor_ corresponds to the current path
    bool same_path = true;
    if(trajectory_processor_->getPath().size() != (current_path_->getConnectionsSize()+1)) //number of waypoints is number of connections + 1
      same_path = false;
    else
    {
      //if same size, check each waypoint
      for(size_t i=0;i<trajectory_processor_->getPath().size();i++)
      {
        const auto wp = current_path_->getWaypoints();
        if(trajectory_processor_->getPath()[i] != wp[i])
        {
          same_path = false;
          break;
        }
      }
    }

    if(not same_path)
      trajectory_processor_->setPath(current_path_->getWaypoints()); //trj is now an empty vector
  }

  if(trajectory_processor_->getTrj().empty()) //if trj was not computed externally
    trajectory_processor_->computeTrj();

  pnt_          = std::make_shared<TrjPoint>();
  pnt_unscaled_ = std::make_shared<TrjPoint>();
  pnt_replan_   = std::make_shared<TrjPoint>();

  trajectory_processor_->interpolate(t_replan_,pnt_replan_  ,scaling            );
  trajectory_processor_->interpolate(t_       ,pnt_         ,scaling            );
  trajectory_processor_->interpolate(t_       ,pnt_unscaled_,scaling_from_param_);

  Eigen::VectorXd point2project(joint_names.size());
  for(size_t i=0; i<pnt_replan_->state_->pos_.size();i++)
    point2project(i) = pnt_replan_->state_->pos_[i];

  configuration_replan_  = current_path_shared_->projectOnPath(point2project);
  current_configuration_ = current_path_shared_->getStartNode()->getConfiguration();

  initReplanner();
  replanner_->setVerbosity(replanner_verbosity_);

  obj_ids_.clear();

  new_joint_state_.position                 = pnt_->state_->pos_              ;
  new_joint_state_.velocity                 = pnt_->state_->vel_              ;
  new_joint_state_.name                     = joint_names                     ;
  new_joint_state_.header.frame_id          = kinematic_model->getModelFrame();
  new_joint_state_.header.stamp             = ros::Time::now()                ;
  new_joint_state_unscaled_.position        = pnt_unscaled_->state_->pos_     ;
  new_joint_state_unscaled_.velocity        = pnt_unscaled_->state_->vel_     ;
  new_joint_state_unscaled_.name            = joint_names                     ;
  new_joint_state_unscaled_.header.frame_id = kinematic_model->getModelFrame();
  new_joint_state_unscaled_.header.stamp    = ros::Time::now()                ;
}

void ReplannerManagerBase::overrideCallback(const std_msgs::Int64ConstPtr& msg, const std::string& override_name)
{
  double ovr;
  double global_override = 1.0;

  if (msg->data>100)
    ovr=1.0;
  else if (msg->data<0)
    ovr=0.0;
  else
    ovr=msg->data*0.01;

  overrides_.at(override_name)=ovr;
  for(const std::pair<std::string,double>& p: overrides_)
    global_override *= p.second;

  ovr_mtx_.lock();
  global_override_ = global_override;
  ovr_mtx_.unlock();
}

void ReplannerManagerBase::subscribeTopicsAndServices()
{
  scaling_topics_vector_.clear();
  for(const std::string &scaling_topic_name : scaling_topics_names_)
  {
    auto cb = boost::bind(&ReplannerManagerBase::overrideCallback,this,_1,scaling_topic_name);
    scaling_topics_vector_.push_back(std::make_shared<ros_helper::SubscriptionNotifier<std_msgs::Int64>>
                                     (nh_,scaling_topic_name,1,cb));

    overrides_.insert(std::pair<std::string,double>(scaling_topic_name,0.0));
    CNR_INFO(logger_,"Subscribing speed override topic "<<scaling_topic_name.c_str());
  }

  target_pub_          = nh_.advertise<sensor_msgs::JointState>(joint_target_topic_,         10);
  unscaled_target_pub_ = nh_.advertise<sensor_msgs::JointState>(unscaled_joint_target_topic_,10);

  if(benchmark_)
    text_overlay_pub_ = nh_.advertise<jsk_rviz_plugins::OverlayText>("/rviz_text_overlay_replanner_bench",1);

  plannning_scene_client_ = nh_.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");

  if(not plannning_scene_client_.waitForExistence(ros::Duration(10)))
    throw std::runtime_error("unable to connect to /get_planning_scene");

  if(spawn_objs_)
  {
    obj_pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>(obs_pose_topic_,10);

    add_obj_    = nh_.serviceClient<cnr_scene_manager_msgs::AddObjects   >("/cnr_scene_manager/add_objects"   );
    move_obj_   = nh_.serviceClient<cnr_scene_manager_msgs::MoveObjects  >("/cnr_scene_manager/move_objects"  );
    remove_obj_ = nh_.serviceClient<cnr_scene_manager_msgs::RemoveObjects>("/cnr_scene_manager/remove_objects");

    if(not add_obj_.waitForExistence(ros::Duration(10)))
    {
      CNR_ERROR(logger_,"unable to connect to /cnr_scene_manager/add_objects");
      spawn_objs_ = false;
    }

    if(not move_obj_.waitForExistence(ros::Duration(10)))
    {
      CNR_ERROR(logger_,"unable to connect to /cnr_scene_manager/move_objects");
      spawn_objs_ = false;
    }

    if(not remove_obj_.waitForExistence(ros::Duration(10)))
    {
      CNR_ERROR(logger_,"unable to connect to /cnr_scene_manager/remove_objects");
      spawn_objs_ = false;
    }
  }
}

void ReplannerManagerBase::updateSharedPath()
{
  current_path_shared_ = current_path_->clone();
  current_path_shared_->setChecker(checker_cc_);
  current_path_sync_needed_ = true;

  download_scene_info_ = false;
}

void ReplannerManagerBase::downloadPathCost()
{
  std::vector<ConnectionPtr> current_path_conn        = current_path_       ->getConnections();
  std::vector<ConnectionPtr> current_path_shared_conn = current_path_shared_->getConnections();

  std::vector<ConnectionPtr>::iterator it        = current_path_conn       .end();
  std::vector<ConnectionPtr>::iterator it_shared = current_path_shared_conn.end();

  while(it>current_path_conn.begin() && it_shared>current_path_shared_conn.begin())
  {
    it--;
    it_shared--;

    if((*it)->getParent()->getConfiguration() == (*it_shared)->getParent()->getConfiguration() &&
       (*it)->getChild() ->getConfiguration() == (*it_shared)->getChild() ->getConfiguration())
    {
      (*it)->setCost((*it_shared)->getCost());
    }
    else
      break;
  }

  current_path_->cost(); //update path cost
}

bool ReplannerManagerBase::uploadPathCost(const PathPtr& current_path_updated_copy)
{
  bool updated = true;

  if(not current_path_sync_needed_)
  {
    std::vector<ConnectionPtr> current_path_conns      = current_path_shared_     ->getConnections();
    std::vector<ConnectionPtr> current_path_copy_conns = current_path_updated_copy->getConnections();
    for(size_t z=0;z<current_path_conns.size();z++)
    {
      assert(current_path_conns.size() == current_path_copy_conns.size());
      assert((current_path_conns.at(z)->getParent()->getConfiguration() == current_path_copy_conns.at(z)->getParent()->getConfiguration()) &&
             (current_path_conns.at(z)->getChild() ->getConfiguration() == current_path_copy_conns.at(z)->getChild() ->getConfiguration()));

      current_path_conns.at(z)->setCost(current_path_copy_conns.at(z)->getCost());
    }
    current_path_shared_->cost();

  }
  else
    updated = false;

  if(current_path_shared_->getCostFromConf(current_configuration_) == std::numeric_limits<double>::infinity() && (display_timing_warning_ || display_replanning_success_))
    CNR_INFO(logger_,RESET()<<BOLDMAGENTA()<<"Obstacle detected!"<<RESET());

  return updated;
}

void ReplannerManagerBase::replanningThread()
{
  ros::WallRate lp(replanning_thread_frequency_);
  ros::WallRate fast_lp(2000);

  ros::WallTime tic,toc,tic_rep,toc_rep;

  PathPtr path2project_on;
  Eigen::VectorXd current_configuration;
  Eigen::VectorXd point2project(pnt_replan_->state_->pos_.size());

  size_t n_size_before;
  bool success = false;
  bool path_changed = false;
  bool path_obstructed = true;
  double replanning_duration = 0.0;
  double duration, abscissa_current_configuration, abscissa_replan_configuration;

  Eigen::VectorXd projection = configuration_replan_;
  Eigen::VectorXd past_projection = configuration_replan_;
  Eigen::VectorXd goal_conf = replanner_->getGoal()->getConfiguration();
  PathPtr trj_path;

  while((not stop_) && ros::ok())
  {
    tic = ros::WallTime::now();

    if(not download_scene_info_)
    {
      fast_lp.sleep(); //wait for 0.5 ms
      continue;
    }

    trj_mtx_.lock();
    trajectory_processor_->interpolate(t_replan_,pnt_replan_,scaling_);

    for(size_t i=0; i<pnt_replan_->state_->pos_.size();i++)
      point2project(i) = pnt_replan_->state_->pos_[i];

    current_configuration = current_configuration_;
    trj_mtx_.unlock();

    if((point2project-goal_conf).norm()>goal_tol_)
    {
      paths_mtx_.lock();
      path2project_on = current_path_shared_->clone();
      paths_mtx_.unlock();

      projection = path2project_on->projectOnPath(point2project,past_projection,false);
      past_projection = projection;

      abscissa_replan_configuration  = path2project_on->curvilinearAbscissaOfPoint(projection);
      abscissa_current_configuration = path2project_on->curvilinearAbscissaOfPoint(current_configuration);

      if(abscissa_replan_configuration<=abscissa_current_configuration)
        projection = path2project_on->pointOnCurvilinearAbscissa(abscissa_current_configuration+0.05);  //5% step forward

      replanner_mtx_.lock();
      configuration_replan_ = projection;
      replanner_mtx_.unlock();

      scene_mtx_.lock();
      checker_replanning_->setPlanningSceneMsg(planning_scene_diff_msg_);
      paths_mtx_.lock();
      downloadPathCost();
      paths_mtx_.unlock();
      planning_scene_msg_benchmark_ = planning_scene_msg_;
      scene_mtx_.unlock();

      replanner_mtx_.lock();
      if(not (current_path_->findConnection(configuration_replan_)))
      {
        CNR_INFO(logger_,RESET()<<BOLDYELLOW()<<"configuration replan not found on path"<<RESET());
        trj_mtx_.lock();
        configuration_replan_ = current_configuration_;
        trj_mtx_.unlock();
      }

      replanner_->setCurrentPath(current_path_);
      replanner_->setChecker(checker_replanning_);
      replanner_->setCurrentConf(configuration_replan_);

      path_obstructed = (current_path_->getCostFromConf(configuration_replan_) == std::numeric_limits<double>::infinity());
      replanner_mtx_.unlock();

      success = false;
      path_changed = false;
      replanning_duration = 0.0;

      if(haveToReplan(path_obstructed))
      {
        n_size_before = current_path_->getConnectionsSize();

        tic_rep=ros::WallTime::now();
        path_changed = replan();      //path may have changed even though replanning was unsuccessful
        toc_rep=ros::WallTime::now();

        replanning_duration = (toc_rep-tic_rep).toSec();
        success = replanner_->getSuccess();

        bench_mtx_.lock();
        if(success)
          replanning_time_ = replanning_duration;
        bench_mtx_.unlock();

        assert(((not path_changed) && (n_size_before == current_path_->getConnectionsSize())) || (path_changed));
      }

      if(replanning_duration>=dt_replan_/0.9 && display_timing_warning_)
        CNR_INFO(logger_,RESET()<<BOLDYELLOW()<<"Replanning duration: "<<replanning_duration<<RESET());
      if(display_replanning_success_)
        CNR_INFO(logger_,RESET()<<BOLDWHITE()<<"Success: "<< success <<" in "<< replanning_duration <<" seconds"<<RESET());

      if(path_changed && (not stop_))
      {
        trj_mtx_.lock();
        Eigen::VectorXd current_conf = current_configuration_;
        startReplannedPathFromNewCurrentConf(current_conf);
        trj_mtx_.unlock();

        trj_path = replanner_->getReplannedPath()->clone();
        trj_path->resample(solver_->getMaxDistance()/10.0); //add more nodes to ensure better tracking

        replanner_mtx_.lock();
        trj_mtx_.lock();

        tic_trj_ = ros::WallTime::now();

        if(success)
        {
          trajectory_processor_->setPath(trj_path->getWaypoints());
          trajectory_processor_->computeTrj();

          t_ = scaling_*(ros::WallTime::now()-tic_trj_).toSec(); //0.0
          t_replan_ = t_+time_shift_*scaling_;
        }

        current_path_ = replanner_->getReplannedPath();
        replanner_->setCurrentPath(current_path_);

        paths_mtx_.lock();
        updateSharedPath();
        paths_mtx_.unlock();

        past_projection = current_conf;

        trj_mtx_.unlock();
        replanner_mtx_.unlock();
      }

      toc=ros::WallTime::now();
      duration = (toc-tic).toSec();

      if(display_timing_warning_ && duration>(dt_replan_/0.9))
        CNR_INFO(logger_,RESET()<<BOLDYELLOW()<<"Replanning thread time expired: duration-> "<<duration<<" replanning time-> "<<replanning_duration<<RESET());
    }

    lp.sleep();
  }

  CNR_INFO(logger_,RESET()<<BOLDCYAN()<<"Replanning thread is over"<<RESET());
}

void ReplannerManagerBase::collisionCheckThread()
{
  moveit_msgs::GetPlanningScene ps_srv;
  Eigen::VectorXd current_configuration_copy;

  PathPtr current_path_copy = current_path_shared_->clone();
  current_path_copy->setChecker(checker_cc_);

  double duration;
  ros::WallTime tic,toc;
  ros::WallRate lp(collision_checker_thread_frequency_);

  moveit_msgs::PlanningScene planning_scene_msg;

  while ((not stop_) && ros::ok())
  {
    tic = ros::WallTime::now();

    /* Update the planning scene:
     * moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY +
     * moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS */
    ps_srv.request.components.components = 20;

    if(not plannning_scene_client_.call(ps_srv))
    {
      CNR_ERROR(logger_,"call to srv not ok");
      stop_ = true;
      break;
    }

    scene_mtx_.lock();
    planning_scene_msg.world = ps_srv.response.scene.world;
    planning_scene_msg.is_diff = true;
    checker_cc_->setPlanningSceneMsg(planning_scene_msg);
    scene_mtx_.unlock();

    trj_mtx_.lock();

    current_configuration_copy = current_configuration_;

    paths_mtx_.lock();
    if(current_path_sync_needed_)
    {
      current_path_copy = current_path_shared_->clone();
      current_path_copy->setChecker(checker_cc_);
      current_path_sync_needed_ = false;
    }
    paths_mtx_.unlock();
    trj_mtx_.unlock();

    if((current_configuration_copy-replanner_->getGoal()->getConfiguration()).norm()<goal_tol_)
    {
      stop_ = true;
      break;
    }

    size_t conn_idx;
    if(not current_path_copy->findConnection(current_configuration_copy,conn_idx))
      continue;
    else
      current_path_copy->isValidFromConf(current_configuration_copy,conn_idx,checker_cc_);

    scene_mtx_.lock();
    paths_mtx_.lock();
    if(uploadPathCost(current_path_copy)) //if path cost can be updated, update also the planning scene used to check the path
    {
      planning_scene_msg_.world = ps_srv.response.scene.world;  //not diff,it contains all pln scn info but only world is updated
      planning_scene_diff_msg_ = planning_scene_msg;            //diff, contains only world

      download_scene_info_ = true;      //dowloadPathCost can be called because the scene and path cost are referred now to the last path found
    }
    paths_mtx_.unlock();
    scene_mtx_.unlock();

    toc=ros::WallTime::now();
    duration = (toc-tic).toSec();

    if(duration>(1.0/double(collision_checker_thread_frequency_)) && display_timing_warning_)
      CNR_INFO(logger_,RESET()<<BOLDYELLOW()<<"Collision checking thread time expired: total duration-> "<<duration<<RESET());

    lp.sleep();
  }

  CNR_INFO(logger_,RESET()<<BOLDCYAN()<<"Collision check thread is over"<<RESET());
}

bool ReplannerManagerBase::replan()
{
  return replanner_->replan();
}

bool ReplannerManagerBase::joinThreads()
{
  if(trj_exec_thread_                         .joinable()) trj_exec_thread_  .join();
  if(replanning_enabled_ && replanning_thread_.joinable()) replanning_thread_.join();
  if(col_check_thread_                        .joinable()) col_check_thread_ .join();
  if(display_thread_                          .joinable()) display_thread_   .join();
  if(benchmark_          && benchmark_thread_ .joinable()) benchmark_thread_ .join();
  if(spawn_objs_         && spawn_obj_thread_ .joinable()) spawn_obj_thread_ .join();

  return true;
}

bool ReplannerManagerBase::stop()
{
  stop_ = true ;
  return joinThreads();
}

bool ReplannerManagerBase::run()
{
  ros::AsyncSpinner spinner(4);
  spinner.start();

  attributeInitialization();

  target_pub_         .publish(new_joint_state_         );
  unscaled_target_pub_.publish(new_joint_state_unscaled_);

  CNR_INFO(logger_,RESET()<<BOLDWHITE()<<"Launching threads.."<<RESET());

  display_thread_         = std::thread(&ReplannerManagerBase::displayThread            ,this);  //it must be the first one launched, otherwise the first paths will be not displayed in time
  if(spawn_objs_)
    spawn_obj_thread_     = std::thread(&ReplannerManagerBase::spawnObjectsThread       ,this);
  if(benchmark_)
    benchmark_thread_     = std::thread(&ReplannerManagerBase::benchmarkThread          ,this);
  if(replanning_enabled_)
    replanning_thread_    = std::thread(&ReplannerManagerBase::replanningThread         ,this);
  col_check_thread_       = std::thread(&ReplannerManagerBase::collisionCheckThread     ,this);
  ros::Duration(0.1).sleep();
  trj_exec_thread_        = std::thread(&ReplannerManagerBase::trajectoryExecutionThread,this);

  return true;
}

bool ReplannerManagerBase::start()
{
  run();
  joinThreads();

  return true;
}

double ReplannerManagerBase::readScalingTopics()
{
  ovr_mtx_.lock();
  double ovr = global_override_;
  ovr_mtx_.unlock();

  return ovr;
}

void ReplannerManagerBase::trajectoryExecutionThread()
{
  double  duration;
  ros::WallTime tic,toc;
  PathPtr path2project_on;
  Eigen::VectorXd point2project(pnt_->state_->pos_.size());
  Eigen::VectorXd goal_conf = replanner_->getGoal()->getConfiguration();

  ros::WallRate lp(trj_exec_thread_frequency_);

  while((not stop_) && ros::ok())
  {
    tic = ros::WallTime::now();

    trj_mtx_.lock();

    scaling_ = scaling_from_param_;

    if(read_safe_scaling_)
      scaling_ = scaling_*readScalingTopics();

    real_time_ += dt_;
    t_+= scaling_*dt_;
    t_replan_ = t_+time_shift_*scaling_;

    trajectory_processor_->interpolate(t_,pnt_         ,scaling_           );
    trajectory_processor_->interpolate(t_,pnt_unscaled_,scaling_from_param_);

    for(size_t i=0; i<pnt_->state_->pos_.size();i++)
      point2project[i] = pnt_->state_->pos_[i];

    paths_mtx_.lock();
    path2project_on = current_path_shared_->clone();
    paths_mtx_.unlock();

    if(path2project_on->findConnection(current_configuration_))
      current_configuration_ = path2project_on->projectOnPath(point2project,current_configuration_);
    else
      current_configuration_ = path2project_on->projectOnPath(point2project);

    trj_mtx_.unlock();

    if((point2project-goal_conf).norm()<goal_tol_)
    {
      stop_ = true;
      goal_reached_ = true;
    }

    new_joint_state_.position              = pnt_->state_->pos_;
    new_joint_state_.velocity              = pnt_->state_->vel_;
    new_joint_state_.header.stamp          = ros::Time::now();

    new_joint_state_unscaled_.position     = pnt_unscaled_->state_->pos_;
    new_joint_state_unscaled_.velocity     = pnt_unscaled_->state_->vel_;
    new_joint_state_unscaled_.header.stamp = ros::Time::now();

    target_pub_         .publish(new_joint_state_)         ;
    unscaled_target_pub_.publish(new_joint_state_unscaled_);

    toc = ros::WallTime::now();
    duration = (toc-tic).toSec();
    if(duration>(1.0/double(trj_exec_thread_frequency_)) && display_timing_warning_)
      CNR_INFO(logger_,RESET()<<BOLDYELLOW()<<"Trj execution thread time expired: duration-> "<<duration<<RESET());

    lp.sleep();
  }

  stop_ = true;

  for(size_t i=0; i<pnt_->state_->pos_.size();i++)
    point2project(i) = pnt_->state_->pos_[i];

  if(goal_reached_ && (point2project-goal_conf).norm()>goal_tol_)
    throw std::runtime_error("goal toll not respected! goal toll "+std::to_string(goal_tol_)+" dist "+std::to_string((point2project-goal_conf).norm()));

  CNR_INFO(logger_,RESET()<<BOLDCYAN()<<"Trajectory execution thread is over"<<RESET());
}

void ReplannerManagerBase::displayThread()
{
  PathPtr initial_path = current_path_shared_->clone();
  planning_scene::PlanningScenePtr planning_scene = planning_scene::PlanningScene::clone(planning_scn_cc_);

  DisplayPtr disp = std::make_shared<Display>(planning_scene,group_name_,which_link_display_path_);

  PathPtr current_path;
  TrjPoint pnt, pnt_replan;
  Eigen::VectorXd current_configuration, configuration_replan;

  Eigen::VectorXd point2project(pnt_->state_->pos_.size());

  int path_id,node_id,wp_id;
  std::vector<double> marker_scale(3,0.01);
  std::vector<double> marker_scale_sphere(3,0.02);
  std::vector<double> marker_color_initial_path   = {0.0,1.0,0.0,1.0};
  std::vector<double> marker_color_current_path   = {1.0,1.0,0.0,1.0};
  std::vector<double> marker_color_current_config = {1.0,0.0,1.0,1.0};
  std::vector<double> marker_color_current_pnt    = {0.0,1.0,0.0,1.0};
  std::vector<double> marker_color_replan_config  = {0.0,0.0,0.0,1.0};
  std::vector<double> marker_color_replan_pnt     = {0.5,0.5,0.5,1.0};

  disp->clearMarkers();

  unsigned int display_thread_frequency = 2*trj_exec_thread_frequency_;
  ros::WallRate lp(display_thread_frequency);

  while((not stop_) && ros::ok())
  {
    paths_mtx_.lock();
    current_path = current_path_shared_->clone();
    paths_mtx_.unlock();

    replanner_mtx_.lock();
    trj_mtx_.lock();
    *pnt.state_ = *(pnt_->state_);
    pnt.time_from_start_ = pnt_->time_from_start_;

    *pnt_replan.state_ = *(pnt_replan_->state_);
    pnt_replan.time_from_start_ = pnt_replan_->time_from_start_;

    configuration_replan  = configuration_replan_ ;
    current_configuration = current_configuration_;
    trj_mtx_.unlock();
    replanner_mtx_.unlock();

    path_id = 10;
    node_id = 1000;
    wp_id = 10000;

    disp->changeConnectionSize(marker_scale);
    disp->displayPathAndWaypoints(current_path,path_id,wp_id,"graph_display",marker_color_current_path);

    disp->displayPathAndWaypoints(initial_path,path_id+2000,wp_id+2000,"graph_display",marker_color_initial_path);
    disp->defaultConnectionSize();

    disp->changeNodeSize(marker_scale_sphere);

    if(display_current_config_)
      disp->displayNode(std::make_shared<Node>(current_configuration),node_id,"graph_display",marker_color_current_config);

    if(display_current_trj_point_)
    {
      for(size_t i=0; i<pnt.state_->pos_.size();i++)
        point2project[i] = pnt.state_->pos_[i];

      node_id +=1;
      disp->displayNode(std::make_shared<Node>(point2project),node_id,"graph_display",marker_color_current_pnt);
    }

    if(display_replan_config_)
    {
      node_id +=1;
      disp->displayNode(std::make_shared<Node>(configuration_replan),node_id,"graph_display",marker_color_replan_config);
    }

    if(display_replan_trj_point_)
    {
      for(size_t i=0; i<pnt_replan.state_->pos_.size();i++)
        point2project[i] = pnt_replan.state_->pos_[i];

      node_id +=1;
      disp->displayNode(std::make_shared<Node>(point2project),node_id,"graph_display",marker_color_replan_pnt);
    }

    disp->defaultNodeSize();

    lp.sleep();
  }

  disp->clearMarkers();
  CNR_INFO(logger_,RESET()<<BOLDCYAN()<<"Display thread is over"<<RESET());
}

void ReplannerManagerBase::spawnObjectsThread()
{
  cnr_scene_manager_msgs::AddObjects srv_add_object;
  cnr_scene_manager_msgs::MoveObjects srv_move_objects;
  cnr_scene_manager_msgs::RemoveObjects srv_remove_object;

  CollisionCheckerPtr checker = checker_cc_->clone();
  planning_scene::PlanningScenePtr planning_scene = planning_scene::PlanningScene::clone(planning_scn_cc_);
  std::string last_link = planning_scene->getRobotModel()->getJointModelGroup(group_name_)->getLinkModelNames().back();

  PathPtr current_path;
  Eigen::VectorXd obj_conf, replan_conf;
  Eigen::VectorXd goal_conf = current_path_shared_->getGoalNode()->getConfiguration();

  Eigen::Vector3d replan_pose, obj_pose;
  Eigen::Vector3d goal_pose = forwardIk(goal_conf,last_link,planning_scene);

  std::vector<std::string> ids;
  std::vector<double> moving_time;
  std::vector<unsigned int> n_move;
  std::vector<Eigen::Vector3d> velocities;
  std::vector<Eigen::Vector3d> objects_locations;
  std::vector<cnr_scene_manager_msgs::Object> spawned_objects;

  bool obs_update;
  geometry_msgs::Quaternion q;
  q.x = 0.0; q.y = 0.0; q.z = 0.0; q.w = 1.0;

  std::random_device rseed;
  std::mt19937 gen(rseed());
  std::uniform_real_distribution<double> random_vel(-1.0,1.0);
  std::uniform_real_distribution<double> random_abs(0.2,0.8);

  cnr_scene_manager_msgs::Object new_obj;
  new_obj.object_type = obj_type_;
  new_obj.pose.header.frame_id = "world";
  new_obj.pose.pose.orientation = q;

  std::reverse(spawn_instants_.begin(),spawn_instants_.end());
  ros::WallRate lp(100);

  while(not stop_ && ros::ok())
  {
    obs_update = false;

    srv_add_object.request.objects.clear();
    srv_move_objects.request.poses.clear();
    srv_move_objects.request.obj_ids.clear();

    if(not spawn_instants_.empty())
    {
      if(real_time_>=spawn_instants_.back())
      {
        spawn_instants_.pop_back();

        replanner_mtx_.lock();
        paths_mtx_.lock();

        current_path = current_path_shared_->clone();
        replan_conf = configuration_replan_;

        paths_mtx_.unlock();
        replanner_mtx_.unlock();

        current_path->setChecker(checker);
        current_path = current_path->getSubpathFromConf(replan_conf,true);

        replan_pose = forwardIk(replan_conf,last_link,planning_scene);

        double obj_abscissa = 0.0;
        while(not stop_ && ros::ok())
        {
          obj_abscissa = random_abs(gen); //0.2~0.8

          obj_conf = current_path->pointOnCurvilinearAbscissa(obj_abscissa);
          obj_pose = forwardIk(obj_conf,last_link,planning_scene);

          // to no collide with the robot or the goal
          if((obj_pose-replan_pose).norm()>obj_max_size_ && (obj_conf-replan_conf).norm()>obj_max_size_ &&
             (obj_pose-goal_pose  ).norm()>obj_max_size_ && (obj_conf-goal_conf  ).norm()>obj_max_size_  )
            break;
        }

        new_obj.pose.pose.position.x = obj_pose[0];
        new_obj.pose.pose.position.y = obj_pose[1];
        new_obj.pose.pose.position.z = obj_pose[2];

        srv_add_object.request.objects.push_back(new_obj);

        if(stop_ || not ros::ok())
          break;
      }
    }

    if(not objects_locations.empty() && obj_vel_>0.0)
    {
      for(size_t i=0;i<objects_locations.size();i++)
      {
        if(real_time_>(moving_time.at(i)+dt_move_))
        {
          if(n_move.at(i)>direction_change_) //change direction of motion
          {
            Eigen::Vector3d v;
            v << (random_vel(gen)),(random_vel(gen)),(random_vel(gen));
            v = (v/v.norm())*obj_vel_;

            velocities.at(i) = v;
            n_move.at(i) = 0;
          }

          objects_locations.at(i) = objects_locations.at(i) + dt_move_*velocities.at(i);
          spawned_objects.at(i).pose.pose.position.x = objects_locations.at(i)[0];
          spawned_objects.at(i).pose.pose.position.y = objects_locations.at(i)[1];
          spawned_objects.at(i).pose.pose.position.z = objects_locations.at(i)[2];
          spawned_objects.at(i).pose.pose.orientation = q;

          n_move.at(i) = n_move.at(i)+1;
          moving_time.at(i) = real_time_;

          srv_move_objects.request.obj_ids.push_back(ids.at(i));
          srv_move_objects.request.poses.push_back(spawned_objects.at(i).pose.pose);
        }

        if(stop_ || not ros::ok())
          break;
      }

      if(stop_ || not ros::ok())
        break;
    }

    if(not srv_add_object.request.objects.empty())
    {
      if(not add_obj_.call(srv_add_object))
      {
        CNR_ERROR(logger_,"call to add obj srv not ok");

        stop_ = true;
        break;
      }

      if(not srv_add_object.response.success)
        CNR_ERROR(logger_,"add obj srv error");
      else
      {
        CNR_INFO(logger_,RESET()<<BOLDMAGENTA()<<"Obstacle spawned!"<<RESET());
        obs_update = true;

        Eigen::Vector3d v;
        v << (random_vel(gen)),(random_vel(gen)),(random_vel(gen));
        v = (v/v.norm())*obj_vel_;

        n_move.push_back(0);
        velocities.push_back(v);
        moving_time.push_back(real_time_);
        spawned_objects.push_back(new_obj);
        objects_locations.push_back(obj_pose);
        ids.push_back(srv_add_object.response.ids.front());

        for (const std::string& str:srv_add_object.response.ids)
          srv_remove_object.request.obj_ids.push_back(str);
      }
    }

    if(not srv_move_objects.request.poses.empty())
    {
      if(not move_obj_.call(srv_move_objects))
        CNR_ERROR(logger_,"call to move obj srv not ok");

      if(not srv_move_objects.response.success)
        CNR_ERROR(logger_,"move obj srv error");
      else
        obs_update = true;
    }

    if(obs_update)
    {
      geometry_msgs::PoseArray pose_array;
      pose_array.header.frame_id = "world";
      pose_array.header.stamp = ros::Time::now();

      geometry_msgs::Pose pose;
      pose.orientation = q;

      bench_mtx_.lock();
      obj_ids_ = ids;  //also contains the new added obj

      obj_pos_.clear();
      for(const Eigen::Vector3d& ol: objects_locations) //also contains the new added obj
      {
        Eigen::VectorXd vector = ol.head<3>();
        obj_pos_.push_back(vector);

        pose.position.x = ol[0];
        pose.position.y = ol[1];
        pose.position.z = ol[2];

        pose_array.poses.push_back(pose);
      }
      bench_mtx_.unlock();

      obj_pose_pub_.publish(pose_array); //publish poses for SSM node
    }
    lp.sleep();
  }

  if (not remove_obj_.call(srv_remove_object))
    CNR_ERROR(logger_,"call to remove obj srv not ok");
  if(not srv_remove_object.response.success)
    CNR_ERROR(logger_,"remove obj srv error");

  CNR_INFO(logger_,RESET()<<BOLDCYAN()<<"Spawn objects thread is over"<<RESET());
}

void ReplannerManagerBase::benchmarkThread()
{
  bool success = true;
  double path_length = 0.0;
  TrjPoint pnt;
  PathPtr current_path;
  ConnectionPtr current_conn;
  std::vector<std::string> obj_ids;
  std::vector<Eigen::VectorXd> obj_pos;
  std::vector<std::string>::iterator it;
  std::vector<double> replanning_time_vector;
  std::vector<std::string> already_collided_obj;
  Eigen::VectorXd old_current_configuration, current_configuration, current_configuration_3d, old_pnt_conf, pnt_conf;

  planning_scene::PlanningScenePtr planning_scene = planning_scene::PlanningScene::clone(planning_scn_cc_);

  std::string last_link = planning_scene->getRobotModel()->getJointModelGroup(group_name_)->getLinkModelNames().back();
  MoveitCollisionCheckerPtr checker = std::make_shared<MoveitCollisionChecker>(planning_scene,group_name_,logger_);

  paths_mtx_.lock();
  Eigen::VectorXd start = current_path_shared_->getStartNode()->getConfiguration();
  Eigen::VectorXd goal  = current_path_shared_->getGoalNode ()->getConfiguration();
  double initial_path_length = current_path_shared_->computeEuclideanNorm();
  paths_mtx_.unlock();

  Eigen::VectorXd goal_3d = forwardIk(goal,last_link,planning_scene);

  pnt_conf = start;
  current_configuration = start;

  double distance;
  double distance_start_goal = (goal-start).norm();

  int n_collisions = 0;

  std::string text;

  std_msgs::ColorRGBA fg_color_green, fg_color_red, bg_color;
  fg_color_green.r = 0;   fg_color_red.r = 1;   bg_color.r = 0;
  fg_color_green.g = 0.8; fg_color_red.g = 0;   bg_color.g = 0;
  fg_color_green.b = 0.2; fg_color_red.b = 0;   bg_color.b = 0;
  fg_color_green.a = 0.8; fg_color_red.a = 0.8; bg_color.a = 0;

  jsk_rviz_plugins::OverlayText overlayed_text;
  overlayed_text.font = "FreeSans";
  overlayed_text.bg_color = bg_color;
  overlayed_text.height = 70;
  overlayed_text.left = 10;
  overlayed_text.top = 80;
  overlayed_text.line_width = 2;
  overlayed_text.text_size = 15;
  overlayed_text.width = 1000;

  overlayed_text.action = overlayed_text.DELETE;
  text_overlay_pub_.publish(overlayed_text);

  overlayed_text.action = overlayed_text.ADD;

  double cycle_duration;
  ros::WallTime tic, toc;
  unsigned int freq = 2*trj_exec_thread_frequency_;
  ros::WallRate lp(freq);

  while((not stop_) && ros::ok())
  {
    tic = ros::WallTime::now();

    if(success)
    {
      text = "Success: TRUE \nCollided objects: 0";
      overlayed_text.text = text;
      overlayed_text.fg_color = fg_color_green;
      text_overlay_pub_.publish(overlayed_text);
    }

    old_pnt_conf = pnt_conf;
    old_current_configuration = current_configuration;

    trj_mtx_.lock();
    paths_mtx_.lock();
    *pnt.state_ = *(pnt_->state_);
    current_path = current_path_shared_->clone();
    current_configuration = current_configuration_;
    paths_mtx_.unlock();
    trj_mtx_.unlock();

    current_configuration_3d = forwardIk(current_configuration,last_link,planning_scene);

    for(size_t i=0; i<pnt.state_->pos_.size();i++)
      pnt_conf(i) = pnt.state_->pos_[i];

    /* Replanning time */
    bench_mtx_.lock();
    if(replanning_time_ != 0.0)
      replanning_time_vector.push_back(replanning_time_);

    replanning_time_ = 0.0;
    bench_mtx_.unlock();

    /* Path length */
    distance = (pnt_conf-old_pnt_conf).norm();
    if(distance>0.3)
    {
      //current_configuration = old_current_configuration;
      CNR_INFO(logger_,RESET()<<BOLDRED()<<"Skipping path length increment! Distance: "<<distance<<RESET());
    }
    else
      path_length += distance;

    /* Collisions with mobile obstacles */
    bench_mtx_.lock();
    obj_ids = obj_ids_;
    obj_pos = obj_pos_;
    bench_mtx_.unlock();

    for(size_t i=0;i<obj_pos.size();i++)
    {
      if((current_configuration_3d-obj_pos[i]).norm()<obj_max_size_ && ((goal_3d-obj_pos[i]).norm()>obj_max_size_))
      {
        it = std::find(already_collided_obj.begin(),already_collided_obj.end(),obj_ids[i]);
        if(it>=already_collided_obj.end())
        {
          scene_mtx_.lock();
          checker->setPlanningSceneMsg(planning_scene_msg_benchmark_);
          scene_mtx_.unlock();

          if(not checker->check(current_configuration)) //Did replanner know about this obstacle? If check(current_configuration) is false, replanner knew the obstacle
          {
            current_conn = current_path->findConnection(current_configuration);

            if(current_conn && (current_conn->getCost() != std::numeric_limits<double>::infinity()))
              throw std::runtime_error("current conn cost should be infinite! ");

            n_collisions++;
            already_collided_obj.push_back(obj_ids[i]);
            success = false;

            text = "Success: FALSE \nCollided objects: "+std::to_string(n_collisions);
            overlayed_text.text = text;
            overlayed_text.fg_color = fg_color_red;
            text_overlay_pub_.publish(overlayed_text);

            break;
          }
        }
      }
    }

    toc = ros::WallTime::now();
    cycle_duration = (toc-tic).toSec();
    if(cycle_duration>(1/freq) && display_timing_warning_)
      CNR_INFO(logger_,RESET()<<BOLDYELLOW()<<"Benchmark thread time expired: duration-> "<<cycle_duration<<RESET());

    lp.sleep();
  }

  double sum, mean, std_dev, variance, max_replanning_time;
  sum = std::accumulate(replanning_time_vector.begin(),replanning_time_vector.end(),0.0);
  mean = sum/replanning_time_vector.size();

  variance = 0.0;
  for(const double& d:replanning_time_vector)
    variance += std::pow(d - mean, 2);

  std_dev = std::sqrt(variance / replanning_time_vector.size());

  if(not replanning_time_vector.empty())
    max_replanning_time = *(std::max_element(replanning_time_vector.begin(),replanning_time_vector.end()));
  else
    max_replanning_time = 0.0;

  bench_mtx_.lock();
  size_t number_of_objects = obj_ids_.size();
  bench_mtx_.unlock();

  std::string replanner_type;
  get_param(logger_,param_ns_,"replanner_type",replanner_type,std::string("replanner"));

  std::string test_name;
  get_param(logger_,param_ns_,"test_name",test_name,std::string("test"));

  std::string bench_name;
  get_param(logger_,param_ns_,"bench_name",bench_name,std::string("bench"));

  std::string path = "./openmore_replanners_benchmark";
  std::string file_name = path+"/"+bench_name+"/"+replanner_type+"/"+test_name+".bin";

  boost::filesystem::path dir(path);
  if(not (boost::filesystem::exists(dir)))
    boost::filesystem::create_directory(dir);

  boost::filesystem::path dir2(path+"/"+bench_name);
  if(not (boost::filesystem::exists(dir2)))
    boost::filesystem::create_directory(dir2);

  boost::filesystem::path dir3(path+"/"+bench_name+"/"+replanner_type);
  if(not (boost::filesystem::exists(dir3)))
    boost::filesystem::create_directory(dir3);

  std::ofstream file;
  file.open(file_name,std::ios::out | std::ios::binary);

  const size_t bufsize = 1024 * 1024;
  std::unique_ptr<char[]> buf;
  buf.reset(new char[bufsize]);

  file.rdbuf()->pubsetbuf(buf.get(), bufsize);

  file.write((char*) &success,             sizeof(success            ));
  file.write((char*) &number_of_objects,   sizeof(number_of_objects  ));
  file.write((char*) &n_collisions,        sizeof(n_collisions       ));
  file.write((char*) &path_length,         sizeof(path_length        ));
  file.write((char*) &distance_start_goal, sizeof(distance_start_goal));
  file.write((char*) &initial_path_length, sizeof(initial_path_length));
  file.write((char*) &real_time_,          sizeof(real_time_         ));
  file.write((char*) &mean,                sizeof(mean               ));
  file.write((char*) &std_dev,             sizeof(std_dev            ));
  file.write((char*) &max_replanning_time, sizeof(max_replanning_time));

  file.flush();
  file.close();

  CNR_INFO(logger_,RESET()<<BOLDBLUE()<<
           "\nFile "<<file_name<<" saved!\n* success: "<<success
           <<"\n* number_of_objects: "<<number_of_objects
           <<"\n* number_of_collisions: "<<n_collisions
           <<"\n* path length: "<<path_length
           <<"\n* distance start-goal: "<<distance_start_goal
           <<"\n* initial path length: "<<initial_path_length
           <<"\n* time: "<<real_time_
           <<"\n* replanning time mean: "<<mean
           <<"\n* replanning time std dev: "<<std_dev
           <<"\n* max replanning time: "<<max_replanning_time
           <<RESET());

  if(n_collisions == 0 && not success)
    throw std::runtime_error("no collisions but success false!");

  CNR_INFO(logger_,RESET()<<BOLDCYAN()<<"Benchamrk thread is over"<<RESET());
}

Eigen::Vector3d ReplannerManagerBase::forwardIk(const Eigen::VectorXd& conf, const std::string& last_link, const planning_scene::PlanningScenePtr& planning_scene)
{
  geometry_msgs::Pose pose;
  return forwardIk(conf,last_link,planning_scene,pose);
}


Eigen::Vector3d ReplannerManagerBase::forwardIk(const Eigen::VectorXd& conf, const std::string& last_link, const planning_scene::PlanningScenePtr& planning_scene, geometry_msgs::Pose& pose)
{
  moveit::core::RobotState state = fromWaypoint2State(conf,planning_scene,group_name_);
  tf::poseEigenToMsg(state.getGlobalLinkTransform(last_link),pose);

  std::vector<double> v(3);
  v[0] = pose.position.x;
  v[1] = pose.position.y;
  v[2] = pose.position.z;

  Eigen::Vector3d position(v.size());
  for(size_t i=0;i<v.size();i++)
    position[i] = v[i];

  return position;
}

void ReplannerManagerBase::displayTrj(const DisplayPtr& disp)
{
  TrajectoryPtr interpolator = trajectory_processor_;

  double t=0;
  double t_max = interpolator->getTrjDuration();

  NodePtr node;
  TrjPointPtr pnt;
  std::vector<NodePtr> nodes;

  while(t<t_max)
  {
    interpolator->interpolate(t,pnt,scaling_);

    Eigen::VectorXd conf(pnt->state_->pos_.size());
    for(size_t i=0; i<pnt->state_->pos_.size();i++)
      conf(i) = pnt->state_->pos_[i];

    node = std::make_shared<Node>(conf,logger_);
    nodes.push_back(node);

    t+=0.001;
  }

  PathPtr path = std::make_shared<Path>(nodes,current_path_shared_->getMetrics(),current_path_shared_->getChecker(),logger_);
  disp->displayPath(path,"graph_display",{1,0,0,1});
}
}
