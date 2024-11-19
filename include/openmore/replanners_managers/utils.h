#pragma once

#include <mutex>
#include <thread>
#include <random>

#include <std_msgs/Int64.h>
#include <std_msgs/ColorRGBA.h>
#include <jsk_rviz_plugins/OverlayText.h>

#include <cnr_scene_manager_msgs/AddObjects.h>
#include <cnr_scene_manager_msgs/MoveObjects.h>
#include <cnr_scene_manager_msgs/RemoveObjects.h>

#include <openmore/replanners/replanner_base.h>
#include <subscription_notifier/subscription_notifier.h> //Sostituire?
#include <openmore/trajectories_processors/trajectory_processor_base.h>
#include <moveit_collision_checker/collision_checkers/parallel_moveit_collision_checker.h>

namespace openmore
{
using namespace cnr_logger;
using namespace graph::core;
using namespace graph::display;
using namespace graph::collision_check;
using namespace trajectories_processors;

using TrajectoryPtr = TrajectoryProcessorBasePtr;

/**
 * @brief fromWaypoint2State transforms a Eigen::VectorXd waypoint into a moveit::core::RobotState at the provided configuration.
 * @param waypoint the Eigen::VectorXd representing a joints configuration.
 * @param planning_scene the moveit planning scene.
 * @param group_name the name of the robot group to be considered.
 * @return the corresponding moveit::core::RobotState
 */
inline moveit::core::RobotState fromWaypoint2State(const Eigen::VectorXd& waypoint,
                                                   const planning_scene::PlanningScenePtr& planning_scene,
                                                   const std::string& group_name)
{
  moveit::core::RobotState state=planning_scene->getCurrentState();
  state.setJointGroupPositions(group_name,waypoint);
  state.update();

  return state;
}

/**
 * @brief fromWaypoints2State transforms a vector of Eigen::VectorXd waypoints into a vector of moveit::core::RobotState at the provided configurations.
 * @param waypoints the vector of Eigen::VectorXd representing joints configurations.
 * @param planning_scene the moveit planning scene.
 * @param group_name the name of the robot group to be considered.
 * @param states the corresponding moveit::core::RobotState
 * @return true if successfull
 */
inline bool fromWaypoints2States(const std::vector<Eigen::VectorXd>& waypoints,
                                 const planning_scene::PlanningScenePtr& planning_scene,
                                 const std::string& group_name,
                                 std::vector<moveit::core::RobotState>& states)
{
  states.clear();
  states.reserve(waypoints.size());

  for(const auto& waypoint: waypoints)
  {
    auto state = fromWaypoint2State(waypoint,planning_scene,group_name);
    states.push_back(state);
  }

  return true;
}
}


