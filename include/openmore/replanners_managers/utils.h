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

#include <mutex>
#include <thread>
#include <random>
#include <shared_mutex>

#include <std_msgs/Int64.h>
#include <std_msgs/ColorRGBA.h>
#include <jsk_rviz_plugins/OverlayText.h>

#include <cnr_scene_manager_msgs/AddObjects.h>
#include <cnr_scene_manager_msgs/MoveObjects.h>
#include <cnr_scene_manager_msgs/RemoveObjects.h>

#include <openmore/replanners/replanner_base.h>
#include <subscription_notifier/subscription_notifier.h>
#include <openmore/trajectories_processors/trajectory_processor_base.h>
#include <moveit_collision_checker/collision_checkers/parallel_moveit_collision_checker.h>

namespace openmore
{
using namespace cnr_logger;
using namespace graph::core;
using namespace graph::display;
using namespace graph::collision_check;
using TrajectoryPtr = TrajectoryProcessorBasePtr;

/**
 * @brief fromWaypoint2State transforms a Eigen::VectorXd waypoint into a moveit::core::RobotState at the provided configuration.
 * @param waypoint the Eigen::VectorXd representing a joints configuration.
 * @param planning_scene the moveit planning scene.
 * @param group_name the name of the robot group to be considered.
 * @return the corresponding moveit::core::RobotState
 */
inline moveit::core::RobotState fromWaypoint2State(const Eigen::VectorXd& waypoint, const planning_scene::PlanningScenePtr& planning_scene,
                                                   const std::string& group_name)
{
  moveit::core::RobotState state = planning_scene->getCurrentState();
  state.setJointGroupPositions(group_name, waypoint);
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
inline bool fromWaypoints2States(const std::vector<Eigen::VectorXd>& waypoints, const planning_scene::PlanningScenePtr& planning_scene,
                                 const std::string& group_name, std::vector<moveit::core::RobotState>& states)
{
  states.clear();
  states.reserve(waypoints.size());

  for (const auto& waypoint : waypoints)
  {
    auto state = fromWaypoint2State(waypoint, planning_scene, group_name);
    states.push_back(state);
  }

  return true;
}
}  // namespace openmore
