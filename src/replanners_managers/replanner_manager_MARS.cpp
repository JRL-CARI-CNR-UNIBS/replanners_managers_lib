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

#include "openmore/replanners_managers/replanner_manager_MARS.h"

namespace openmore
{
ReplannerManagerMARS::ReplannerManagerMARS(const PathPtr& current_path, const TrajectoryPtr& trajectory_processor, const TreeSolverPtr& solver,
                                           const std::string& param_ns, const TraceLoggerPtr& logger)
  : ReplannerManagerBase(current_path, trajectory_processor, solver, param_ns, logger)
{
  ReplannerManagerMARS::MARSadditionalParams();
}

ReplannerManagerMARS::ReplannerManagerMARS(const PathPtr& current_path, const TrajectoryPtr& trajectory_processor, const TreeSolverPtr& solver,
                                           const std::string& param_ns, const TraceLoggerPtr& logger, const std::vector<PathPtr>& other_paths)
  : ReplannerManagerMARS(current_path, trajectory_processor, solver, param_ns, logger)
{
  other_paths_ = other_paths;
  if (replanner_)
  {
    MARSPtr MARS_replanner = std::static_pointer_cast<MARS>(replanner_);
    MARS_replanner->setOtherPaths(other_paths);
  }
}

void ReplannerManagerMARS::MARSadditionalParams()
{
  get_param(logger_, param_ns_, "MARS/reverse_start_nodes", reverse_start_nodes_, false);
  get_param(logger_, param_ns_, "MARS/full_net_search", full_net_search_, true);
  get_param(logger_, param_ns_, "MARS/verbosity_level", verbosity_level_, 0);
  get_param(logger_, param_ns_, "MARS/display_other_paths", display_other_paths_, true);
}

void ReplannerManagerMARS::attributeInitialization()
{
  ReplannerManagerBase::attributeInitialization();

  MARSPtr replanner = std::static_pointer_cast<MARS>(replanner_);

  if (replanner_verbosity_)
  {
    if (verbosity_level_ > 2)
      verbosity_level_ = 2;
    else if (verbosity_level_ < 0)
      verbosity_level_ = 0;

    switch (verbosity_level_)
    {
      case 0:
        replanner->setInformedOnlineReplanningVerbose(false);
        replanner->setPathSwitchVerbose(false);
        break;
      case 1:
        replanner->setInformedOnlineReplanningVerbose(true);
        replanner->setPathSwitchVerbose(false);
        break;
      case 2:
        replanner->setInformedOnlineReplanningVerbose(true);
        replanner->setPathSwitchVerbose(true);
        break;
    }
  }

  first_replanning_ = true;
  old_current_node_ = nullptr;

  initial_path_ = current_path_;

  other_paths_shared_.clear();
  other_paths_sync_needed_.clear();
  for (const PathPtr& p : other_paths_)
  {
    PathPtr other_path = p->clone();
    other_paths_shared_.push_back(other_path);

    other_path->setChecker(checker_cc_);
    p->setChecker(checker_replanning_);

    other_paths_sync_needed_.push_back(false);
  }
}

bool ReplannerManagerMARS::replan()
{
  bool path_changed = replanner_->replan();

  // CHANGE WITH PATH_CHANGED?
  if (replanner_->getSuccess() && first_replanning_)  // add the initial path to the other paths
  {
    first_replanning_ = false;

    other_paths_mtx_.lock();

    PathPtr another_path = initial_path_->clone();
    another_path->setChecker(checker_cc_);

    other_paths_shared_.push_back(another_path);
    other_paths_sync_needed_.push_back(false);

    MARSPtr replanner = std::static_pointer_cast<MARS>(replanner_);
    replanner->addOtherPath(initial_path_, false);  // replanner->getCurrentPath() can be slightly different (some new nodes)

    assert(another_path->getConnectionsSize() == initial_path_->getConnectionsSize());

    other_paths_.push_back(initial_path_);  // not move from here

    other_paths_mtx_.unlock();
  }

  return path_changed;
}

void ReplannerManagerMARS::startReplannedPathFromNewCurrentConf(const Eigen::VectorXd& configuration)
{
  MARSPtr replanner = std::static_pointer_cast<MARS>(replanner_);

  PathPtr current_path = replanner->getCurrentPath();
  PathPtr replanned_path = replanner->getReplannedPath();
  NodePtr node_replan = replanned_path->getStartNode();

  TreePtr tree = current_path->getTree();

  assert(current_path->findConnection(configuration) != nullptr);

  if (old_current_node_ && ((old_current_node_->getConfiguration() - configuration).norm() > TOLERANCE) && old_current_node_ != node_replan &&
      tree->isInTree(old_current_node_))
  {
    if ((old_current_node_->getParentConnectionsSize() + old_current_node_->getNetParentConnectionsSize()) == 1)
    {
      if ((old_current_node_->getChildConnectionsSize() + old_current_node_->getNetChildConnectionsSize()) == 1)
      {
        /* Remove the old node detaching it from the tree, restore the old connection and set it as initial
         * connection of current path to be able to insert the new current node */

        ConnectionPtr parent_conn;
        (old_current_node_->getParentConnectionsSize() > 0) ? (parent_conn = old_current_node_->parentConnection(0)) :
                                                              (parent_conn = old_current_node_->netParentConnection(0));
        assert([&]() -> bool {
          if (tree->isInTree(parent_conn->getParent()))
            return true;
          else
          {
            for (const ConnectionPtr c : old_current_node_->getParentConnections())
              CNR_INFO(logger_, "parent conn " << *c);

            for (const ConnectionPtr c : old_current_node_->getNetParentConnections())
              CNR_INFO(logger_, "net parent conn " << *c);

            CNR_INFO(logger_, "current path " << *current_path);
            CNR_INFO(logger_, "old current node " << *old_current_node_ << old_current_node_);

            return false;
          }
        }());

        std::vector<ConnectionPtr> new_conns = current_path->getConnections();
        new_conns.insert(new_conns.begin(), parent_conn);
        current_path->setConnections(new_conns);

        ConnectionPtr restored_conn;
        if (current_path->removeNode(old_current_node_, {}, restored_conn))
        {
          std::vector<PathPtr> paths = other_paths_;
          paths.push_back(replanned_path);
          for (PathPtr& p : paths)
          {
            p->restoreConnection(restored_conn, old_current_node_);

            assert([&]() -> bool {
              for (const NodePtr& n : p->getNodes())
                if (n == old_current_node_)
                {
                  return false;
                }

              return true;
            }());

            assert(not tree->isInTree(old_current_node_));
          }

          CNR_DEBUG(logger_, RESET() << BC() << "OLD CURRENT NODE REMOVED");
        }
      }
    }
  }

  size_t conn_idx;
  bool is_a_new_node;
  PathPtr tmp_p = current_path->clone();
  ConnectionPtr conn = current_path->findConnection(configuration, conn_idx);
  NodePtr current_node = current_path->addNodeAtCurrentConfig(configuration, conn, true, is_a_new_node);

  assert([&]() -> bool {
    if ((current_node == node_replan && ((configuration - node_replan->getConfiguration()).norm() > TOLERANCE)) ||
        (current_node != node_replan && ((configuration - node_replan->getConfiguration()).norm() <= TOLERANCE)))
    {
      CNR_INFO(logger_, "current node: " << current_node << " " << *current_node);
      CNR_INFO(logger_, "is a new node: " << is_a_new_node);
      CNR_INFO(logger_, "replan node: " << node_replan << " " << *node_replan);
      CNR_INFO(logger_, "conn: " << *conn);
      CNR_INFO(logger_, "conf: " << configuration.transpose());
      CNR_INFO(logger_, "TOLERANCE: " << TOLERANCE << " norm: " << (configuration - node_replan->getConfiguration()).norm());
      CNR_INFO(logger_, "curr p:" << *current_path);
      CNR_INFO(logger_, "tmp p: " << *tmp_p);

      tmp_p->findConnection(configuration, conn_idx, true);

      return false;
    }
    return true;
  }());

  if (is_a_new_node)
  {
    old_current_node_ = current_node;
    for (PathPtr& p : other_paths_)
    {
      p->splitConnection(current_path->getConnectionsConst().at(conn_idx), current_path->getConnectionsConst().at(conn_idx + 1), conn);
    }
  }
  else
    old_current_node_ = nullptr;

  if (not replanner->getSuccess())
  {
    replanned_path->setConnections(current_path->getSubpathFromNode(current_node)->getConnections());
  }
  else
  {
    std::vector<NodePtr> nodes = current_path->getNodes();

    std::vector<NodePtr>::iterator it_current_node = std::find(nodes.begin(), nodes.end(), current_node);
    std::vector<NodePtr>::iterator it_node_replan = std::find(nodes.begin(), nodes.end(), node_replan);

    int distance = std::distance(nodes.begin(), it_node_replan) - std::distance(nodes.begin(), it_current_node);

    if (distance == 0)
    {
      CNR_DEBUG(logger_, RESET() << BC() << "DISTANCE ZERO");

      if (node_replan != current_node)
        throw std::runtime_error("shoudl be the same node");
    }
    else if (distance < 0)  // replan node before current node on current path
    {
      CNR_DEBUG(logger_, RESET() << BC() << "DISTANCE <0");

      size_t idx;
      ConnectionPtr current_conn = replanned_path->findConnection(configuration, idx, true);
      if (current_conn != nullptr)  // current node is on replanned path
      {
        CNR_DEBUG(logger_, RESET() << BC() << "\t -> CASE 1");

        if (current_conn->getParent() == current_node || current_conn->getChild() == current_node)
        {
          CNR_DEBUG(logger_, RESET() << BC() << "\t -> CASE 1.1");

          replanned_path->setConnections(replanned_path->getSubpathFromNode(current_node)->getConnections());
        }
        else
        {
          CNR_DEBUG(logger_, RESET() << BC() << "\t -> CASE 1.2");

          if (conn != current_conn)
          {
            CNR_INFO(logger_, "conf " << configuration.transpose());
            CNR_INFO(logger_, "conn " << *conn << "\n" << conn);
            CNR_INFO(logger_, "current_conn " << *current_conn << "\n" << current_conn);

            CNR_INFO(logger_, RESET() << BOLDBLUE() << "CURRENT PATH " << *current_path << RESET());
            CNR_INFO(logger_, RESET() << BOLDBLUE() << "REPLANNED PATH " << *replanned_path << RESET());

            stop_ = true;
            if (not display_thread_.joinable())
              display_thread_.join();

            DisplayPtr disp = std::make_shared<Display>(planning_scn_cc_, group_name_);
            disp->clearMarkers();

            ros::Duration(1).sleep();

            disp->displayConnection(conn, "graph_display", { 1, 0, 0, 1 });

            disp->displayConnection(current_conn, "graph_display", { 0, 0, 1, 1 });

            disp->displayNode(std::make_shared<Node>(configuration), "graph_display", { 1, 0, 0, 0.5 });

            ros::Duration(1).sleep();

            throw std::runtime_error("err");
          }

          if (not replanned_path->splitConnection(current_path->getConnectionsConst().at(conn_idx), current_path->getConnectionsConst().at(conn_idx + 1),
                                                  current_conn))
            CNR_DEBUG(logger_, RESET() << BOLDBLUE() << "CONNECTION NOT SPLITTED" << RESET());

          replanned_path->setConnections(replanned_path->getSubpathFromNode(current_node)->getConnections());
        }
      }
      else
      {
        CNR_DEBUG(logger_, RESET() << BC() << "\t -> CASE 2");

        // current node should be very close to replan node, minimal difference between the connections

        std::vector<ConnectionPtr> replanned_path_conns = replanned_path->getConnections();

        ConnectionPtr first_conn = replanned_path_conns.front();
        NodePtr child = first_conn->getChild();

        ConnectionPtr new_conn = std::make_shared<Connection>(current_node, child, logger_, first_conn->isNet());
        (first_conn->getCost() < std::numeric_limits<double>::infinity()) ?
            new_conn->setCost(replanned_path->getMetrics()->cost(current_node->getConfiguration(), child->getConfiguration())) :
            new_conn->setCost(std::numeric_limits<double>::infinity());

        first_conn->remove();
        new_conn->add();

        replanned_path_conns[0] = new_conn;
        replanned_path->setConnections(replanned_path_conns);
      }
    }
    else  // distance>0 //replan node after current node on current path
    {
      CNR_DEBUG(logger_, RESET() << BC() << "DISTANCE > 0");

      assert(current_node != node_replan);
      assert((current_node->getConfiguration() - node_replan->getConfiguration()).norm() > TOLERANCE);

      PathPtr tmp_subpath;
      tmp_subpath = current_path->getSubpathFromNode(current_node);
      try
      {
        tmp_subpath = tmp_subpath->getSubpathToNode(node_replan);
      }
      catch (...)
      {
        CNR_INFO(logger_, "current path: " << *current_path);  // elimina
        CNR_INFO(logger_, "tmp subpath: " << *tmp_subpath);    // elimina
        CNR_INFO(logger_, "current node: " << current_node << " " << *current_node);
        CNR_INFO(logger_, "node_replan : " << node_replan << " " << *node_replan);
        throw std::runtime_error("runtime err");
      }
      std::vector<ConnectionPtr> new_conns = tmp_subpath->getConnections();

      new_conns.insert(new_conns.end(), replanned_path->getConnectionsConst().begin(), replanned_path->getConnectionsConst().end());
      replanned_path->setConnections(new_conns);

      CNR_DEBUG(logger_, RESET() << BC() << "NEW PATH\n" << *replanned_path);

      if (old_current_node_ != nullptr)
      {
        std::vector<NodePtr> pn = replanned_path->getNodes();
        if (std::find(pn.begin(), pn.end(), old_current_node_) >= pn.end())
        {
          CNR_INFO(logger_, "rp " << *replanned_path);
          CNR_INFO(logger_, "old cur node " << *old_current_node_ << old_current_node_);

          throw std::runtime_error("error");
        }
      }
    }

    if (replanner->replanNodeIsANewNode() && ((node_replan->getConfiguration() - configuration).norm() > TOLERANCE) && node_replan != old_current_node_)
    {
      ConnectionPtr restored_conn;
      if (replanned_path->removeNode(node_replan, {}, restored_conn))
      {
        std::vector<PathPtr> paths = other_paths_;
        paths.push_back(current_path);

        for (PathPtr& p : paths)
        {
          assert(p->getTree() != nullptr);
          p->restoreConnection(restored_conn, node_replan);

          assert(not tree->isInTree(node_replan));
          assert([&]() -> bool {
            std::vector<NodePtr> p_nodes = p->getNodes();
            if (std::find(p_nodes.begin(), p_nodes.end(), node_replan) < p_nodes.end())
            {
              CNR_INFO(logger_, "other path: " << *p);
              return false;
            }
            else
            {
              return true;
            }
          }());
        }
      }
    }

    assert([&]() -> bool {
      for (const NodePtr& n : replanned_path->getNodes())
      {
        if (n->getParentConnectionsSize() != 1)
        {
          for (const NodePtr& nn : replanned_path->getNodes())
          {
            CNR_INFO(logger_, nn << " " << *nn);
          }
          CNR_INFO(logger_, *replanned_path);

          return false;
        }
      }
      return true;
    }());
  }
}

bool ReplannerManagerMARS::haveToReplan(const bool path_obstructed)
{
  return alwaysReplan();
}

void ReplannerManagerMARS::updateSharedPath()
{
  ReplannerManagerBase::updateSharedPath();

  other_paths_mtx_.lock();
  bool sync_needed;

  for (unsigned int i = 0; i < other_paths_shared_.size(); i++)
  {
    sync_needed = false;
    if (other_paths_shared_.at(i)->getConnectionsSize() == other_paths_.at(i)->getConnectionsSize())
    {
      for (unsigned int j = 0; j < other_paths_shared_.at(i)->getConnectionsSize(); j++)
      {
        if (other_paths_shared_.at(i)->getConnections().at(j)->getParent()->getConfiguration() !=
            other_paths_.at(i)->getConnections().at(j)->getParent()->getConfiguration())
        {
          sync_needed = true;
          break;
        }
      }

      if (other_paths_shared_.at(i)->getConnections().back()->getChild()->getConfiguration() !=
          other_paths_shared_.at(i)->getConnections().back()->getChild()->getConfiguration())
        sync_needed = true;
    }
    else
    {
      sync_needed = true;
    }

    if (sync_needed)
    {
      other_paths_sync_needed_.at(i) = true;  // NB: sync needed false set by the collision check thread

      CollisionCheckerPtr checker = other_paths_shared_.at(i)->getChecker();
      other_paths_shared_.at(i) = other_paths_.at(i)->clone();
      other_paths_shared_.at(i)->setChecker(checker);
    }
  }
  other_paths_mtx_.unlock();
}

void ReplannerManagerMARS::downloadPathCost()
{
  ReplannerManagerBase::downloadPathCost();

  other_paths_mtx_.lock();

  unsigned int other_paths_size = std::min(other_paths_.size(), other_paths_shared_.size());
  for (unsigned int i = 0; i < other_paths_size; i++)
  {
    std::vector<ConnectionPtr> other_path_conn = other_paths_.at(i)->getConnections();
    std::vector<ConnectionPtr> other_path_shared_conn = other_paths_shared_.at(i)->getConnections();

    std::vector<ConnectionPtr>::iterator it = other_path_conn.end();
    std::vector<ConnectionPtr>::iterator it_shared = other_path_shared_conn.end();
    while (it > other_path_conn.begin() && it_shared > other_path_shared_conn.begin())
    {
      it--;
      it_shared--;

      if ((*it)->getParent()->getConfiguration() == (*it_shared)->getParent()->getConfiguration() &&
          (*it)->getChild()->getConfiguration() == (*it_shared)->getChild()->getConfiguration())
      {
        (*it)->setCost((*it_shared)->getCost());
      }
      else
        break;
    }

    other_paths_.at(i)->cost();  // update path cost
  }
  other_paths_mtx_.unlock();
}

void ReplannerManagerMARS::initReplanner()
{
  double time_for_repl = 0.9 * dt_replan_;
  MARSPtr replanner = std::make_shared<MARS>(configuration_replan_, current_path_, time_for_repl, solver_, logger_, other_paths_);

  replanner->reverseStartNodes(reverse_start_nodes_);
  replanner->setFullNetSearch(full_net_search_);
  replanner_ = replanner;

#ifdef GRAPH_DISPLAY_AVAILABLE
  DisplayPtr disp = std::make_shared<Display>(planning_scn_cc_, group_name_);
  replanner_->setDisp(disp);
#endif
}

bool ReplannerManagerMARS::checkPathTask(const PathPtr& path)
{
  bool valid = path->isValid();
  path->cost();

  return valid;
}

void ReplannerManagerMARS::collisionCheckThread()
{
  moveit_msgs::GetPlanningScene ps_srv;
  Eigen::VectorXd current_configuration_copy;

  PathPtr current_path_copy = current_path_shared_->clone();
  current_path_copy->setChecker(checker_cc_);

  std::vector<PathPtr> other_paths_copy;
  std::vector<MoveitCollisionCheckerPtr> checkers;
  for (const PathPtr& p : other_paths_shared_)
  {
    PathPtr path_copy = p->clone();
    MoveitCollisionCheckerPtr checker = std::static_pointer_cast<MoveitCollisionChecker>(checker_cc_->clone());

    checkers.push_back(checker);
    path_copy->setChecker(checker);
    other_paths_copy.push_back(path_copy);
  }

  size_t other_path_size = other_paths_copy.size();

  ros::WallRate lp(collision_checker_thread_frequency_);
  graph_time_point tic;

  moveit_msgs::PlanningScene planning_scene_msg;

  while ((not stop_) && ros::ok())
  {
    tic = graph_time::now();

    /* Update the planning scene:
     * moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY +
     * moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS */
    ps_srv.request.components.components = 20;

    if (not plannning_scene_client_.call(ps_srv))
    {
      CNR_ERROR(logger_, "call to srv not ok");
      stop_ = true;
      break;
    }

    scene_mtx_.lock();
    planning_scene_msg.world = ps_srv.response.scene.world;
    planning_scene_msg.is_diff = true;
    checker_cc_->setPlanningSceneMsg(planning_scene_msg);

    for (const MoveitCollisionCheckerPtr& checker : checkers)
      checker->setPlanningSceneMsg(planning_scene_msg);
    scene_mtx_.unlock();

    /* Update paths if they have been changed */
    trj_mtx_.lock();

    current_configuration_copy = current_configuration_;

    paths_mtx_.lock();
    if (current_path_sync_needed_)
    {
      current_path_copy = current_path_shared_->clone();
      current_path_copy->setChecker(checker_cc_);
      current_path_sync_needed_ = false;
    }

    other_paths_mtx_.lock();
    if (other_path_size < other_paths_shared_.size())  // if the previous current path has been added, update the vector of copied paths
    {
      assert(other_path_size == (other_paths_shared_.size() - 1));

      MoveitCollisionCheckerPtr checker = std::static_pointer_cast<MoveitCollisionChecker>(checker_cc_->clone());
      PathPtr path_copy = other_paths_shared_.back()->clone();

      checkers.push_back(checker);
      path_copy->setChecker(checker);
      other_paths_copy.push_back(path_copy);

      other_path_size = other_paths_copy.size();
    }

    for (unsigned int i = 0; i < other_paths_shared_.size(); i++)  // sync other_paths_shared with its copy
    {
      if (other_paths_sync_needed_.at(i))
      {
        other_paths_copy.at(i) = other_paths_shared_.at(i)->clone();
        other_paths_copy.at(i)->setChecker(checkers.at(i));
        other_paths_sync_needed_.at(i) = false;
      }
    }

    other_paths_mtx_.unlock();
    paths_mtx_.unlock();
    trj_mtx_.unlock();

    /* Launch collision check tasks */

    std::vector<std::shared_future<bool>> tasks;
    for (unsigned int i = 0; i < other_paths_copy.size(); i++)
    {
      tasks.push_back(std::async(std::launch::async, &ReplannerManagerMARS::checkPathTask, this, other_paths_copy.at(i)));
    }

    // current_path_copy->isValidFromConf(current_configuration_copy,checker_cc_);
    size_t conn_idx;
    if (not current_path_copy->findConnection(current_configuration_copy, conn_idx))
      continue;
    else
      current_path_copy->isValidFromConf(current_configuration_copy, conn_idx, checker_cc_);

    for (unsigned int i = 0; i < tasks.size(); i++)
      tasks.at(i).wait();  // wait for the end of each task

    /* Update the cost of the paths */
    scene_mtx_.lock();
    paths_mtx_.lock();
    other_paths_mtx_.lock();
    if (uploadPathsCost(current_path_copy, other_paths_copy))
    {
      planning_scene_msg_.world = ps_srv.response.scene.world;  // not diff,it contains all pln scn info but only world is updated
      planning_scene_diff_msg_ = planning_scene_msg;            // diff, contains only world

      cost_updated_ = true;  // dowloadPathCost can be called because the scene and path cost are referred now to the last path found
    }
    other_paths_mtx_.unlock();
    paths_mtx_.unlock();
    scene_mtx_.unlock();

    double duration = toSeconds(graph_time::now(), tic);

    if (duration > (1.0 / collision_checker_thread_frequency_) && display_timing_warning_)
      CNR_INFO(logger_, RESET() << BOLDYELLOW() << "Collision checking thread time expired: total duration-> " << duration << RESET());

    lp.sleep();
  }

  CNR_INFO(logger_, RESET() << BOLDCYAN() << "Collision check thread is over" << RESET());
}

bool ReplannerManagerMARS::uploadPathsCost(const PathPtr& current_path_updated_copy, const std::vector<PathPtr>& other_paths_updated_copy)
{
  /*
   * Ensure all path costs are updated simultaneously.
   * If this isn't possible, avoid updating any path to maintain consistency and simplicity.
   */
  bool syncronized =
      (not current_path_sync_needed_) && std::all_of(other_paths_sync_needed_.begin(), other_paths_sync_needed_.end(), [](bool v) { return !v; });
  if (!syncronized)
  {
    return false;
  }
  else
  {
    // Update current path cost
    std::vector<ConnectionPtr> current_path_conns = current_path_shared_->getConnectionsConst();
    std::vector<ConnectionPtr> current_path_copy_conns = current_path_updated_copy->getConnectionsConst();

    for (unsigned int j = 0; j < current_path_conns.size(); j++)
    {
      assert([&] {
        if (current_path_conns.at(j)->getParent()->getConfiguration() != current_path_copy_conns.at(j)->getParent()->getConfiguration())
        {
          CNR_ERROR(logger_, "current parent\n" << *current_path_conns.at(j)->getParent());
          CNR_ERROR(logger_, "copied parent\n" << *current_path_copy_conns.at(j)->getParent());
          return false;
        }

        if (current_path_conns.at(j)->getChild()->getConfiguration() != current_path_copy_conns.at(j)->getChild()->getConfiguration())
        {
          CNR_ERROR(logger_, "current child\n" << *current_path_conns.at(j)->getChild());
          CNR_ERROR(logger_, "copied child\n" << *current_path_copy_conns.at(j)->getChild());
          return false;
        }
        return true;
      }());

      current_path_conns.at(j)->setCost(current_path_copy_conns.at(j)->getCost());
    }
    current_path_shared_->cost();

    if (current_path_shared_->getCostFromConf(current_configuration_) == std::numeric_limits<double>::infinity() &&
        (display_timing_warning_ || display_replanning_success_))
      CNR_INFO(logger_, RESET() << BOLDMAGENTA() << "Obstacle detected!" << RESET());

    // Update other paths costs
    for (unsigned int i = 0; i < other_paths_updated_copy.size(); i++)
    {
      std::vector<ConnectionPtr> path_conns = other_paths_shared_.at(i)->getConnectionsConst();
      std::vector<ConnectionPtr> path_copy_conns = other_paths_updated_copy.at(i)->getConnectionsConst();

      assert(path_conns.size() == path_copy_conns.size());
      for (unsigned int j = 0; j < path_conns.size(); j++)
      {
        path_conns.at(j)->setCost(path_copy_conns.at(j)->getCost());

        assert((path_conns.at(j)->getParent()->getConfiguration() - path_copy_conns.at(j)->getParent()->getConfiguration()).norm() < 1e-06 &&
               (path_conns.at(j)->getChild()->getConfiguration() - path_copy_conns.at(j)->getChild()->getConfiguration()).norm() < 1e-06);
      }
      other_paths_shared_.at(i)->cost();
    }

    return true;
  }
}

void ReplannerManagerMARS::displayCurrentPath()
{
  ReplannerManagerBase::displayThread();
}

void ReplannerManagerMARS::displayOtherPaths()
{
  DisplayPtr disp = std::make_shared<Display>(planning_scn_cc_, group_name_);
  disp->clearMarkers();

  int path_id, wp_id;
  std::vector<PathPtr> other_paths;
  std::vector<double> marker_color = { 1.0, 0.5, 0.3, 1.0 };
  std::vector<double> marker_scale = { 0.01, 0.01, 0.01 };

  disp->changeNodeSize(marker_scale);

  double display_thread_frequency = 0.75 * trj_exec_thread_frequency_;
  ros::WallRate lp(display_thread_frequency);

  while ((not stop_) && ros::ok())
  {
    other_paths.clear();

    other_paths_mtx_.lock();
    for (const PathPtr& p : other_paths_shared_)
      other_paths.push_back(p->clone());
    other_paths_mtx_.unlock();

    path_id = 20000;
    wp_id = 25000;

    for (const PathPtr& p : other_paths)
    {
      disp->displayPathAndWaypoints(p, path_id, wp_id, "graph_display", marker_color);

      path_id += 1;
      wp_id += 1000;
    }

    lp.sleep();
  }

  disp->clearMarkers();
  CNR_INFO(logger_, RESET() << BOLDCYAN() << "Display other paths thread is over" << RESET());
}

void ReplannerManagerMARS::displayThread()
{
  std::thread current_path_display_thread = std::thread(&ReplannerManagerMARS::displayCurrentPath, this);

  if (display_other_paths_)
  {
    std::thread other_paths_display_thread = std::thread(&ReplannerManagerMARS::displayOtherPaths, this);

    if (other_paths_display_thread.joinable())
      other_paths_display_thread.join();
  }

  if (current_path_display_thread.joinable())
    current_path_display_thread.join();
}
}  // namespace openmore
