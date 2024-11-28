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

#include "openmore/replanners_managers/replanner_manager_DRRTStar.h"

namespace openmore
{
ReplannerManagerDRRTStar::ReplannerManagerDRRTStar(const PathPtr &current_path,
                                                   const TrajectoryPtr& trajectory_processor,
                                                   const TreeSolverPtr &solver,
                                                   const std::string &param_ns,
                                                   const TraceLoggerPtr& logger):
  ReplannerManagerBase(current_path,trajectory_processor,solver,param_ns,logger)
{
  RRTStarPtr tmp_solver = std::make_shared<RRTStar>(solver_->getMetrics(), checker_replanning_, solver_->getSampler(),logger_);
  tmp_solver->importFromSolver(solver);

  solver_  = tmp_solver;
}

void ReplannerManagerDRRTStar::startReplannedPathFromNewCurrentConf(const Eigen::VectorXd& configuration)
{
  PathPtr current_path = replanner_->getCurrentPath();
  PathPtr replanned_path = replanner_->getReplannedPath();
  TreePtr tree = current_path->getTree();

  bool was_a_new_node;
  if(not old_current_node_)
    was_a_new_node = false;
  else
    was_a_new_node = is_a_new_node_;

  if(not tree->changeRoot(current_path->getStartNode()))
    throw std::runtime_error("root can not be changed");

  NodePtr current_node;
  ConnectionPtr conn = current_path->findConnection(configuration);

  if(conn->isValid())
    current_node = current_path->addNodeAtCurrentConfig(configuration,conn,true,is_a_new_node_);
  else  //if the conn of current conf is the conn before the replan goal, it is not valid
  {
    assert(conn->getParent() != nullptr && conn->getParent() != nullptr);

    current_node = current_path->addNodeAtCurrentConfig(configuration,conn,false);
    conn = std::make_shared<Connection>(conn->getParent(),current_node,logger_);
    conn->setCost(tree->getMetrics()->cost(conn->getParent(),current_node));
    conn->add();

    tree->addNode(current_node);
  }

  if(not tree->changeRoot(current_node))
    throw std::runtime_error("root can not be changed");

  if(old_current_node_ && old_current_node_ != tree->getRoot()) //remove old current node before computing new path
  {
    if(was_a_new_node)
    {
      if((old_current_node_->getParentConnectionsSize() + old_current_node_->getChildConnectionsSize()) == 2)
      {
        ConnectionPtr parent_conn = old_current_node_->getParentConnections().front();
        ConnectionPtr child_conn  = old_current_node_->getChildConnections().front();

        if(parent_conn->isParallel(child_conn))
        {
          NodePtr parent = parent_conn->getParent();
          NodePtr child = child_conn->getChild();

          double restored_cost = parent_conn->getCost()+child_conn->getCost();

          ConnectionPtr restored_conn = std::make_shared<Connection>(parent,child,logger_);
          restored_conn->setCost(restored_cost);
          restored_conn->add();

          tree->removeNode(old_current_node_);
        }
      }
    }
  }

  std::vector<ConnectionPtr> new_conns = tree->getConnectionToNode(replanned_path->getGoalNode());
  replanned_path->setConnections(new_conns);

  old_current_node_ = current_node;
}

bool ReplannerManagerDRRTStar::haveToReplan(const bool path_obstructed)
{
  return replanIfObstructed(path_obstructed);
}

void ReplannerManagerDRRTStar::initReplanner()
{
  double time_for_repl = 0.9*dt_replan_;
  replanner_ = std::make_shared<DynamicRRTStar>(configuration_replan_,current_path_,time_for_repl,solver_,logger_);
}

}
