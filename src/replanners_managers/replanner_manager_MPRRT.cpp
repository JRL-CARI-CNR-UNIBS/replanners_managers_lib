﻿/*
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

#include "openmore/replanners_managers/replanner_manager_MPRRT.h"

namespace openmore
{
ReplannerManagerMPRRT::ReplannerManagerMPRRT(const PathPtr& current_path, const TrajectoryPtr& trajectory_processor, const TreeSolverPtr& solver,
                                             const std::string& param_ns, const TraceLoggerPtr& logger)
  : ReplannerManagerBase(current_path, trajectory_processor, solver, param_ns, logger)
{
  RRTPtr tmp_solver = std::make_shared<RRT>(solver_->getMetrics(), checker_replanning_, solver_->getSampler(), logger_);
  tmp_solver->importFromSolver(solver);

  solver_ = tmp_solver;

  additionalParams();
}

void ReplannerManagerMPRRT::additionalParams()
{
  unsigned int default_n_threads = 5;

  if (get_param(logger_, param_ns_, "MPRRT/n_threads_replan", n_threads_replan_, default_n_threads))
  {
    if (n_threads_replan_ < 1)
    {
      CNR_ERROR(logger_, "n_threads_replan can not be less than 1, set 1");
      n_threads_replan_ = 1;
    }
  }
}

void ReplannerManagerMPRRT::attributeInitialization()
{
  ReplannerManagerBase::attributeInitialization();
  path_optimizer_ = std::make_shared<PathLocalOptimizer>(checker_replanning_, solver_->getMetrics(), logger_);
}

void ReplannerManagerMPRRT::startReplannedPathFromNewCurrentConf(const Eigen::VectorXd& configuration)
{
  std::vector<ConnectionPtr> path_connections;
  PathPtr replanned_path = replanner_->getReplannedPath();
  Eigen::VectorXd replanned_path_start_conf = replanned_path->getStartNode()->getConfiguration();
  std::vector<ConnectionPtr> conn_replanned = replanned_path->getConnections();

  // If the configuration matches to a node of the replanned path
  for (const Eigen::VectorXd& wp : replanned_path->getWaypoints())
  {
    if ((wp - configuration).norm() < TOLERANCE)
    {
      assert(wp != replanned_path->getWaypoints().back());
      replanned_path = replanned_path->getSubpathFromNode(configuration);

      return;
    }
  }

  // Otherwise, if the configuration does not match to any path node..
  PathPtr current_path = replanner_->getCurrentPath();

  PathPtr path_conf2replanned;
  size_t idx_current_conf, idx_replanned_path_start;

  double abscissa_current_conf = current_path->curvilinearAbscissaOfPoint(configuration, idx_current_conf);
  double abscissa_replanned_path_start = current_path->curvilinearAbscissaOfPoint(replanned_path_start_conf, idx_replanned_path_start);

  assert(abscissa_current_conf != abscissa_replanned_path_start);

  if (abscissa_current_conf < abscissa_replanned_path_start)  // the replanned path starts from a position after the current one
  {
    path_conf2replanned = current_path->clone();
    NodePtr n1 = path_conf2replanned->addNodeAtCurrentConfig(configuration, true);
    NodePtr n2 = path_conf2replanned->addNodeAtCurrentConfig(replanned_path_start_conf, true);

    path_conf2replanned = path_conf2replanned->getSubpathFromNode(n1);
    path_conf2replanned = path_conf2replanned->getSubpathToNode(n2);

    path_connections = path_conf2replanned->getConnections();

    assert((path_connections.back()->getChild()->getConfiguration() - conn_replanned.front()->getParent()->getConfiguration()).norm() < TOLERANCE);

    ConnectionPtr conn = std::make_shared<Connection>(path_connections.back()->getParent(), conn_replanned.front()->getParent(), logger_);
    conn->setCost(path_connections.back()->getCost());
    conn->add();

    path_connections.back()->remove();
    path_connections.pop_back();
    path_connections.push_back(conn);

    path_connections.insert(path_connections.end(), conn_replanned.begin(), conn_replanned.end());
  }
  else
  {
    path_conf2replanned = current_path->clone();
    NodePtr n1 = path_conf2replanned->addNodeAtCurrentConfig(replanned_path_start_conf, true);
    NodePtr n2 = path_conf2replanned->addNodeAtCurrentConfig(configuration, true);

    path_conf2replanned = path_conf2replanned->getSubpathFromNode(n1);
    path_conf2replanned = current_path->getSubpathToNode(n2);

    path_conf2replanned->flip();
    path_connections = path_conf2replanned->getConnections();

    ConnectionPtr conn = std::make_shared<Connection>(path_connections.back()->getParent(), conn_replanned.front()->getParent(), logger_);
    conn->setCost(path_connections.back()->getCost());
    conn->add();

    path_connections.back()->remove();
    path_connections.pop_back();
    path_connections.push_back(conn);

    path_connections.insert(path_connections.end(), conn_replanned.begin(), conn_replanned.end());
  }

  replanned_path->setConnections(path_connections);
  path_optimizer_->setPath(replanned_path);
  path_optimizer_->simplify(0.01);
}

bool ReplannerManagerMPRRT::haveToReplan(const bool path_obstructed)
{
  return alwaysReplan();
}

void ReplannerManagerMPRRT::initReplanner()
{
  double time_for_repl = 0.9 * dt_replan_;

  replanner_ = std::make_shared<MPRRT>(configuration_replan_, current_path_, time_for_repl, solver_, logger_, n_threads_replan_);
}

}  // namespace openmore
