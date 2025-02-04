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

#include <openmore/replanners_managers/replanner_manager_base.h>
#include <openmore/replanners/MARS.h>
#include <future>

namespace openmore
{
class ReplannerManagerMARS;
typedef std::shared_ptr<ReplannerManagerMARS> ReplannerManagerMARSPtr;

class ReplannerManagerMARS : public ReplannerManagerBase
{
protected:
  bool full_net_search_;
  bool first_replanning_;
  bool reverse_start_nodes_;
  bool display_other_paths_;
  int verbosity_level_;
  NodePtr old_current_node_;
  PathPtr initial_path_;
  std::mutex other_paths_mtx_;
  std::vector<PathPtr> other_paths_;
  std::vector<PathPtr> other_paths_shared_;
  std::vector<bool> other_paths_sync_needed_;

  bool checkPathTask(const PathPtr& path);
  void MARSadditionalParams();
  void displayCurrentPath();
  void displayOtherPaths();
  void downloadPathCost() override;
  bool uploadPathsCost(const PathPtr& current_path_updated_copy, const std::vector<PathPtr>& other_paths_updated_copy);
  void displayThread() override;
  bool haveToReplan(const bool path_obstructed) override;
  virtual void updateSharedPath() override;
  virtual void attributeInitialization() override;

  virtual bool replan() override;
  virtual void initReplanner() override;
  virtual void collisionCheckThread() override;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ReplannerManagerMARS(const PathPtr& current_path, const TrajectoryPtr& trajectory_processor, const TreeSolverPtr& solver, const std::string& param_ns,
                       const TraceLoggerPtr& logger);

  ReplannerManagerMARS(const PathPtr& current_path, const TrajectoryPtr& trajectory_processor, const TreeSolverPtr& solver, const std::string& param_ns,
                       const TraceLoggerPtr& logger, const std::vector<PathPtr>& other_paths);

  virtual void setOtherPaths(const std::vector<PathPtr>& other_paths)
  {
    other_paths_ = other_paths;
  }

  virtual void startReplannedPathFromNewCurrentConf(const Eigen::VectorXd& configuration) override;
};

}  // namespace openmore
