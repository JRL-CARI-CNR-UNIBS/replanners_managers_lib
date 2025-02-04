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

#include <graph_core/solvers/path_optimizers/path_local_optimizer.h>
#include <openmore/replanners_managers/replanner_manager_base.h>
#include <openmore/replanners/MPRRT.h>

namespace openmore
{
class ReplannerManagerMPRRT;
typedef std::shared_ptr<ReplannerManagerMPRRT> ReplannerManagerMPRRTPtr;

class ReplannerManagerMPRRT : public ReplannerManagerBase
{
protected:
  PathLocalOptimizerPtr path_optimizer_;

  unsigned int n_threads_replan_;

  bool haveToReplan(const bool path_obstructed) override;
  void initReplanner() override;
  void additionalParams();
  void attributeInitialization() override;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ReplannerManagerMPRRT(const PathPtr& current_path, const TrajectoryPtr& trajectory_processor, const TreeSolverPtr& solver, const std::string& param_ns,
                        const TraceLoggerPtr& logger);

  void startReplannedPathFromNewCurrentConf(const Eigen::VectorXd& configuration) override;
};

}  // namespace openmore
