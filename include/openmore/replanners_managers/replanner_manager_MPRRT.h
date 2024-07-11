#pragma once

#include <graph_core/solvers/path_optimizers/path_local_optimizer.h>
#include <openmore/replanners_managers/replanner_manager_base.h>
#include <openmore/replanners/MPRRT.h>

namespace openmore
{
class ReplannerManagerMPRRT;
typedef std::shared_ptr<ReplannerManagerMPRRT> ReplannerManagerMPRRTPtr;

class ReplannerManagerMPRRT: public ReplannerManagerBase
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

  ReplannerManagerMPRRT(const PathPtr &current_path,
                        const TrajectoryPtr& trajectory_processor,
                        const TreeSolverPtr &solver,
                        const std::string &param_ns,
                        const TraceLoggerPtr& logger);

  void startReplannedPathFromNewCurrentConf(const Eigen::VectorXd &configuration) override;
};

}
