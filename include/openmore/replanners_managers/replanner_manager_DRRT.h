#pragma once

#include <openmore/replanners_managers/replanner_manager_base.h>
#include <openmore/replanners/DRRT.h>

namespace openmore
{
class ReplannerManagerDRRT;
typedef std::shared_ptr<ReplannerManagerDRRT> ReplannerManagerDRRTPtr;

class ReplannerManagerDRRT: public ReplannerManagerBase
{
protected:

  virtual bool haveToReplan(const bool path_obstructed) override;
  virtual void initReplanner() override;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ReplannerManagerDRRT(const PathPtr &current_path,
                       const TrajectoryPtr& trajectory_processor,
                       const TreeSolverPtr &solver,
                       const std::string &param_ns,
                       const TraceLoggerPtr& logger);

  void startReplannedPathFromNewCurrentConf(const Eigen::VectorXd& configuration) override;
};

}
