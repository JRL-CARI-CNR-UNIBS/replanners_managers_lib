#pragma once

#include <openmore/replanners_managers/replanner_manager_base.h>
#include <openmore/replanners/DRRTStar.h>

namespace openmore
{
class ReplannerManagerDRRTStar;
typedef std::shared_ptr<ReplannerManagerDRRTStar> ReplannerManagerDRRTStarPtr;

class ReplannerManagerDRRTStar: public ReplannerManagerBase
{
protected:
  NodePtr old_current_node_ = nullptr;
  bool is_a_new_node_;

  bool haveToReplan(const bool path_obstructed) override;
  void initReplanner() override;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ReplannerManagerDRRTStar(const PathPtr &current_path,
                           const TrajectoryPtr& trajectory_processor,
                           const TreeSolverPtr &solver,
                           const std::string &param_ns,
                           const TraceLoggerPtr& logger);

  void startReplannedPathFromNewCurrentConf(const Eigen::VectorXd &configuration) override;
};

}
