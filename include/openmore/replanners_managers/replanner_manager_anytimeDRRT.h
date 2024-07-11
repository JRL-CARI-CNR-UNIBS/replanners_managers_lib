#pragma once

#include <openmore/replanners_managers/replanner_manager_DRRT.h>
#include <openmore/replanners/anytimeDRRT.h>

namespace openmore
{
class ReplannerManagerAnytimeDRRT;
typedef std::shared_ptr<ReplannerManagerAnytimeDRRT> ReplannerManagerAnytimeDRRTPtr;

class ReplannerManagerAnytimeDRRT: public ReplannerManagerDRRT
{
protected:

  bool haveToReplan(const bool path_obstructed) override;
  void initReplanner() override;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ReplannerManagerAnytimeDRRT(const PathPtr &current_path,
                              const TrajectoryPtr& trajectory_processor,
                              const TreeSolverPtr &solver,
                              const std::string &param_ns,
                              const TraceLoggerPtr& logger);
};
}
