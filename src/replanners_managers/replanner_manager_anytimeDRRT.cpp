#include "openmore/replanners_managers/replanner_manager_anytimeDRRT.h"

namespace openmore
{

ReplannerManagerAnytimeDRRT::ReplannerManagerAnytimeDRRT(const PathPtr &current_path,
                                                         const TrajectoryPtr& trajectory_processor,
                                                         const TreeSolverPtr &solver,
                                                         const std::string &param_ns,
                                                         const TraceLoggerPtr& logger):
  ReplannerManagerDRRT(current_path,trajectory_processor,solver,param_ns,logger)
{
  AnytimeRRTPtr tmp_solver = std::make_shared<AnytimeRRT>(solver_->getMetrics(),checker_replanning_,
                                                          solver_->getSampler(),logger_);
  tmp_solver->importFromSolver(solver);

  solver_  = tmp_solver;
}

bool ReplannerManagerAnytimeDRRT::haveToReplan(const bool path_obstructed)
{
  return alwaysReplan();
}

void ReplannerManagerAnytimeDRRT::initReplanner()
{  
  double time_for_repl = 0.9*dt_replan_;
  replanner_ = std::make_shared<AnytimeDynamicRRT>(configuration_replan_,current_path_,
                                                   time_for_repl,solver_,logger_);
}
}
