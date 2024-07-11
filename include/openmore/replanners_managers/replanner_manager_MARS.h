#pragma once

#include <openmore/replanners_managers/replanner_manager_base.h>
#include <openmore/replanners/MARS.h>
#include <future>

namespace openmore
{
class ReplannerManagerMARS;
typedef std::shared_ptr<ReplannerManagerMARS> ReplannerManagerMARSPtr;

class ReplannerManagerMARS: public ReplannerManagerBase
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

  ReplannerManagerMARS(const PathPtr &current_path,
                       const TrajectoryPtr& trajectory_processor,
                       const TreeSolverPtr &solver,
                       const std::string &param_ns,
                       const TraceLoggerPtr& logger);

  ReplannerManagerMARS(const PathPtr &current_path,
                       const TrajectoryPtr& trajectory_processor,
                       const TreeSolverPtr &solver,
                       const std::string &param_ns,
                       const TraceLoggerPtr& logger,
                       const std::vector<PathPtr> &other_paths);


  virtual void setOtherPaths(const std::vector<PathPtr>& other_paths)
  {
    other_paths_ = other_paths;
  }

  virtual void startReplannedPathFromNewCurrentConf(const Eigen::VectorXd& configuration) override;
};

}
