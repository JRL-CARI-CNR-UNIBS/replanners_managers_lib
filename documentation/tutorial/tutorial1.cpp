// graph_core required headers
#include <graph_core/solvers/rrt.h>
#include <graph_core/samplers/uniform_sampler.h>
#include <graph_core/metrics/euclidean_metrics.h>
#include <graph_core/collision_checkers/cube_3d_collision_checker.h>

// replanners_lib required headers
#include <openmore/replanners/DRRT.h>

int main(int argc, char** argv)
{
  // Logger (see graph_core/logger.h)
  graph::core::LoggerPtr logger = std::make_shared<graph::core::ConsoleLogger>("openmore_tutorial", graph::core::LogLevel::Info);

  // Parameters under "openmore_tutorial" in a YAML file (see graph_core/params.h)
  std::string path_to_config_folder = "path/to/config/folder";
  graph::core::ParamMap params =
      graph::core::paramsFromYAML(YAML::LoadFile(path_to_config_folder + "/params.yaml")["openmore_tutorial"]);

  // Define the collision checker (foo collision checker)
  double min_cc_distance;
  double default_min_cc_distance = 0.01;
  graph::core::get_param(logger, params, "min_cc_distance", min_cc_distance, default_min_cc_distance);

  if (min_cc_distance <= 0)
    min_cc_distance = default_min_cc_distance;

  double joints_threshold = 0.0;

  graph::collision_check::CollisionCheckerPtr collision_checker =
      std::make_shared<graph::collision_check::Cube3dCollisionChecker>(logger, joints_threshold, min_cc_distance);

  // Define a cost function (Euclidean metrics)
  graph::core::MetricsPtr metrics = std::make_shared<graph::core::EuclideanMetrics>(logger);

  // Define lower/upper bounds
  size_t dof = 3;
  Eigen::VectorXd lb(dof);
  lb.setConstant(-2.5);
  Eigen::VectorXd ub(dof);
  ub.setConstant(2.5);

  // Define a sampler (uniform sampler)
  graph::core::SamplerPtr sampler = std::make_shared<graph::core::UniformSampler>(lb, ub, logger);

  // Define the solver (RRT)
  graph::core::TreeSolverPtr solver = std::make_shared<graph::core::RRT>(metrics, collision_checker, sampler, logger);

  // Define start and goal nodes
  Eigen::VectorXd start_configuration(dof);
  start_configuration << -1.5, -1.5, -1.5;

  Eigen::VectorXd goal_configuration(dof);
  goal_configuration << 1.5, 1.5, 1.5;

  // Optionally, you can load these directly from params
  // graph::core::get_param(logger, params, "start_configuration",start_configuration);
  // graph::core::get_param(logger, params, "goal_configuration",goal_configuration)

  graph::core::NodePtr start_node = std::make_shared<graph::core::Node>(start_configuration, logger);
  graph::core::NodePtr goal_node = std::make_shared<graph::core::Node>(goal_configuration, logger);

  // Compute the initial robot's path
  double max_time = 10.0;  // seconds
  size_t max_iter = 1000000;
  graph::core::PathPtr initial_path;

  bool found = solver->computePath(start_node, goal_node, params, initial_path, max_time, max_iter);

  if (found)
    GC_INFO(logger, "Path found! Cost: " << initial_path->cost());
  else
  {
    GC_ERROR(logger, "Initial path not found!");
    return 0;
  }

  // Simulate a new obstacle on the path and update its cost
  joints_threshold = 1.0;  // increases the obstacle's size, from 0.0 to 1.0 on each robot's joint
  graph::collision_check::CollisionCheckerPtr new_obs_collision_checker =
      std::make_shared<graph::collision_check::Cube3dCollisionChecker>(logger, joints_threshold, min_cc_distance);
  initial_path->setCollisionChecker(new_obs_collision_checker);
  initial_path->isValid();

  GC_INFO(logger, "Updated path's cost: " << initial_path->cost());

  // Create a replanner object
  graph::core::TreeSolverPtr replanning_solver = std::make_shared<graph::core::RRT>(metrics, new_obs_collision_checker, sampler, logger);

  double max_replanning_time = 0.200;  // 200 ms
  Eigen::VectorXd current_configuration = start_configuration;
  openmore::ReplannerBasePtr replanner = std::make_shared<openmore::DynamicRRT>(current_configuration, initial_path, max_time, replanning_solver, logger);

  // Replan
  auto tic = graph_time::now();
  replanner->replan();
  success = replanner->getSuccess();
  auto toc = graph_time::now();

  double elapsed_time = graph::core::toSeconds(toc, tic);

  if (success)
    GC_INFO(logger, "New path found in " << elapsed_time << " seconds. Cost: " << replanner->getReplannedPath()->cost());
  else
    GC_ERROR(logger, "No valid path found in " << elapsed_time << " seconds.");

  return 0;
}
