#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GncOptimizer.h>

#include <boost/filesystem.hpp>
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <sstream>
#include <cstdlib>
#include <string>
#include <iostream>


struct Config
{
  std::string name;
  std::string dataset;
  std::string ground_truth;
  std::string output;
  bool visualize;
  int canonic_inliers;
  double fast_reject_th;
  int fast_reject_iter_base;
  double slow_reject_th;
  int slow_reject_iter_base;
};


void readConfig(const std::string& cfg_filepath, Config& out_cfg);
void store3D(const std::string& output_filepath, const gtsam::Values& result);
void store2D(const std::string& output_filepath, const gtsam::Values& result);

void addPrior3D(gtsam::NonlinearFactorGraph& nfg);
void addPrior2D(gtsam::NonlinearFactorGraph& nfg);