#include <iostream>
#include <yaml-cpp/yaml.h>
#include <random>
#include <chrono>
#include <vector>
#include <algorithm>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/stuff/command_args.h"
#include "g2o/types/slam2d/types_slam2d.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/core/optimizable_graph.h"


struct Config
{
  std::string dataset;
  std::string output;
  int canonic_inliers;
  int maxiters;
  double inlier_th;
  
  // Switchable parameters
  double switch_prior;
  
  // MaxMix parameters
  double maxmix_weight;
  double nu_constraints;
  double nu_nullHypothesis;
};

// Using edges to initialize graph
template <class EDGE, class VERTEX>
void odometryInitialization(g2o::SparseOptimizer& optimizer);

// Sets the whole optimization problem
template <class T, class EDGE, class VERTEX>
void setProblem(const std::string& problem_file, 
                g2o::SparseOptimizer& optimizer,
                std::vector<T>& init_poses,
                std::vector<VERTEX*>& v_poses);


template <class EDGE, class VERTEX>
void getOdometryEdges(const g2o::SparseOptimizer& optimizer, g2o::OptimizableGraph::EdgeContainer& odometry_edges);

void writeVertex(std::ofstream& out_data, g2o::VertexSE2* v);
void writeVertex(std::ofstream& out_data, g2o::VertexSE3* v);

template <class T>
void readSolutionFile(std::vector<T>& poses, const std::string& path);
void readConfig(const std::string& cfg_filepath, Config& out_cfg);
void readLine(std::ifstream& in_data, Eigen::Isometry2d& pose); 
void readLine(std::ifstream& in_data, Eigen::Isometry3d& pose);