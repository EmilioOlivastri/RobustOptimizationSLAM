#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

void evaluateAte(std::vector<Eigen::Isometry3d>& sol,
                 const std::vector<Eigen::Isometry3d>& gt,
                 double& ate_rotation,
                 double& ate_translation,
                 double& rmse_translation);

void evaluateRpe(const std::vector<Eigen::Isometry3d>& sol,
                 const std::vector<Eigen::Isometry3d>& gt,
                 double& rpe_rotation,
                 double& rpe_translation);
                
void computeAlignmentTransform(const std::vector<Eigen::Isometry3d>& sol,
                               const std::vector<Eigen::Isometry3d>& gt,
                               Eigen::Isometry3d& delta);

void align(std::vector<Eigen::Isometry3d>& sol, const Eigen::Isometry3d& T);

void readSolutionFile2D(std::vector<Eigen::Isometry3d>& poses, const std::string& path);
void readSolutionFile3D(std::vector<Eigen::Isometry3d>& poses, const std::string& path);
