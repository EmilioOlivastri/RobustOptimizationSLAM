#include "metric_eval.hpp"

#define LINESIZE 81920

double getAngle(const Eigen::Matrix3d& R) 
{
  // to compute the angular error we refer to the axis-angle representation of the error
  // rotation matrix from that we can extract the sin and cos of the angle and then use the
  // atan2 (see Alessandro De Luca slides for the Robotics 1 course)
  const Eigen::Matrix3d skew_R = R - R.transpose();
  // sin of the error angle
  double sin_angle = 0.5 * std::sqrt(skew_R(0, 1) * skew_R(0, 1) + skew_R(0, 2) * skew_R(0, 2) +
                                     skew_R(1, 2) * skew_R(1, 2));
  // cos of the error angle
  double cos_angle = 0.5 * (R.trace() - 1.0);

  double angle = std::atan2(sin_angle, cos_angle);
  return angle;
}

/**/
void fixRotation(Eigen::Matrix3d& R) 
{
  Eigen::Matrix3d E = R.transpose() * R;
  //E.diagonal().array() -= Eigen::Vector3d::Ones(E.diagonal().array().size());
  for ( size_t idx = 0; idx < 3; ++idx ) E(idx, idx) -= 1.0;

  R -= R * E * 0.5;

  return;
}
/**/

void computeAlignmentTransform(const std::vector<Eigen::Isometry3d>& graph,
                               const std::vector<Eigen::Isometry3d>& target_graph,
                               Eigen::Isometry3d& delta) 
{
  std::vector<Eigen::Vector3d> target, query;
  target.reserve(graph.size());
  query.reserve(graph.size());

  // Vector of positions
  for ( size_t i = 0 ; i < target_graph.size(); ++i )
  {
    query.emplace_back(graph[i].translation());
    target.emplace_back(target_graph[i].translation());
  }

  // Computing mean
  Eigen::Vector3d mean_query = Eigen::Vector3d::Zero(), mean_target = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < query.size(); ++i) 
  {
    mean_query += query.at(i);
    mean_target += target.at(i);
  }

  const double inv_size = 1.0 / (double) query.size();
  mean_query *= inv_size;
  mean_target *= inv_size;
  Eigen::Matrix3d sigma = Eigen::Matrix3d::Zero();
  for (size_t i = 0; i < query.size(); ++i)
    sigma += (target.at(i) - mean_target) * (query.at(i) - mean_query).transpose();
  
  sigma *= inv_size;
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(sigma, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d W = Eigen::Matrix3d::Identity();
  if (svd.matrixU().determinant() * svd.matrixV().determinant() < 0)
    W(2, 2) = -1;

  Eigen::Matrix3d R = svd.matrixU() * W * svd.matrixV().transpose();
  
  fixRotation(R);
  delta.translation() = mean_target - R * mean_query;
  delta.linear()      = R;

  return;
}

void align(std::vector<Eigen::Isometry3d>& sol, const Eigen::Isometry3d& T) 
{
  for ( size_t i = 0 ; i < sol.size() ; ++i )
  {
    Eigen::Isometry3d estimate = sol[i];
    Eigen::Isometry3d aligned_estimate = T * estimate;
    sol[i] = aligned_estimate;  
  }

  return;
}



void evaluateAte(std::vector<Eigen::Isometry3d>& sol,
                 const std::vector<Eigen::Isometry3d>& gt,
                 double& ate_rotation,
                 double& ate_translation,
                 double& rmse_translation) 
{
  auto& gt_variables    = gt;
  auto& query_variables = sol;

  // tg check it the two graph are same number of variables
  if (gt_variables.size() != query_variables.size())
  {
    std::cout << "GT_SIZE = " << gt_variables.size() << std::endl;
    std::cout << "QUERY_SIZE = " << query_variables.size() << std::endl;
    throw std::runtime_error("|number of variables mismatch between query and ground truth");
  }

  Eigen::Isometry3d delta_alignment = Eigen::Isometry3d::Identity();
  computeAlignmentTransform(query_variables, gt_variables, delta_alignment);
  //delta_alignment = query_variables[0].inverse();

  //std::cout << "T = " << delta_alignment.translation() << std::endl;
  align(query_variables, delta_alignment);

  // tg counter for skipped variables (due to nans) and processed
  size_t skipped   = 0;
  size_t processed = 0;
  // tg for each ground truth variable
  for ( size_t id = 0; id < gt_variables.size(); ++id )
  {
    // Corresponding posese
    const Eigen::Isometry3d target = gt_variables[id];
    const Eigen::Isometry3d query  = query_variables[id];

    // extract rotation and translation
    const Eigen::Matrix3d Ri = target.linear();
    const Eigen::Matrix3d Rj = query.linear();
    const Eigen::Vector3d ti = target.translation();
    const Eigen::Vector3d tj = query.translation();

    // compute rotation error
    Eigen::Matrix3d R_error = Ri * Rj.transpose();
    
    // fix round off due to the multiplication
    fixRotation(R_error);

    // compute rotation error
    double angle_error           = getAngle(R_error);
    Eigen::Vector3d t_error_ATE = ti - R_error * tj;
    Eigen::Vector3d t_error     = ti - tj;

    // check for nans
    if (std::isnan(t_error_ATE.norm()) || std::isnan(angle_error)) 
    {
      std::cout << "WARNING, detected nan, skipping vertex [ " << id << " ]\n";
      ++skipped;
      continue;
    }

    // accumulate squared errors
    ate_translation  += t_error_ATE.transpose() * t_error_ATE;
    ate_rotation     += angle_error * angle_error;
    rmse_translation += t_error.transpose() * t_error;
    ++processed;

  }

  std::cout << "processed variables [ " << processed << "/" << gt_variables.size() << " ]\n";
  std::cout << "skipped variables [ " << skipped << "/" << gt_variables.size() << " ]\n";
  if (!processed)
    throw std::runtime_error("invalid types");

  // get number of variables processed
  const double inverse_num_variables = 1.0 / (double) (processed);
  // compute final ATE
  ate_translation  *= inverse_num_variables;
  ate_rotation     *= inverse_num_variables;
  rmse_translation *= inverse_num_variables;

  ate_translation    = std::sqrt(ate_translation);
  ate_rotation       = std::sqrt(ate_rotation);
  rmse_translation   = std::sqrt(rmse_translation);

  return;
}

void evaluateRpe(const std::vector<Eigen::Isometry3d>& sol,
                 const std::vector<Eigen::Isometry3d>& gt,
                 double& rpe_rotation,
                 double& rpe_translation) 
{
  auto& gt_variables    = gt;
  auto& query_variables = sol;

  // tg check it the two graph are same number of variables
  if (gt_variables.size() != query_variables.size())
  {
    std::cout << "GT_SIZE = " << gt_variables.size() << std::endl;
    std::cout << "QUERY_SIZE = " << query_variables.size() << std::endl;
    throw std::runtime_error("|number of variables mismatch between query and ground truth");
  }

  // tg counter for skipped variables (due to nans) and processed
  size_t skipped   = 0;
  size_t processed = 0;
  // tg for each ground truth variable
  for ( size_t id = 0; id < gt_variables.size() - 1; ++id )
  {
    // Corresponding posese
    const Eigen::Isometry3d target = gt_variables[id].inverse() * gt_variables[id + 1];
    const Eigen::Isometry3d query  = query_variables[id].inverse() * query_variables[id + 1];

    // extract rotation and translation
    const Eigen::Isometry3d delta = target.inverse() * query;

    // compute rotation error
    Eigen::Matrix3d R_error = delta.linear().matrix();
    
    // compute rotation error
    double angle_error           = getAngle(R_error);
    Eigen::Vector3d t_error = delta.translation();

    // check for nans
    if (std::isnan(t_error.norm()) || std::isnan(angle_error)) 
    {
      std::cout << "WARNING, detected nan, skipping vertex [ " << id << " ]\n";
      ++skipped;
      continue;
    }

    // accumulate squared errors
    rpe_translation  += t_error.transpose() * t_error;
    rpe_rotation     += angle_error * angle_error;
    ++processed;

  }

  std::cout << "processed variables [ " << processed << "/" << gt_variables.size() - 1 << " ]\n";
  std::cout << "skipped variables [ " << skipped << "/" << gt_variables.size() - 1<< " ]\n";
  if (!processed)
    throw std::runtime_error("invalid types");

  // compute final ATE
  const double inverse_num_variables = 1.f / (double) (processed);
  rpe_translation  *= inverse_num_variables;
  rpe_rotation     *= inverse_num_variables;

  rpe_translation    = std::sqrt(rpe_translation);
  rpe_rotation       = std::sqrt(rpe_rotation);

  return;
}


void readSolutionFile2D(std::vector<Eigen::Isometry3d>& poses, const std::string& path)
{
  std::ifstream in_data(path.c_str());
  if (!in_data )
    throw std::invalid_argument("Cannot find file : " + path);

  // Keep going until the file has been read
  double x, y, yaw;
  while ( !in_data.eof() )
  {
    in_data >> x >> y >> yaw;

    Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
    p.translation() = Eigen::Vector3d(x, y, 0.0);
    p.linear() = (Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
                  Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())).toRotationMatrix();

    poses.push_back(p);

    in_data.ignore(LINESIZE, '\n');
  }

  return;
}

void readSolutionFile3D(std::vector<Eigen::Isometry3d>& poses, const std::string& path)
{
  std::ifstream in_data(path.c_str());
  if (!in_data ) throw std::invalid_argument("Cannot find file : " + path);

  // Keep going until the file has been read
  double x, y, z, roll, pitch, yaw;
  while ( !in_data.eof() )
  {
    in_data >> x >> y >> z >> roll >> pitch >> yaw;

    Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
    p.translation() = Eigen::Vector3d(x, y, z);
    p.linear() = (Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
                  Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())).toRotationMatrix();

    poses.push_back(p);

    in_data.ignore(LINESIZE, '\n');
  }

  return;
}