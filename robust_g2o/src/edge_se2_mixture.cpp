#include "edge_se2_mixture.hpp"

using namespace g2o;
// ================================================
EdgeSE2Mixture::EdgeSE2Mixture() : EdgeSE2()
{
  nullHypothesisMoreLikely = false;
  determinant_set = false;
}

// ================================================
bool EdgeSE2Mixture::read(std::istream& is)
{
    
    return true;
}
// ================================================
bool EdgeSE2Mixture::write(std::ostream& os) const
{
    return os.good();
}

// ================================================
void EdgeSE2Mixture::linearizeOplus()
{
  EdgeSE2::linearizeOplus();
  if (nullHypothesisMoreLikely) {
    _jacobianOplusXi *= weight;
    _jacobianOplusXj *= weight;
  }
  return;
}

// ================================================
void EdgeSE2Mixture::computeError()
{
  // determine the likelihood for constraint and null hypothesis
  determinant_c = determinant_set ? determinant_c : information_constraint.inverse().determinant();
  determinant_null = determinant_set ? determinant_null : information_nullHypothesis.inverse().determinant();  

  determinant_set = true;

  double mahal_constraint = _error.transpose() * information_constraint * _error;
  double mahal_nullHypothesis  = _error.transpose() * (information_nullHypothesis) * _error;

  double neglogprob_null = -log(nu_nullHypothesis) + 
                           0.5 * log(determinant_null) + 
                           0.5 * mahal_nullHypothesis;

  double neglogprob_cost = -log(nu_constraint) + 
                           0.5 * log(determinant_c) + 
                           0.5 * mahal_constraint;

  // if the nullHypothesis is more likely ...
  if (neglogprob_cost > neglogprob_null) {
    _information = information_nullHypothesis;
    nullHypothesisMoreLikely = true;
    //std::cout << "Using the null Hypothesis" << std::endl;
  }
  else {
    _information = information_constraint;
    nullHypothesisMoreLikely = false;
  }

  // calculate the error for this constraint
  EdgeSE2::computeError();

  return;
}



bool EdgeSE2Mixture::isOutlier()
{
  return nullHypothesisMoreLikely;
}