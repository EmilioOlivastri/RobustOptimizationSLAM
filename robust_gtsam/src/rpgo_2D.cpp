#include "utils.hpp"

#include <stdlib.h>
#include <memory>
#include <string>

#include "KimeraRPGO/RobustSolver.h"
#include "KimeraRPGO/SolverParams.h"
#include "KimeraRPGO/Logger.h"
#include "KimeraRPGO/utils/GeometryUtils.h"
#include "KimeraRPGO/utils/TypeUtils.h"

using namespace KimeraRPGO;
using namespace gtsam;
using namespace std;

int main(int argc, char* argv[]) 
{

  string cfg_file = "";
  int maxIterations = 2000; // default
  string g2oFile = ""; // default

  // Parse user's inputs
  string output_file = "res.txt";
  if (argc > 1) g2oFile = argv[1]; 
  if (argc > 2) cfg_file = argv[2];
  if (argc > 3) output_file = argv[3]; // Number of iterations

  Config cfg;
  readConfig(cfg_file, cfg);

  typedef Pose2 PoseType;
  vector<PoseType> poses;
  vector<NonlinearFactor::shared_ptr> loops;

  // reading file and creating factor graph
  ifstream in_data(g2oFile.c_str());
  if (!in_data )
    throw invalid_argument("Cannot find file : " + g2oFile);

  RobustSolverParams params;
  NonlinearFactorGraph::shared_ptr graph;
  Values::shared_ptr initial;
  bool is3D = false;
  boost::tie(graph, initial) = readG2o(g2oFile, is3D);
  Values new_init = *initial;

  int dof = 3;
  double th = 0.5 * Chi2inv(0.9, dof);
  params.setPcm2DParams(th, th, Verbosity::QUIET);
  params.setLmDiagonalDamping(is3D);
  params.setGncInlierCostThresholdsAtProbability(0.9);
  unique_ptr<RobustSolver> pgo = KimeraRPGO::make_unique<RobustSolver>(params);  
  for (const auto& factor : *graph) 
  {
    // convert to between factor
    if (new_init.exists(factor->front())) 
    {
      BetweenFactor<PoseType>& btwn =
          *boost::dynamic_pointer_cast<BetweenFactor<PoseType>>(factor);
      new_init.update(
          factor->back(),
          new_init.at<PoseType>(factor->front()).compose(btwn.measured()));
    }

    int delta = factor->front() - factor->back();
    if ( abs(delta) > 1 ) loops.push_back(factor);    
  }

  std::cout << "Adding prior on pose 0 " << std::endl;
  NonlinearFactorGraph nfg = *graph;
  addPrior2D(nfg);
  
  cout << "Starting optimization" << endl;
  chrono::steady_clock::time_point begin = chrono::steady_clock::now();
  pgo->update(nfg, new_init);
  chrono::steady_clock::time_point end = chrono::steady_clock::now();
  Values result = pgo->calculateBestEstimate();
  double dt = chrono::duration_cast<chrono::microseconds>(end - begin).count() / 1000000.0;

  double barcSq = 0.5 * Chi2inv(0.99, dof);
  int tp  = 0; int tn = 0;
  int fp  = 0; int fn = 0;
  for ( int idx = 0; idx < cfg.canonic_inliers; ++idx)
  {
    const BetweenFactor<PoseType>& btwn = *boost::dynamic_pointer_cast<BetweenFactor<PoseType>>(loops[idx]);
    Values tmp;
    tmp.insert(loops[idx]->back(), result.at<PoseType>(loops[idx]->back()));
    tmp.insert(loops[idx]->front(), result.at<PoseType>(loops[idx]->front()));
    double v = btwn.error(tmp);
    if ( v < barcSq ) ++tp;
    else ++fn;
  }

  for ( int idx = cfg.canonic_inliers; idx < loops.size(); ++idx)
  {
    const BetweenFactor<PoseType>& btwn = *boost::dynamic_pointer_cast<BetweenFactor<PoseType>>(loops[idx]);
    Values tmp;
    tmp.insert(loops[idx]->back(), result.at<PoseType>(loops[idx]->back()));
    tmp.insert(loops[idx]->front(), result.at<PoseType>(loops[idx]->front()));
    double v = btwn.error(tmp);
    if ( v < barcSq ) ++fp;
    else ++tn;
  }
  float precision = tp / (float)(tp + fp);
  float recall    = tp / (float)(tp + fn); 

  cout << "Optimization complete in " << dt << " [s]" << endl;
  cout << "Precision  = " << precision << endl;
  cout << "Recall = " << recall << endl;
  cout << "initial error=" << nfg.error(*initial)<< endl;
  cout << "final error=" << nfg.error(result)<< endl;


  store2D(output_file, result);

  ofstream outfile;
  string out2 = output_file.substr(0, output_file.size() - 3) + "PR";
  outfile.open(out2.c_str());
  outfile << precision << " " << recall << endl;
  outfile << dt << endl;
  outfile.close();

  return 0;
}