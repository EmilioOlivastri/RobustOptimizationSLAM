#include "utils.hpp"

using namespace std;
using namespace gtsam;

int main(int argc, char **argv) 
{
  string cfg_file = "";
  int maxIterations = 1000; // default
  string g2oFile = ""; // default

  // Parse user's inputs
  string output_file = "res.txt";
  if (argc > 1) g2oFile = argv[1]; 
  if (argc > 2) cfg_file = argv[2];
  if (argc > 3) output_file = argv[3]; // Number of iterations

  Config cfg;
  readConfig(cfg_file, cfg);

  /**
  bool is3D = true;
  typedef Pose3 PoseType;
  /**/

  bool is3D = false;
  typedef Pose2 PoseType;
  /**/

  vector<PoseType> poses;
  vector<NonlinearFactor::shared_ptr> loops;

  // reading file and creating factor graph
  ifstream in_data(g2oFile.c_str());
  if (!in_data ) throw invalid_argument("Cannot find file : " + g2oFile);

  NonlinearFactorGraph::shared_ptr graph;
  Values::shared_ptr initial;
  boost::tie(graph, initial) = readG2o(g2oFile, is3D);
  Values new_init = *initial;

  int edge_counter = 0;
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
    if ( std::abs(delta) > 1 ) loops.push_back(factor);    
  }
  
  int inliers = cfg.canonic_inliers;
 
  // Add prior on the pose having index (key) = 0
  NonlinearFactorGraph nfg = *graph;
  std::cout << "Adding prior on pose 0 " << std::endl;
  if (is3D) addPrior3D(nfg);
  else addPrior2D(nfg);

  LevenbergMarquardtParams lmParams;
  GncParams<LevenbergMarquardtParams> gncParams(lmParams);
  auto gnc = GncOptimizer<GncParams<LevenbergMarquardtParams>>(nfg, new_init, gncParams);
  std::cout << "Optimizing the factor graph" << std::endl;

  chrono::steady_clock::time_point begin = chrono::steady_clock::now();
  Values result = gnc.optimize();
  chrono::steady_clock::time_point end = chrono::steady_clock::now();
  chrono::microseconds delta_time = chrono::duration_cast<chrono::microseconds>(end - begin);

  int dof = is3D ? 6 : 3;
  double barcSq = 0.5 * Chi2inv(0.9, dof);
  int tp  = 0; int tn = 0;
  int fp  = 0; int fn = 0;
  for ( int idx = 0; idx < inliers; ++idx)
  {
    const BetweenFactor<PoseType>& btwn = *boost::dynamic_pointer_cast<BetweenFactor<PoseType>>(loops[idx]);
    Values tmp;
    tmp.insert(loops[idx]->back(), result.at<PoseType>(loops[idx]->back()));
    tmp.insert(loops[idx]->front(), result.at<PoseType>(loops[idx]->front()));
    double v = btwn.error(tmp);
    if ( v < barcSq ) ++tp;
    else ++fn;
  }

  for ( int idx = inliers; idx < loops.size(); ++idx)
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
  float dt = delta_time.count() / 1000000.0;

  std::cout << "Optimization complete in " << dt << " [s]" << std::endl;
  std::cout << "Precision  = " << precision << std::endl;
  std::cout << "Recall = " << recall << std::endl;
  std::cout << "initial error=" <<graph->error(*initial)<< std::endl;
  std::cout << "final error=" <<graph->error(result)<< std::endl;
  

  if (is3D) store3D(output_file, result);
  else store2D(output_file, result);

  ofstream outfile;
  string out_pr = output_file.substr(0, output_file.size() - 3) + "PR";
  outfile.open(out_pr.c_str());
  outfile << precision << " " << recall << endl;
  outfile << dt << endl;
  outfile.close();


  return 0;
}
