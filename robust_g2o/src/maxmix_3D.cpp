#include "utils.hpp"
#include "maxmixConstraints/include/edge_se3_mixture.hpp"

using namespace std;
using namespace g2o;

// we use the 2D and 3D SLAM types here
G2O_USE_TYPE_GROUP(slam3d);
G2O_USE_OPTIMIZATION_LIBRARY(eigen);

void initializeMixture(EdgeSE3Mixture* e_maxmix, EdgeSE3* e_odom);

int main(int argc, char** argv) 
{
  // Command line parsing
  string cfg_file;
  CommandArgs arg;
  
  arg.param("cfg", cfg_file, "",
            "Configuration File(.yaml)");
  arg.parseArgs(argc, argv);

  // Storing initial guess and vertices of optimization
  vector<Eigen::Isometry3d> init_poses;
  vector<VertexSE3*> v_poses;

  Config cfg;
  readConfig(cfg_file, cfg);
  string input_dataset = cfg.dataset;
  int maxIterations = cfg.maxiters;
  double inlier_th = cfg.inlier_th;
  int inliers = cfg.canonic_inliers;
  double maxmix_weight = cfg.maxmix_weight;
  double nu_constraints = cfg.nu_constraints;
  double nu_nullHypothesis = cfg.nu_nullHypothesis;

  // create the optimizer to load the data and carry out the optimization
  SparseOptimizer optimizer;
  setProblem<Eigen::Isometry3d, EdgeSE3, VertexSE3>(input_dataset, optimizer, init_poses, v_poses);
  
  /**/
  std::vector<EdgeSE3*> e2remove;
  std::vector<EdgeSE3Mixture*> e2add;

  for ( auto it_e = optimizer.edges().begin(); it_e != optimizer.edges().end(); ++it_e )
  {
    EdgeSE3* edge_odom = dynamic_cast<EdgeSE3*>(*it_e);
    /* Odom edge loop closure */ 
    if ( edge_odom != nullptr && abs(edge_odom->vertices()[1]->id() - edge_odom->vertices()[0]->id()) > 1  )
    {
      /**/      
      EdgeSE3Mixture* e_maxmix = new EdgeSE3Mixture();
      e_maxmix->setVertex(0, edge_odom->vertices()[0]);
      e_maxmix->setVertex(1, edge_odom->vertices()[1]);
      initializeMixture(e_maxmix, edge_odom);
      e_maxmix->setInformation(edge_odom->information());
      e_maxmix->weight = maxmix_weight;
      e_maxmix->information_constraint = edge_odom->information();
      e_maxmix->nu_constraint = nu_constraints;
      e_maxmix->nu_nullHypothesis = nu_nullHypothesis;

      e2add.push_back(e_maxmix);
      e2remove.push_back(edge_odom);
    }

  }
  /**/
  for ( int i = 0 ; i < e2remove.size() ; ++i )
  {
    optimizer.addEdge(e2add[i]);
    optimizer.removeEdge(e2remove[i]);
  }

  std::cout << "Starting optimization : " << std::endl;
  optimizer.initializeOptimization();
  chrono::steady_clock::time_point begin = chrono::steady_clock::now();
  optimizer.optimize(maxIterations);
  chrono::steady_clock::time_point end = chrono::steady_clock::now();
  chrono::microseconds delta_time = chrono::duration_cast<chrono::microseconds>(end - begin);

  int tp  = 0; int tn = 0; int fp  = 0; int fn = 0; 
  for ( size_t idx = 0; idx < cfg.canonic_inliers; ++idx)
  {
    if ( !e2add[idx]->isOutlier() ) ++tp;
    else ++fn;
  }


  for ( size_t idx = cfg.canonic_inliers; idx < e2add.size(); ++idx)
  {
    if ( e2add[idx]->isOutlier() ) ++tn;
    else ++fp;
  }
  
  std::cout << "Optimtization Concluded!" << std::endl;

  ofstream outfile;
  string output_file_trj = cfg.output;
  outfile.open(output_file_trj.c_str()); 
  for (size_t it = 0; it < optimizer.vertices().size(); ++it )
  {
      VertexSE3* v = dynamic_cast<VertexSE3*>(optimizer.vertex(it));
      writeVertex(outfile, v);
  }
  outfile.close();

  float precision = tp + fp > 0 ? tp / (float)(tp + fp) : 0.0;
  float recall    = tp + fn > 0 ? tp / (float)(tp + fn) : 0.0; 
  float dt = delta_time.count() / 1000000.0;

  std::cout << "Loops size = " << e2add.size() << std::endl;
  std::cout << "Canonic Inliers = " << cfg.canonic_inliers << std::endl;
  std::cout << "Prec = " << precision << std::endl;
  std::cout << "Rec = " << recall << std::endl;
  std::cout << "TP = " << tp << std::endl;
  std::cout << "TN = " << tn << std::endl;
  std::cout << "FP = " << fp << std::endl;
  std::cout << "FN = " << fn << std::endl;


  string output_file_pr = output_file_trj.substr(0, output_file_trj.size() - 3) + "PR";
  outfile.open(output_file_pr.c_str());
  outfile << precision << " " << recall << endl;
  outfile << dt << endl;
  outfile.close();

  return 0;
}

void initializeMixture(EdgeSE3Mixture* e_maxmix, EdgeSE3* e_odom)
{
  typedef Eigen::Matrix<double, 6, 6> Matrix6d;

  double m[7];
  e_odom->getMeasurementData(m);
  Eigen::Map<const Vector7> v(m);
  e_maxmix->setMeasurement(internal::fromVectorQT(v));
  e_maxmix->information_nullHypothesis = Matrix6d::Identity();

  return;
}