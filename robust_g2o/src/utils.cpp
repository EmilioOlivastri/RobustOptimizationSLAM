#include "utils.hpp"

using namespace std;
using namespace g2o;
using namespace Eigen;


/*-------------------------------------------------------------------*/

void betterInitializationPoint(SparseOptimizer& optimizer)
{
    // Iterating trough the edges
    for ( auto it_e = optimizer.edges().begin(); it_e != optimizer.edges().end(); ++it_e )
    {
        auto edge_odom = dynamic_cast<EdgeSE2*>(*it_e);
        if ( edge_odom != nullptr )
        {
            auto v_fn = dynamic_cast<VertexSE2*>(edge_odom->vertices()[1]);
            auto v_st = dynamic_cast<VertexSE2*>(edge_odom->vertices()[0]);

            int id_st = v_st->id();
            int id_fn = v_fn->id();

            auto est = id_fn - id_st == 1 ? v_st->estimate() * edge_odom->measurement() : v_fn->estimate();
            v_fn->setEstimate(est);
        }
    }

    return;
}


/*-------------------------------------------------------------------*/

template <class T, class EDGE, class VERTEX>
void setProblem(const string& problem_file, 
                SparseOptimizer& optimizer,
                vector<T>& init_poses,
                vector<VERTEX*>& v_poses)
{
    optimizer.setVerbose(false);
    
    // allocate the solver
    OptimizationAlgorithmProperty solverProperty;
    optimizer.setAlgorithm(OptimizationAlgorithmFactory::instance()->construct("dl_var", solverProperty));
    
    // Loading the g2o file
    ifstream ifs(problem_file.c_str());
    if (!ifs) 
    {
        cerr << "unable to open " << problem_file << endl;
        return;
    }
    optimizer.load(ifs);
    //odometryInitialization<EDGE, VERTEX>(optimizer);
    init_poses.resize(optimizer.vertices().size());
    v_poses.resize(optimizer.vertices().size());
    for ( auto it = optimizer.vertices().begin() ; it != optimizer.vertices().end() ; ++it )
    {
        auto v1 = dynamic_cast<VERTEX*>(it->second);
        v_poses[v1->id()] = v1;
        init_poses[v1->id()]= v1->estimate();
    }

    return;
}

template void setProblem<SE2, EdgeSE2, VertexSE2>(const string& problem_file, 
                                                  SparseOptimizer& optimizer,
                                                  vector<SE2>& init_poses,
                                                  vector<VertexSE2*>& v_poses);
template void setProblem<Isometry3d, EdgeSE3, VertexSE3>(const string& problem_file, 
                                                         SparseOptimizer& optimizer,
                                                         vector<Isometry3d>& init_poses,
                                                         vector<VertexSE3*>& v_poses);

/*-------------------------------------------------------------------*/

template <class EDGE, class VERTEX>
void odometryInitialization(SparseOptimizer& optimizer)
{
    // Iterating trough the edges
    for ( auto it_e = optimizer.edges().begin(); it_e != optimizer.edges().end(); ++it_e )
    {
        auto edge_odom = dynamic_cast<EDGE*>(*it_e);
        if ( edge_odom != nullptr )
        {
            auto v_fn = dynamic_cast<VERTEX*>(edge_odom->vertices()[1]);
            auto v_st = dynamic_cast<VERTEX*>(edge_odom->vertices()[0]);

            int id_st = v_st->id();
            int id_fn = v_fn->id();

            auto est = id_fn - id_st == 1 ? v_st->estimate() * edge_odom->measurement(): v_fn->estimate();
            v_fn->setEstimate(est);
        }
    }

    return;
}

template void odometryInitialization<EdgeSE2, VertexSE2>(SparseOptimizer& optimizer);
template void odometryInitialization<EdgeSE3, VertexSE3>(SparseOptimizer& optimizer);
/*-------------------------------------------------------------------*/

template <class T>
void readSolutionFile(vector<T>& poses, const string& path)
{
    const int LINESIZE = 81920;

    ifstream in_data(path.c_str());
    if (!in_data )
        throw invalid_argument("Cannot find file : " + path);

    // Keep going until the file has been read
    while ( !in_data.eof() )
    {
        T p = T::Identity();
        readLine(in_data, p);
        poses.push_back(p);

        in_data.ignore(LINESIZE, '\n');
    }

    return;
}

template void readSolutionFile<Isometry2d>(vector<Isometry2d>& poses, const string& path);
template void readSolutionFile<Isometry3d>(vector<Isometry3d>& poses, const string& path);

/*-------------------------------------------------------------------*/

void writeVertex(ofstream& out_data, VertexSE2* v)
{
    out_data << v->estimate()[0] << " " 
             << v->estimate()[1] << " " 
             << v->estimate()[2] << endl;
    return;
}


void writeVertex(ofstream& out_data, VertexSE3* v)
{
    Isometry3d pose = v->estimate();
    Quaterniond q(pose.linear());
    Vector3d eul = pose.linear().eulerAngles(0, 1, 2);
    Vector3d t = pose.translation();

    out_data << t[0] << " " << t[1] << " " << t[2] << " "
             << eul[0] << " " <<eul[1] << " " << eul[2] << endl;

    return;
}

/*-------------------------------------------------------------------*/

void readLine(ifstream& in_data, Isometry2d& pose)
{
    double x, y, yaw;
    in_data >> x >> y >> yaw;

    pose = Isometry2d::Identity();
    pose.translation() = Vector2d(x, y);
    pose.linear() = Eigen::Rotation2D<double>(yaw).toRotationMatrix();

    return;
}

void readLine(ifstream& in_data, Isometry3d& pose)
{
    double x, y, z, qx, qy, qz, qw;
    in_data >> x >> y >> z >> qx >> qy >> qz >> qw;

    pose = Isometry3d::Identity();
    pose.translation() = Vector3d(x, y, z);
    pose.linear() = Quaterniond(qw, qx, qy, qz).toRotationMatrix();

    return;
}


/*-------------------------------------------------------------------*/



void readConfig(const std::string& cfg_filepath, Config& out_cfg)
{
    const YAML::Node config = YAML::LoadFile(cfg_filepath);

    // Filter parameters
    out_cfg.dataset = config["dataset"].as<std::string>();
    out_cfg.output = config["output"].as<std::string>();
    out_cfg.canonic_inliers = config["canonic_inliers"].as<int>();
    out_cfg.maxiters = config["max_iters"].as<int>();
    out_cfg.inlier_th = config["inlier_th"].as<double>();
    
    // Switchable variables
    out_cfg.switch_prior = config["switch_prior"].as<double>();

    // MaxMix parameters
    out_cfg.maxmix_weight = config["maxmix_weight"].as<double>();
    out_cfg.nu_constraints = config["nu_constraint"].as<double>();
    out_cfg.nu_nullHypothesis = config["nu_nullHypothesis"].as<double>();


    return;
}