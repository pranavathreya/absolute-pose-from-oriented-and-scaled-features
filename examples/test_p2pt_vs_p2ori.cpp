#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>
#include <algorithm>

#include <p2ori/p2ori.h>
#include <up1sift/up1sift.h>
#include <problem_generator/problem_generator.h>

/* This include statement causes the segmentation fault in generator.make_random_problem*/
#include <openMVG/multiview/solver_resection_p2pt_fabbri.hpp>

//using namespace openMVG::euclidean_resection;
//using namespace openMVG;
using namespace problem_generator;

int main( int argc, char **argv )
{
    /* Code to instantiate Fabbri's solver */
	// // Prepare your data as columns (Eigen matrices or compatible Mat type)
    // Mat bearing_vectors(3, 2);   // 3x2 matrix
    // Mat tangent_vectors(3, 2);   // 3x2 matrix
    // Mat X(3, 2);                 // 3D points, 3x2 matrix
    // Mat T(3, 2);                 // 3D tangents, 3x2 matrix

    // // ... Fill in the matrices with your data ...

    // std::vector<Mat34> models;
    // P2PtSolver_Fabbri solver;
    // solver.Solve(bearing_vectors, tangent_vectors, X, T, &models);

    srand(1234);

    ProblemGeneratorConfig config; // can set focal length and noise parameters here
    ProblemGenerator generator(config);
    
    const int nprob = 10000;

    double mean_p2ori_rot_err = 0;
    double mean_p2ori_pos_err = 0;
    double mean_up1sift_rot_err = 0;
    double mean_up1sift_pos_err = 0;

    
    for ( int j = 0; j < nprob; j++ )
    {
       problem_generator::Problem problem = generator.make_random_problem(2);
       
       Eigen::Matrix3d R_ref[2];
       Eigen::Vector3d t_ref[2];
       double s_ref[2];
       double c_ref[2];
       double s_query[2];
       double c_query[2];
       double q[2];
       Eigen::Vector2d p_ref[2];
       double d[2];
       Eigen::Vector3d n[2];
       Eigen::Vector2d p_query[2];

       for ( int i = 0; i < 2; i++ )
       {
           R_ref[i] = problem.correspondences[i].R_ref;
           t_ref[i] = problem.correspondences[i].t_ref;
           s_ref[i] = problem.correspondences[i].s_ref;
           c_ref[i] = problem.correspondences[i].c_ref;
           s_query[i] = problem.correspondences[i].s_query;
           c_query[i] = problem.correspondences[i].c_query;
           q[i] = problem.correspondences[i].q;
           p_ref[i] = problem.correspondences[i].p_ref;
           d[i] = problem.correspondences[i].d;
           n[i] = problem.correspondences[i].n;
           p_query[i] = problem.correspondences[i].p_query;
       }

       std::vector<Eigen::Matrix3d> p2ori_Rsolns;
       std::vector<Eigen::Vector3d> p2ori_tsolns;
       P2ORI::solve(R_ref,t_ref,s_ref,c_ref,s_query,c_query,p_ref,d,n,p_query,p2ori_Rsolns,p2ori_tsolns);

       std::vector<Eigen::Matrix3d> up1sift_Rsolns;
       std::vector<Eigen::Vector3d> up1sift_tsolns;
       UP1SIFT::solve(R_ref[0],t_ref[0],s_ref[0],c_ref[0],s_query[0],c_query[0],q[0],p_ref[0],d[0],n[0],p_query[0],problem.Rxz,up1sift_Rsolns,up1sift_tsolns);
       
       std::vector<double> p2ori_rot_err;
       std::vector<double> p2ori_pos_err;
       std::vector<double> up1sift_rot_err;
       std::vector<double> up1sift_pos_err;
       
       for ( int i = 0; i < p2ori_Rsolns.size(); i++ )
       {
           p2ori_rot_err.push_back(problem.rot_error( p2ori_Rsolns[i] ));
           p2ori_pos_err.push_back(problem.pos_error( p2ori_Rsolns[i], p2ori_tsolns[i] ));
       }
       for ( int i = 0; i < up1sift_Rsolns.size(); i++ )
       {
           up1sift_rot_err.push_back(problem.rot_error( up1sift_Rsolns[i] ));
           up1sift_pos_err.push_back(problem.pos_error( up1sift_Rsolns[i], up1sift_tsolns[i] ));
       }
       
       if ( p2ori_Rsolns.empty() ) std::cout << "num P2ORI: " << p2ori_Rsolns.size() << "\n";
       if ( up1sift_Rsolns.empty() ) std::cout << "num UP1SIFT: " << up1sift_Rsolns.size() << "\n";
       
       mean_p2ori_rot_err += *std::min_element(p2ori_rot_err.begin(),p2ori_rot_err.end());
       mean_p2ori_pos_err += *std::min_element(p2ori_pos_err.begin(),p2ori_pos_err.end());

       mean_up1sift_rot_err += *std::min_element(up1sift_rot_err.begin(),up1sift_rot_err.end());
       mean_up1sift_pos_err += *std::min_element(up1sift_pos_err.begin(),up1sift_pos_err.end());
    }

    mean_p2ori_rot_err /= nprob;
    mean_p2ori_pos_err /= nprob;

    mean_up1sift_rot_err /= nprob;
    mean_up1sift_pos_err /= nprob;

    std::cout << "tested " << nprob << " problems.\n";
    std::cout << "average P2ORI rotation error: " << mean_p2ori_rot_err << "\n";
    std::cout << "average P2ORI position error: " << mean_p2ori_pos_err << "\n";
    std::cout << "average UP2SIFT rotation error: " << mean_up1sift_rot_err << "\n";
    std::cout << "average UP2SIFT position error: " << mean_up1sift_pos_err << "\n";

    return 0;
}
