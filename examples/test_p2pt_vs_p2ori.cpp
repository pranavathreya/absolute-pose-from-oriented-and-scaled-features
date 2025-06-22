#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>
#include <algorithm>

#include <p2ori/p2ori.h>
#include <up1sift/up1sift.h>
#include <problem_generator/problem_generator.h>
#include <openMVG/multiview/solver_resection_p2pt_fabbri.hpp>

using namespace openMVG::euclidean_resection;
using namespace openMVG;
using namespace problem_generator;

int main( int argc, char **argv )
{
    // Set up problem infrastructure
    srand(1234);
    ProblemGeneratorConfig config; // can set focal length and noise parameters here
    ProblemGenerator generator(config);
    const int nprob = 1;
    double mean_p2ori_rot_err = 0;
    double mean_p2ori_pos_err = 0;
    double mean_p2pt_rot_err = 0;
    double mean_p2pt_pos_err = 0;
    problem_generator::Problem problem = generator.make_random_problem(2);

    std::cout << "Testing P2Pt vs P2ORI solvers\n";

    /* Initialize Fabbri's solver */
    // TODO: Fill in the values for all the vars
    Mat bearing_vectors(3, 2);   // Pair of 2D points
    Mat tangent_vectors(3, 2);   // Pair of 2D tangents of the above 2D points
    Mat X(3, 2);                 // Pair of 3D points
    Mat T(3, 2);                 // Pair of 3D tangents of the above 3D points
    P2PtSolver_Fabbri p2pt_solver;

    /* Initialize P2ORI Solver */
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
    std::vector<Mat34> p2pt_solns;
    
    // Run the p2pt and p2ori solvers
    P2ORI::solve(R_ref, t_ref, s_ref, c_ref, s_query, c_query, p_ref, d, n, p_query, p2ori_Rsolns, p2ori_tsolns);
    p2pt_solver.Solve(bearing_vectors, tangent_vectors, X, T, &p2pt_solns);

    return 0;
}
