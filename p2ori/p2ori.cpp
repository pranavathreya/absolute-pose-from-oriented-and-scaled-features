#include <Eigen/Dense>
#include <p2ori/p2ori.h>
#include <PoseLib/misc/re3q3.h>

namespace P2ORI
{
    static
    Eigen::Matrix<double,3,12> build_constraints(
        const Eigen::Matrix3d &R_ref,
        const Eigen::Vector3d &t_ref,
        const double s_ref,
        const double c_ref,
        const double s_query,
        const double c_query,
        const Eigen::Vector2d &p_ref,
        const double d,
        const Eigen::Vector3d &n,
        const Eigen::Vector2d &p_query )
{
        const Eigen::Vector3d n_ref = R_ref * n;
        Eigen::Matrix<double,3,12> M;
        M << -1, 0, p_query(0), R_ref(0,0)*t_ref(0) - R_ref(2,0)*d + R_ref(1,0)*t_ref(1) + R_ref(2,0)*t_ref(2) - R_ref(0,0)*d*p_ref(0) - R_ref(1,0)*d*p_ref(1), 0, R_ref(2,0)*d*p_query(0) - R_ref(0,0)*t_ref(0)*p_query(0) - R_ref(1,0)*t_ref(1)*p_query(0) - R_ref(2,0)*t_ref(2)*p_query(0) + R_ref(0,0)*d*p_ref(0)*p_query(0) + R_ref(1,0)*d*p_ref(1)*p_query(0), R_ref(0,1)*t_ref(0) - R_ref(2,1)*d + R_ref(1,1)*t_ref(1) + R_ref(2,1)*t_ref(2) - R_ref(0,1)*d*p_ref(0) - R_ref(1,1)*d*p_ref(1), 0, R_ref(2,1)*d*p_query(0) - R_ref(0,1)*t_ref(0)*p_query(0) - R_ref(1,1)*t_ref(1)*p_query(0) - R_ref(2,1)*t_ref(2)*p_query(0) + R_ref(0,1)*d*p_ref(0)*p_query(0) + R_ref(1,1)*d*p_ref(1)*p_query(0), R_ref(0,2)*t_ref(0) - R_ref(2,2)*d + R_ref(1,2)*t_ref(1) + R_ref(2,2)*t_ref(2) - R_ref(0,2)*d*p_ref(0) - R_ref(1,2)*d*p_ref(1), 0, R_ref(2,2)*d*p_query(0) - R_ref(0,2)*t_ref(0)*p_query(0) - R_ref(1,2)*t_ref(1)*p_query(0) - R_ref(2,2)*t_ref(2)*p_query(0) + R_ref(0,2)*d*p_ref(0)*p_query(0) + R_ref(1,2)*d*p_ref(1)*p_query(0),
        0, -1, p_query(1), 0, R_ref(0,0)*t_ref(0) - R_ref(2,0)*d + R_ref(1,0)*t_ref(1) + R_ref(2,0)*t_ref(2) - R_ref(0,0)*d*p_ref(0) - R_ref(1,0)*d*p_ref(1), R_ref(2,0)*d*p_query(1) - R_ref(0,0)*t_ref(0)*p_query(1) - R_ref(1,0)*t_ref(1)*p_query(1) - R_ref(2,0)*t_ref(2)*p_query(1) + R_ref(0,0)*d*p_ref(0)*p_query(1) + R_ref(1,0)*d*p_ref(1)*p_query(1), 0, R_ref(0,1)*t_ref(0) - R_ref(2,1)*d + R_ref(1,1)*t_ref(1) + R_ref(2,1)*t_ref(2) - R_ref(0,1)*d*p_ref(0) - R_ref(1,1)*d*p_ref(1), R_ref(2,1)*d*p_query(1) - R_ref(0,1)*t_ref(0)*p_query(1) - R_ref(1,1)*t_ref(1)*p_query(1) - R_ref(2,1)*t_ref(2)*p_query(1) + R_ref(0,1)*d*p_ref(0)*p_query(1) + R_ref(1,1)*d*p_ref(1)*p_query(1), 0, R_ref(0,2)*t_ref(0) - R_ref(2,2)*d + R_ref(1,2)*t_ref(1) + R_ref(2,2)*t_ref(2) - R_ref(0,2)*d*p_ref(0) - R_ref(1,2)*d*p_ref(1), R_ref(2,2)*d*p_query(1) - R_ref(0,2)*t_ref(0)*p_query(1) - R_ref(1,2)*t_ref(1)*p_query(1) - R_ref(2,2)*t_ref(2)*p_query(1) + R_ref(0,2)*d*p_ref(0)*p_query(1) + R_ref(1,2)*d*p_ref(1)*p_query(1),
        0, 0, 0, R_ref(0,0)*c_ref*d*n_ref(2)*s_query - R_ref(2,0)*c_ref*d*n_ref(0)*s_query + R_ref(1,0)*d*n_ref(2)*s_ref*s_query - R_ref(2,0)*d*n_ref(1)*s_ref*s_query + R_ref(0,0)*c_ref*d*n_ref(1)*s_query*p_ref(1) - R_ref(1,0)*c_ref*d*n_ref(0)*s_query*p_ref(1) - R_ref(0,0)*d*n_ref(1)*s_ref*s_query*p_ref(0) + R_ref(1,0)*d*n_ref(0)*s_ref*s_query*p_ref(0), R_ref(2,0)*c_ref*c_query*d*n_ref(0) - R_ref(0,0)*c_ref*c_query*d*n_ref(2) - R_ref(1,0)*c_query*d*n_ref(2)*s_ref + R_ref(2,0)*c_query*d*n_ref(1)*s_ref - R_ref(0,0)*c_ref*c_query*d*n_ref(1)*p_ref(1) + R_ref(1,0)*c_ref*c_query*d*n_ref(0)*p_ref(1) + R_ref(0,0)*c_query*d*n_ref(1)*s_ref*p_ref(0) - R_ref(1,0)*c_query*d*n_ref(0)*s_ref*p_ref(0), R_ref(0,0)*c_ref*c_query*d*n_ref(2)*p_query(1) - R_ref(2,0)*c_ref*c_query*d*n_ref(0)*p_query(1) - R_ref(0,0)*c_ref*d*n_ref(2)*s_query*p_query(0) + R_ref(1,0)*c_query*d*n_ref(2)*s_ref*p_query(1) + R_ref(2,0)*c_ref*d*n_ref(0)*s_query*p_query(0) - R_ref(2,0)*c_query*d*n_ref(1)*s_ref*p_query(1) - R_ref(1,0)*d*n_ref(2)*s_ref*s_query*p_query(0) + R_ref(2,0)*d*n_ref(1)*s_ref*s_query*p_query(0) + R_ref(0,0)*c_ref*c_query*d*n_ref(1)*p_ref(1)*p_query(1) - R_ref(1,0)*c_ref*c_query*d*n_ref(0)*p_ref(1)*p_query(1) - R_ref(0,0)*c_ref*d*n_ref(1)*s_query*p_ref(1)*p_query(0) - R_ref(0,0)*c_query*d*n_ref(1)*s_ref*p_ref(0)*p_query(1) + R_ref(1,0)*c_ref*d*n_ref(0)*s_query*p_ref(1)*p_query(0) + R_ref(1,0)*c_query*d*n_ref(0)*s_ref*p_ref(0)*p_query(1) + R_ref(0,0)*d*n_ref(1)*s_ref*s_query*p_ref(0)*p_query(0) - R_ref(1,0)*d*n_ref(0)*s_ref*s_query*p_ref(0)*p_query(0), R_ref(0,1)*c_ref*d*n_ref(2)*s_query - R_ref(2,1)*c_ref*d*n_ref(0)*s_query + R_ref(1,1)*d*n_ref(2)*s_ref*s_query - R_ref(2,1)*d*n_ref(1)*s_ref*s_query + R_ref(0,1)*c_ref*d*n_ref(1)*s_query*p_ref(1) - R_ref(1,1)*c_ref*d*n_ref(0)*s_query*p_ref(1) - R_ref(0,1)*d*n_ref(1)*s_ref*s_query*p_ref(0) + R_ref(1,1)*d*n_ref(0)*s_ref*s_query*p_ref(0), R_ref(2,1)*c_ref*c_query*d*n_ref(0) - R_ref(0,1)*c_ref*c_query*d*n_ref(2) - R_ref(1,1)*c_query*d*n_ref(2)*s_ref + R_ref(2,1)*c_query*d*n_ref(1)*s_ref - R_ref(0,1)*c_ref*c_query*d*n_ref(1)*p_ref(1) + R_ref(1,1)*c_ref*c_query*d*n_ref(0)*p_ref(1) + R_ref(0,1)*c_query*d*n_ref(1)*s_ref*p_ref(0) - R_ref(1,1)*c_query*d*n_ref(0)*s_ref*p_ref(0), R_ref(0,1)*c_ref*c_query*d*n_ref(2)*p_query(1) - R_ref(2,1)*c_ref*c_query*d*n_ref(0)*p_query(1) - R_ref(0,1)*c_ref*d*n_ref(2)*s_query*p_query(0) + R_ref(1,1)*c_query*d*n_ref(2)*s_ref*p_query(1) + R_ref(2,1)*c_ref*d*n_ref(0)*s_query*p_query(0) - R_ref(2,1)*c_query*d*n_ref(1)*s_ref*p_query(1) - R_ref(1,1)*d*n_ref(2)*s_ref*s_query*p_query(0) + R_ref(2,1)*d*n_ref(1)*s_ref*s_query*p_query(0) + R_ref(0,1)*c_ref*c_query*d*n_ref(1)*p_ref(1)*p_query(1) - R_ref(1,1)*c_ref*c_query*d*n_ref(0)*p_ref(1)*p_query(1) - R_ref(0,1)*c_ref*d*n_ref(1)*s_query*p_ref(1)*p_query(0) - R_ref(0,1)*c_query*d*n_ref(1)*s_ref*p_ref(0)*p_query(1) + R_ref(1,1)*c_ref*d*n_ref(0)*s_query*p_ref(1)*p_query(0) + R_ref(1,1)*c_query*d*n_ref(0)*s_ref*p_ref(0)*p_query(1) + R_ref(0,1)*d*n_ref(1)*s_ref*s_query*p_ref(0)*p_query(0) - R_ref(1,1)*d*n_ref(0)*s_ref*s_query*p_ref(0)*p_query(0), R_ref(0,2)*c_ref*d*n_ref(2)*s_query - R_ref(2,2)*c_ref*d*n_ref(0)*s_query + R_ref(1,2)*d*n_ref(2)*s_ref*s_query - R_ref(2,2)*d*n_ref(1)*s_ref*s_query + R_ref(0,2)*c_ref*d*n_ref(1)*s_query*p_ref(1) - R_ref(1,2)*c_ref*d*n_ref(0)*s_query*p_ref(1) - R_ref(0,2)*d*n_ref(1)*s_ref*s_query*p_ref(0) + R_ref(1,2)*d*n_ref(0)*s_ref*s_query*p_ref(0), R_ref(2,2)*c_ref*c_query*d*n_ref(0) - R_ref(0,2)*c_ref*c_query*d*n_ref(2) - R_ref(1,2)*c_query*d*n_ref(2)*s_ref + R_ref(2,2)*c_query*d*n_ref(1)*s_ref - R_ref(0,2)*c_ref*c_query*d*n_ref(1)*p_ref(1) + R_ref(1,2)*c_ref*c_query*d*n_ref(0)*p_ref(1) + R_ref(0,2)*c_query*d*n_ref(1)*s_ref*p_ref(0) - R_ref(1,2)*c_query*d*n_ref(0)*s_ref*p_ref(0), R_ref(0,2)*c_ref*c_query*d*n_ref(2)*p_query(1) - R_ref(2,2)*c_ref*c_query*d*n_ref(0)*p_query(1) - R_ref(0,2)*c_ref*d*n_ref(2)*s_query*p_query(0) + R_ref(1,2)*c_query*d*n_ref(2)*s_ref*p_query(1) + R_ref(2,2)*c_ref*d*n_ref(0)*s_query*p_query(0) - R_ref(2,2)*c_query*d*n_ref(1)*s_ref*p_query(1) - R_ref(1,2)*d*n_ref(2)*s_ref*s_query*p_query(0) + R_ref(2,2)*d*n_ref(1)*s_ref*s_query*p_query(0) + R_ref(0,2)*c_ref*c_query*d*n_ref(1)*p_ref(1)*p_query(1) - R_ref(1,2)*c_ref*c_query*d*n_ref(0)*p_ref(1)*p_query(1) - R_ref(0,2)*c_ref*d*n_ref(1)*s_query*p_ref(1)*p_query(0) - R_ref(0,2)*c_query*d*n_ref(1)*s_ref*p_ref(0)*p_query(1) + R_ref(1,2)*c_ref*d*n_ref(0)*s_query*p_ref(1)*p_query(0) + R_ref(1,2)*c_query*d*n_ref(0)*s_ref*p_ref(0)*p_query(1) + R_ref(0,2)*d*n_ref(1)*s_ref*s_query*p_ref(0)*p_query(0) - R_ref(1,2)*d*n_ref(0)*s_ref*s_query*p_ref(0)*p_query(0);
        return M;
    }

    void solve(
        const Eigen::Matrix3d R_ref[],
        const Eigen::Vector3d t_ref[],
        const double s_ref[],
        const double c_ref[],
        const double s_querp_query[],
        const double c_querp_query[],
        const Eigen::Vector2d p_ref[],
        const double d[],
        const Eigen::Vector3d n[],
        const Eigen::Vector2d p_query[],
        std::vector<Eigen::Matrix3d> &Rsolns, std::vector<Eigen::Vector3d> &tsolns
    )
    {
        const Eigen::Matrix<double,3,12> M1 = build_constraints(R_ref[0],t_ref[0],s_ref[0],c_ref[0],s_querp_query[0],c_querp_query[0],p_ref[0],d[0],n[0],p_query[0]);
        const Eigen::Matrix<double,3,12> M2 = build_constraints(R_ref[1],t_ref[1],s_ref[1],c_ref[1],s_querp_query[1],c_querp_query[1],p_ref[1],d[1],n[1],p_query[1]);
        Eigen::Matrix<double,6,12> M;
        M << M1, M2;

        // G-J elimination
        const Eigen::Matrix<double,6,9> G = M.block<6,6>(0,0).partialPivLu().solve(M.block<6,9>(0,3));
        const Eigen::Matrix<double,3,9> T = G.block<3,9>(0,0);
        const Eigen::Matrix<double,3,9> C = G.block<3,9>(3,0);
        
        Eigen::Matrix<double,3,8> cayley_solutions;
        Eigen::Matrix<double,3,10> C3Q3;
        poselib::re3q3::rotation_to_3q3(C,&C3Q3);
        int nsolns = poselib::re3q3::re3q3(C3Q3, &cayley_solutions);

        for ( int i = 0; i < nsolns; i++ )
        {
            Eigen::Matrix3d R;
            poselib::re3q3::cayley_param(cayley_solutions.col(i),&R);
            
            Eigen::Matrix<double,9,1> X;
            X << R(0,0),R(1,0),R(2,0), R(0,1),R(1,1),R(2,1), R(0,2),R(1,2),R(2,2);
            Eigen::Vector3d t = -T*X;
            
            Rsolns.push_back(R);
            tsolns.push_back(t);
        }
    }

}
