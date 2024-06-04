
#include <Eigen/Dense>
#include <vector>

namespace UP1SIFT {

    void solve(
        const Eigen::Matrix3d &R_ref, // rotation of reference image
        const Eigen::Vector3d &t_ref, // translation of reference image
        const double s_ref, //sin of angle in reference image
        const double c_ref, //cos of angle in reference image
        const double s_query, //sin of angle in query image
        const double c_query, //cos of angle in query image
        const double q, //ratio of scales (scale in query image / scale in reference image)
        const Eigen::Vector2d &p_ref, //point observation in reference image
        const double d, //depth of point in reference image
        const Eigen::Vector3d &n, //surface normal in world coordinates
        const Eigen::Vector2d &p_query, //point observation in query image
        const Eigen::Matrix3d &Rxz, //query image gravity rotation such that R=Ry*Rxz
        std::vector<Eigen::Matrix3d> &Rsolns, std::vector<Eigen::Vector3d> &tsolns // query pose solutions in world coordinate system
    );
}
