
#include <Eigen/Dense>
#include <vector>

namespace P2ORI {

    void solve(
        const Eigen::Matrix3d R_ref[], // rotation of reference image (world-to-camera)
        const Eigen::Vector3d t_ref[], // translation of reference image (world-to-camera)
        const double s_ref[], // sine of feature orientation in reference image
        const double c_ref[], // cosine of feature orientation in reference image
        const double s_query[], // sine of feature orientation in query image
        const double c_query[], // cosine of feature orientation in query image
        const Eigen::Vector2d p_ref[], // point in reference image
        const double d[], // depth of point in reference image
        const Eigen::Vector3d n[], // normal of point in world coordinate system
        const Eigen::Vector2d p_query[], // point in query image
        std::vector<Eigen::Matrix3d> &Rsolns, std::vector<Eigen::Vector3d> &tsolns // query pose solutions in world coordinate system
    );

}
