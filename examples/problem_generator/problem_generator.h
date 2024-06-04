
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>

#include <problem_generator/random.h>

namespace problem_generator
{
    Eigen::Matrix3d decomposeRotation( const Eigen::Matrix3d &R );
    
    struct Correspondence
    {
        Eigen::Matrix3d R_ref; // reference rotation
        Eigen::Vector3d t_ref; // reference translation
        Eigen::Matrix2d A; // affine
        double s_ref; // sin of angle in reference view
        double c_ref; // cos of angle in reference view
        double s_query; // sin of angle in query view
        double c_query; // cos of angle in query view
        double q; // ratio of scales
        Eigen::Vector3d X; // 3D point in world coordinates
        Eigen::Vector2d p_ref; // 2D point in reference image
        double d; // depth in reference image
        Eigen::Vector3d n; // normal in world coordinates
        Eigen::Vector2d p_query; // 2D point in query image

        void randomize( RandomGenerator &random_generator )
        {
            R_ref = Eigen::Quaterniond::UnitRandom().toRotationMatrix();
            t_ref = random_generator.rand_unit_vector();
            s_ref = random_generator.rand();
            c_ref = random_generator.rand();
            s_query = random_generator.rand();
            c_query = random_generator.rand();
            q = exp(random_generator.randn());
            X = random_generator.randn3();
            d = random_generator.randn();
            p_ref = random_generator.randn2();
            n = random_generator.rand_unit_vector();
            A << random_generator.randn(), random_generator.randn(), random_generator.randn(), random_generator.randn();
            p_query = random_generator.randn2();
        }
    };    

    struct Problem
    {
        std::vector<Correspondence> correspondences;
        Eigen::Matrix3d Rxz; // R = Ry*Rxz
        Eigen::Matrix3d R; 
        Eigen::Vector3d t;
    
        double rot_error( const Eigen::Matrix3d &Rsoln ) const
        {
            const Eigen::Quaterniond quat(R);
            const Eigen::Quaterniond quatsoln(Rsoln);
            return quat.angularDistance(quatsoln);
        }
        double pos_error( const Eigen::Matrix3d &Rsoln, const Eigen::Vector3d &tsoln ) const
        {
            const Eigen::Vector3d c(-R.transpose()*t);
            const Eigen::Vector3d csoln(-Rsoln.transpose()*tsoln);
            return (c-csoln).norm();
        }
    };

    struct ProblemGeneratorConfig
    {
        double focal;
        double gravity_noise;
        double affine_noise;
        double point_noise;
        double angle_noise;
        double scale_noise;
        double normal_noise;
        bool zero_rotation;
        bool zero_translation;
        ProblemGeneratorConfig() : 
            focal(1),
            gravity_noise(0),
            affine_noise(0),
            point_noise(0),
            angle_noise(0),
            scale_noise(0),
            normal_noise(0),
            zero_rotation(false),
            zero_translation(false)
            { }
    };

    class ProblemGenerator
    {
        RandomGenerator generator;
        ProblemGeneratorConfig config;
    public:
        ProblemGenerator( ProblemGeneratorConfig _config = ProblemGeneratorConfig() ) : config(_config) { }
        Problem make_random_problem(int ncorr);
    };
}
