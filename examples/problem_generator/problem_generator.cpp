
#include <Eigen/Geometry>

#include <problem_generator/random.h>
#include <problem_generator/problem_generator.h>

#include <affine2sift/affine2sift.h>

namespace problem_generator
{
    Eigen::Matrix3d decomposeRotation( const Eigen::Matrix3d &R )
    {
        // decompose s.t. R = Ry * Rxz
        // returns Rxz
        const Eigen::Vector3d Y = R.row(1);
        const Eigen::Vector3d up(0,1,0);
        if ( Y.dot(up) == 1 ) return Eigen::Matrix3d::Identity();
        const Eigen::Vector3d axis = Y.cross(up);
        const Eigen::Vector3d rxz = axis/axis.norm();
        const double thetaxz = acos(Y.dot(up));
        return Eigen::AngleAxisd(thetaxz,rxz).toRotationMatrix();

        // Ry = R*Rxz.transpose();
    }

    Eigen::Matrix3d gravityToRotation( const Eigen::Vector3d &gravity )
    {
        const Eigen::Vector3d up(0,1,0);
        if ( gravity.dot(up) == 1 ) return Eigen::Matrix3d::Identity();
        const Eigen::Vector3d axis = gravity.cross(up);
        const Eigen::Vector3d rxz = axis/axis.norm();
        const double thetaxz = acos(gravity.dot(up));
        return Eigen::AngleAxisd(thetaxz,rxz).toRotationMatrix();
    }

    static Eigen::Matrix3d calculateHomography( const Eigen::Vector3d &X, const Eigen::Vector3d &n, const Eigen::Matrix3d &R, const Eigen::Vector3d &t )
    {
        double d = -n.dot(X);
        
        Eigen::Vector3d v = -n/d;
        Eigen::Matrix3d H = R + t * v.transpose();
        
        return H;
    }

    static Eigen::Matrix2d affineFromHomography( const Eigen::Matrix3d &H, const Eigen::Vector2d &x, const Eigen::Vector2d &y )
    {
        double s = H(2,0)*x(0) + H(2,1)*x(1) + H(2,2);
        
        Eigen::Matrix2d A;
        A << (H(0,0)-H(2,0)*y(0))/s, (H(0,1)-H(2,1)*y(0))/s,
             (H(1,0)-H(2,0)*y(1))/s, (H(1,1)-H(2,1)*y(1))/s;
        
        return A;
    }

    static Eigen::Matrix3d lookAt( const Eigen::Vector3d &eye, const Eigen::Vector3d &target, const Eigen::Vector3d &up )
    {
        Eigen::Vector3d zaxis = (target - eye);
        zaxis.normalize();
        
        Eigen::Vector3d xaxis = up.cross(zaxis);
        xaxis.normalize();
        
        Eigen::Vector3d yaxis = zaxis.cross(xaxis);
        yaxis.normalize();
        
        Eigen::Matrix3d R;
        R.col(0) = xaxis;
        R.col(1) = yaxis;
        R.col(2) = zaxis;
        
        return R.transpose();
    }

    Problem ProblemGenerator::make_random_problem(int ncorr)
    {
        Problem problem;
        problem.correspondences.resize(ncorr);

        // iterate until valid configuration found
        Eigen::Vector3d target = generator.rand3() * -0.5;

        Eigen::Vector3d c_query = generator.rand_unit_vector() * (generator.rand() * 0.5 + 1.5);
        problem.R = lookAt( c_query, target, Eigen::Vector3d(0,-1,0) );
        problem.t = -problem.R * c_query;
        problem.Rxz = decomposeRotation(problem.R);

        // add noise to Rxz
        if ( config.gravity_noise > 0 )
        {
            Eigen::Vector3d gravity = problem.Rxz.transpose() * Eigen::Vector3d(0,1,0);
            Eigen::Vector3d noiser = generator.rand_unit_vector();
            double rand_angle = generator.randn() * config.gravity_noise * M_PI / 180.;
            Eigen::Matrix3d randR = Eigen::AngleAxisd(rand_angle,noiser).toRotationMatrix();
            gravity = randR * gravity;
            problem.Rxz = gravityToRotation(gravity);
        }

        for ( int i = 0; i < ncorr; i++ )
        {
            while ( true )
            {
                Eigen::Vector3d c_ref = generator.rand_unit_vector() * (generator.rand() * 0.5 + 1.5);
                if ( config.zero_translation ) {
                    c_ref = c_query + generator.rand_unit_vector()*.00001;
                }

                problem.correspondences[i].R_ref = lookAt( c_ref, target, Eigen::Vector3d(0,-1,0) );
                if ( config.zero_rotation ) problem.correspondences[i].R_ref = problem.R;

                problem.correspondences[i].t_ref = -problem.correspondences[i].R_ref * c_ref;
                
                Eigen::Matrix3d R_rel = problem.R * problem.correspondences[i].R_ref.transpose();
                Eigen::Vector3d t_rel = problem.R * (c_ref-c_query);
            
                if ( Eigen::AngleAxisd(R_rel).angle() >= M_PI ) continue;

                // make 3D point
                problem.correspondences[i].X = generator.randn3();
                    
                // compute depth in reference camera
                Eigen::Vector3d X_ref = problem.correspondences[i].R_ref * problem.correspondences[i].X + problem.correspondences[i].t_ref;
                problem.correspondences[i].d = X_ref(2);
                
                // check if behind reference camera
                if ( problem.correspondences[i].d < 0 ) continue;
                
                // project to reference camera
                problem.correspondences[i].p_ref = X_ref.head(2)/X_ref(2);
                    
                // sample random normal vector
                problem.correspondences[i].n = generator.rand_unit_vector();
                
                // rotate normal vector to reference frame
                Eigen::Vector3d n_ref = problem.correspondences[i].R_ref * problem.correspondences[i].n;
                
                // project to query camera
                Eigen::Vector3d X_query = problem.R * problem.correspondences[i].X + problem.t;
                problem.correspondences[i].p_query = X_query.head(2)/X_query(2);

                // check if behind query camera
                if ( X_query(2) < 0 ) continue; 
                    
                // calculate homography
                Eigen::Matrix3d H = calculateHomography(X_ref,n_ref,R_rel,t_rel);
                
                // calculate affine from homography
                problem.correspondences[i].A = affineFromHomography(H,problem.correspondences[i].p_ref,problem.correspondences[i].p_query);
                
                // check if determinant is positive
                if ( problem.correspondences[i].A.determinant() < 0 ) continue;
                
                // convert affine to SIFT
                affine2sift(problem.correspondences[i].A, problem.correspondences[i].s_ref, problem.correspondences[i].c_ref, problem.correspondences[i].s_query, problem.correspondences[i].c_query, problem.correspondences[i].q);

                // solve for w
                // qu * qv = q
                double qu = problem.correspondences[i].q;
                double qv = problem.correspondences[i].q;
                double w;
                if ( abs(problem.correspondences[i].c_query*problem.correspondences[i].s_ref) > abs(problem.correspondences[i].c_ref*problem.correspondences[i].c_query) )
                    w = (problem.correspondences[i].c_ref*problem.correspondences[i].c_query*qu - problem.correspondences[i].A(0,0) + qv*problem.correspondences[i].s_ref*problem.correspondences[i].s_query)/(problem.correspondences[i].c_query*problem.correspondences[i].s_ref);
                else
                    w = (problem.correspondences[i].A(0,1) - problem.correspondences[i].c_query*qu*problem.correspondences[i].s_ref + problem.correspondences[i].c_ref*qv*problem.correspondences[i].s_query)/(problem.correspondences[i].c_ref*problem.correspondences[i].c_query);
                
                // add noise to affine
                if ( config.affine_noise > 0 )
                {
                    problem.correspondences[i].A(0,0) += (problem.correspondences[i].A(0,0)*config.affine_noise) * generator.randn();
                    problem.correspondences[i].A(0,1) += (problem.correspondences[i].A(0,1)*config.affine_noise) * generator.randn();
                    problem.correspondences[i].A(1,0) += (problem.correspondences[i].A(1,0)*config.affine_noise) * generator.randn();
                    problem.correspondences[i].A(1,1) += (problem.correspondences[i].A(1,1)*config.affine_noise) * generator.randn();
                }
                
                // add noise to points
                if ( config.point_noise > 0 ) 
                {
                    problem.correspondences[i].p_ref += generator.randn2() * config.point_noise / config.focal;
                    problem.correspondences[i].p_query += generator.randn2() * config.point_noise / config.focal;
                }

                // add noise to angles
                if ( config.angle_noise > 0 )
                {
                    double theta2 = atan2(problem.correspondences[i].s_query,problem.correspondences[i].c_query);

                    double rand_angle2 = generator.randn() * config.angle_noise * M_PI / 180.;
                    
                    theta2 += rand_angle2;
                        
                    problem.correspondences[i].c_query = cos(theta2);
                    problem.correspondences[i].s_query = sin(theta2);
                }

                // add noise to normal vector
                if ( config.normal_noise > 0 )
                {
                    Eigen::Vector3d noiser = generator.rand_unit_vector();
                    double rand_angle = generator.randn() * config.normal_noise * M_PI / 180.;
                    Eigen::Matrix3d randR = Eigen::AngleAxisd(rand_angle,noiser).toRotationMatrix();
                    problem.correspondences[i].n = randR * problem.correspondences[i].n;
                }
                
                // add noise to scale
                if ( config.scale_noise > 0 )
                {
                    double log_q = log(problem.correspondences[i].q);
                    double log_noise = generator.randn() * config.scale_noise;
                    log_q += log_noise;
                    problem.correspondences[i].q = exp(log_q);
                    qv = problem.correspondences[i].q;
                }
                
                // re-make affine matrix if necessary
                if ( config.angle_noise > 0 || config.scale_noise > 0 )
                {
                    while ( true )
                    {
                        Eigen::Matrix2d R2, U, R1t;
                        R2 << problem.correspondences[i].c_query, -problem.correspondences[i].s_query, problem.correspondences[i].s_query, problem.correspondences[i].c_query;
                        U << qu, w, 0, qv;
                        R1t << problem.correspondences[i].c_ref, problem.correspondences[i].s_ref, -problem.correspondences[i].s_ref, problem.correspondences[i].c_ref;
                        problem.correspondences[i].A = R2 * U * R1t;
                        if ( problem.correspondences[i].A.determinant() > 0 ) break;
                    }
                }
            
                break;
            }
        }

        return problem;
    }
}

