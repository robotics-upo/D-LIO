#ifndef __DLL6DSOLVER_HPP__
#define __DLL6DSOLVER_HPP__

#include <vector>
#include <utility>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "ceres/cost_function_to_functor.h"
#include "ceres/autodiff_cost_function.h"
#include "glog/logging.h"
#include <dlio/tdf3d_64.hpp>
#include <pcl/point_cloud.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>

using ceres::CostFunction;
using ceres::SizedCostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using ceres::HuberLoss;

class DistanceFunction : public ceres::SizedCostFunction<1, 3> 
{
 public:

    DistanceFunction(TDF3D64 &grid)
      : _grid(grid), _res(grid.getResolution()),
        _penalty(35.0 * grid.getResolution()),  // max distance in metres
        _grid_min_x(grid.getMinX()), _grid_max_x(grid.getMaxX()),
        _grid_min_y(grid.getMinY()), _grid_max_y(grid.getMaxY()),
        _grid_min_z(grid.getMinZ()), _grid_max_z(grid.getMaxZ())
    {
    }

    virtual ~DistanceFunction(void) 
    {
    }

    virtual bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) const 
    {
        double x = parameters[0][0];
        double y = parameters[0][1];
        double z = parameters[0][2];

        if(_grid.isIntoGrid(x, y, z))
        {
            TrilinearParams p = _grid.computeDistInterpolation(x, y, z);
            // scale to metres (interpolation is in voxel units)
            residuals[0] = _res * (p.a0 + p.a1*x + p.a2*y + p.a3*z + p.a4*x*y + p.a5*x*z + p.a6*y*z + p.a7*x*y*z);
            if (jacobians != NULL && jacobians[0] != NULL) 
            {
                jacobians[0][0] = _res * (p.a1 + p.a5*z + p.a4*y + p.a7*z*y);
                jacobians[0][1] = _res * (p.a2 + p.a6*z + p.a4*x + p.a7*z*x);
                jacobians[0][2] = _res * (p.a3 + p.a5*x + p.a6*y + p.a7*x*y);
            }
        }
        else
        {
            // Out of grid: constant max-distance penalty, zero gradient.
            // Must stay at _penalty (not 0): a 0 reads as "on a surface" and would
            // reward pushing points into empty space.
            residuals[0] = _penalty;
            if (jacobians != NULL && jacobians[0] != NULL) 
            {
                jacobians[0][0] = 0.0;
                jacobians[0][1] = 0.0;
                jacobians[0][2] = 0.0;
            }
        }

        return true;
  }

  private:

    TDF3D64 &_grid;
    double _res;
    double _penalty;
    // kept for reference; unused by the zero-gradient else branch
    double _grid_min_x, _grid_max_x;
    double _grid_min_y, _grid_max_y;
    double _grid_min_z, _grid_max_z;
};

class DLL6DCostFunctor
{
 public:
    DLL6DCostFunctor(double px, double py, double pz, TDF3D64 &grid, double w = 1.0)
      : _px(px), _py(py), _pz(pz), _grid(grid), _w(w), _distanceFunctor(new DistanceFunction(grid))
    {
    }

    virtual ~DLL6DCostFunctor(void) 
    {
    }

    template <typename T>
    bool operator()(const T* t,const T* q, T* residuals) const
    {   
        const T tx = t[0];
        const T ty = t[1];
        const T tz = t[2];
        const T qx = q[1];
        const T qy = q[2];
        const T qz = q[3];
        const T qw = q[0];

        // transformed point
        T r00, r01, r02, r10, r11, r12, r20, r21, r22;
        T p[3], dist;
        r00 = 1.0-2.0*(qy*qy+qz*qz);    r01 = 2.0*(qx*qy-qz*qw);        r02 = 2.0*(qx*qz+qy*qw); 
        r10 = 2.0*(qx*qy+qz*qw);        r11 = 1.0-2.0*(qx*qx+qz*qz);    r12 = 2.0*(qy*qz-qx*qw);
        r20 = 2.0*(qx*qz-qy*qw);        r21 = 2.0*(qy*qz+qx*qw);        r22 = 1.0-2.0*(qx*qx+qy*qy);
        p[0] = r00*_px + r01*_py + r02*_pz + tx;
        p[1] = r10*_px + r11*_py + r12*_pz + ty;
        p[2] = r20*_px + r21*_py + r22*_pz + tz;

        // distance
        _distanceFunctor(p, &dist);

        // weighted residual
        residuals[0] = _w * dist;
        
        return true;
    }

  private:

    // Point to be evaluated
    double _px; 
    double _py; 
    double _pz; 

    // Constraint weighting
    double _w;

    // Distance grid
    TDF3D64 &_grid;

    // distance function differentiation
    ceres::CostFunctionToFunctor<1, 3> _distanceFunctor;
};

// QuatNormCostFunction removed: QuaternionManifold handles normalization.


class DLL6DSolver
{
  private:

    // Distance grid
    TDF3D64 &_grid;

    // Optimizer parameters
    int _max_num_iterations;
    int _max_threads;
    double _robusKernelScale;

  public:

    DLL6DSolver(TDF3D64 *grid) : _grid(*grid)
    {
        //google::InitGoogleLogging("DLL6DSolver");
        _max_num_iterations = 50;
        _max_threads = 5;

        // cauchy_scale = resolution * _robusKernelScale * (0.1 + 0.1*range).
        // Default 5 gives ~5 cm at close range; set robust_outlier_dist: 5.0 in YAML to match.
        _robusKernelScale = 5;
    }

    ~DLL6DSolver(void)
    {

    } 

    bool setMaxIterations(int n)
    {
        if(n>0)
        {
            _max_num_iterations = n;
            return true;
        } 
        else
            return false;
    }

    bool setMaxThreads(int n)
    {
        if(n>0)
        {
            _max_threads = n;
            return true;
        } 
        else
            return false;
    }

    bool setRobustKernelScale(double s)
    {
        // s > 0.0: residual is in metres, so a few cm is a legitimate scale.
        if(s > 0.0)
        {
            _robusKernelScale = s;
            return true;
        } 
        else
            return false;
    }

    

    bool solve(std::vector<pcl::PointXYZ> &p, double &tx, double &ty, double &tz, 
                                              double &roll, double &pitch, double &yaw,
                                              double *covMatrix = NULL,
                                              int    *numInliers = nullptr,
                                              double *finalCost  = nullptr)
    {
        // Initial solution
        bool converged = false;
        double t[3];
        t[0] = 0.0; t[1] = 0.0; t[2] = 0.0; 
        double q[4];		
        q[0] = 1.0; q[1] = 0.0;q[2] = 0.0; q[3] = 0.0; 

        // Build the problem.   
        Problem problem;
        problem.AddParameterBlock(t, 3);
        problem.AddParameterBlock(q, 4);

        ceres::Manifold* quaternion_manifold = new ceres::QuaternionManifold();
        problem.SetManifold(q, quaternion_manifold);

        // residual is in metres; scale the Cauchy threshold by resolution
        const double res = _grid.getResolution();

        int n=0;
        double nx, ny, nz;
        tf2::Quaternion q_pre;
        q_pre.setRPY(roll, pitch, yaw);
        tf2::Transform transform;
        transform.setRotation(q_pre);
        transform.setOrigin(tf2::Vector3(tx, ty, tz));
        
        // one cost function per cloud point
        for(unsigned int i=0; i<p.size(); i++)
        {
            // apply the initial transform
            tf2::Vector3 point(p[i].x, p[i].y, p[i].z);
            tf2::Vector3 transformed_point = transform * point;

            nx = transformed_point.x();
            ny = transformed_point.y();
            nz = transformed_point.z();

            // drop points outside the grid
            if(_grid.isIntoGrid(nx, ny, nz))
            {
                n++;

                // hit-counter weighting disabled (under test): uniform weight
                double confidence = 1.0;

                float d = sqrt(p[i].x*p[i].x + p[i].y*p[i].y + p[i].z*p[i].z);
                double cauchy_scale = res * _robusKernelScale * (0.1 + 0.1 * d);
                CostFunction* cost_function = new ceres::AutoDiffCostFunction<DLL6DCostFunctor, 1, 3, 4>( new DLL6DCostFunctor(nx, ny, nz, _grid, confidence) );
                problem.AddResidualBlock(cost_function,  new ceres::CauchyLoss(cauchy_scale) , t,q); 
            }
        }
        
        // Run the solver!
        Solver::Options options;
        options.minimizer_progress_to_stdout = false;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = _max_num_iterations;
        options.num_threads = _max_threads; 
        Solver::Summary summary;
        Solve(options, &problem, &summary);

        if(summary.termination_type == ceres::CONVERGENCE)
            converged = true;

        // quality scalars (already computed)
        if (numInliers != nullptr) *numInliers = n;
        if (finalCost  != nullptr) *finalCost  = summary.final_cost;

        // Direction-aware measurement covariance from the Hessian (only if covMatrix
        // != NULL). Output: row-major 6x6 [px,py,pz, rx,ry,rz] to match the ESKF.
        // Degenerate DoF (e.g. yaw on a flat floor) gets a large capped variance.
        if (covMatrix != nullptr)
        {
            const double res       = _grid.getResolution();
            const double floor_pos = (0.5*res)*(0.5*res);   // ~2.5 cm (half a voxel)
            const double floor_ori = 0.01*0.01;             // ~0.57 deg
            const double cap_pos   = 0.5*0.5;               // 0.5 m
            const double cap_ori   = 0.5*0.5;               // ~28 deg

            // large isotropic default: if not estimable, the filter ~ignores LiDAR
            double var[6] = { cap_pos, cap_pos, cap_pos, cap_ori, cap_ori, cap_ori };

            if (converged && n > 6)
            {
                ceres::Covariance::Options cov_opt;
                cov_opt.algorithm_type      = ceres::DENSE_SVD;  // tolerant to ill-conditioning
                cov_opt.apply_loss_function = true;              // account for the Cauchy weights
                cov_opt.num_threads         = _max_threads;
                ceres::Covariance covariance(cov_opt);

                std::vector<std::pair<const double*, const double*>> blocks;
                blocks.emplace_back(t, t);
                blocks.emplace_back(q, q);

                if (covariance.Compute(blocks, &problem))
                {
                    double Ctt[9], Cqq[9];
                    covariance.GetCovarianceBlockInTangentSpace(t, t, Ctt);
                    covariance.GetCovarianceBlockInTangentSpace(q, q, Cqq);

                    // a-posteriori factor: scales (J^T J)^-1 to the real fit quality
                    const double dof    = std::max(1, n - 6);
                    const double sigma2 = std::max(1e-9, 2.0 * summary.final_cost / dof);

                    // diagonals of each 3x3 (indices 0,4,8)
                    const double raw[6] = { Ctt[0], Ctt[4], Ctt[8],
                                            Cqq[0], Cqq[4], Cqq[8] };
                    for (int i = 0; i < 6; ++i)
                    {
                        const double lo = (i < 3) ? floor_pos : floor_ori;
                        const double hi = (i < 3) ? cap_pos   : cap_ori;
                        var[i] = std::min(hi, std::max(lo, sigma2 * raw[i]));
                    }
                }
            }

            Eigen::Matrix<double,6,6> C = Eigen::Matrix<double,6,6>::Zero();
            for (int i = 0; i < 6; ++i) C(i,i) = var[i];
            Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> mappedCovMatrix(covMatrix);
            mappedCovMatrix = C;
        }

        if (converged) {
            
            double d = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
            if (d < 1e-7) d = 1e-7;

            Eigen::Quaterniond q_new(q[0] / d, q[1] / d, q[2] / d, q[3] / d);
            tf2::Quaternion q_init;
            q_init.setRPY(roll, pitch, yaw);

            Eigen::Quaterniond q_eigen(q_init.w(), q_init.x(), q_init.y(), q_init.z());

            Eigen::Matrix4d T_initial = getTransformMatrix(tx, ty, tz, q_eigen.normalized());
            Eigen::Matrix4d T_solver = getTransformMatrix(t[0], t[1], t[2], q_new.normalized());

            Eigen::Matrix4d T_result = T_solver * T_initial;
            
            tx = T_result(0, 3);
            ty = T_result(1, 3);
            tz = T_result(2, 3);

            Eigen::Matrix3d R_result = T_result.block<3, 3>(0, 0);
            Eigen::Quaterniond q_final(R_result);
            q_final.normalize();

            tf2::Quaternion q_final_tf(q_final.x(), q_final.y(), q_final.z(), q_final.w());
            tf2::Matrix3x3 m_final(q_final_tf.normalize());
            m_final.getRPY(roll, pitch, yaw);

        }

        return converged; 
    }


    Eigen::Matrix4d getTransformMatrix(double x, double y, double z, const Eigen::Quaterniond& q) {

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        Eigen::Matrix3d R_eigen = q.toRotationMatrix();

        T.block<3,3>(0,0) = R_eigen;
        T(0, 3) = x;
        T(1, 3) = y;
        T(2, 3) = z;

        return T;
    }


};

#endif