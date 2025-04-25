#ifndef __DLL6DSOLVER_HPP__
#define __DLL6DSOLVER_HPP__

#include <vector>
#include <utility>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "ceres/cost_function_to_functor.h"
#include "ceres/autodiff_cost_function.h"
#include "glog/logging.h"
#include <dlo3d/df3d.hpp>
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
using ceres::Covariance;

class DistanceFunction : public ceres::SizedCostFunction<1, 3> 
{
 public:

    DistanceFunction(DF3D &grid)
      : _grid(grid)
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
            residuals[0] = p.a0 + p.a1*x + p.a2*y + p.a3*z + p.a4*x*y + p.a5*x*z + p.a6*y*z + p.a7*x*y*z;
            if (jacobians != NULL && jacobians[0] != NULL) 
            {
                jacobians[0][0] = p.a1 + p.a5*z + p.a4*y + p.a7*z*y;
                jacobians[0][1] = p.a2 + p.a6*z + p.a4*x + p.a7*z*x;
                jacobians[0][2] = p.a3 + p.a5*x + p.a6*y + p.a7*x*y;
            }
        }
        else
        {
            residuals[0] = 0;
            if (jacobians != NULL && jacobians[0] != NULL) 
            {
                jacobians[0][0] = 0;
                jacobians[0][1] = 0;
                jacobians[0][2] = 0;
            }
        }

        return true;
  }

  private:

    DF3D &_grid;
};

class DLL6DCostFunctor
{
 public:
    DLL6DCostFunctor(double px, double py, double pz, DF3D &grid, double w = 1.0)
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

        // Compute transformed point
        T r00, r01, r02, r10, r11, r12, r20, r21, r22;
        T p[3], dist;
        r00 = 1.0-2.0*(qy*qy+qz*qz);    r01 = 2.0*(qx*qy-qz*qw);        r02 = 2.0*(qx*qz+qy*qw); 
        r10 = 2.0*(qx*qy+qz*qw);        r11 = 1.0-2.0*(qx*qx+qz*qz);    r12 = 2.0*(qy*qz-qx*qw);
        r20 = 2.0*(qx*qz-qy*qw);        r21 = 2.0*(qy*qz+qx*qw);        r22 = 1.0-2.0*(qx*qx+qy*qy);
        p[0] = r00*_px + r01*_py + r02*_pz + tx;
        p[1] = r10*_px + r11*_py + r12*_pz + ty;
        p[2] = r20*_px + r21*_py + r22*_pz + tz;

        // Compute distance
        _distanceFunctor(p, &dist);

        // Compute weigthed residual
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
    DF3D &_grid;

    // Distance funtion diferenciation
    ceres::CostFunctionToFunctor<1, 3> _distanceFunctor;
};

class QuatNormCostFunction
  : public SizedCostFunction<1 /* number of residuals */,
                             7 /* size of first parameter */> 
{
 public:
    QuatNormCostFunction(double w = 1.0)
      : _w(w)
    {
    }

    virtual ~QuatNormCostFunction(void) 
    {

    }

    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const 
    {
        double qx = parameters[0][3];
        double qy = parameters[0][4];
        double qz = parameters[0][5];
        double qw = parameters[0][6];
        double mod = sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
       
        residuals[0] =  _w*( mod - 1);
        if (jacobians != NULL && jacobians[0] != NULL) 
        {
            jacobians[0][0] = 0;
            jacobians[0][1] = 0;
            jacobians[0][2] = 0;
            jacobians[0][3] = qx*_w/mod;
            jacobians[0][4] = qy*_w/mod;
            jacobians[0][5] = qz*_w/mod;
            jacobians[0][6] = qw*_w/mod;
        }
        
        return true;
    }

  private:

    // Constraint weighting
    double _w;
};


class DLL6DSolver
{
  private:

    // Distance grid
    DF3D &_grid;

    // Optimizer parameters
    int _max_num_iterations;
    int _max_threads;
    double _robusKernelScale;

  public:

    DLL6DSolver(DF3D *grid) : _grid(*grid)
    {
        //google::InitGoogleLogging("DLL6DSolver");
        _max_num_iterations = 50;
        _max_threads = 5;
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
        if(s > 0.9)
        {
            _robusKernelScale = s;
            return true;
        } 
        else
            return false;
    }

    

    bool solve(std::vector<pcl::PointXYZ> &p, double &tx, double &ty, double &tz, 
                                              double &roll, double &pitch, double &yaw,
                                              double *covMatrix = NULL)
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

        int n=0;
        double nx, ny, nz;
        tf2::Quaternion q_pre;
        q_pre.setRPY(roll, pitch, yaw);
        tf2::Transform transform;
        transform.setRotation(q_pre);
        transform.setOrigin(tf2::Vector3(tx, ty, tz));
        
        // Set up a cost funtion per point into the cloud
        for(unsigned int i=0; i<p.size(); i++)
        {
            // Compute position of the point into the grid according to initial transform
            tf2::Vector3 point(p[i].x, p[i].y, p[i].z);
            tf2::Vector3 transformed_point = transform * point;

            nx = transformed_point.x();
            ny = transformed_point.y();
            nz = transformed_point.z();

            // Outlier rejection. Points out of the grid are discarded
            if(_grid.isIntoGrid(nx, ny, nz))
            {
                n++;
                float d = sqrt(p[i].x*p[i].x + p[i].y*p[i].y + p[i].z*p[i].z);
                CostFunction* cost_function = new ceres::AutoDiffCostFunction<DLL6DCostFunctor, 1, 3, 4>( new DLL6DCostFunctor(nx, ny, nz, _grid) );
                problem.AddResidualBlock(cost_function,  new ceres::CauchyLoss( _robusKernelScale * (0.1+0.1*sqrt(p[i].x*p[i].x + p[i].y*p[i].y + p[i].z*p[i].z)) ) , t,q); 
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

        if (converged) {
            
            double d = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
            if (d < 1e-7) d = 1e-7;

            Eigen::Quaterniond q_new(q[0] / d, q[1] / d, q[2] / d, q[3] / d);
            Eigen::Vector3d rpy = q_new.normalized().toRotationMatrix().eulerAngles(2,1,0);
            double roll_out = rpy(0);
            double pitch_out = rpy(1);
            double yaw_out = rpy(2);
            
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
