#ifndef __ESKF_HPP__
#define __ESKF_HPP__

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <vector>
#include <cmath>
#include <fstream>
#include <iomanip>

#define YEL "\033[33m"
#define RST "\033[0m"

/**
 * Error-State Kalman Filter (ESKF) for IMU + 6DoF pose updates (e.g. LiDAR).
 *
 * Nominal state:  p, v, q, ba, bg   (q is a unit quaternion)
 * Error state:    [dp, dv, dtheta, dba, dbg] in R^15
 *
 * Orientation lives on SO(3), stored as a unit quaternion. The error dtheta is
 * a right perturbation in the tangent space: q_true = q_nom * Exp(dtheta).
 * Exp/Log go through Eigen::AngleAxis. After each update the error is injected
 * (q <- q * Exp(dtheta)), dtheta is reset to zero, and the reset Jacobian is
 * applied to P.
 *
 * IMU convention: right-handed body frame; accel reports specific force
 * (~(0,0,+g) at rest, in m/s^2); gyro in rad/s. World gravity is (0,0,-g) so
 * that R*acc + g_world = 0 at rest.
 */
class ESKF
{
public:

    ESKF()
    {
        init = false;
        sec  = 0;

        odom_file.open("predict_data.csv", std::ios::out);
        if (!odom_file.is_open()) {
            throw std::runtime_error("Unable to open odom file.");
        }
        odom_file << "time,field.header.seq,field.header.stamp,"
                  << "field.pose.pose.position.x,field.pose.pose.position.y,field.pose.pose.position.z,"
                  << "field.pose.pose.orientation.x,field.pose.pose.orientation.y,field.pose.pose.orientation.z,field.pose.pose.orientation.w,"
                  << "field.twist.twist.linear.x,field.twist.twist.linear.y,field.twist.twist.linear.z,"
                  << "field.twist.twist.angular.x,field.twist.twist.angular.y,field.twist.twist.angular.z,"
                  << "gbx,gby,gbz,abx,aby,abz\n";
    }

    ~ESKF()
    {
        if (odom_file.is_open()) {
            odom_file.close();
        }
    }

    // --- Setup ---
    // _T              nominal IMU period [s] (fallback dt)
    // _calibTime      static-alignment window [s]; 0 disables alignment
    // _gyr_dev        gyro white noise PSD [rad/s/sqrt(Hz)]
    // _gyr_rw_dev     gyro bias random walk PSD [rad/s^2/sqrt(Hz)]
    // _acc_dev        accel white noise PSD [m/s^2/sqrt(Hz)]
    // _acc_rw_dev     accel bias random walk PSD [m/s^3/sqrt(Hz)]
    // x/y/z_init,     initial nominal pose (world frame)
    // roll/pitch/yaw_init
    // gravity_norm    |g| at the operating site (default 9.81)
    // P0_*            initial 1-sigma uncertainty for each error block
    bool setup(double _T, double _calibTime,
               double _gyr_dev, double _gyr_rw_dev,
               double _acc_dev, double _acc_rw_dev,
               double x_init = 0.0, double y_init = 0.0, double z_init = 0.0,
               double roll_init = 0.0, double pitch_init = 0.0, double yaw_init = 0.0,
               double gravity_norm = 9.81,
               double P0_pos = 1e-2, double P0_vel = 1e-2,
               double P0_ori = 1e-3, double P0_ba  = 1e-2,
               double P0_bg  = 1e-4)
    {
        acc_dev    = _acc_dev;
        gyr_dev    = _gyr_dev;
        gyr_rw_dev = _gyr_rw_dev;
        acc_rw_dev = _acc_rw_dev;

        gravity = Eigen::Vector3d(0.0, 0.0, -gravity_norm);

        T  = _T;
        T2 = _T * _T;
        calibTime = _calibTime;

        // Nominal state init
        p = Eigen::Vector3d(x_init, y_init, z_init);
        v.setZero();

        Eigen::AngleAxisd rollAngle (roll_init,  Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch_init, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle  (yaw_init,   Eigen::Vector3d::UnitZ());
        q = yawAngle * pitchAngle * rollAngle;
        q.normalize();

        ba.setZero();
        bg.setZero();

        // Calibration buffer
        if (calibTime <= 0.0) {
            calibSize = 1;
        } else {
            calibSize = static_cast<int>(_calibTime / _T);
            if (calibSize < 1) calibSize = 1;
        }
        calibData.resize(calibSize);
        calibIndex = 0;

        // Error-state covariance (variances, not std-dev)
        P.setZero();
        P.block<3,3>(0,0).diagonal().setConstant(P0_pos * P0_pos);
        P.block<3,3>(3,3).diagonal().setConstant(P0_vel * P0_vel);
        P.block<3,3>(6,6).diagonal().setConstant(P0_ori * P0_ori);
        P.block<3,3>(9,9).diagonal().setConstant(P0_ba  * P0_ba );
        P.block<3,3>(12,12).diagonal().setConstant(P0_bg * P0_bg);

        init = false;
        sec  = 0;

        std::cout << "ESKF setup -> T=" << T << " calibTime=" << calibTime
                  << " gyr_dev=" << gyr_dev << " gyr_rw=" << gyr_rw_dev
                  << " acc_dev=" << acc_dev << " acc_rw=" << acc_rw_dev
                  << " init_xyz=(" << x_init << "," << y_init << "," << z_init << ")"
                  << " init_rpy=(" << roll_init << "," << pitch_init << "," << yaw_init << ")"
                  << " |g|=" << gravity_norm << std::endl;
        return true;
    }

    // --- Initialization (static alignment or moving start) ---
    bool initialize(sensor_msgs::msg::Imu &msg,
                    double init_abx = 0.0, double init_aby = 0.0, double init_abz = 0.0)
    {
        // Moving start: trust the configured pose, skip IMU alignment
        if (calibSize <= 1 || calibTime < 0.1) {
            bg.setZero();
            ba = Eigen::Vector3d(init_abx, init_aby, init_abz);
            // gravity already set in setup()
            // q stays as configured in setup() (keeps yaw)
            RCLCPP_WARN(rclcpp::get_logger("ESKF"),
                "MOVING START (calibTime<=0). Skipping IMU alignment, using configured pose.");
            init = true;
            return true;
        }

        // Fill the calibration buffer
        calibData[calibIndex++ % calibSize] = msg;
        if (calibIndex < calibSize) return false;

        // Mean gyro and accel over the calibration window
        double gx_m = 0.0, gy_m = 0.0, gz_m = 0.0;
        double ax_m = 0.0, ay_m = 0.0, az_m = 0.0;
        const int samples = static_cast<int>(calibData.size());

        for (const auto& data : calibData) {
            if (!std::isnan(data.angular_velocity.x)) gx_m += data.angular_velocity.x;
            if (!std::isnan(data.angular_velocity.y)) gy_m += data.angular_velocity.y;
            if (!std::isnan(data.angular_velocity.z)) gz_m += data.angular_velocity.z;
            if (!std::isnan(data.linear_acceleration.x)) ax_m += data.linear_acceleration.x;
            if (!std::isnan(data.linear_acceleration.y)) ay_m += data.linear_acceleration.y;
            if (!std::isnan(data.linear_acceleration.z)) az_m += data.linear_acceleration.z;
        }
        gx_m /= samples; gy_m /= samples; gz_m /= samples;
        ax_m /= samples; ay_m /= samples; az_m /= samples;

        // Roll/pitch from specific force at rest:
        // a_meas ~ R^T * (-g_world), i.e. (0,0,+g) rotated into the body frame
        const double roll  = std::atan2(ay_m, az_m);
        const double pitch = std::atan2(-ax_m, std::sqrt(ay_m*ay_m + az_m*az_m));

        // Preserve previously configured yaw
        const double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
        const double cosy_cosp = 1.0 - 2.0 * (q.y()*q.y() + q.z()*q.z());
        const double current_yaw = std::atan2(siny_cosp, cosy_cosp);

        Eigen::AngleAxisd rollAngle (roll,        Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch,       Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle  (current_yaw, Eigen::Vector3d::UnitZ());
        q = yawAngle * pitchAngle * rollAngle;
        q.normalize();

        // Bias and velocity init
        bg = Eigen::Vector3d(gx_m, gy_m, gz_m);
        ba = Eigen::Vector3d(init_abx, init_aby, init_abz);
        v.setZero();

        RCLCPP_INFO(rclcpp::get_logger("ESKF"),
            "Calibration done. Samples: %d", samples);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("ESKF"),
            "Init gyro bias: " << bg.transpose());
        RCLCPP_INFO(rclcpp::get_logger("ESKF"),
            "Init RPY: %.4f, %.4f, %.4f", roll, pitch, current_yaw);

        init = true;
        return true;
    }

    // --- Prediction (IMU integration + covariance propagation) ---
    bool predict(double gx, double gy, double gz,
                 double ax, double ay, double az,
                 double stamp, double dt)
    {
        if (!init) return false;

        T  = dt;
        T2 = dt * dt;

        const Eigen::Vector3d acc_meas(ax, ay, az);
        const Eigen::Vector3d gyr_meas(gx, gy, gz);

        const Eigen::Vector3d acc_unbiased = acc_meas - ba;
        const Eigen::Vector3d gyr_unbiased = gyr_meas - bg;

        const Eigen::Matrix3d R = q.toRotationMatrix();

        // Nominal state propagation
        const Eigen::Vector3d acc_world = R * acc_unbiased + gravity;

        p = p + v * T + 0.5 * acc_world * T2;
        v = v + acc_world * T;

        // Orientation step: q <- q * Exp(omega*dt), via AngleAxis
        const Eigen::Vector3d angle_vec = gyr_unbiased * T;
        const double angle_norm = angle_vec.norm();
        if (angle_norm > 1e-8) {
            const Eigen::AngleAxisd delta_rot(angle_norm, angle_vec.normalized());
            q = q * delta_rot;
            q.normalize();
        }

        // Error-state transition F (15x15), first-order Euler: F = I + A*dt
        // Order: [dp(0:3), dv(3:6), dtheta(6:9), dba(9:12), dbg(12:15)]
        Eigen::Matrix<double,15,15> F = Eigen::Matrix<double,15,15>::Identity();

        // dp/dv = I*dt
        F.block<3,3>(0,3) = Eigen::Matrix3d::Identity() * T;

        // dv/dtheta = -R*[a]x*dt
        F.block<3,3>(3,6) = -R * skew(acc_unbiased) * T;

        // dv/dba = -R*dt
        F.block<3,3>(3,9) = -R * T;

        // dtheta self-term (right perturbation): I - [omega]x*dt
        F.block<3,3>(6,6) = Eigen::Matrix3d::Identity() - skew(gyr_unbiased) * T;

        // dtheta/dbg = -I*dt
        F.block<3,3>(6,12) = -Eigen::Matrix3d::Identity() * T;

        // Process noise Q (discrete, var ~ sigma^2 * dt)
        Eigen::Matrix<double,15,15> Q = Eigen::Matrix<double,15,15>::Zero();

        // velocity: accel white noise
        Q.block<3,3>(3,3).diagonal().setConstant(acc_dev * acc_dev * T);

        // orientation: gyro white noise
        Q.block<3,3>(6,6).diagonal().setConstant(gyr_dev * gyr_dev * T);

        // Bias random walks
        Q.block<3,3>(9,9).diagonal().setConstant(acc_rw_dev * acc_rw_dev * T);
        Q.block<3,3>(12,12).diagonal().setConstant(gyr_rw_dev * gyr_rw_dev * T);

        // Covariance propagation
        P = F * P * F.transpose() + Q;
        // keep P symmetric (guards against numerical drift)
        P = 0.5 * (P + P.transpose());

        // Logging
        gxf = gx; gyf = gy; gzf = gz;

        odom_file << std::fixed << std::setprecision(0)
                  << stamp * 1e9 << "," << sec << "," << stamp * 1e9 << ","
                  << std::fixed << std::setprecision(9)
                  << p.x() << "," << p.y() << "," << p.z() << ","
                  << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << ","
                  << v.x() << "," << v.y() << "," << v.z() << ","
                  << bg.x() << "," << bg.y() << "," << bg.z() << ","
                  << ba.x() << "," << ba.y() << "," << ba.z() << "\n";
        sec++;

        return true;
    }

    // --- Pose update (position + orientation, e.g. from LiDAR) ---
    // var_pos / var_ori: scalar variances [m^2 / rad^2], isotropic.
    // bool update_pose(const Eigen::Vector3d& meas_p,
    //                  const Eigen::Quaterniond& meas_q,
    //                  double var_pos, double var_ori,
    //                  double stamp)
    // {
    //     if (!init) return false;

    //     // ---- Innovation -----------------------------------------------------
    //     // Position: linear difference in world frame
    //     const Eigen::Vector3d y_p = meas_p - p;

    //     // Orientation: log map on SO(3). q_err = q^{-1} * meas_q lives in the
    //     // tangent space at q (right perturbation). Using the full atan2-based
    //     // Log instead of the small-angle 2*vec() approximation -> robust to
    //     // large attitude residuals after solver fixes.
    //     Eigen::Quaterniond q_err = q.conjugate() * meas_q;
    //     q_err.normalize();
    //     const Eigen::Vector3d y_theta = log_SO3(q_err);

    //     Eigen::Matrix<double,6,1> y;
    //     y.head<3>() = y_p;
    //     y.tail<3>() = y_theta;

    //     // ---- H (6x15): observation Jacobian --------------------------------
    //     // We observe position (block 0:3) and orientation (block 6:9) of the
    //     // error state.
    //     Eigen::Matrix<double,6,15> H;
    //     H.setZero();
    //     H.block<3,3>(0,0).setIdentity();
    //     H.block<3,3>(3,6).setIdentity();

    //     // ---- R (6x6): measurement covariance -------------------------------
    //     Eigen::Matrix<double,6,6> R_cov = Eigen::Matrix<double,6,6>::Zero();
    //     R_cov.block<3,3>(0,0).diagonal().setConstant(var_pos);
    //     R_cov.block<3,3>(3,3).diagonal().setConstant(var_ori);

    //     // ---- Kalman gain ---------------------------------------------------
    //     const Eigen::Matrix<double,6,6>  S = H * P * H.transpose() + R_cov;
    //     const Eigen::Matrix<double,15,6> K = P * H.transpose() * S.inverse();

    //     const Eigen::Matrix<double,15,1> delta_x = K * y;

    //     // ---- Joseph-form covariance update (numerically stable) ------------
    //     const Eigen::Matrix<double,15,15> I15 = Eigen::Matrix<double,15,15>::Identity();
    //     const Eigen::Matrix<double,15,15> IKH = I15 - K * H;
    //     P = IKH * P * IKH.transpose() + K * R_cov * K.transpose();
    //     P = 0.5 * (P + P.transpose());

    //     // ---- Inject error into nominal state -------------------------------
    //     p  += delta_x.segment<3>(0);
    //     v  += delta_x.segment<3>(3);
    //     ba += delta_x.segment<3>(9);
    //     bg += delta_x.segment<3>(12);

    //     // Lie manifold injection: q <- q * Exp(dtheta)
    //     const Eigen::Vector3d delta_theta = delta_x.segment<3>(6);
    //     q = q * exp_SO3(delta_theta);
    //     q.normalize();

    //     // Reset Jacobian on orientation block (small but improves consistency
    //     // when correcting large attitude errors). G = I - 0.5 * [dtheta]x.
    //     // Applied only to the orientation 3x3 block of P.
    //     const Eigen::Matrix3d G = Eigen::Matrix3d::Identity() - 0.5 * skew(delta_theta);
    //     P.block<3,3>(6,6) = G * P.block<3,3>(6,6) * G.transpose();

    //     // ---- Logging -------------------------------------------------------
    //     odom_file << std::fixed << std::setprecision(0)
    //               << stamp * 1e9 << "," << sec << "," << stamp * 1e9 << ","
    //               << std::fixed << std::setprecision(9)
    //               << p.x() << "," << p.y() << "," << p.z() << ","
    //               << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << ","
    //               << v.x() << "," << v.y() << "," << v.z() << ","
    //               << gxf  << "," << gyf  << "," << gzf  << ","
    //               << bg.x() << "," << bg.y() << "," << bg.z() << ","
    //               << ba.x() << "," << ba.y() << "," << ba.z() << "\n";
    //     sec++;
    //     return true;
    // }

    // Full 6x6 covariance version (e.g. from the solver Hessian)
    bool update_pose(const Eigen::Vector3d& meas_p,
                     const Eigen::Quaterniond& meas_q,
                     const Eigen::Matrix<double,6,6>& R_cov,
                     double stamp)
    {
        if (!init) return false;

        const Eigen::Vector3d y_p = meas_p - p;

        Eigen::Quaterniond q_err = q.conjugate() * meas_q;
        q_err.normalize();
        const Eigen::Vector3d y_theta = log_SO3(q_err);

        Eigen::Matrix<double,6,1> y;
        y.head<3>() = y_p;
        y.tail<3>() = y_theta;

        Eigen::Matrix<double,6,15> H;
        H.setZero();
        H.block<3,3>(0,0).setIdentity();
        H.block<3,3>(3,6).setIdentity();

        const Eigen::Matrix<double,6,6>  S = H * P * H.transpose() + R_cov;
        const Eigen::Matrix<double,15,6> K = P * H.transpose() * S.inverse();
        const Eigen::Matrix<double,15,1> delta_x = K * y;

        const Eigen::Matrix<double,15,15> I15 = Eigen::Matrix<double,15,15>::Identity();
        const Eigen::Matrix<double,15,15> IKH = I15 - K * H;
        P = IKH * P * IKH.transpose() + K * R_cov * K.transpose();
        P = 0.5 * (P + P.transpose());

        p  += delta_x.segment<3>(0);
        v  += delta_x.segment<3>(3);
        ba += delta_x.segment<3>(9);
        bg += delta_x.segment<3>(12);

        const Eigen::Vector3d delta_theta = delta_x.segment<3>(6);
        q = q * exp_SO3(delta_theta);
        q.normalize();

        const Eigen::Matrix3d G = Eigen::Matrix3d::Identity() - 0.5 * skew(delta_theta);
        P.block<3,3>(6,6) = G * P.block<3,3>(6,6) * G.transpose();

        odom_file << std::fixed << std::setprecision(0)
                  << stamp * 1e9 << "," << sec << "," << stamp * 1e9 << ","
                  << std::fixed << std::setprecision(9)
                  << p.x() << "," << p.y() << "," << p.z() << ","
                  << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << ","
                  << v.x() << "," << v.y() << "," << v.z() << ","
                  << gxf  << "," << gyf  << "," << gzf  << ","
                  << bg.x() << "," << bg.y() << "," << bg.z() << ","
                  << ba.x() << "," << ba.y() << "," << ba.z() << "\n";
        sec++;
        return true;
    }

    // Scalar (isotropic) version: builds a diagonal R_cov
    bool update_pose(const Eigen::Vector3d& meas_p,
                     const Eigen::Quaterniond& meas_q,
                     double var_pos, double var_ori,
                     double stamp)
    {
        Eigen::Matrix<double,6,6> R_cov = Eigen::Matrix<double,6,6>::Zero();
        R_cov.block<3,3>(0,0).diagonal().setConstant(var_pos);
        R_cov.block<3,3>(3,3).diagonal().setConstant(var_ori);
        return update_pose(meas_p, meas_q, R_cov, stamp);
    }

    // --- Accessors ---
    bool getEuler(double &roll, double &pitch, double &yaw)
    {
        // Manual ZYX extraction (avoids Eigen eulerAngles range jumps)
        const double sinr_cosp = 2.0 * (q.w()*q.x() + q.y()*q.z());
        const double cosr_cosp = 1.0 - 2.0 * (q.x()*q.x() + q.y()*q.y());
        roll = std::atan2(sinr_cosp, cosr_cosp);

        const double sinp = 2.0 * (q.w()*q.y() - q.z()*q.x());
        pitch = (std::abs(sinp) >= 1.0) ? std::copysign(M_PI/2.0, sinp) : std::asin(sinp);

        const double siny_cosp = 2.0 * (q.w()*q.z() + q.x()*q.y());
        const double cosy_cosp = 1.0 - 2.0 * (q.y()*q.y() + q.z()*q.z());
        yaw = std::atan2(siny_cosp, cosy_cosp);
        return true;
    }

    bool getQuat(double &qx, double &qy, double &qz, double &qw)
    {
        qx = q.x(); qy = q.y(); qz = q.z(); qw = q.w();
        return true;
    }

    bool getVelocities(double &vx_, double &vy_, double &vz_)
    {
        vx_ = v.x(); vy_ = v.y(); vz_ = v.z();
        return true;
    }

    bool getposition(double &x_, double &y_, double &z_)
    {
        x_ = p.x(); y_ = p.y(); z_ = p.z();
        return true;
    }

    bool getBIAS(double &gbx_, double &gby_, double &gbz_)
    {
        gbx_ = bg.x(); gby_ = bg.y(); gbz_ = bg.z();
        return true;
    }

    bool getAccelBias(double &abx_, double &aby_, double &abz_)
    {
        abx_ = ba.x(); aby_ = ba.y(); abz_ = ba.z();
        return true;
    }

    double get_sec() { return sec; }
    bool   isInit() const { return init; }

protected:

    // --- ESKF nominal state ---
    Eigen::Vector3d    p;   // position (world)
    Eigen::Vector3d    v;   // velocity (world)
    Eigen::Quaterniond q;   // orientation body->world
    Eigen::Vector3d    ba;  // accel bias (body)
    Eigen::Vector3d    bg;  // gyro bias  (body)

    // --- Error-state covariance ---
    // Order: [dp, dv, dtheta, dba, dbg]
    Eigen::Matrix<double,15,15> P;

    // --- Tuning / runtime ---
    double acc_dev, gyr_dev;
    double acc_rw_dev, gyr_rw_dev;
    double T, T2;
    double sec;
    bool   init;
    Eigen::Vector3d gravity;
    double gxf, gyf, gzf;            // last raw gyro for logging

    // --- Calibration buffer ---
    double calibTime;
    int    calibIndex, calibSize;
    std::vector<sensor_msgs::msg::Imu> calibData;
    std::ofstream odom_file;

    // --- Lie-algebra helpers on SO(3) ---

    // Skew-symmetric (hat) operator: R^3 -> so(3)
    static Eigen::Matrix3d skew(const Eigen::Vector3d& w)
    {
        Eigen::Matrix3d m;
        m <<     0.0, -w.z(),  w.y(),
              w.z(),     0.0, -w.x(),
             -w.y(),  w.x(),     0.0;
        return m;
    }

    // Exp map: so(3) (rotation vector) -> SO(3) (unit quaternion)
    static Eigen::Quaterniond exp_SO3(const Eigen::Vector3d& phi)
    {
        const double n = phi.norm();
        if (n < 1e-12) return Eigen::Quaterniond::Identity();
        return Eigen::Quaterniond(Eigen::AngleAxisd(n, phi / n));
    }

    // Log map: SO(3) (unit quaternion) -> so(3) (rotation vector)
    // Sign-corrected for the shortest-arc rotation vector.
    static Eigen::Vector3d log_SO3(const Eigen::Quaterniond& qq)
    {
        const double w  = qq.w();
        const Eigen::Vector3d vec = qq.vec();
        const double n  = vec.norm();
        if (n < 1e-10) {
            // Small-angle approximation (avoids 0/0 in atan2)
            return 2.0 * (w >= 0 ? vec : -vec);
        }
        const double angle = 2.0 * std::atan2(n, std::abs(w));
        const double sign  = (w >= 0) ? 1.0 : -1.0;
        return (sign * angle / n) * vec;
    }
};

#endif // __ESKF_HPP__