#include <vector>
#include <ctime>
#include <algorithm>
#include <queue>
#include <mutex>
#include <thread>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"


#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" 
#include "tf2/transform_datatypes.h"

#include "pcl_ros/transforms.hpp"
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include "pcl_conversions/pcl_conversions.h"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <dlo3d/tdf3d_64.hpp>
#include <dlo3d/dlo6dsolver_lie.hpp>
#include "ElapsedTime.hpp"
#include <filter/imufilter.hpp>

using std::isnan;
struct Filter_Data {
    double timestamp;
    double x, y, z;
    double roll, pitch, yaw;
};

struct Closest_Filter_Result {
    Filter_Data Filter_data;
    bool found;
};

struct PointXYZT
{
    PCL_ADD_POINT4D;
    uint32_t t;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZT,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (uint32_t, t, t))

   
//Class Definition
class DLO3DNode : public rclcpp::Node
{
public:

//!Default contructor
    DLO3DNode(const std::string &node_name)
        : Node(node_name), m_solver(&m_grid3d),imuFilter() 
    {
        // Read DLO3D parameters
        m_inCloudTopic     = this->declare_parameter<std::string>("in_cloud", "/cloud");
        m_inCloudAuxTopic  = this->declare_parameter<std::string>("in_cloud_aux", "/cloud_aux");
        m_inImuTopic       = this->declare_parameter<std::string>("in_imu", "/imu");
        m_baseFrameId      = this->declare_parameter<std::string>("base_frame_id", "base_link");
        m_odomFrameId      = this->declare_parameter<std::string>("odom_frame_id", "odom");

        T_pcl              = 1.0 / this->declare_parameter<double>("hz_cloud", 10.0);
        T_imu              = 1.0 / this->declare_parameter<double>("hz_imu", 100.0);
        calibTime          = this->declare_parameter<double>("calibration_time", 1.0);
        aux_lidar_en       = this->declare_parameter<bool>("aux_lidar_en", false);

        m_keyFrameDist      = this->declare_parameter<double>("keyframe_dist", 1.0);
        m_keyFrameRot       = this->declare_parameter<double>("keyframe_rot", 0.1);
        m_tdfGridSizeX_low  = this->declare_parameter<double>("tdfGridSizeX_low", 10.0);
        m_tdfGridSizeX_high = this->declare_parameter<double>("tdfGridSizeX_high", 10.0);
        m_tdfGridSizeY_low  = this->declare_parameter<double>("tdfGridSizeY_low", 10.0);
        m_tdfGridSizeY_high = this->declare_parameter<double>("tdfGridSizeY_high", 10.0);
        m_tdfGridSizeZ_low  = this->declare_parameter<double>("tdfGridSizeZ_low", 10.0);
        m_tdfGridSizeZ_high = this->declare_parameter<double>("tdfGridSizeZ_high", 10.0);
        m_tdfGridRes        = this->declare_parameter<double>("tdf_grid_res", 0.05);
        m_solverMaxIter     = this->declare_parameter<int>("solver_max_iter", 75);
        m_solverMaxThreads  = this->declare_parameter<int>("solver_max_threads", 20);
        m_minRange          = this->declare_parameter<double>("min_range", 1.0);
        m_maxRange          = this->declare_parameter<double>("max_range", 100.0);
        m_PcDownsampling    = this->declare_parameter<int>("pc_downsampling", 1);
        m_robusKernelScale  = this->declare_parameter<double>("robust_kernel_scale", 1.0);

        // Init internal variables
        m_tfImuCache            = false;
        m_tfPointCloudCache     = false;
        m_tfPointCloudAuxCache  = false;
        m_keyFrameInit          = false;
        converged               = false;

        // Pose auxiliar variables
        m_tx = m_ty = m_tz    = 0.0;
        m_trx = m_try = m_trz = 0.0;
        m_kx = m_ky = m_kz    = 0.0;
        m_krx = m_kry = m_krz = 0.0;
        m_rx = m_ry = m_rz    = 0.0;
        
        //Init buffers
        m_tfBr = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        m_tfBuffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        m_tfListener = std::make_shared<tf2_ros::TransformListener>(*m_tfBuffer);
        
        // Launch publishers
        m_keyframePub = this->create_publisher<sensor_msgs::msg::PointCloud2>("keyframe", 10);
        m_cloudPub = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 10);
        m_odomPub = this->create_publisher<nav_msgs::msg::Odometry>("/odometry_pose", 10);

        m_imuSub = this->create_subscription<sensor_msgs::msg::Imu>(
            m_inImuTopic,  5000,
            [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
                
                std::lock_guard<std::mutex> lock(queue_mutex_);
                imu_queue_.push(msg);

            });
        // Create subscribers
         m_pcSub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            m_inCloudTopic,  5000,
            [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
                
                std::lock_guard<std::mutex> lock(queue_mutex_);
                pcl_queue_.push(msg);
            });

         m_pcSub_aux = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            m_inCloudAuxTopic,  5000,
            [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
                if(aux_lidar_en){
                    std::lock_guard<std::mutex> lock(queue_mutex_);
                    pcl_aux_queue_.push(msg);}
            });
        
        // Create Services
        save_service_csv_ = this->create_service<std_srvs::srv::Trigger>(
            "/save_grid_csv",
            std::bind(&DLO3DNode::saveGridCSV, this, std::placeholders::_1, std::placeholders::_2)
        );

        save_service_pcd_ = this->create_service<std_srvs::srv::Trigger>(
            "/save_grid_pcd",
            std::bind(&DLO3DNode::saveGridPCD, this, std::placeholders::_1, std::placeholders::_2)
        );

        // Kalman Filter Setup
        imuFilter.setup(T_imu,calibTime,
                this->declare_parameter<double>("gyr_dev", 1.0),
                this->declare_parameter<double>("gyr_rw_dev", 1.0),
                this->declare_parameter<double>("acc_dev", 1.0),
                this->declare_parameter<double>("acc_rw_dev", 1.0)

        );

        // TDF grid Setup
        m_grid3d.setup(m_tdfGridSizeX_low, m_tdfGridSizeX_high,
                    m_tdfGridSizeY_low, m_tdfGridSizeY_high,
                    m_tdfGridSizeZ_low, m_tdfGridSizeZ_high,
                    m_tdfGridRes);
                      
        std::cout << "DLO3D is ready to execute! " << std::endl;
        std::cout << "Grid Created. Size: " 
                << fabs(m_tdfGridSizeX_low) + fabs(m_tdfGridSizeX_high) << " x " 
                << fabs(m_tdfGridSizeY_low) + fabs(m_tdfGridSizeY_high) << " x " 
                << fabs(m_tdfGridSizeZ_low) + fabs(m_tdfGridSizeZ_high) << "." 
                << std::endl;
        
        // Solver Setup
        m_solver.setMaxIterations(m_solverMaxIter);
        m_solver.setMaxThreads(m_solverMaxThreads);
        m_solver.setRobustKernelScale(m_robusKernelScale);

        // Runtime File for time analysis
        times_dlo.open("dlo3d_runtime.csv", std::ios::out);
        times_dlo << "total_time, optimized,opt_time,updated,update_time";

        // Launch the processing thread
        processing_thread_ = std::thread(&DLO3DNode::processQueues, this);
    }

    // Default Destructor
    ~DLO3DNode(){

        stop_processing_ = true;
        queue_condition_.notify_all();

        if (processing_thread_.joinable()) {
            processing_thread_.join();
        }

        if (times_dlo.is_open()) {
            times_dlo.close();
        }

        RCLCPP_INFO(this->get_logger(), "Node closed successfully.");   
    }

private:

    // ROS2 parameters
    std::string m_inCloudTopic;
    std::string m_inCloudAuxTopic;
    std::string m_inImuTopic;
    std::string m_baseFrameId;
    std::string m_odomFrameId;
    double m_minRange, m_maxRange;
    ImuFilter imuFilter;

    // 3D distance grid
    TDF3D64 m_grid3d;
    double m_tdfGridSizeX_low, m_tdfGridSizeX_high, m_tdfGridSizeY_low, m_tdfGridSizeY_high, m_tdfGridSizeZ_low, m_tdfGridSizeZ_high, m_tdfGridRes;

    // Non-linear optimization solver
    DLL6DSolver m_solver;
    int m_solverMaxIter, m_solverMaxThreads;
    int m_PcDownsampling;
    double m_robusKernelScale;

    // Odometry variables
    double m_rx, m_ry, m_rz;
    double m_tx, m_ty, m_tz, m_trx, m_try, m_trz;
    double m_kx, m_ky, m_kz, m_krx, m_kry, m_krz;
    double m_lx, m_ly, m_lz, m_lrx, m_lry, m_lrz;
    double m_x, m_y, m_z, a, vx, vy, vz;
    double m_x_last, m_y_last, m_z_last;

    // Key frame thresholds
    double m_keyFrameDist, m_keyFrameRot;
    bool m_keyFrameInit, aux_lidar_en, converged;

    // Time parameters
    double T_pcl, T_imu, calibTime;
    
    // Transformations
    bool m_tfPointCloudCache, m_tfImuCache, m_tfPointCloudAuxCache;
    geometry_msgs::msg::TransformStamped m_staticTfImu, m_staticTfPointCloud, m_staticTfPointCloudAux;
    
    // Lidar Unwrap data
    std::deque<Filter_Data> Filter_filtered_queue_;
    std::mutex Filter_queue_mutex_;
    
    // File management
    std::ofstream times_dlo;

    // ROS2 subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imuSub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_pcSub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_pcSub_aux;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_keyframePub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_cloudPub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odomPub;

    // ROS2 transform management
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tfBr;
    std::shared_ptr<tf2_ros::Buffer> m_tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> m_tfListener;

    // Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_csv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_pcd_;

    // Queue management for processing
    std::mutex queue_mutex_;
    std::condition_variable queue_condition_;
    bool stop_processing_;

    std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> pcl_queue_;
    std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> pcl_aux_queue_;
    std::queue<sensor_msgs::msg::Imu::ConstSharedPtr> imu_queue_;
    std::thread processing_thread_;

    // Function declarations
    void processQueues();
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud, 
                            const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_aux, 
                            bool is_aux_avaliable);

    void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg);
    bool PointCloud2_to_PointXYZ_unwrap(sensor_msgs::msg::PointCloud2 &in, std::vector<pcl::PointXYZ> &out, double scan_start_time);
    Closest_Filter_Result findClosestFilterData(const std::deque<Filter_Data>& Filter_queue, double target_timestamp);

    Eigen::Matrix4f getTransformMatrix(const geometry_msgs::msg::TransformStamped& transform_stamped);
    
    double adjustYaw(double angle, double reference);

    void saveGridCSV(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void saveGridPCD(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response);

};

void DLO3DNode::saveGridCSV(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                 std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    RCLCPP_INFO(this->get_logger(), "Received request to save CSV. Starting in a separate thread...");
    
    std::thread([this]() {
        RCLCPP_INFO(this->get_logger(), "Generating CSV...");
        m_grid3d.exportGridToCSV("grid_data.csv",m_tdfGridSizeX_low, m_tdfGridSizeX_high,
                    m_tdfGridSizeY_low, m_tdfGridSizeY_high,
                    m_tdfGridSizeZ_low, m_tdfGridSizeZ_high,
                      1);
        RCLCPP_INFO(this->get_logger(), "CSV saved successfully..");
    }).detach();

    response->success = true;
    response->message = "CSV export started in the background.";

}

void DLO3DNode::saveGridPCD(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                 std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Received request to save PCD. Starting in a separate thread...");

    std::thread([this]() {
        RCLCPP_INFO(this->get_logger(), "Generating PCD...");
        m_grid3d.exportGridToPCD("grid_data.pcd",1); 
        RCLCPP_INFO(this->get_logger(), "PCD saved successfully.");
    }).detach();

    response->success = true;
    response->message = "Exportación del PCD iniciada en segundo plano.";
}


void DLO3DNode::processQueues() {
    while (!stop_processing_) {
        std::unique_lock<std::mutex> lock(queue_mutex_);

        // Check if processing should stop
        if (stop_processing_) {
            break;
        }
        // Process if there are messages in both queues
        if (!imu_queue_.empty() && !pcl_queue_.empty()) {
            auto imu_msg = imu_queue_.front();
            auto pcl_msg = pcl_queue_.front();

            // Compare timestamps of IMU and point cloud messages
            if (imu_msg->header.stamp.sec < pcl_msg->header.stamp.sec || 
                (imu_msg->header.stamp.sec == pcl_msg->header.stamp.sec && imu_msg->header.stamp.nanosec < pcl_msg->header.stamp.nanosec)) {

                imu_queue_.pop();
                lock.unlock();
                imuCallback(imu_msg); // Call IMU processing function
                lock.lock();

            } else {
                auto pcl_msg_safe = pcl_msg;
                
                pcl_queue_.pop();
                lock.unlock();

                // Check if an auxiliary point cloud is available
                bool is_aux_available = false;
                sensor_msgs::msg::PointCloud2::ConstSharedPtr pcl_msg_aux_safe;
                
                double pcl_timestamp = pcl_msg_safe->header.stamp.sec + pcl_msg_safe->header.stamp.nanosec * 1e-9;
                double min_diff = std::numeric_limits<double>::max();
                sensor_msgs::msg::PointCloud2::ConstSharedPtr best_match = nullptr;

                // Find the closest auxiliary point cloud in time if enable
                std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> pcl_aux_queue_copy = pcl_aux_queue_;
                while (aux_lidar_en && !pcl_aux_queue_copy.empty()) {
                    auto pcl_aux_candidate = pcl_aux_queue_copy.front();
                    double aux_timestamp = pcl_aux_candidate->header.stamp.sec + pcl_aux_candidate->header.stamp.nanosec * 1e-9;
                    double diff = std::abs(aux_timestamp - pcl_timestamp);

                    if (diff <= 0.1) {  
                        if (diff < min_diff) {
                            min_diff = diff;
                            best_match = pcl_aux_candidate;
                        }
                    } else if (aux_timestamp > pcl_timestamp + 0.1) {
                        break;
                    }

                    pcl_aux_queue_copy.pop();
                }

                // Use the best auxiliary match if found
                if (best_match) {
                    is_aux_available = true;
                    pcl_msg_aux_safe = best_match;
                } else {
                    pcl_msg_aux_safe = pcl_msg_safe;
                    is_aux_available = false;
                }

                // Call point cloud processing function
                pointcloudCallback(pcl_msg_safe, pcl_msg_aux_safe, is_aux_available);
                lock.lock();
            }
        }
    }

}

//! IMU callback
void DLO3DNode::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
{
    
    // Pre-cache transform for point-cloud to base frame and transform the pc 
    if(!m_tfImuCache)
    {	
        try
        {
            geometry_msgs::msg::TransformStamped transformStamped;
            transformStamped = m_tfBuffer->lookupTransform(m_baseFrameId, msg->header.frame_id, tf2::TimePointZero,tf2::durationFromSec(2.0));
            m_staticTfImu = transformStamped;
            m_tfImuCache = true;
            return;
        }
        catch (tf2::TransformException & ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
            return;
        }
    }

    // Compute r difference between measurements
    static double prev_stamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    double current_stamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    double dt = current_stamp - prev_stamp;
    prev_stamp = current_stamp;

    // Adapt IMU velocity and aceleration to base system reference
    tf2::Quaternion q_static = tf2::Quaternion(
        m_staticTfImu.transform.rotation.x,
        m_staticTfImu.transform.rotation.y,
        m_staticTfImu.transform.rotation.z,
        m_staticTfImu.transform.rotation.w
    );
    
    // Rotate angular velocities to base system reference 
    tf2::Vector3 v(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    tf2::Matrix3x3 rot_matrix(q_static);
    tf2::Vector3 v_base = rot_matrix * v;

    // Rotate and compense for accelerations produced by off-centering the IMU (base frame must be in the center)
    tf2::Vector3 a(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    static tf2::Vector3 v_base_prev = v_base;
    tf2::Vector3 a_base = rot_matrix * a;
    
    a_base += ((v_base - v_base_prev) / T_imu).cross(tf2::Vector3(
        m_staticTfImu.transform.translation.x,
        m_staticTfImu.transform.translation.y,
        m_staticTfImu.transform.translation.z
    )) + v_base.cross(v_base.cross(tf2::Vector3(
        m_staticTfImu.transform.translation.x,
        m_staticTfImu.transform.translation.y,
        m_staticTfImu.transform.translation.z
    )));
    v_base_prev = v_base;

    // Modified msg for imuFilter -> v (rad/s) -- a (m/s2)
    sensor_msgs::msg::Imu modified_msg = *msg;
    modified_msg.angular_velocity.x = v_base.x();
    modified_msg.angular_velocity.y = v_base.y();
    modified_msg.angular_velocity.z = v_base.z();
    modified_msg.linear_acceleration.x = a_base.x();
    modified_msg.linear_acceleration.y = a_base.y();
    modified_msg.linear_acceleration.z = a_base.z();

    // Prediction Step
    if (!imuFilter.isInit()) {

        imuFilter.initialize(modified_msg); 

    } else {
        
        imuFilter.predict(
            v_base.x(),v_base.y(),v_base.z(),
            a_base.x(),a_base.y(), a_base.z(),current_stamp);

        imuFilter.getAngles(m_rx, m_ry, m_rz);

    }

    // EKF Data Storage for LiDAR Deeskewing
    Filter_Data data;
    data.timestamp = current_stamp;
    imuFilter.getposition(data.x, data.y, data.z);
    imuFilter.getAngles(data.roll, data.pitch, data.yaw);

    {
        std::lock_guard<std::mutex> lock(Filter_queue_mutex_);
        Filter_filtered_queue_.emplace_back(data);
        while(current_stamp - Filter_filtered_queue_.front().timestamp > 5.0 ){
            Filter_filtered_queue_.pop_front();
        }
    }
    
}

//! 3D point-cloud callback
void DLO3DNode::pointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud, const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_aux, bool is_aux_avaliable)
{
    // Time analysis variables
    ElapsedTime total_timer,opt_timer,update_timer;
    total_timer.tick();

    int updated = 0;
    int optimized = 0;
    double opt_time = 0.0;
    double update_time = 0.0;

    // Compute time difference between measurements
    static double prev_stamp = cloud->header.stamp.sec + cloud->header.stamp.nanosec * 1e-9;
    double current_stamp = cloud->header.stamp.sec + cloud->header.stamp.nanosec * 1e-9;
    double dt = current_stamp - prev_stamp;
    prev_stamp = current_stamp;

    // Current pose to optimize from EKF Filter
    imuFilter.getposition(m_x, m_y, m_z);
    imuFilter.getAngles(m_rx, m_ry, m_rz);

    // Pre-cache transform for point-clouds to base frame and transform the pc 
    if (!m_tfPointCloudCache)
    {	
        try
        {
            m_staticTfPointCloud = m_tfBuffer->lookupTransform(
                m_baseFrameId, 
                cloud->header.frame_id, 
                tf2::TimePointZero, 
                tf2::durationFromSec(2.0)); 
            
            m_tfPointCloudCache = true;
            return;
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
            return;
        }
    }

    if (!m_tfPointCloudAuxCache && is_aux_avaliable)
    {	
        try
        {
            m_staticTfPointCloudAux = m_tfBuffer->lookupTransform(
                m_baseFrameId, 
                cloud_aux->header.frame_id,
                tf2::TimePointZero, 
                tf2::durationFromSec(2.0)); 
            
            m_tfPointCloudAuxCache = true;
            return;
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
            return;
        }
    }

    // Cloud Preproccessing
    pcl::PointCloud<PointXYZT> pcl_cloud;
    pcl::fromROSMsg(*cloud, pcl_cloud);

    Eigen::Matrix4f transform_matrix = getTransformMatrix(m_staticTfPointCloud);
    pcl::PointCloud<PointXYZT> transformed_cloud;
    pcl::transformPointCloud(pcl_cloud, transformed_cloud, transform_matrix);
    pcl::PointCloud<PointXYZT> final_cloud = transformed_cloud; 

    // Fuse auxiliar cloud if avaliable
    if (is_aux_avaliable) {

        Eigen::Matrix4f transform_aux_matrix = getTransformMatrix(m_staticTfPointCloudAux);
        pcl::PointCloud<PointXYZT> pcl_cloud_aux;
        pcl::fromROSMsg(*cloud_aux, pcl_cloud_aux);

        pcl::PointCloud<PointXYZT> transformed_cloud_aux;
        pcl::transformPointCloud(pcl_cloud_aux, transformed_cloud_aux, transform_aux_matrix);
        final_cloud += transformed_cloud_aux;  
    }

    sensor_msgs::msg::PointCloud2 final_cloud_msg;
    pcl::toROSMsg(final_cloud, final_cloud_msg);
    double scan_end_time = cloud->header.stamp.sec + cloud->header.stamp.nanosec * 1e-9;
    std::vector<pcl::PointXYZ> c;

    bool unwrap_success = PointCloud2_to_PointXYZ_unwrap(final_cloud_msg, c, scan_end_time);

    // If unwarp not succedded, use raw cloud
    if (!unwrap_success) {

        c.clear();
        c.reserve(final_cloud.size());
        for (const auto& point : final_cloud) {
            c.emplace_back(point.x, point.y, point.z);
        }
    }

    // Cloud Processing
    converged = false;

    if(imuFilter.isInit()){

        if(!m_keyFrameInit)
        {
            // Reset key-frame variables 
            m_lx = m_ly = m_lz = m_lrx = m_lry = m_lrz = 0.0;
            m_tx = m_ty = m_tz = m_trx = m_try = m_trz = 0.0;

            m_kx = m_x;
            m_ky = m_y;
            m_kz = m_z;
            m_krx = m_rx;
            m_kry = m_ry;
            m_krz = m_rz;

            m_x_last = m_x;
            m_y_last = m_y;
            m_z_last = m_z;

            vx = vy = vz = 0.0;

            m_grid3d.clear();

            // Load current point-cloud as new key-frame
            m_grid3d.loadCloud(c, m_kx, m_ky, m_kz, m_krx,m_kry,m_krz);

            // Set flag
            m_keyFrameInit = converged = true;

            // Publish current point cloud
            m_keyframePub->publish(*cloud);

        }
        else
        {
            // Map Frame -> Grid Frame
            tf2::Transform grid_base;

            tf2::Quaternion grid_q;
            grid_q.setRPY(m_trx, m_try, m_trz);
            
            grid_base.setRotation(grid_q.normalize());
            grid_base.setOrigin(tf2::Vector3(m_tx, m_ty, m_tz));
            tf2::Vector3 point_global(m_x, m_y, m_z);

            tf2::Vector3 point_local = grid_base.inverse() * point_global;

            // Grid local Variables
            double x_local = point_local.x();
            double y_local = point_local.y();
            double z_local = point_local.z();
            double roll_local = m_rx - m_trx;
            double pitch_local = m_ry - m_try;
            double yaw_local = m_rz - m_trz;
            double yaw_local_prev;

            yaw_local_prev = yaw_local;
           
            // Optimization
            optimized = 1;
            opt_timer.tick();
            converged = m_solver.solve(c, x_local, y_local, z_local, roll_local, pitch_local, yaw_local);
            opt_time = opt_timer.tock();

            // Adjust yaw to maintain continuity after solver, which constrains yaw within [-π, π].
            yaw_local = adjustYaw(yaw_local_prev, yaw_local);

            // Update EKF if converged
            if(converged){

                point_local = tf2::Vector3(x_local, y_local, z_local);
                point_global = grid_base * point_local;

                m_x = point_global.x();
                m_y = point_global.y();
                m_z = point_global.z();
                m_rx = m_trx + roll_local;
                m_ry = m_try + pitch_local;
                m_rz = m_trz + yaw_local;

                vx = (m_x - m_x_last)/(T_pcl);
                vy = (m_y - m_y_last)/(T_pcl);
                vz = (m_z - m_z_last)/(T_pcl);

                imuFilter.update_opt_full(  m_x, m_y, m_z, 
                                            m_rx, m_ry, m_rz,
                                            vx, vy, vz,
                                            0.01 * 0.01, 0.01*0.01,
                                            0.001 * 0.001, 0.001 * 0.001,
                                            0.01 *0.01,0.01 *0.01, scan_end_time);

                imuFilter.getAngles(m_rx, m_ry, m_rz);
                imuFilter.getposition(m_x, m_y, m_z);

                m_x_last = m_x;
                m_y_last = m_y;
                m_z_last = m_z; 
             
                point_global = tf2::Vector3(m_x, m_y, m_z);
                point_local = grid_base.inverse() * point_global;

                m_kx = point_local.x();
                m_ky = point_local.y();
                m_kz = point_local.z();
                m_krx = m_rx - m_trx;
                m_kry = m_ry - m_try;
                m_krz = m_rz - m_trz;
                
            }

        }

        // Create a new map if the limit is reached or if the solver does not converge.
        if( m_kx > m_tdfGridSizeX_high || m_kx < m_tdfGridSizeX_low || 
            m_ky > m_tdfGridSizeY_high || m_ky < m_tdfGridSizeY_low ||
            m_kz > m_tdfGridSizeZ_high || m_kz < m_tdfGridSizeZ_low || !converged)
        {     
            if(!converged){
                std::cout<<" NOT CONVERGED. CREATING A NEW MAP!  ····························································"<<std::endl;
            }else{
                std::cout<<" MAP LIMIT REACHED. CREATING A NEW MAP!  ························································"<<std::endl;
            }

            // Add keyframe transform into odom
            m_tx  = m_x;
            m_ty  = m_y;
            m_tz  = m_z;
            m_trx = m_rx;
            m_try = m_ry;
            m_trz = m_rz;

            // Reset Grid and key-frame variables 
            m_kx = m_ky = m_kz = m_krx = m_kry = m_krz = 0.0;
            m_lx = m_ly = m_lz = m_lrx = m_lry = m_lrz = 0.0;

            m_grid3d.clear();

            // Load current point-cloud as new key-frame
            m_grid3d.loadCloud(c);
  
        }
        else if( (m_kx-m_lx)*(m_kx-m_lx) + (m_ky-m_ly)*(m_ky-m_ly) + (m_kz-m_lz)*(m_kz-m_lz) > m_keyFrameDist*m_keyFrameDist) // Update drid if threshold reached
        {

            m_lx = m_kx;
            m_ly = m_ky;
            m_lz = m_kz;

            m_lrx = m_krx;
            m_lry = m_kry;
            m_lrz = m_krz;

            updated = 1;
            update_timer.tick();
            m_grid3d.loadCloud(c, m_kx, m_ky, m_kz, m_krx,m_kry,m_krz);
            update_time = update_timer.tock();
            
        }

        // Create and publish odom transform
        geometry_msgs::msg::TransformStamped odomTf;
        odomTf.header.stamp = cloud->header.stamp;
        odomTf.header.frame_id = m_odomFrameId;
        odomTf.child_frame_id = m_baseFrameId;

        imuFilter.getposition(m_x, m_y, m_z);
        imuFilter.getAngles(m_rx, m_ry, m_rz);

        odomTf.transform.translation.x = m_x;
        odomTf.transform.translation.y = m_y;
        odomTf.transform.translation.z = m_z;

        tf2::Quaternion q;
        q.setRPY(m_rx, m_ry, m_rz);
        odomTf.transform.rotation = tf2::toMsg(q.normalize());
        m_tfBr->sendTransform(odomTf);

        // Create and publish odom message
        nav_msgs::msg::Odometry odomMsg;
        odomMsg.header.stamp = cloud->header.stamp;
        odomMsg.header.frame_id = m_odomFrameId;
        odomMsg.child_frame_id = m_baseFrameId;

        odomMsg.pose.pose.position.x = m_x;
        odomMsg.pose.pose.position.y = m_y;
        odomMsg.pose.pose.position.z = m_z;

        odomMsg.pose.pose.orientation = tf2::toMsg(q.normalize());
        m_odomPub->publish(odomMsg);

        // Publish LiDAR Cloud
        std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> pcl_points_from_vector(c.begin(), c.end());
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud_from_vector;
        pcl_cloud_from_vector.points = pcl_points_from_vector;
        pcl_cloud_from_vector.width = pcl_points_from_vector.size();  
        pcl_cloud_from_vector.height = 1; 
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(pcl_cloud_from_vector, cloud_msg);
        cloud_msg.header.frame_id = "base_link"; 
        m_cloudPub->publish(cloud_msg);
    }

    double total_time = total_timer.tock();
    times_dlo << std::fixed << std::setprecision(9)<<
            total_time << ","<<
            optimized <<","<<
            opt_time << ","<<
            updated <<","<<
            update_time <<"\n";

}

Closest_Filter_Result DLO3DNode::findClosestFilterData(const std::deque<Filter_Data>& Filter_queue, double target_timestamp) {

    Closest_Filter_Result result;
    result.found = false;
    
    // Initialize the search with the most recent filter data (last entry in the queue)
    result.Filter_data = Filter_queue.back();
    double min_time_diff = std::abs(target_timestamp - result.Filter_data.timestamp);

    // Iterate over the filter queue to find the closest match based on timestamp
    for (const auto& Filter_data : Filter_queue) {
        double time_diff = std::abs(target_timestamp - Filter_data.timestamp);
        if (time_diff < min_time_diff) {
            min_time_diff = time_diff;
            result.Filter_data = Filter_data;
        }
    }

    // Check if the closest filter data is within an acceptable time range (e.g., 5 times the IMU time step)
    if (min_time_diff < 5 * T_imu) {
        result.found = true;   
    }

    return result;
}

bool DLO3DNode::PointCloud2_to_PointXYZ_unwrap(
    sensor_msgs::msg::PointCloud2 &in, 
    std::vector<pcl::PointXYZ> &out,
    double scan_end_time)
{
    out.clear();
    int points = 0;

    // Check if IMU data is available
    if (Filter_filtered_queue_.empty())
    {
        RCLCPP_WARN(this->get_logger(), "IMU queue is empty. Unwrap cannot be performed.");
        return false;
    }

    double error_cnt = 0.0;

    // Define the time window for filtering the data
    double search_start_time = scan_end_time - 0.2;
    double search_end_time = scan_end_time + 0.2;

    // Filter the IMU data within the desired time window
    std::deque<Filter_Data> filtered_deque;
    for (const auto& Filter_data : Filter_filtered_queue_) {
        if (Filter_data.timestamp >= search_start_time && Filter_data.timestamp <= search_end_time) {
            filtered_deque.push_back(Filter_data);  // Añadir al std::deque
        }
    }

    // Check if filtered data is available within the time window
    if (filtered_deque.empty()) {
        RCLCPP_WARN(this->get_logger(), "No data found in the specified time range.");
        return false;
    }

    // Define iterators for point cloud data (x, y, z, t)
    sensor_msgs::PointCloud2Iterator<float> iterX(in, "x");
    sensor_msgs::PointCloud2Iterator<float> iterY(in, "y");
    sensor_msgs::PointCloud2Iterator<float> iterZ(in, "z");
    sensor_msgs::PointCloud2Iterator<uint32_t> iterT(in, "t");
    tf2::Transform T_base;
    
    // Find the timestamp of the last point in the scan
    uint32_t last_stamp = 0;
    auto iterT_start = iterT;
    for (; iterT != iterT.end(); ++iterT) {
        last_stamp = *iterT;
    }
    iterT = iterT_start;

    // Find the Filter transform corresponding to the time of the last point
    Closest_Filter_Result Filter_f = findClosestFilterData(filtered_deque, scan_end_time);
    
    tf2::Quaternion q_base;
    q_base.setRPY(Filter_f.Filter_data.roll, Filter_f.Filter_data.pitch, Filter_f.Filter_data.yaw);
    tf2::Vector3 base_position(Filter_f.Filter_data.x, Filter_f.Filter_data.y, Filter_f.Filter_data.z);
    T_base.setRotation(q_base);
    T_base.setOrigin(base_position);

    double total_points = static_cast<double>(in.width) * static_cast<double>(in.height);
   
    // Iterate over each point in the point cloud
    for (; iterX != iterX.end(); ++iterX, ++iterY, ++iterZ, ++iterT)
    {
        
        tf2::Vector3 point_pcl(*iterX, *iterY, *iterZ);
        double point_time = scan_end_time - static_cast<double>(last_stamp)*1e-9 + static_cast<double>(*iterT)* 1e-9;
        Closest_Filter_Result closest_Filter_data = findClosestFilterData(filtered_deque, point_time);

        if(!closest_Filter_data.found){
            error_cnt++;  
        }

        // Transform corresponding to the closest Filter data
        tf2::Quaternion closest_Filter_q;
        closest_Filter_q.setRPY(closest_Filter_data.Filter_data.roll, closest_Filter_data.Filter_data.pitch, closest_Filter_data.Filter_data.yaw);
        tf2::Vector3 closest_Filter_pos(closest_Filter_data.Filter_data.x, closest_Filter_data.Filter_data.y, closest_Filter_data.Filter_data.z);
        tf2::Transform T_i(closest_Filter_q,closest_Filter_pos);

        // Apply transformation to the point
        tf2::Transform T = T_base.inverse() * T_i;
        point_pcl = T * point_pcl;

        // Range filtering and downsampling
        double min_sq = m_minRange * m_minRange;
        double max_sq = m_maxRange * m_maxRange;
        double d2 = point_pcl.x() * point_pcl.x() + point_pcl.y()  * point_pcl.y()  + point_pcl.z()  * point_pcl.z() ;

        pcl::PointXYZ compensated_p;
        compensated_p.x = point_pcl.x();
        compensated_p.y = point_pcl.y();
        compensated_p.z = point_pcl.z();
       
        // Add point to output if within range and if downsampling condition is met
        if (d2 > min_sq && d2 < max_sq && (points % m_PcDownsampling == 0))
        {
            out.push_back(compensated_p);
        }
        points++;

    }

    // Return true if less than 10% of points had errors
    return (total_points - error_cnt)/total_points < 0.9;

}

//! Auxiliar Functions
Eigen::Matrix4f DLO3DNode::getTransformMatrix(const geometry_msgs::msg::TransformStamped& transform_stamped)
    {
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

        Eigen::Quaternionf q(
            transform_stamped.transform.rotation.w,
            transform_stamped.transform.rotation.x,
            transform_stamped.transform.rotation.y,
            transform_stamped.transform.rotation.z
        );
        Eigen::Matrix3f rotation = q.toRotationMatrix();

        Eigen::Vector3f translation(
            transform_stamped.transform.translation.x,
            transform_stamped.transform.translation.y,
            transform_stamped.transform.translation.z
        );

        transform.block<3,3>(0,0) = rotation;
        transform.block<3,1>(0,3) = translation;

        return transform;
    }

double DLO3DNode::adjustYaw(double yaw_prev, double yaw_solved) {
    
    double delta_yaw = yaw_solved - yaw_prev;
    while (delta_yaw > M_PI) delta_yaw -= 2 * M_PI;
    while (delta_yaw < -M_PI) delta_yaw += 2 * M_PI;

    return yaw_prev + delta_yaw;
}
  


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<DLO3DNode>("dll3d_node");
        rclcpp::spin(node);

    } catch (const std::exception &e) {
        std::cerr << "Error creating or running the node: " << e.what() << std::endl;
    }

    rclcpp::shutdown();

    return 0;
}
