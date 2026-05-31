#include <vector>
#include <algorithm>
#include <queue>
#include <mutex>
#include <thread>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <rclcpp/qos.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" 
#include "tf2/transform_datatypes.h"
#include "pcl_ros/transforms.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/console/print.h>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <dlio/tdf3d_64.hpp>
#include <dlio/dlo6dsolver.hpp>
#include <dlio/point_cloud_processor.hpp>
#include "ElapsedTime.hpp"
#include <ekf_filter/eskf.hpp>
#include <pcl/common/transforms.h>

#define RST  "\033[0m"    
#define GRN  "\033[32m"
#define BOLD "\033[1m"
#define ORANGE "\033[38;5;208m"


using std::isnan;
using namespace std;

// ---------------------------------------------------------------------------
class DLO3DNode : public rclcpp::Node
{
public:
    /// Constructor: declares parameters, sets up ROS I/O and launches the processing thread.
    DLO3DNode(const std::string &node_name)
        : Node(node_name), m_solver(&m_grid3d),imuFilter() 
    {
        // ROS parameters
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
        m_robustOutlierDist = this->declare_parameter<double>("robust_outlier_dist", 0.5);
        m_kGridMarginFactor  = this->declare_parameter<double>("kGridMarginFactor", 0.8);
        m_maxload  = this->declare_parameter<double>("maxload", 5.0);
        m_maxCells  = this->declare_parameter<int>("maxCells", 10000.0);
        m_PubDownsampling    = this->declare_parameter<int>("PubDownsampling", 1);
        use_fixed_imu_dt_ = this->declare_parameter<bool>("use_fixed_imu_dt", true);
        bool imu_acc_in_ms2 = this->declare_parameter<bool>("imu_acc_in_ms2", true);
        acc_scale_factor_ = imu_acc_in_ms2 ? 1.0 : 9.80665;

        // Point cloud processor: leaf_size_map (fine, for TDF) and leaf_size_opt (coarse, for solver)
        PointCloudProcessor::Config pcl_cfg;
        pcl_cfg.lidar_type     = this->declare_parameter<std::string>("lidar_type", "ouster");
        pcl_cfg.leaf_size_map  = this->declare_parameter<double>("leaf_size_map", 0.10);
        pcl_cfg.leaf_size_opt  = this->declare_parameter<double>("leaf_size_opt", 0.25);
        pcl_cfg.min_range      = this->declare_parameter<double>("min_range", 1.0);
        pcl_cfg.max_range      = this->declare_parameter<double>("max_range", 100.0);
        pcl_cfg.timestamp_mode = this->declare_parameter<std::string>("timestamp_mode", "START_OF_SCAN");
        m_timestampMode = pcl_cfg.timestamp_mode;
        pcl_cfg.T_pcl          = T_pcl;
        pcl_cfg.T_imu          = T_imu;
        pcl_processor_.configure(pcl_cfg);
        std::cout << "Pcl_info: lidar_type=" << pcl_cfg.lidar_type
                  << " leaf_size_map=" << pcl_cfg.leaf_size_map
                  << " leaf_size_opt=" << pcl_cfg.leaf_size_opt
                  << " min_range=" << pcl_cfg.min_range
                  << " max_range=" << pcl_cfg.max_range
                  << " timestamp_mode=" << pcl_cfg.timestamp_mode
                  << " T_pcl=" << pcl_cfg.T_pcl
                  << " T_imu=" << pcl_cfg.T_imu << std::endl;

        // State init
        m_tfImuCache            = false;
        m_tfPointCloudCache     = false;
        m_tfPointCloudAuxCache  = false;
        m_keyFrameInit          = false;
        converged               = false;

        // Pose state
        m_tx = m_ty = m_tz    = 0.0;
        m_trx = m_try = m_trz = 0.0;
        m_kx = m_ky = m_kz    = 0.0;
        m_krx = m_kry = m_krz = 0.0;
        m_rx = m_ry = m_rz    = 0.0;
        
        // TF infrastructure
        m_tfBr = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        m_tfBuffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        m_tfListener = std::make_shared<tf2_ros::TransformListener>(*m_tfBuffer);
        
        // Publishers
        m_keyframePub = this->create_publisher<sensor_msgs::msg::PointCloud2>("keyframe", 10);
        m_cloudPub = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 10);
        m_odomPub = this->create_publisher<nav_msgs::msg::Odometry>("/odometry_pose", 10);
        grid_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/grid_pointcloud", 1);
           

        // Subscribers
        m_imuSub = this->create_subscription<sensor_msgs::msg::Imu>(
            m_inImuTopic,  5000,
            [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
                
                std::lock_guard<std::mutex> lock(queue_mutex_);
                imu_queue_.push(msg);
                queue_condition_.notify_one();

            });

        if (pcl_cfg.lidar_type == "livox") {
            m_livoxSub = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
                m_inCloudTopic, 1000,
                std::bind(&DLO3DNode::livoxCallback, this, std::placeholders::_1)
            );
        } else {
            m_pcSub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                m_inCloudTopic,  1000,
                [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
                    
                    std::lock_guard<std::mutex> lock(queue_mutex_);
                    pcl_queue_.push(msg);
                    message_count_++;
                    queue_condition_.notify_one();
                });
        }

        m_pcSub_aux = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            m_inCloudAuxTopic,  1000,
            [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
                if(aux_lidar_en){
                    std::lock_guard<std::mutex> lock(queue_mutex_);
                    pcl_aux_queue_.push(msg);
                    queue_condition_.notify_one();}
        });
        
        // Services
        save_service_pcd_ = this->create_service<std_srvs::srv::Trigger>(
            "/save_grid_pcd",
            std::bind(&DLO3DNode::saveGridPCD, this, std::placeholders::_1, std::placeholders::_2)
        );
        export_grid_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/pub_grid",
            std::bind(&DLO3DNode::exportGridService, this, std::placeholders::_1, std::placeholders::_2)
        );  
        save_service_mesh_ = this->create_service<std_srvs::srv::Trigger>( "/save_grid_mesh",
            std::bind(&DLO3DNode::saveGridMesh, this, std::placeholders::_1, std::placeholders::_2));


        // ESKF setup
        imuFilter.setup(T_imu,calibTime,
                this->declare_parameter<double>("gyr_dev", 1.0),
                this->declare_parameter<double>("gyr_rw_dev", 1.0),
                this->declare_parameter<double>("acc_dev", 1.0),
                this->declare_parameter<double>("acc_rw_dev", 1.0),
                this->declare_parameter<double>("m_init_x", 0.0),
                this->declare_parameter<double>("m_init_y", 0.0),
                this->declare_parameter<double>("m_init_z", 0.0),
                this->declare_parameter<double>("m_init_roll", 0.0),
                this->declare_parameter<double>("m_init_pitch", 0.0),
                this->declare_parameter<double>("m_init_yaw", 0.0)
        );

        // TDF grid
        m_grid3d.setup(m_tdfGridSizeX_low, m_tdfGridSizeX_high,
                    m_tdfGridSizeY_low, m_tdfGridSizeY_high,
                    m_tdfGridSizeZ_low, m_tdfGridSizeZ_high,
                    m_tdfGridRes,m_maxCells);
                      
        RCLCPP_INFO(this->get_logger(), GRN "DLO3D is ready to execute!" RST);

        RCLCPP_INFO_STREAM(this->get_logger(), GRN "Grid Created. Size: " 
                << (fabs(m_tdfGridSizeX_low) + fabs(m_tdfGridSizeX_high)) << " x " 
                << (fabs(m_tdfGridSizeY_low) + fabs(m_tdfGridSizeY_high)) << " x " 
                << (fabs(m_tdfGridSizeZ_low) + fabs(m_tdfGridSizeZ_high)) << "." RST);

        // Solver
        m_solver.setMaxIterations(m_solverMaxIter);
        m_solver.setMaxThreads(m_solverMaxThreads);
        m_solver.setRobustKernelScale(m_robustOutlierDist);

        // Runtime profiling log
        times_dlo.open("dlo3d_runtime.csv", std::ios::out);
        times_dlo << "total_time, optimized,opt_time,updated,update_time" << std::endl;

        // Start processing thread
        processing_thread_ = std::thread(&DLO3DNode::processQueues, this);
    }

    /// Destructor: stops the processing thread and exports the final PCD.
    ~DLO3DNode(){

        stop_processing_ = true;
        queue_condition_.notify_all();

        if (processing_thread_.joinable()) {
            processing_thread_.join();
        }

        if (times_dlo.is_open()) {
            times_dlo.close();
        }

        try {
            size_t idx = pcd_save_counter_++;
            std::string filename = "grid_data_" + std::to_string(idx) + ".pcd";
            RCLCPP_INFO(this->get_logger(), "Exporting PCD before closing: %s", filename.c_str());
            m_grid3d.exportGridToPCD(filename, 1);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error exporting PCD: %s", e.what());
        }

        RCLCPP_INFO(this->get_logger(), "Node closed successfully.");   
    }

private:
    int message_count_ = 0; 
    int message_processed_ = 0; 

    // ROS I/O
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imuSub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_pcSub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_pcSub_aux;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr m_livoxSub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_keyframePub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_cloudPub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odomPub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr grid_cloud_pub_;

    // Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_pcd_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr export_grid_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_mesh_;

    // Topic names and frame IDs
    std::string m_inCloudTopic;
    std::string m_inCloudAuxTopic;
    std::string m_inImuTopic;

    std::string m_baseFrameId;
    std::string m_odomFrameId;
    double T_pcl, T_imu;

    // TF caching
    bool m_tfPointCloudCache, m_tfImuCache, m_tfPointCloudAuxCache;
    geometry_msgs::msg::TransformStamped m_staticTfImu, m_staticTfPointCloud, m_staticTfPointCloudAux;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tfBr;
    std::shared_ptr<tf2_ros::Buffer> m_tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> m_tfListener;

    // Processing modules
    PointCloudProcessor pcl_processor_;

    ESKF imuFilter;
    double calibTime;

    // TDF grid and solver
    TDF3D64 m_grid3d;
    double m_tdfGridSizeX_low, m_tdfGridSizeX_high, m_tdfGridSizeY_low, m_tdfGridSizeY_high, m_tdfGridSizeZ_low, m_tdfGridSizeZ_high, m_tdfGridRes;

    DLL6DSolver m_solver;
    int m_solverMaxIter, m_solverMaxThreads;
    double m_robustOutlierDist;

    // Odometry state
    double m_rx, m_ry, m_rz;
    double m_tx, m_ty, m_tz, m_trx, m_try, m_trz;
    double m_kx, m_ky, m_kz, m_krx, m_kry, m_krz;
    double m_lx, m_ly, m_lz, m_lrx, m_lry, m_lrz;
    double m_x, m_y, m_z, vx, vy, vz;
    double m_x_last, m_y_last, m_z_last;
    double acc_scale_factor_;

    double m_keyFrameDist, m_keyFrameRot, m_kGridMarginFactor;
    double m_maxload;
    int m_maxCells;
    bool m_keyFrameInit, aux_lidar_en, converged;
    
    // Deskew buffer
    std::deque<Filter_Data> Filter_filtered_queue_;
    std::mutex Filter_queue_mutex_;
    int m_PubDownsampling;
    std::string m_timestampMode;
    bool use_fixed_imu_dt_;
    std::ofstream times_dlo;           // runtime profiling log

    // Thread-safe message queues
    std::mutex queue_mutex_;
    std::condition_variable queue_condition_;
    bool stop_processing_ = false;
    size_t pcd_save_counter_ = 0;
    std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> pcl_queue_;
    std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> pcl_aux_queue_;
    std::queue<sensor_msgs::msg::Imu::ConstSharedPtr> imu_queue_;
    std::thread processing_thread_;
    
    void processQueues();

    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud, 
                            const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_aux, 
                            bool is_aux_avaliable);
    void livoxCallback(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& msg);
    void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg);
    
    void exportGridService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void saveGridPCD(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void saveGridMesh(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
   
    Eigen::Matrix4f getTransformMatrix(const geometry_msgs::msg::TransformStamped& transform_stamped);
    double adjustYaw(double angle, double reference);
    
};



// -- Service callbacks -------------------------------------------------------

void DLO3DNode::saveGridPCD(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response){
    
    RCLCPP_INFO(this->get_logger(), "[saveGridPCD] Service invoked!");
    size_t idx = pcd_save_counter_++;
    std::string filename = "grid_data_" + std::to_string(idx) + ".pcd";
    RCLCPP_INFO(this->get_logger(), "[saveGridPCD] File Name: %s", filename.c_str());

    std::thread([this, filename]() {
        m_grid3d.exportGridToPCD(filename, 1);
    }).detach();
    response->success = true;
    response->message = "PCD export initiated: " + filename;
    RCLCPP_INFO(this->get_logger(), "PCD saved correctly!");
}

void DLO3DNode::exportGridService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response){
    
    auto cloud = m_grid3d.extractPointCloudFromGrid(m_PubDownsampling);
    if (!cloud || cloud->empty()) {
        response->success = false;
        response->message = "Grid empty or no points extracted.";
        RCLCPP_WARN(this->get_logger(), "saveGridPCD service: cloud empty.");
        return;
    }
    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(*cloud, out_msg);
    out_msg.header.stamp = this->now();
    out_msg.header.frame_id = m_odomFrameId;
    grid_cloud_pub_->publish(out_msg);

    response->success = true;
    response->message = "Grid published to /grid_pointcloud (no file saved). Points: " + std::to_string(cloud->size());
    RCLCPP_INFO(this->get_logger(), "saveGridPCD service: published %zu points.", cloud->size());
}

void DLO3DNode::saveGridMesh(const std::shared_ptr<std_srvs::srv::Trigger::Request>, 
        std::shared_ptr<std_srvs::srv::Trigger::Response> response){

    RCLCPP_INFO(this->get_logger(),"Received request to save Mesh. Starting in background...");

    float iso = 0.00f;

    std::thread([this, iso]() {
        try {
            m_grid3d.exportMesh("mesh.stl", iso); 
            RCLCPP_INFO(this->get_logger(), "Mesh saved successfully to mesh.stl (iso=%.3f)", iso);
            } 
        catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(),"Failed to extract mesh: %s", e.what());
        }
    }).detach();

    response->success = true;
    response->message = "Mesh export started in background.";
}

// Process queues - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void DLO3DNode::processQueues(){

    double SCAN_DURATION_MARGIN;
    if(m_timestampMode == "START_OF_SCAN"){
        SCAN_DURATION_MARGIN = T_pcl + 0.02;
    }else{
        SCAN_DURATION_MARGIN =  0.02;
    }

    while (!stop_processing_) {
        std::unique_lock<std::mutex> lock(queue_mutex_);

        queue_condition_.wait(lock, [this] {
            return stop_processing_ || (!imu_queue_.empty() && !pcl_queue_.empty());
        });
        // Process if there are messages in both queues
        if (!imu_queue_.empty() && !pcl_queue_.empty()) {
            auto imu_msg = imu_queue_.front();
            auto pcl_msg = pcl_queue_.front();

            // Compare timestamps of IMU and point cloud messages
            double imu_time = imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec * 1e-9;
            double pcl_time = pcl_msg->header.stamp.sec + pcl_msg->header.stamp.nanosec * 1e-9;


            if (imu_time < (pcl_time + SCAN_DURATION_MARGIN)) {
                imu_queue_.pop();
                lock.unlock();
                imuCallback(imu_msg);
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

                    if (diff <= T_pcl/2.0) {  
                        if (diff < min_diff) {
                            min_diff = diff;
                            best_match = pcl_aux_candidate;
                        }
                    } else if (aux_timestamp > pcl_timestamp + T_pcl/2.0) {
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
                message_processed_ ++; 
                RCLCPP_DEBUG_STREAM(this->get_logger(),
                    "Frames processed: " << message_processed_ 
                    << " / " << message_count_ 
                    << " -- queue size: " << pcl_queue_.size());
                lock.lock();
            }
        }
    }

}

// -- IMU callback ------------------------------------------------------------
void DLO3DNode::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
{   
    
    // Cache IMU-to-base TF on first message
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

    // Compute time difference between measurements
    double current_stamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    static double prev_stamp = current_stamp;
    double dt_real = current_stamp - prev_stamp;

    if (dt_real <= 0.0) {
        prev_stamp = current_stamp;
        return;
    }
    if (dt_real > 1.0) {
        RCLCPP_WARN(this->get_logger(), "Large IMU gap (%.6f s). Resetting integration.", dt_real);
        prev_stamp = current_stamp;
        return;
    }

    double dt;
    if (use_fixed_imu_dt_) {
        dt = T_imu;
    } else {
        dt = dt_real;
    }
    prev_stamp = current_stamp;

    // Rotate angular velocity and acceleration to base frame
    tf2::Quaternion q_static = tf2::Quaternion(
        m_staticTfImu.transform.rotation.x,
        m_staticTfImu.transform.rotation.y,
        m_staticTfImu.transform.rotation.z,
        m_staticTfImu.transform.rotation.w
    );
    
    // Rotate angular velocity to base frame
    tf2::Vector3 v(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    tf2::Matrix3x3 rot_matrix(q_static);
    tf2::Vector3 v_base = rot_matrix * v;

    // Rotate and compensate centripetal acceleration from IMU offset
    tf2::Vector3 a(
        msg->linear_acceleration.x * acc_scale_factor_, 
        msg->linear_acceleration.y * acc_scale_factor_, 
        msg->linear_acceleration.z * acc_scale_factor_
    );
    static tf2::Vector3 v_base_prev = v_base;
    tf2::Vector3 a_base = rot_matrix * a;
    
    a_base -= ((v_base - v_base_prev) / T_imu).cross(tf2::Vector3(
        m_staticTfImu.transform.translation.x,
        m_staticTfImu.transform.translation.y,
        m_staticTfImu.transform.translation.z
    )) + v_base.cross(v_base.cross(tf2::Vector3(
        m_staticTfImu.transform.translation.x,
        m_staticTfImu.transform.translation.y,
        m_staticTfImu.transform.translation.z
    )));
    v_base_prev = v_base;

    // Overwrite msg with base-frame quantities
    sensor_msgs::msg::Imu modified_msg = *msg;
    modified_msg.angular_velocity.x = v_base.x();
    modified_msg.angular_velocity.y = v_base.y();
    modified_msg.angular_velocity.z = v_base.z();
    modified_msg.linear_acceleration.x = a_base.x();
    modified_msg.linear_acceleration.y = a_base.y();
    modified_msg.linear_acceleration.z = a_base.z();

    // Propagate ESKF
    if (!imuFilter.isInit()) {

        imuFilter.initialize(modified_msg); 

    } else {
        
        imuFilter.predict(
            v_base.x(),v_base.y(),v_base.z(),
            a_base.x(),a_base.y(), a_base.z(),current_stamp, dt);
    }

    // Store pose for deskewing
    Filter_Data data;
    data.timestamp = current_stamp;
    imuFilter.getposition(data.x, data.y, data.z);
    imuFilter.getEuler(data.roll, data.pitch, data.yaw);

    
    std::lock_guard<std::mutex> lock(Filter_queue_mutex_);
    Filter_filtered_queue_.emplace_back(data);
    constexpr std::size_t kMaxQueueSize = 1000; 
    while (Filter_filtered_queue_.size() > kMaxQueueSize) {
        Filter_filtered_queue_.pop_front();
    }
    
    
}

// -- Livox callback ----------------------------------------------------------
void DLO3DNode::livoxCallback(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& msg)
{
    // Convert Livox CustomMsg to point cloud format and pack it as a PointCloud2 message
    // so it uses the same pipeline logic. Alternatively, we just do it here and push to pcl_queue_.
    
    pcl::PointCloud<PointXYZT> pcl_cloud;
    pcl_processor_.convertLivoxToPCL(msg, pcl_cloud);

    sensor_msgs::msg::PointCloud2::SharedPtr pc2_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(pcl_cloud, *pc2_msg);
    pc2_msg->header = msg->header; // Copy the header

    std::lock_guard<std::mutex> lock(queue_mutex_);
    pcl_queue_.push(pc2_msg);
    message_count_++;
    queue_condition_.notify_one();
}

// -- Point cloud callback ----------------------------------------------------

void DLO3DNode::pointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud, const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_aux, bool is_aux_avaliable)
{
    RCLCPP_DEBUG(this->get_logger(), BOLD ORANGE "Processing new cloud ......................" RST);
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
    imuFilter.getEuler(m_rx, m_ry, m_rz);
    imuFilter.getVelocities(vx,vy,vz);

    // Cache LiDAR-to-base TF on first cloud
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

    // Preprocessing: convert, transform to base frame
    pcl::PointCloud<PointXYZT> pcl_cloud;
    // Check if the cloud was already converted (from livox callback)
    // If it has "timestamp" and "t" fields, it's already PointXYZT. Our convertLivoxToPCL generates a clean PointXYZT cloud.
    bool is_already_pxyzt = false;
    for (const auto& field : cloud->fields) {
        if (field.name == "timestamp" || field.name == "t") {
            is_already_pxyzt = true;
            break;
        }
    }

    if (is_already_pxyzt) {
        pcl::console::VERBOSITY_LEVEL old_level = pcl::console::getVerbosityLevel();
        pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
        pcl::fromROSMsg(*cloud, pcl_cloud);
        pcl::console::setVerbosityLevel(old_level);
    } else {
        pcl_processor_.convertAndAdapt(cloud, pcl_cloud);
    }

    static bool printed_fields = false;
    if (!printed_fields) {
        printed_fields = true;
        RCLCPP_INFO(this->get_logger(), " ☁️  PointCloud fields ☁️");
            for (const auto& field : cloud->fields) {
                RCLCPP_INFO(this->get_logger(), "Field: %s (type: %d), offset: %u", 
                            field.name.c_str(), field.datatype, field.offset);
            }
    }
    
    Eigen::Matrix4f transform_matrix = getTransformMatrix(m_staticTfPointCloud);
    pcl::PointCloud<PointXYZT> transformed_cloud;
    pcl::transformPointCloud(pcl_cloud, transformed_cloud, transform_matrix);
    pcl::PointCloud<PointXYZT> final_cloud = transformed_cloud; 

    // Fuse auxiliary LiDAR cloud if available
    if (is_aux_avaliable) {

        Eigen::Matrix4f transform_aux_matrix = getTransformMatrix(m_staticTfPointCloudAux);
        pcl::PointCloud<PointXYZT> pcl_cloud_aux;
        pcl_processor_.convertAndAdapt(cloud_aux, pcl_cloud_aux);
        pcl::PointCloud<PointXYZT> transformed_cloud_aux;
        pcl::transformPointCloud(pcl_cloud_aux, transformed_cloud_aux, transform_aux_matrix);
        final_cloud += transformed_cloud_aux;  
    }

    double scan_time = cloud->header.stamp.sec + cloud->header.stamp.nanosec * 1e-9;

    // Deskew once into a single base cloud; the per-stage downsampling diverges from here.
    std::vector<pcl::PointXYZ> c_base;
    bool unwrap_success = pcl_processor_.deskew(final_cloud, c_base, scan_time, Filter_filtered_queue_, Filter_queue_mutex_);

    if (!unwrap_success) 
    {
        RCLCPP_WARN(this->get_logger(), "Unwrap failed (IMU missing/sync error). Using RAW cloud.");

        c_base.clear();
        c_base.reserve(final_cloud.size());
        for (const auto& pt : final_cloud.points) {
            if (pcl::isFinite(pt)) {
                c_base.emplace_back(pt.x, pt.y, pt.z);
            }
        }
    }

    // Deskew and create two downsampled copies: coarse (solver) and fine (map)
    std::vector<pcl::PointXYZ> c_solver = c_base;
    pcl_processor_.downsampleForSolver(c_solver);

    std::vector<pcl::PointXYZ> c_map = c_base;
    pcl_processor_.downsampleForMap(c_map);

    // Cloud processing
    converged = true;
    static double yaw_local_prev;

    if(imuFilter.isInit()){

        if(!m_keyFrameInit)
        {
            // Reset key-frame variables 
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

            m_grid3d.clear();

            // Initialize last-keyframe position to current position
            m_lx = m_kx;
            m_ly = m_ky;
            m_lz = m_kz;
            m_lrx = m_krx;
            m_lry = m_kry;
            m_lrz = m_krz;

            // Load current point-cloud as new key-frame (use the fine-resolution copy)
            m_grid3d.loadCloudFiltered(c_map, m_kx, m_ky, m_kz, m_krx,m_kry,m_krz,m_maxload);

            m_keyFrameInit = true;
            yaw_local_prev = m_rz;

            // Publish current point cloud
            m_keyframePub->publish(*cloud);

        }
        else
        {
            
            // Grid local Variables
            double x_solver = m_x;
            double y_solver = m_y;
            double z_solver = m_z;
            double roll_solver = m_rx;
            double pitch_solver = m_ry;
            double yaw_solver = m_rz;

            // Optimization (uses the coarse-resolution copy for speed)
            optimized = 1;
            opt_timer.tick();
            // converged = m_solver.solve(c_solver, x_solver, y_solver, z_solver, roll_solver, pitch_solver, yaw_solver);
            double  lidar_cov[36];
            int     lidar_inliers = 0;
            double  lidar_cost    = 0.0;
            converged = m_solver.solve(c_solver, x_solver, y_solver, z_solver,
                                       roll_solver, pitch_solver, yaw_solver,
                                       lidar_cov, &lidar_inliers, &lidar_cost);
            opt_time = opt_timer.tock();

            // Adjust yaw to maintain continuity after solver, which constrains yaw within [-π, π].
            yaw_solver = adjustYaw(yaw_local_prev, yaw_solver);
            yaw_local_prev = yaw_solver;    

            // Update EKF if converged
            if(converged){


                m_x = x_solver;
                m_y = y_solver;
                m_z = z_solver;
                m_rx =  roll_solver;
                m_ry =  pitch_solver;
                m_rz =  yaw_solver;

                // Build quaternion from solver Euler angles
                Eigen::Quaterniond q_solver;
                q_solver = Eigen::AngleAxisd(m_rz, Eigen::Vector3d::UnitZ())
                         * Eigen::AngleAxisd(m_ry, Eigen::Vector3d::UnitY())
                         * Eigen::AngleAxisd(m_rx, Eigen::Vector3d::UnitX());
                q_solver.normalize();

                // imuFilter.update_pose(
                //     Eigen::Vector3d(m_x, m_y, m_z),
                //     q_solver,
                //     0.01 * 0.01,    // var_pos
                //     0.025 * 0.025,  // var_ori
                //     scan_time);
                Eigen::Matrix<double,6,6> R_cov =
                    Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>>(lidar_cov);
                imuFilter.update_pose(
                    Eigen::Vector3d(m_x, m_y, m_z),
                    q_solver,
                    R_cov,
                    scan_time);
                

                imuFilter.getEuler(m_rx, m_ry, m_rz);
                imuFilter.getposition(m_x, m_y, m_z);

                m_x_last = m_x;
                m_y_last = m_y;
                m_z_last = m_z; 
             
               
                m_kx = m_x ;
                m_ky =  m_y ;
                m_kz =  m_z ;
                m_krx = m_rx;
                m_kry = m_ry;
                m_krz = m_rz;
                
            }

            // Create a new map if the limit is reached or if the solver does not converge.
        if(m_kx > m_tdfGridSizeX_high * m_kGridMarginFactor || m_kx < m_tdfGridSizeX_low * m_kGridMarginFactor || 
            m_ky > m_tdfGridSizeY_high * m_kGridMarginFactor|| m_ky < m_tdfGridSizeY_low * m_kGridMarginFactor||
            m_kz > m_tdfGridSizeZ_high * m_kGridMarginFactor|| m_kz < m_tdfGridSizeZ_low * m_kGridMarginFactor||!converged)
            {     
                if(!converged){
                    RCLCPP_INFO(this->get_logger(), "NOT CONVERGED. CREATING A NEW MAP!  ····························································");
                }else{
                    RCLCPP_INFO(this->get_logger(), "MAP LIMIT REACHED. CREATING A NEW MAP!  ························································");
                }

                
                size_t idx = pcd_save_counter_++;
                std::string filename = "grid_data_" + std::to_string(idx) + ".pcd";
                RCLCPP_INFO(this->get_logger(), "Exporting PCD before grid reset: %s", filename.c_str());
                m_grid3d.exportGridToPCD(filename, 1);
                RCLCPP_INFO(this->get_logger(), "PCD export done.");

                m_lx = m_kx;
                m_ly = m_ky;
                m_lz = m_kz;

                m_lrx = m_krx;
                m_lry = m_kry;
                m_lrz = m_krz;

                // Reset Grid and key-frame variables 
                m_kx = m_x ;
                m_ky =  m_y ;
                m_kz =  m_z ;
                m_krx = m_rx;
                m_kry = m_ry;
                m_krz = m_rz;
                yaw_local_prev = m_rz;

                m_grid3d.clear();

                // Load current point-cloud as new key-frame (fine-resolution copy)
                m_grid3d.loadCloudFiltered(c_map, m_kx, m_ky, m_kz, m_krx,m_kry,m_krz,m_maxload);
  
            }
            else if( ((m_kx-m_lx)*(m_kx-m_lx) + (m_ky-m_ly)*(m_ky-m_ly) + (m_kz-m_lz)*(m_kz-m_lz) > m_keyFrameDist*m_keyFrameDist) || 
                     (std::fabs(std::atan2(std::sin(m_krz - m_lrz), std::cos(m_krz - m_lrz))) > m_keyFrameRot) ) // Update grid if threshold reached            {
            {
                m_lx = m_kx;
                m_ly = m_ky;
                m_lz = m_kz;

                m_lrx = m_krx;
                m_lry = m_kry;
                m_lrz = m_krz;

                updated = 1;
                update_timer.tick();
                // Load the fine-resolution copy into the TDF
                m_grid3d.loadCloudFiltered(c_map, m_kx, m_ky, m_kz, m_krx,m_kry,m_krz,m_maxload);
                update_time = update_timer.tock();

                // Publish LiDAR Cloud (use the solver cloud for visual feedback - lighter)
                std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> pcl_points_from_vector(c_solver.begin(), c_solver.end());
                pcl::PointCloud<pcl::PointXYZ> pcl_cloud_from_vector;
                pcl_cloud_from_vector.points = pcl_points_from_vector;
                pcl_cloud_from_vector.width = pcl_points_from_vector.size();  
                pcl_cloud_from_vector.height = 1; 
                sensor_msgs::msg::PointCloud2 cloud_msg;
                pcl::toROSMsg(pcl_cloud_from_vector, cloud_msg);
                cloud_msg.header.frame_id = m_baseFrameId; 
                m_keyframePub->publish(cloud_msg);

            }
        }

        // Create and publish odom transform
        geometry_msgs::msg::TransformStamped odomTf;
        odomTf.header.stamp = cloud->header.stamp;
        odomTf.header.frame_id = m_odomFrameId;
        odomTf.child_frame_id = m_baseFrameId;
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

        // Publish LiDAR Cloud (use the solver cloud - already coarse, lighter to publish)
        std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> pcl_points_from_vector(c_solver.begin(), c_solver.end());
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud_from_vector;
        pcl_cloud_from_vector.points = pcl_points_from_vector;
        pcl_cloud_from_vector.width = pcl_points_from_vector.size();  
        pcl_cloud_from_vector.height = 1; 
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(pcl_cloud_from_vector, cloud_msg);
        cloud_msg.header.frame_id = m_baseFrameId; 
        m_cloudPub->publish(cloud_msg);

    
    }

    double total_time = total_timer.tock();
    times_dlo << std::fixed << std::setprecision(9)
          << total_time << "," << optimized << "," << opt_time << ","
          << updated << "," << update_time << "\n";
    
    times_dlo.flush();

    RCLCPP_DEBUG_STREAM(this->get_logger(), std::fixed << std::setprecision(5)
    << "+ Time Info[s]-> Total: " << total_time 
    << " | Opt: " << opt_time 
    << " | Update: " << update_time);

}


// -- Utility functions -------------------------------------------------------

Eigen::Matrix4f DLO3DNode::getTransformMatrix(const geometry_msgs::msg::TransformStamped& transform_stamped){
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
        auto node = std::make_shared<DLO3DNode>("DLIO_node");
        rclcpp::spin(node);

    } catch (const std::exception &e) {

        std::cerr << "Error creating or running the node: " << e.what() << std::endl;

    }

    rclcpp::shutdown();
    return 0;
}