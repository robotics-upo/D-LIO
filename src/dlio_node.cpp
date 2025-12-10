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
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/impl/pcl_base.hpp>       
#include "pcl_conversions/pcl_conversions.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <dlio/tdf3d_64.hpp>
#include <dlio/dlo6dsolver_lie.hpp>
#include "ElapsedTime.hpp"
#include <ekf_filter/ekf_filter.hpp>
#include <pcl/console/print.h>


#define RST  "\033[0m"    
#define GRN  "\033[32m"
#define BOLD "\033[1m"
#define ORANGE "\033[38;5;208m"


using std::isnan;
using namespace std;
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
    double timestamp;                
    uint32_t t;                       
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZT,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (double, timestamp, timestamp)
    (uint32_t, t, t)
)

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
        m_PubDownsampling    = this->declare_parameter<int>("PubDownsampling", 1);
        m_robusKernelScale  = this->declare_parameter<double>("robust_kernel_scale", 1.0);
        m_kGridMarginFactor  = this->declare_parameter<double>("kGridMarginFactor", 0.8);
        m_maxload  = this->declare_parameter<double>("maxload", 100.0);
        m_maxCells  = this->declare_parameter<int>("maxCells", 10000.0);
        m_lidarType = this->declare_parameter<std::string>("lidar_type", "ouster");
        m_leafSize = this->declare_parameter<double>("leaf_size", 0.1);
        m_timestampMode = this->declare_parameter<std::string>("timestamp_mode", "START_OF_SCAN");

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
        grid_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/grid_pointcloud", 1);
           

        // Create subscribers
        m_imuSub = this->create_subscription<sensor_msgs::msg::Imu>(
            m_inImuTopic,  5000,
            [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
                
                std::lock_guard<std::mutex> lock(queue_mutex_);
                imu_queue_.push(msg);


            });

        m_pcSub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            m_inCloudTopic,  1000,
            [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
                
                std::lock_guard<std::mutex> lock(queue_mutex_);
                pcl_queue_.push(msg);
                message_count_++;
            });

        m_pcSub_aux = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            m_inCloudAuxTopic,  1000,
            [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
                if(aux_lidar_en){
                    std::lock_guard<std::mutex> lock(queue_mutex_);
                    pcl_aux_queue_.push(msg);}
        });
        
        // Create Services
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
                    m_tdfGridRes,m_maxCells);
                      
        RCLCPP_INFO(this->get_logger(), GRN "DLO3D is ready to execute!" RST);

        RCLCPP_INFO_STREAM(this->get_logger(), GRN "Grid Created. Size: " 
                << (fabs(m_tdfGridSizeX_low) + fabs(m_tdfGridSizeX_high)) << " x " 
                << (fabs(m_tdfGridSizeY_low) + fabs(m_tdfGridSizeY_high)) << " x " 
                << (fabs(m_tdfGridSizeZ_low) + fabs(m_tdfGridSizeZ_high)) << "." RST);

        // Solver Setup
        m_solver.setMaxIterations(m_solverMaxIter);
        m_solver.setMaxThreads(m_solverMaxThreads);
        m_solver.setRobustKernelScale(m_robusKernelScale);

        // Runtime File for time analysis
        times_dlo.open("dlo3d_runtime.csv", std::ios::out);
        times_dlo << "total_time, optimized,opt_time,updated,update_time" << std::endl;

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

    // ROS2 subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imuSub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_pcSub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_pcSub_aux;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_keyframePub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_cloudPub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odomPub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr grid_cloud_pub_;

    // Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_pcd_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr export_grid_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_mesh_;

    // ROS2 parameters
    std::string m_inCloudTopic;
    std::string m_inCloudAuxTopic;
    std::string m_inImuTopic;

    std::string m_baseFrameId;
    std::string m_odomFrameId;
    double T_pcl, T_imu;
    std::string m_lidarType,m_lidarTimeField;
    double m_leafSize;

    // ROS2 transform management
    bool m_tfPointCloudCache, m_tfImuCache, m_tfPointCloudAuxCache;
    geometry_msgs::msg::TransformStamped m_staticTfImu, m_staticTfPointCloud, m_staticTfPointCloudAux;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tfBr;
    std::shared_ptr<tf2_ros::Buffer> m_tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> m_tfListener;

    // IMU Filter
    ImuFilter imuFilter;
    double calibTime;

    // 3D distance grid
    TDF3D64 m_grid3d;
    double m_tdfGridSizeX_low, m_tdfGridSizeX_high, m_tdfGridSizeY_low, m_tdfGridSizeY_high, m_tdfGridSizeZ_low, m_tdfGridSizeZ_high, m_tdfGridRes;

    // Non-linear optimization solver
    DLL6DSolver m_solver;
    int m_solverMaxIter, m_solverMaxThreads;
    double m_robusKernelScale;

    // Odometry variables
    double m_rx, m_ry, m_rz;
    double m_tx, m_ty, m_tz, m_trx, m_try, m_trz;
    double m_kx, m_ky, m_kz, m_krx, m_kry, m_krz;
    double m_lx, m_ly, m_lz, m_lrx, m_lry, m_lrz;
    double m_x, m_y, m_z, a, vx, vy, vz;
    double m_x_last, m_y_last, m_z_last;

    double m_keyFrameDist, m_keyFrameRot, m_kGridMarginFactor;
    double m_maxload;
    int m_maxCells;
    bool m_keyFrameInit, aux_lidar_en, converged;
    double m_minRange, m_maxRange;
    
    // Lidar Unwrap
    std::deque<Filter_Data> Filter_filtered_queue_;
    std::mutex Filter_queue_mutex_;
    int m_PubDownsampling;
    std::string m_timestampMode;

    // File management
    std::ofstream times_dlo;

    // Queue management for processing
    std::mutex queue_mutex_;
    std::condition_variable queue_condition_;
    bool stop_processing_;
    size_t pcd_save_counter_ = 0;
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
    
    void convertAndAdapt(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& in_msg, 
                         pcl::PointCloud<PointXYZT>& out_cloud);
   
    bool PointCloud2_to_PointXYZ_unwrap(pcl::PointCloud<PointXYZT> &in, std::vector<pcl::PointXYZ> &out, double scan_start_time);
    
    void exportGridService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void saveGridPCD(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void saveGridMesh(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
   
    Closest_Filter_Result findClosestFilterData(const std::deque<Filter_Data>& Filter_queue, double target_timestamp);
    Eigen::Matrix4f getTransformMatrix(const geometry_msgs::msg::TransformStamped& transform_stamped);
    double adjustYaw(double angle, double reference);
    
};



// Services Callbacks - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

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

        // Check if processing should stop
        if (stop_processing_) {
            break;
        }
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

//! IMU callback - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
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
    }

    // EKF Data Storage for LiDAR Deeskewing
    Filter_Data data;
    data.timestamp = current_stamp;
    imuFilter.getposition(data.x, data.y, data.z);
    imuFilter.getAngles(data.roll, data.pitch, data.yaw);

    
    std::lock_guard<std::mutex> lock(Filter_queue_mutex_);
    Filter_filtered_queue_.emplace_back(data);
    constexpr std::size_t kMaxQueueSize = 1000; 
    while (Filter_filtered_queue_.size() > kMaxQueueSize) {
        Filter_filtered_queue_.pop_front();
    }
    
    
}

//! 3D point-cloud callback - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

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
    imuFilter.getAngles(m_rx, m_ry, m_rz);
    imuFilter.getVelocities(vx,vy,vz);

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
    convertAndAdapt(cloud, pcl_cloud);

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

    // Fuse auxiliar cloud if avaliable
    if (is_aux_avaliable) {

        Eigen::Matrix4f transform_aux_matrix = getTransformMatrix(m_staticTfPointCloudAux);
        pcl::PointCloud<PointXYZT> pcl_cloud_aux;
        convertAndAdapt(cloud_aux, pcl_cloud_aux);
        pcl::PointCloud<PointXYZT> transformed_cloud_aux;
        pcl::transformPointCloud(pcl_cloud_aux, transformed_cloud_aux, transform_aux_matrix);
        final_cloud += transformed_cloud_aux;  
    }

    double scan_time = cloud->header.stamp.sec + cloud->header.stamp.nanosec * 1e-9;
    std::vector<pcl::PointXYZ> c;

    bool unwrap_success = PointCloud2_to_PointXYZ_unwrap(final_cloud, c, scan_time);

    if (!unwrap_success) 
    {
        RCLCPP_WARN(this->get_logger(), "Unwrap failed (IMU missing/sync error). Using RAW cloud.");

        c.clear();
        c.reserve(final_cloud.size());
        for (const auto& pt : final_cloud.points) {
            if (pcl::isFinite(pt)) {
                c.emplace_back(pt.x, pt.y, pt.z);
            }
        }
    }

    if (m_leafSize > 0.001) 
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_filter(new pcl::PointCloud<pcl::PointXYZ>());
        cloud_to_filter->width = c.size();
        cloud_to_filter->height = 1;
        cloud_to_filter->points.resize(c.size());
        
        for (size_t i = 0; i < c.size(); ++i) {
            cloud_to_filter->points[i] = c[i];
        }

        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud_to_filter);
        sor.setLeafSize(m_leafSize, m_leafSize, m_leafSize);
        
        pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
        sor.filter(cloud_filtered);

        c.clear();
        c.reserve(cloud_filtered.size());
        for (const auto& p : cloud_filtered.points) {
            c.push_back(p);
        }
        RCLCPP_DEBUG_STREAM(this->get_logger(), "+ Downsample Info:  " << cloud_to_filter->size() << " -> " << c.size() << " points.");
    }

    // Cloud Processing
    converged = true;
    static double yaw_local_prev;

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

            m_grid3d.clear();

            // Load current point-cloud as new key-frame
            m_grid3d.loadCloudFiltered(c, m_kx, m_ky, m_kz, m_krx,m_kry,m_krz,m_maxload);

            m_keyFrameInit = true;

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

            // Optimization
            optimized = 1;
            opt_timer.tick();
            converged = m_solver.solve(c, x_solver, y_solver, z_solver,roll_solver,pitch_solver,yaw_solver);
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

                vx = (m_x - m_x_last)/(T_pcl);
                vy = (m_y - m_y_last)/(T_pcl);
                vz = (m_z - m_z_last)/(T_pcl);

                imuFilter.update_opt_full(  m_x, m_y, m_z, 
                                            m_rx, m_ry, m_rz,
                                            vx, vy, vz,
                                            0.01 * 0.01, 0.01*0.01,
                                            0.001 * 0.001, 0.001 * 0.001,
                                            0.01 *0.01,0.01 *0.01, scan_time);

                imuFilter.getAngles(m_rx, m_ry, m_rz);
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
                RCLCPP_INFO(this->get_logger(), "Exportando PCD antes de limpiar: %s", filename.c_str());
                m_grid3d.exportGridToPCD(filename, 1);
                RCLCPP_INFO(this->get_logger(), "Exportación completada.");

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

                // Load current point-cloud as new key-frame
                m_grid3d.loadCloudFiltered(c, m_kx, m_ky, m_kz, m_krx,m_kry,m_krz,m_maxload);
  
            }
            else if( ((m_kx-m_lx)*(m_kx-m_lx) + (m_ky-m_ly)*(m_ky-m_ly) + (m_kz-m_lz)*(m_kz-m_lz) > m_keyFrameDist*m_keyFrameDist)||(std::fabs(m_krz - m_lrz)>m_keyFrameRot)) // Update drid if threshold reached
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

                // Publish LiDAR Cloud
                std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> pcl_points_from_vector(c.begin(), c.end());
                pcl::PointCloud<pcl::PointXYZ> pcl_cloud_from_vector;
                pcl_cloud_from_vector.points = pcl_points_from_vector;
                pcl_cloud_from_vector.width = pcl_points_from_vector.size();  
                pcl_cloud_from_vector.height = 1; 
                sensor_msgs::msg::PointCloud2 cloud_msg;
                pcl::toROSMsg(pcl_cloud_from_vector, cloud_msg);
                cloud_msg.header.frame_id = m_baseFrameId; 
                m_keyframePub->publish(cloud_msg);

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

        // Publish LiDAR Cloud
        std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> pcl_points_from_vector(c.begin(), c.end());
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

// Adapt Lidar pcl2 msg to custom PointXYZT data 
void DLO3DNode::convertAndAdapt(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& in_msg, pcl::PointCloud<PointXYZT>& out_cloud){
    pcl::console::VERBOSITY_LEVEL old_level = pcl::console::getVerbosityLevel();

    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

    pcl::fromROSMsg(*in_msg, out_cloud);

    pcl::console::setVerbosityLevel(old_level);

}

//! LiDAR Unwrap Functions - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

Closest_Filter_Result DLO3DNode::findClosestFilterData(const std::deque<Filter_Data>& Filter_queue, double target_timestamp) {
    
    Closest_Filter_Result result;
    result.found = false;

    auto it = std::lower_bound(Filter_queue.begin(), Filter_queue.end(), target_timestamp,
        [](const Filter_Data& data, double value) {
            return data.timestamp < value;
        });

    const Filter_Data* closest = nullptr;

    if (it == Filter_queue.begin()) {
        closest = &(*it);
    } 
    else if (it == Filter_queue.end()) {
        closest = &Filter_queue.back();
    } 
    else {
        auto prev_it = std::prev(it);
        if (std::abs(it->timestamp - target_timestamp) < std::abs(prev_it->timestamp - target_timestamp)) {
            closest = &(*it);
        } else {
            closest = &(*prev_it);
        }
    }

    if (closest && std::abs(target_timestamp - closest->timestamp) < 3 * T_imu) {
        result.Filter_data = *closest;
        result.found = true;
    }

    return result;
}

bool DLO3DNode::PointCloud2_to_PointXYZ_unwrap(pcl::PointCloud<PointXYZT> &in, std::vector<pcl::PointXYZ> &out, double scan_header_time){
   
    out.clear();

    double min_sq = m_minRange * m_minRange;
    double max_sq = m_maxRange * m_maxRange;

    if (m_lidarType != "ouster" && m_lidarType != "hesai") {
        
        out.reserve(in.size()); 

        for (const auto& pt : in.points) {
            if (!pcl::isFinite(pt)) continue;

            double d2 = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;

            if (d2 > min_sq && d2 < max_sq) {
                out.emplace_back(pt.x, pt.y, pt.z);
            }
        }
        return true; 
    }

    double valid_points_count = 0.0;
    double error_cnt = 0.0;

    double total_raw_pcl_points = static_cast<double>(in.points.size());
    double down_points = 0.0;
    double successfull_points = 0.0;
    double max_rel_time = 0.0;
    
    
    double scan_start_time = scan_header_time;
    double target_deskew_time  = scan_header_time;

    if (m_timestampMode == "START_OF_SCAN") {
        
        scan_start_time    = scan_header_time;
        target_deskew_time = scan_header_time + T_pcl; 
    } 
    else { 
        scan_start_time    = scan_header_time - T_pcl;
        target_deskew_time = scan_header_time;
    }

    double search_start_time  = target_deskew_time - 2*T_pcl;
    double search_end_time    = target_deskew_time + T_pcl;

    // Filter the IMU data within the desired time window
    std::deque<Filter_Data> filtered_deque;
    {
       
        std::lock_guard<std::mutex> lock(Filter_queue_mutex_);

        if (Filter_filtered_queue_.empty())
        {
            RCLCPP_WARN(this->get_logger(), "IMU queue is empty. Unwrap cannot be performed.");
            return false;
        }

        
        for (const auto& Filter_data : Filter_filtered_queue_) {
            if (Filter_data.timestamp >= search_start_time && Filter_data.timestamp <= search_end_time) {
                filtered_deque.push_back(Filter_data);
            }
        }
    }

    // Check if filtered data is available within the time window
    if (filtered_deque.empty()) {
        RCLCPP_WARN(this->get_logger(), "No data found in the specified time range.");
        return false;
    }
   
    // Find the Filter transform corresponding to the time of the last point
    Closest_Filter_Result Filter_f = findClosestFilterData(filtered_deque, target_deskew_time);
    
    tf2::Quaternion q_base;
    q_base.setRPY(Filter_f.Filter_data.roll, Filter_f.Filter_data.pitch, Filter_f.Filter_data.yaw);
    tf2::Vector3 base_position(Filter_f.Filter_data.x, Filter_f.Filter_data.y, Filter_f.Filter_data.z);
    tf2::Transform T_base;
    T_base.setRotation(q_base);
    T_base.setOrigin(base_position);


    // Iterate over each point in the point cloud
    for (const auto& pt : in.points)
    {
       
        if (!pcl::isFinite(pt) || (std::abs(pt.x) < 1e-4 && std::abs(pt.y) < 1e-4 && std::abs(pt.z) < 1e-4)) {
            continue; 
        }
        valid_points_count++;

        // Find the absolute point timestamp depending on the sensor
        tf2::Vector3 point_pcl(pt.x, pt.y, pt.z);
        double point_time, point_timestamp;

        if (m_lidarType == "ouster") 
        {
            
            double pt_rel_time = static_cast<double>(pt.t) * 1e-9;
            point_timestamp = scan_start_time + pt_rel_time;
            
        }

        else if (m_lidarType == "hesai") // pt.timestamp = secs.nanosecs -> Absolute
        {
            point_timestamp = pt.timestamp;
            if (point_timestamp < (target_deskew_time - T_pcl - 0.05) || point_timestamp > (target_deskew_time + 0.05)) {
                point_timestamp = target_deskew_time; 
            }
        }
        
        Closest_Filter_Result closest_Filter_data = findClosestFilterData(filtered_deque, point_timestamp);
        if(!closest_Filter_data.found){
            error_cnt++; 
            continue; 
            
        }else{
            successfull_points++;
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
        
        double d2 = point_pcl.x() * point_pcl.x() + point_pcl.y()  * point_pcl.y()  + point_pcl.z()  * point_pcl.z() ;

        pcl::PointXYZ compensated_p;
        compensated_p.x = point_pcl.x();
        compensated_p.y = point_pcl.y();
        compensated_p.z = point_pcl.z();
       
        // Add point to output if within range and if downsampling condition is met
        if (d2 > min_sq && d2 < max_sq)
        {
            out.push_back(compensated_p);
            down_points ++;
        }
        
    }
    if (valid_points_count == 0) return false;
    double success_ratio = (valid_points_count - error_cnt) / valid_points_count;
    RCLCPP_DEBUG_STREAM(this->get_logger(),
        "+ UnWarp INFO -> Succeed Ratio: " << (success_ratio * 100.0) 
        << "%. Total pcl points: " << total_raw_pcl_points 
        << ", valid_points: " << valid_points_count 
        << ", successful unwarped points: " << successfull_points 
        << ", error cnt: " << error_cnt);
    return success_ratio > 0.9; 
}

//! Auxiliar Functions - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

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