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

//!Default contructord
    DLO3DNode(const std::string &node_name)
        : Node(node_name), m_solver(&m_grid3d),imuFilter(1.0 / 388.0, 2.0) 
    {
        // Read node parameters
        this->declare_parameter<std::string>("in_cloud", "/cloud");
        this->declare_parameter<std::string>("in_cloud_aux", "/cloud_aux");
        this->declare_parameter<std::string>("in_imu", "/imu");
        this->declare_parameter<std::string>("base_frame_id", "base_link");
        this->declare_parameter<std::string>("odom_frame_id", "odom");
        this->declare_parameter<double>("hz_imu", 100.0);
        this->declare_parameter<double>("hz_cloud", 10.0);
        this->declare_parameter<bool>("aux_lidar_en", false);

        m_inCloudTopic = this->get_parameter("in_cloud").as_string();
        m_inCloudAuxTopic = this->get_parameter("in_cloud_aux").as_string();
        m_inImuTopic = this->get_parameter("in_imu").as_string();
        m_baseFrameId = this->get_parameter("base_frame_id").as_string();
        m_odomFrameId = this->get_parameter("odom_frame_id").as_string();
        T_pcl = 1 / this->get_parameter("hz_cloud").as_double();
        T_imu = 1 / this->get_parameter("hz_imu").as_double();
        aux_lidar_en = this->get_parameter("aux_lidar_en").as_bool();


        //Read DLO parameters
        this->declare_parameter<double>("keyframe_dist", 1.0);
        this->declare_parameter<double>("keyframe_rot", 0.1);
        this->declare_parameter<double>("tdf_grid_size_x", 50.0);
        this->declare_parameter<double>("tdf_grid_size_y", 50.0);
        this->declare_parameter<double>("tdf_grid_size_z", 50.0);
        this->declare_parameter<double>("tdf_grid_res", 0.05);
        this->declare_parameter<int>("solver_max_iter", 75);
        this->declare_parameter<int>("solver_max_threads", 20);
        this->declare_parameter<double>("min_range", 1.0);
        this->declare_parameter<double>("max_range", 100.0);
        this->declare_parameter<int>("pc_downsampling", 1);
        this->declare_parameter<double>("robust_kernel_scale", 1.0);
        m_keyFrameDist = this->get_parameter("keyframe_dist").as_double();
        m_keyFrameRot = this->get_parameter("keyframe_rot").as_double();
        m_tdfGridSizeX = this->get_parameter("tdf_grid_size_x").as_double();
        m_tdfGridSizeY = this->get_parameter("tdf_grid_size_y").as_double();
        m_tdfGridSizeZ = this->get_parameter("tdf_grid_size_z").as_double();
        m_tdfGridRes = this->get_parameter("tdf_grid_res").as_double();
        m_solverMaxIter = this->get_parameter("solver_max_iter").as_int();
        m_solverMaxThreads = this->get_parameter("solver_max_threads").as_int();
        m_minRange = this->get_parameter("min_range").as_double();
        m_maxRange = this->get_parameter("max_range").as_double();
        m_PcDownsampling = this->get_parameter("pc_downsampling").as_int();
        m_robusKernelScale = this->get_parameter("robust_kernel_scale").as_double();

        // Init internal variables
        m_tfImuCache = false;
        m_tfPointCloudCache = false;
        m_tfPointCloudAuxCache = false;
        m_tx = m_ty = m_tz = m_trx = m_try = m_trz = 0.0;
        m_keyFrameInit = converged = false;
        m_kx = m_ky = m_kz = m_krx = m_kry = m_krz= 0.0;
        m_rx = m_ry = m_rz = 0.0;

        //Init buffers
        m_tfBr = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        m_tfBuffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        m_tfListener = std::make_shared<tf2_ros::TransformListener>(*m_tfBuffer);
        
        // Launch publishers
        m_keyframePub = this->create_publisher<sensor_msgs::msg::PointCloud2>("keyframe", 10);
        m_cloudPub = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 10);
        m_unwrap = this->create_publisher<sensor_msgs::msg::PointCloud2>("unwraped_cloud", 10);
        m_odomPub = this->create_publisher<nav_msgs::msg::Odometry>("/odometry_pose", 10);

        // Services

        save_service_csv_ = this->create_service<std_srvs::srv::Trigger>(
            "/save_grid_csv",
            std::bind(&DLO3DNode::saveGridCSV, this, std::placeholders::_1, std::placeholders::_2)
        );

        save_service_pcd_ = this->create_service<std_srvs::srv::Trigger>(
            "/save_grid_pcd",
            std::bind(&DLO3DNode::saveGridPCD, this, std::placeholders::_1, std::placeholders::_2)
        );


        // // Setup TDF grid
        m_grid3d.setup(-m_tdfGridSizeX * 0.9, m_tdfGridSizeX * 0.1,
                    -m_tdfGridSizeY *0.5, m_tdfGridSizeY* 0.5,
                    -m_tdfGridSizeZ * 0.2, m_tdfGridSizeZ * 0.8,
                    m_tdfGridRes);
                      
        std::cout<<"setup made with "<<m_tdfGridSizeX<<" x "<<m_tdfGridSizeY<<" x "<<m_tdfGridSizeZ<<"."<<std::endl;

        // Solver Setup
        m_solver.setMaxIterations(m_solverMaxIter);
        m_solver.setMaxThreads(m_solverMaxThreads);
        m_solver.setRobustKernelScale(m_robusKernelScale);

        odom_test.open("odom_test.csv", std::ios::out);

        odom_test << "time,field.header.seq,field.header.stamp,field.pose.pose.position.x,field.pose.pose.position.y,field.pose.pose.position.z,field.pose.pose.orientation.x,field.pose.pose.orientation.y,field.pose.pose.orientation.z,field.pose.pose.orientation.w,field.twist.twist.linear.x,field.twist.twist.linear.y,field.twist.twist.linear.z";

  

        m_imuSub = this->create_subscription<sensor_msgs::msg::Imu>(
            m_inImuTopic,  5000,
            [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
                
                std::lock_guard<std::mutex> lock(queue_mutex_);
                imu_queue_.push(msg);

            });
        
         m_pcSub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            m_inCloudTopic,  5000,
            [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
                
                std::lock_guard<std::mutex> lock(queue_mutex_);
                pcl_queue_.push(msg);
            });

         m_pcSub_aux = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            m_inCloudAuxTopic,  5000,
            [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
                
                std::lock_guard<std::mutex> lock(queue_mutex_);
                pcl_aux_queue_.push(msg);
            });

        // Lanzar el hilo de procesamiento
        processing_thread_ = std::thread(&DLO3DNode::processQueues, this);
        std::cout<<"Prueba"<<std::endl;
    }

    // Default Destructor
    ~DLO3DNode(){
        stop_processing_ = true;
        queue_condition_.notify_all();
        if (processing_thread_.joinable()) {
            processing_thread_.join();
        }
        if (odom_test.is_open()) {
            odom_test.close();
        }


        RCLCPP_INFO(this->get_logger(), "Nodo cerrado correctamente.");

        
    }

private:
    //! Indicates that the local transfrom of the sensors is cached
    bool m_tfPointCloudCache, m_tfImuCache, m_tfPointCloudAuxCache;
    geometry_msgs::msg::TransformStamped m_staticTfImu, m_staticTfPointCloud, m_staticTfPointCloudAux;

    //! Key frame thresholds
    double m_keyFrameDist, m_keyFrameRot;

    //! Transform into current key-frame
    bool m_keyFrameInit, aux_lidar_en, converged;
    double T_pcl,T_imu;

    double m_tx, m_ty, m_tz, m_trx,m_try,m_trz;
    double m_kx, m_ky, m_kz, m_krx,m_kry,m_krz;
    double m_lx, m_ly, m_lz, m_lrx,m_lry,m_lrz;
    double m_x, m_y, m_z, a, vx, vy, vz;
    double m_x_last, m_y_last, m_z_last;

    //! IMU data
    double m_rx, m_ry, m_rz, roll_imu, pitch_imu, yaw_imu;
   	std::ofstream odom_test;

    //! Node parameters
    std::string m_inCloudTopic;
    std::string m_inCloudAuxTopic;
    std::string m_inImuTopic;
    std::string m_baseFrameId;
    std::string m_odomFrameId;
    double m_minRange, m_maxRange;
    ImuFilter imuFilter;

    //! ROS2 msgs and data
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tfBr;
    std::shared_ptr<tf2_ros::Buffer> m_tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> m_tfListener;
    
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imuSub;   
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_pcSub; 
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_pcSub_aux; 
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_keyframePub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_cloudPub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_unwrap;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odomPub;


    // Gestion de colas
    std::mutex queue_mutex_;
    std::condition_variable queue_condition_;
    bool stop_processing_;

    std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> pcl_queue_;
    std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> pcl_aux_queue_;
    std::queue<sensor_msgs::msg::Imu::ConstSharedPtr> imu_queue_;
    std::thread processing_thread_;

    // Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_csv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_pcd_;

    //! 3D distance grid
    TDF3D64 m_grid3d;
    double m_tdfGridSizeX, m_tdfGridSizeY, m_tdfGridSizeZ, m_tdfGridRes;

    //! Non-linear optimization solver
    DLL6DSolver m_solver;
    int m_solverMaxIter, m_solverMaxThreads;
    int m_PcDownsampling;
    double m_robusKernelScale;
    std::vector<double> refresh_id;
    std::vector<double> refresh_time;
    std::vector<double> conv_id;
    std::vector<double> conv_time;

    // Lidar Unwrap
    std::deque<Filter_Data> Filter_filtered_queue_;
    std::mutex Filter_queue_mutex_;

    // Variables provisionales
    double cnt_refresh = 0.0;
    double max_pcl_queue = 0.0;

    // Declaracion de funciones
    void processQueues();

    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud, const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_aux, bool is_aux_avaliable);
    void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg);
    void downsamplePointCloud(std::vector<pcl::PointXYZ>& points, float leaf_size);
    bool PointCloud2_to_PointXYZ_unwrap(sensor_msgs::msg::PointCloud2 &in, std::vector<pcl::PointXYZ> &out, double scan_start_time);
    Closest_Filter_Result findClosestFilterData(const std::deque<Filter_Data>& Filter_queue, double target_timestamp);

    Eigen::Matrix4f getTransformMatrix(const geometry_msgs::msg::TransformStamped& transform_stamped);
    
    double adjustYaw(double angle, double reference);
    double Pi2PiRange(double angle);
    double Floor_absolute( double value );

    void saveGridCSV(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void saveGridPCD(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response);

};

void DLO3DNode::saveGridCSV(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                 std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Recibida solicitud para guardar CSV. Iniciando en hilo separado...");

    // Ejecutar la exportación en un hilo separado
    std::thread([this]() {
        RCLCPP_INFO(this->get_logger(), "Generando CSV...");
        m_grid3d.exportGridToCSV("grid_data.csv",-2.0, m_tdfGridSizeX -2.0,
                      -m_tdfGridSizeY *0.5, m_tdfGridSizeY* 0.5,
                      -5.0, m_tdfGridSizeZ-5.0,
                      1);  // Usa el miembro de la clase
        RCLCPP_INFO(this->get_logger(), "CSV guardado correctamente.");
    }).detach();  // Desacoplar el hilo para que no bloquee

    response->success = true;
    response->message = "Exportación del CSV iniciada en segundo plano.";
}

void DLO3DNode::saveGridPCD(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                 std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Recibida solicitud para guardar PCD. Iniciando en hilo separado...");

    // Ejecutar la exportación en un hilo separado
    std::thread([this]() {
        RCLCPP_INFO(this->get_logger(), "Generando PCD...");
        m_grid3d.exportGridToPCD("grid_data.pcd",1);  // Usa el miembro de la clase
        RCLCPP_INFO(this->get_logger(), "PCD guardado correctamente.");
    }).detach();  // Desacoplar el hilo para que no bloquee

    response->success = true;
    response->message = "Exportación del PCD iniciada en segundo plano.";
}


void DLO3DNode::processQueues() {
    while (!stop_processing_) {
        std::unique_lock<std::mutex> lock(queue_mutex_);

        if (stop_processing_) {
            break;
        }

        if (!imu_queue_.empty() && !pcl_queue_.empty()) {
            auto imu_msg = imu_queue_.front();
            auto pcl_msg = pcl_queue_.front();

            if (pcl_queue_.size() > max_pcl_queue) {
                max_pcl_queue = pcl_queue_.size();
            }

            // Comparar timestamps
            if (imu_msg->header.stamp.sec < pcl_msg->header.stamp.sec || 
                (imu_msg->header.stamp.sec == pcl_msg->header.stamp.sec && imu_msg->header.stamp.nanosec < pcl_msg->header.stamp.nanosec)) {

                imu_queue_.pop();
                lock.unlock();
                imuCallback(imu_msg);
                lock.lock();

            } else {
                // Guardamos el mensaje actual antes de liberar el lock
                auto pcl_msg_safe = pcl_msg;
                
                pcl_queue_.pop();
                lock.unlock();

                bool is_aux_available = false;
                sensor_msgs::msg::PointCloud2::ConstSharedPtr pcl_msg_aux_safe;
                
                double pcl_timestamp = pcl_msg_safe->header.stamp.sec + pcl_msg_safe->header.stamp.nanosec * 1e-9;
                double min_diff = std::numeric_limits<double>::max();
                sensor_msgs::msg::PointCloud2::ConstSharedPtr best_match = nullptr;

                // Copia local de la cola auxiliar para evitar modificaciones concurrentes
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

                if (best_match) {
                    //std::cout << "[DEBUG] Best match encontrado con timestamp: " << best_match->header.stamp.sec << std::endl;
                    is_aux_available = true;
                    pcl_msg_aux_safe = best_match;
                } else {
                    //std::cout << "[DEBUG] No se encontró best match, usando pcl_msg" << std::endl;
                    pcl_msg_aux_safe = pcl_msg_safe;
                }

                // Llamada segura a pointcloudCallback
                pointcloudCallback(pcl_msg_safe, pcl_msg_aux_safe, is_aux_available);

                lock.lock();
                std::cout << "Tamaño de cola: " <<pcl_queue_.size() <<"/"<< max_pcl_queue<<"."<<std::endl;
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

   //std::cout<<"dt: "<<1.0/dt<<std::endl;

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

    // PROVISIONAL -----------------------------------------
    tf2::Quaternion q;
    tf2::fromMsg(msg->orientation, q);
    tf2::Matrix3x3(q).getRPY(roll_imu, pitch_imu, yaw_imu);
    pitch_imu = - pitch_imu;
    // -----------------------------------------------------

    // Rotate and compense for accelerations produced by off-centering the IMU (base frame must be in the center)
    tf2::Vector3 a(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    static tf2::Vector3 v_base_prev = v_base;
    tf2::Vector3 a_base = rot_matrix * a;
    /*
    a_base += ((v_base - v_base_prev) / dt).cross(tf2::Vector3(
        m_staticTfImu.transform.translation.x,
        m_staticTfImu.transform.translation.y,
        m_staticTfImu.transform.translation.z
    )) + v_base.cross(v_base.cross(tf2::Vector3(
        m_staticTfImu.transform.translation.x,
        m_staticTfImu.transform.translation.y,
        m_staticTfImu.transform.translation.z
    )));
    v_base_prev = v_base;*/

    // Modified msg
    sensor_msgs::msg::Imu modified_msg = *msg;
    modified_msg.angular_velocity.x = v_base.x();
    modified_msg.angular_velocity.y = v_base.y();
    modified_msg.angular_velocity.z = v_base.z();
    modified_msg.linear_acceleration.x = a_base.x();
    modified_msg.linear_acceleration.y = a_base.y();
    modified_msg.linear_acceleration.z = a_base.z();

    // Prediction
    if (!imuFilter.isInit()) {
        imuFilter.initialize(modified_msg); // v (rad/s) - a (m/s2)
    } else {
        
        imuFilter.predict(
            v_base.x(),v_base.y(),v_base.z(),
            a_base.x(),a_base.y(), a_base.z(),current_stamp);

        if(true){//a_mod>0.9 && a_mod<1.1
          //imuFilter.update_imu(roll_imu,pitch_imu,0.0025*0.0025);
        }

        imuFilter.getAngles(m_rx, m_ry, m_rz);

    }

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


     // Compute r difference between measurements
    static double prev_stamp = cloud->header.stamp.sec + cloud->header.stamp.nanosec * 1e-9;
    double current_stamp = cloud->header.stamp.sec + cloud->header.stamp.nanosec * 1e-9;
    double dt = current_stamp - prev_stamp;
    prev_stamp = current_stamp;

    ElapsedTime t;
    imuFilter.getposition(m_x, m_y, m_z);
    imuFilter.getAngles(m_rx, m_ry, m_rz);

    // Pre-cache transform for point-cloud to base frame and transform the pc 
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

    if (!m_tfPointCloudAuxCache && aux_lidar_en)
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

    pcl::PointCloud<PointXYZT> pcl_cloud;
    pcl::fromROSMsg(*cloud, pcl_cloud);

    Eigen::Matrix4f transform_matrix = getTransformMatrix(m_staticTfPointCloud);
    pcl::PointCloud<PointXYZT> transformed_cloud;
    pcl::transformPointCloud(pcl_cloud, transformed_cloud, transform_matrix);
    pcl::PointCloud<PointXYZT> final_cloud = transformed_cloud; 



    if (is_aux_avaliable) {
        std::cout<<"AUX ENABLE"<<std::endl;

        Eigen::Matrix4f transform_aux_matrix = getTransformMatrix(m_staticTfPointCloudAux);
        pcl::PointCloud<PointXYZT> pcl_cloud_aux;
        pcl::fromROSMsg(*cloud_aux, pcl_cloud_aux);

        pcl::PointCloud<PointXYZT> transformed_cloud_aux;
        pcl::transformPointCloud(pcl_cloud_aux, transformed_cloud_aux, transform_aux_matrix);
        final_cloud += transformed_cloud_aux;  
    }

    sensor_msgs::msg::PointCloud2 transformed_msg;
    pcl::toROSMsg(final_cloud, transformed_msg);

    double scan_end_time = cloud->header.stamp.sec + cloud->header.stamp.nanosec * 1e-9;
    std::vector<pcl::PointXYZ> c;
    bool unwrap_success = PointCloud2_to_PointXYZ_unwrap(transformed_msg, c, scan_end_time);
    if (!unwrap_success) {
        std::cout<<"UNWARP NOT SUCCEDDED"<<std::endl;
        c.clear();

        c.reserve(final_cloud.size());
        for (const auto& point : final_cloud) {
            c.emplace_back(point.x, point.y, point.z);
        }
    }

   // downsamplePointCloud(c, 0.2);

    // Pcl Proccesing
    converged = false;

    if(imuFilter.isInit()){

        bool translation_ok,rotation_ok = true;

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

            // Extraer valores locales
            double x_local = point_local.x();
            double y_local = point_local.y();
            double z_local = point_local.z();
            double roll_local = m_rx - m_trx;
            double pitch_local = m_ry - m_try;
            double yaw_local = m_rz - m_trz;

            double roll_local_prev, pitch_local_prev,yaw_local_prev,x_local_prev,y_local_prev,z_local_prev;

            roll_local_prev = roll_local;
            pitch_local_prev = pitch_local;
            yaw_local_prev = yaw_local;
            x_local_prev = x_local;
            y_local_prev = y_local;
            z_local_prev = z_local;

            converged = m_solver.solve(c, x_local, y_local, z_local, roll_local, pitch_local, yaw_local);
            converged = true;
            yaw_local = adjustYaw(yaw_local_prev, yaw_local);

            const double translation_threshold = 0.2;
            const double rotation_threshold = 2.0 * M_PI / 180.0;

            // Calcular diferencias
            double d_x = std::abs(x_local - x_local_prev);
            double d_y = std::abs(y_local - y_local_prev);
            double d_z = std::abs(z_local - z_local_prev);

            double droll = std::abs(roll_local - roll_local_prev);
            double dpitch = std::abs(pitch_local - pitch_local_prev);
            double dyaw = std::abs(yaw_local - yaw_local_prev);

            // Comprobar si el cambio está por debajo del umbral
            translation_ok = (d_x < translation_threshold) && (d_y < translation_threshold) && (d_z < translation_threshold);
            rotation_ok = (droll < rotation_threshold) && (dpitch < rotation_threshold) && (dyaw < rotation_threshold);

            std::cout << "Δx: " << d_x << ", Δy: " << d_y << ", Δz: " << d_z 
                << " | ΔRoll: " << droll * 180.0 / M_PI << "°, ΔPitch: " << dpitch * 180.0 / M_PI 
                << "°, ΔYaw: " << dyaw * 180.0 / M_PI << "°" << std::endl;

            if ( rotation_ok && translation_ok) {
                std::cout << "El cambio está dentro de los límites aceptables." << std::endl;
            } else {
                std::cout << "Se ha detectado un cambio significativo en la transformación." << std::endl;
            }

            if(converged){

                point_local = tf2::Vector3(x_local, y_local, z_local);

                // Aplicar transformación para obtener coordenadas globales
                point_global = grid_base * point_local;

                // Extraer valores transformados
                m_x = point_global.x();
                m_y = point_global.y();
                m_z = point_global.z();
                //std::cout<<"Position vel    : "<<m_x<<", "<<m_y<<", "<<m_z<<std::endl;
                //std::cout<<"Position vel pre: "<<m_x_last<<", "<<m_y_last<<", "<<m_z_last<<std::endl;

                // Ajuste de los ángulos
                m_rx = m_trx + roll_local;
                m_ry = m_try + pitch_local;
                m_rz = m_trz + yaw_local;

                vx = (m_x - m_x_last)/(T_pcl);
                vy = (m_y - m_y_last)/(T_pcl);
                vz = (m_z - m_z_last)/(T_pcl);
                //std::cout<<"Velocidades estimadas: "<<vx <<" "<<vy<<" "<<vz<<std::endl;
                tf2::Quaternion q_ori;
   		        q_ori.setRPY(m_rx, m_ry, m_rz);
                odom_test << std::fixed << std::setprecision(0)
                  << scan_end_time * 1000000000ULL << ","  // %time
                  << 0.0 << ","
                  << scan_end_time * 1000000000ULL<< ","       // field.header.stamp (poniéndolo a 0)
				  << std::fixed << std::setprecision(9)
                  << m_x << ","
                  << m_y << ","
                  << m_z << ","
                  << q_ori.x() << ","  // field.pose.pose.orientation.x (por ejemplo, 0 si no se tiene valor)
                  << q_ori.y() << ","  // field.pose.pose.orientation.y
                  << q_ori.z() << ","  // field.pose.pose.orientation.z
                  << q_ori.w() << "," 
                  << vx << "," 
                  << vy << "," 
                  << vz << "\n";

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
             
                // Aplicar transformación inversa para obtener coordenadas locales
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
/*
        std::cout<< "m_tx: "<<m_tx <<
                    "m_ty: "<<m_ty <<
                    "m_tz: "<<m_tz<<
                    "m_trx: "<<m_trx/(M_PI/180)<<
                    "m_try: "<<m_try/(M_PI/180)<<
                    "m_trz: "<<m_trz/(M_PI/180)<<std::endl;

        std::cout<<"m_kx: "<<m_kx <<
                    "m_ky: "<< m_ky<<
                    "m_kz: "<<m_kz<<
                    "m_krx: "<<m_krx/(M_PI/180)<<
                    "m_kry: "<<m_kry/(M_PI/180)<<
                    "m_krz: "<<m_krz/(M_PI/180)<<std::endl;*/
         std::cout<< "m_x: "<<m_x<<
                    "m_y: "<<m_y<<
                    "m_z: "<<m_z<<
                   "m_rx: "<<m_rx/(M_PI/180)<<
                    "m_ry: "<<m_ry/(M_PI/180)<<
                    "m_rz: "<<m_rz/(M_PI/180)<<std::endl;


                   
        // Update key-frame
        if(fabs(m_kx) > m_tdfGridSizeX * (2.0/3.0) || fabs(m_ky) > m_tdfGridSizeY* (2.0/3.0) ||fabs(m_kz) > m_tdfGridSizeZ* (2.0/3.0)|| !converged)
        {     
            if(!converged){
                std::cout<<"***** NO CONVERGE EL SOLVER ······················································"<<std::endl;
            }else{
                std::cout<<"***** CREANDO NUEVO MAPA ························································"<<std::endl;
            }

            // Add keyframe transform into odom
            m_tx  = m_x;
            m_ty  = m_y;
            m_tz  = m_z;
            m_trx = (m_rx);
            m_try = (m_ry);
            m_trz = m_rz;

            // Reset key-frame variables 
            m_kx = m_ky = m_kz = m_krx = m_kry = m_krz = 0.0;
            m_lx = m_ly = m_lz = m_lrx = m_lry = m_lrz = 0.0;

            m_grid3d.clear();


            // Load current point-cloud as new key-frame
            m_grid3d.loadCloud(c);

            // Publish current point cloud
           // m_keyframePub->publish(*cloud);

            // PROVISIONAL ---------------------------------------------------------------------
            
            cnt_refresh ++;
            if(!converged){
                conv_id.push_back(imuFilter.get_sec());
                conv_time.push_back(t.tock());
            }else{
           
                refresh_id.push_back(imuFilter.get_sec());
                refresh_time.push_back(t.tock());
            }

            // --------------------------------------------------------------------------------

           
        }
        else if( (m_kx-m_lx)*(m_kx-m_lx) + (m_ky-m_ly)*(m_ky-m_ly) + (m_kz-m_lz)*(m_kz-m_lz) > m_keyFrameDist*m_keyFrameDist)
        {

            std::cout << "Updated Map·················································································\n";

            m_lx = m_kx;
            m_ly = m_ky;
            m_lz = m_kz;

            m_lrx = m_krx;
            m_lry = m_kry;
            m_lrz = m_krz;

            tf2::Quaternion q;
            q.setRPY(m_lrx, m_lry, m_lrz);
            
            // Normalizar el cuaternión
            q.normalize();

            // Convertir de vuelta a RPY
            tf2::Matrix3x3 m(q);
            m.getRPY(m_lrx, m_lry, m_lrz);


            m_grid3d.loadCloud(c, m_kx, m_ky, m_kz, m_krx,m_kry,m_krz);

            std::cout << "Map updated ·················································································\n";

            // Publish current point cloud
            
        }

        // PROVISIONAL ---------------------------------------------------------------------
        std::cout<< "Mapa borrado " <<cnt_refresh<< " veces."<<std::endl;
        for (size_t i = 0; i < refresh_id.size(); ++i) {
            std::cout << "Refresh ID: " << refresh_id[i] << " - Tiempo: " << refresh_time[i] << "\n";
        }

        for (size_t i = 0; i < conv_id.size(); ++i) {
            std::cout << "Conv ID: " << conv_id[i] << " - Tiempo: " << conv_time[i] << "\n";
        }
        
        std::cout << "Optimization time: " << t.tock() << "Sec: " << imuFilter.get_sec()<<std::endl;
        std::cout<<"*******************************************************************************************+"<<std::endl;
        // ----------------------------------------------------------------------------------

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

        nav_msgs::msg::Odometry odomMsg;
        odomMsg.header.stamp = cloud->header.stamp;
        odomMsg.header.frame_id = m_odomFrameId;  // Marco de referencia de la odometría
        odomMsg.child_frame_id = m_baseFrameId;   // Marco del robot

        // Asignar la posición
        odomMsg.pose.pose.position.x = m_x;
        odomMsg.pose.pose.position.y = m_y;
        odomMsg.pose.pose.position.z = m_z;

        // Asignar la orientación
        odomMsg.pose.pose.orientation = tf2::toMsg(q.normalize());
        m_odomPub->publish(odomMsg);

        // PROVISIONAL ------------------------------------------------------------------------------------------------
        std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> pcl_points_from_vector(c.begin(), c.end());
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud_from_vector;
        pcl_cloud_from_vector.points = pcl_points_from_vector;
        pcl_cloud_from_vector.width = pcl_points_from_vector.size();  
        pcl_cloud_from_vector.height = 1; 
        sensor_msgs::msg::PointCloud2 unwrap_msg;
        pcl::toROSMsg(pcl_cloud_from_vector, unwrap_msg);
        unwrap_msg.header.frame_id = "base_link"; 
        m_unwrap->publish(unwrap_msg);
        // -----------------------------------------------------------------------------------------------------------
    }
}

double DLO3DNode::Floor_absolute( double value )
	{
	  if (value < 0.0)
		return ceil( value );
	  else
		return floor( value );
	}


double DLO3DNode::Pi2PiRange(double cont_angle)
	{
		double bound_angle = 0.0;
		if(fabs(cont_angle)<=M_PI)
			bound_angle= cont_angle;
		else
		{
			if(cont_angle > M_PI)
				bound_angle = (cont_angle-2*M_PI) - 2*M_PI*Floor_absolute((cont_angle-M_PI)/(2*M_PI));
			
			if(cont_angle < - M_PI)
				bound_angle = (cont_angle+2*M_PI) - 2*M_PI*Floor_absolute((cont_angle+M_PI)/(2*M_PI));
		}
		
		return bound_angle;
	}
void DLO3DNode::downsamplePointCloud(std::vector<pcl::PointXYZ>& points, float leaf_size) {
    if (points.empty()) {
        std::cerr << "[WARNING] La nube de puntos está vacía. No se aplicará downsampling." << std::endl;
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->points.assign(points.begin(), points.end());
    cloud->width = points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    pcl::PCLPointCloud2::Ptr cloud_pcl2(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud_filtered_pcl2(new pcl::PCLPointCloud2());
    pcl::toPCLPointCloud2(*cloud, *cloud_pcl2);

    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud_pcl2);
    sor.setLeafSize(leaf_size, leaf_size, leaf_size);
    sor.filter(*cloud_filtered_pcl2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloud_filtered_pcl2, *cloud_filtered);

    std::cout << "[DEBUG] VoxelGrid aplicado, tamaño original: " 
              << points.size() << ", tamaño resultante: " << cloud_filtered->points.size() << std::endl;

    if (cloud_filtered->empty()) {
        std::cerr << "[ERROR] La nube de puntos filtrada está vacía tras aplicar VoxelGrid." << std::endl;
        return;
    }

    points.assign(cloud_filtered->points.begin(), cloud_filtered->points.end());
}

Closest_Filter_Result DLO3DNode::findClosestFilterData(const std::deque<Filter_Data>& Filter_queue, double target_timestamp) {

    Closest_Filter_Result result;
    result.found = false;
    
    // Inicializar la búsqueda con el ultimo
    result.Filter_data = Filter_queue.back();
    double min_time_diff = std::abs(target_timestamp - result.Filter_data.timestamp);

    // Recorrer la cola
    for (const auto& Filter_data : Filter_queue) {
        double time_diff = std::abs(target_timestamp - Filter_data.timestamp);
        if (time_diff < min_time_diff) {
            min_time_diff = time_diff;
            result.Filter_data = Filter_data;
        }
    }

    if(min_time_diff < 5 * T_imu){
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

    // Verificar si hay datos de IMU disponibles
    if (Filter_filtered_queue_.empty())
    {
        RCLCPP_WARN(this->get_logger(), "La cola de IMU está vacía. No se puede realizar unwrap.");
        return false;
    }

    double error_cnt = 0.0;
    // Filtrar los datos dentro del intervalo de tiempo deseado
    double search_start_time = scan_end_time - 0.2;
    double search_end_time = scan_end_time + 0.2;
    
    // Filtrar los datos dentro del intervalo de tiempo deseado utilizando std::queue
    std::deque<Filter_Data> filtered_deque;

    // Filtrar los datos de la cola de IMU dentro del intervalo
    for (const auto& Filter_data : Filter_filtered_queue_) {
        if (Filter_data.timestamp >= search_start_time && Filter_data.timestamp <= search_end_time) {
            filtered_deque.push_back(Filter_data);  // Añadir al std::deque
        }
    }

    // Verificación si la cola está vacía
    if (filtered_deque.empty()) {
        RCLCPP_WARN(this->get_logger(), "No se encontraron datos en el intervalo de tiempo especificado.");
        return false;
    }

    // Definir iteradores para x, y, z, t
    sensor_msgs::PointCloud2Iterator<float> iterX(in, "x");
    sensor_msgs::PointCloud2Iterator<float> iterY(in, "y");
    sensor_msgs::PointCloud2Iterator<float> iterZ(in, "z");
    sensor_msgs::PointCloud2Iterator<uint32_t> iterT(in, "t");
    tf2::Transform T_base;
    
    // Encontrar el stamp relativo del ultimo punto del scan
    uint32_t last_stamp = 0;
    auto iterT_start = iterT;
    for (; iterT != iterT.end(); ++iterT) {
        last_stamp = *iterT;
    }
    iterT = iterT_start;

    // Encontrar transformada del Filter asociada al momento en el que se obtiene el ultimo punto del scan
    Closest_Filter_Result Filter_f = findClosestFilterData(filtered_deque, scan_end_time);
    
    tf2::Quaternion q_base;
    q_base.setRPY(Filter_f.Filter_data.roll, Filter_f.Filter_data.pitch, Filter_f.Filter_data.yaw);
    tf2::Vector3 base_position(Filter_f.Filter_data.x, Filter_f.Filter_data.y, Filter_f.Filter_data.z);
    T_base.setRotation(q_base);
    T_base.setOrigin(base_position);

    double total_points = static_cast<double>(in.width) * static_cast<double>(in.height);
   
    // Iterar sobre cada punto de la nube 
    for (; iterX != iterX.end(); ++iterX, ++iterY, ++iterZ, ++iterT)
    {
        
        tf2::Vector3 point_pcl(*iterX, *iterY, *iterZ);
        double point_time = scan_end_time - static_cast<double>(last_stamp)*1e-9 + static_cast<double>(*iterT)* 1e-9;
        Closest_Filter_Result closest_Filter_data = findClosestFilterData(filtered_deque, point_time);

        if(!closest_Filter_data.found){
            error_cnt++;  
        }
    
        // Transformada T_i del Filter 
        tf2::Quaternion closest_Filter_q;
        closest_Filter_q.setRPY(closest_Filter_data.Filter_data.roll, closest_Filter_data.Filter_data.pitch, closest_Filter_data.Filter_data.yaw);
        tf2::Vector3 closest_Filter_pos(closest_Filter_data.Filter_data.x, closest_Filter_data.Filter_data.y, closest_Filter_data.Filter_data.z);
        tf2::Transform T_i(closest_Filter_q,closest_Filter_pos);
        
        // Punto corregido
        tf2::Transform T = T_base.inverse() * T_i;
        point_pcl = T * point_pcl;

       
        // ---------------------------------------------------------------------------------------------------------------------------------------------------------------

        // Filtrar por rango y downsampling
        double min_sq = m_minRange * m_minRange;
        double max_sq = m_maxRange * m_maxRange;
        double d2 = point_pcl.x() * point_pcl.x() + point_pcl.y()  * point_pcl.y()  + point_pcl.z()  * point_pcl.z() ;

        pcl::PointXYZ compensated_p;
        compensated_p.x = point_pcl.x();
        compensated_p.y = point_pcl.y();
        compensated_p.z = point_pcl.z();
       

        if (d2 > min_sq && d2 < max_sq && (points % m_PcDownsampling == 0))
        {
            
            out.push_back(compensated_p);
        }
        points++;

    }

    if((total_points - error_cnt)/total_points < 0.9){

        std::cout<< " ************************** unwrapping error ********************************************************************************"<<std::endl;
        return false;
    }

    
     

    return true;

}

//! Auxiliar Functions

Eigen::Matrix4f DLO3DNode::getTransformMatrix(const geometry_msgs::msg::TransformStamped& transform_stamped)
    {
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

        // Rotación
        Eigen::Quaternionf q(
            transform_stamped.transform.rotation.w,
            transform_stamped.transform.rotation.x,
            transform_stamped.transform.rotation.y,
            transform_stamped.transform.rotation.z
        );
        Eigen::Matrix3f rotation = q.toRotationMatrix();

        // Traslación
        Eigen::Vector3f translation(
            transform_stamped.transform.translation.x,
            transform_stamped.transform.translation.y,
            transform_stamped.transform.translation.z
        );

        // Construir la matriz de transformación 4x4
        transform.block<3,3>(0,0) = rotation;
        transform.block<3,1>(0,3) = translation;

        return transform;
    }

double DLO3DNode::adjustYaw(double yaw_prev, double yaw_solved) {
    double delta_yaw = yaw_solved - yaw_prev;

    // Ajustar el delta para que esté en el rango [-π, π]
    while (delta_yaw > M_PI) delta_yaw -= 2 * M_PI;
    while (delta_yaw < -M_PI) delta_yaw += 2 * M_PI;

    // Devolver el yaw ajustado con acumulación
    return yaw_prev + delta_yaw;
}
  

//

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Crear una instancia de tu nodo
    auto node = std::make_shared<DLO3DNode>("dll3d_node");

    // Ejecutar el nodo
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
