#pragma once

#include <string>
#include <vector>
#include <deque>
#include <mutex>
#include <cmath>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/print.h>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include <livox_ros_driver2/msg/custom_msg.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>

// Custom point type
struct PointXYZT
{
    PCL_ADD_POINT4D;
    double   timestamp;
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

// Auxiliary structs
struct Filter_Data {
    double timestamp;
    double x, y, z;
    double roll, pitch, yaw;
};

struct Closest_Filter_Result {
    Filter_Data Filter_data;
    bool found;
};

// LiDAR pre-processing: convert, deskew, downsample, range-filter.
// New LiDAR types only need a case in getPointTimestamp().
class PointCloudProcessor
{
public:
    struct Config {
        std::string lidar_type     = "ouster";
        std::string timestamp_mode = "START_OF_SCAN";
        double leaf_size_map       = 0.10;  // fine downsampling for the TDF map build
        double leaf_size_opt       = 0.25;  // coarser downsampling for the Ceres solver
        double min_range           = 1.0;
        double max_range           = 100.0;
        double T_pcl               = 0.1;   // 1/hz_cloud
        double T_imu               = 0.01;  // 1/hz_imu
    };

    PointCloudProcessor() = default;
    explicit PointCloudProcessor(const Config& cfg);

    void configure(const Config& cfg);

    // Pipeline stages

    void convertAndAdapt(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& in_msg,
                         pcl::PointCloud<PointXYZT>& out_cloud);

    void convertLivoxToPCL(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& in_msg,
                           pcl::PointCloud<PointXYZT>& out_cloud);

    // Deskew; returns false if IMU data is insufficient.
    bool deskew(pcl::PointCloud<PointXYZT>& in,
                std::vector<pcl::PointXYZ>& out,
                double scan_header_time,
                const std::deque<Filter_Data>& filter_queue,
                std::mutex& filter_queue_mutex);

    void downsample(std::vector<pcl::PointXYZ>& cloud, double leaf);

    void downsampleForMap(std::vector<pcl::PointXYZ>& cloud) { downsample(cloud, cfg_.leaf_size_map); }

    void downsampleForSolver(std::vector<pcl::PointXYZ>& cloud) { downsample(cloud, cfg_.leaf_size_opt); }

    // Helpers (public so the node can call them if needed)

    Closest_Filter_Result findClosestFilterData(
        const std::deque<Filter_Data>& filter_queue,
        double target_timestamp) const;

private:
    Config cfg_;

    // Per-point absolute timestamp; add new LiDAR types here.
    double getPointTimestamp(const PointXYZT& pt,
                            double scan_start_time,
                            double target_deskew_time) const;
};