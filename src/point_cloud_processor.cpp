#include <dlio/point_cloud_processor.hpp>
#include <unordered_map>
#include <algorithm>
#include <cmath>

// --- Constructor / configuration ---

PointCloudProcessor::PointCloudProcessor(const Config& cfg) : cfg_(cfg) {}

void PointCloudProcessor::configure(const Config& cfg) { cfg_ = cfg; }

// --- convertAndAdapt ---

void PointCloudProcessor::convertAndAdapt(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& in_msg,
    pcl::PointCloud<PointXYZT>& out_cloud)
{
    pcl::console::VERBOSITY_LEVEL old_level = pcl::console::getVerbosityLevel();
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
    pcl::fromROSMsg(*in_msg, out_cloud);
    pcl::console::setVerbosityLevel(old_level);
}

// --- convertLivoxToPCL ---

void PointCloudProcessor::convertLivoxToPCL(
    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& in_msg,
    pcl::PointCloud<PointXYZT>& out_cloud)
{
    out_cloud.clear();
    out_cloud.reserve(in_msg->point_num);

    // Header stamp holds the scan base time.

    for (uint i = 0; i < in_msg->point_num; ++i) {
        PointXYZT pt;
        pt.x = in_msg->points[i].x;
        pt.y = in_msg->points[i].y;
        pt.z = in_msg->points[i].z;
        // offset_time is in ns relative to the scan base time
        pt.t = in_msg->points[i].offset_time;
        out_cloud.push_back(pt);
    }
}

// Fast hash for 3D voxel coordinates
struct VoxelHash {
    size_t operator()(const Eigen::Vector3i& voxel) const {
        const size_t p1 = 73856093;
        const size_t p2 = 19349663;
        const size_t p3 = 83492791;
        return (static_cast<size_t>(voxel.x()) * p1) ^ 
               (static_cast<size_t>(voxel.y()) * p2) ^ 
               (static_cast<size_t>(voxel.z()) * p3);
    }
};

// --- downsample ---

void PointCloudProcessor::downsample(std::vector<pcl::PointXYZ>& c, double leaf)
{
    if (leaf < 0.05 || c.empty()) return;

    // Running sum of the points falling in a voxel
    struct VoxelAccumulator {
        double x = 0.0, y = 0.0, z = 0.0;
        int count = 0;
    };

    // Hash map instead of PCL's grid, which avoids its voxel-count memory limit
    std::unordered_map<Eigen::Vector3i, VoxelAccumulator, VoxelHash> grid;
    
    double inv_leaf = 1.0 / leaf;
    const double max_r2 = cfg_.max_range * cfg_.max_range;

    // Accumulate each point into its voxel
    for (const auto& p : c) {
        // Drop NaNs and points beyond max range
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        if ((p.x*p.x + p.y*p.y + p.z*p.z) > max_r2) continue;

        // Voxel index
        Eigen::Vector3i idx(
            static_cast<int>(std::floor(p.x * inv_leaf)),
            static_cast<int>(std::floor(p.y * inv_leaf)),
            static_cast<int>(std::floor(p.z * inv_leaf))
        );

        auto& voxel = grid[idx];
        voxel.x += p.x;
        voxel.y += p.y;
        voxel.z += p.z;
        voxel.count++;
    }

    // Emit the centroid of each voxel
    c.clear();
    c.reserve(grid.size());
    
    for (const auto& pair : grid) {
        c.emplace_back(
            static_cast<float>(pair.second.x / pair.second.count),
            static_cast<float>(pair.second.y / pair.second.count),
            static_cast<float>(pair.second.z / pair.second.count)
        );
    }
}
// --- findClosestFilterData ---

Closest_Filter_Result PointCloudProcessor::findClosestFilterData(
    const std::deque<Filter_Data>& filter_queue,
    double target_timestamp) const
{
    Closest_Filter_Result result;
    result.found = false;

    auto it = std::lower_bound(
        filter_queue.begin(), filter_queue.end(), target_timestamp,
        [](const Filter_Data& data, double value) {
            return data.timestamp < value;
        });

    const Filter_Data* closest = nullptr;

    if (it == filter_queue.begin()) {
        closest = &(*it);
    } else if (it == filter_queue.end()) {
        closest = &filter_queue.back();
    } else {
        auto prev_it = std::prev(it);
        if (std::abs(it->timestamp - target_timestamp) <
            std::abs(prev_it->timestamp - target_timestamp)) {
            closest = &(*it);
        } else {
            closest = &(*prev_it);
        }
    }

    if (closest && std::abs(target_timestamp - closest->timestamp) < 0.05) {
        result.Filter_data = *closest;
        result.found = true;
    }

    return result;
}

// --- getPointTimestamp ---
// Maps a point's per-sensor timing field to an absolute timestamp.
// Add new LiDAR types here.

double PointCloudProcessor::getPointTimestamp(
    const PointXYZT& pt,
    double scan_start_time,
    double target_deskew_time) const
{
    if (cfg_.lidar_type == "ouster") {
        // Ouster: pt.t is relative time in ns from scan start
        double pt_rel_time = static_cast<double>(pt.t) * 1e-9;
        return scan_start_time + pt_rel_time;
    }

    if (cfg_.lidar_type == "hesai") {
        // Hesai: pt.timestamp is absolute (s.ns)
        double ts = pt.timestamp;
        if (ts < (target_deskew_time - cfg_.T_pcl - 0.05) ||
            ts > (target_deskew_time + 0.05)) {
            ts = target_deskew_time;
        }
        return ts;
    }

    if (cfg_.lidar_type == "livox") {
        // Livox CustomMsg: pt.t is the offset time in ns
        double pt_rel_time = static_cast<double>(pt.t) * 1e-9;
        return scan_start_time + pt_rel_time;
    }

    // New LiDAR types go below, e.g.:
    // if (cfg_.lidar_type == "velodyne") {
    //     return scan_start_time + pt.timestamp;
    // }

    // No per-point timestamp available
    return target_deskew_time;
}

// --- deskew (unwarp) ---

bool PointCloudProcessor::deskew(
    pcl::PointCloud<PointXYZT>& in,
    std::vector<pcl::PointXYZ>& out,
    double scan_header_time,
    const std::deque<Filter_Data>& filter_queue_ref,
    std::mutex& filter_queue_mutex)
{
    out.clear();

    double min_sq = cfg_.min_range * cfg_.min_range;
    double max_sq = cfg_.max_range * cfg_.max_range;

    // Non-deskewable LiDARs: range filter only
    if (cfg_.lidar_type != "ouster" && cfg_.lidar_type != "hesai" && cfg_.lidar_type != "livox") {
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

    // Time references
    double scan_start_time   = scan_header_time;
    double target_deskew_time = scan_header_time;

    if (cfg_.timestamp_mode == "START_OF_SCAN") {
        scan_start_time    = scan_header_time;
        target_deskew_time = scan_header_time + cfg_.T_pcl;
    } else {
        scan_start_time    = scan_header_time - cfg_.T_pcl;
        target_deskew_time = scan_header_time;
    }

    double search_start = target_deskew_time - 2 * cfg_.T_pcl;
    double search_end   = target_deskew_time + cfg_.T_pcl;


    // Keep IMU samples within the time window
    std::deque<Filter_Data> filtered_deque;
    {
        std::lock_guard<std::mutex> lock(filter_queue_mutex);
        if (filter_queue_ref.empty()) {
            return false;  // empty IMU queue
        }
        for (const auto& fd : filter_queue_ref) {
            if (fd.timestamp >= search_start && fd.timestamp <= search_end) {
                filtered_deque.push_back(fd);
            }
        }
    }

    if (filtered_deque.empty()) {
        return false;  // no IMU samples in the window
    }

    // Base transform (end-of-scan reference)
    Closest_Filter_Result filter_f = findClosestFilterData(filtered_deque, target_deskew_time);

    tf2::Quaternion q_base;
    q_base.setRPY(filter_f.Filter_data.roll, filter_f.Filter_data.pitch, filter_f.Filter_data.yaw);
    tf2::Vector3 base_position(filter_f.Filter_data.x, filter_f.Filter_data.y, filter_f.Filter_data.z);
    tf2::Transform T_base;
    T_base.setRotation(q_base);
    T_base.setOrigin(base_position);

    // Deskew each point
    double valid_points  = 0.0;
    double error_cnt     = 0.0;

    for (const auto& pt : in.points) {
        if (!pcl::isFinite(pt) ||
            (std::abs(pt.x) < 1e-4 && std::abs(pt.y) < 1e-4 && std::abs(pt.z) < 1e-4)) {
            continue;
        }
        valid_points++;

        double point_timestamp = getPointTimestamp(pt, scan_start_time, target_deskew_time);

        Closest_Filter_Result closest = findClosestFilterData(filtered_deque, point_timestamp);
        if (!closest.found) {
            error_cnt++;
            continue;
        }

        tf2::Quaternion q_i;
        q_i.setRPY(closest.Filter_data.roll, closest.Filter_data.pitch, closest.Filter_data.yaw);
        tf2::Vector3 pos_i(closest.Filter_data.x, closest.Filter_data.y, closest.Filter_data.z);
        tf2::Transform T_i(q_i, pos_i);

        tf2::Transform T = T_base.inverse() * T_i;
        tf2::Vector3 point_pcl = T * tf2::Vector3(pt.x, pt.y, pt.z);

        double d2 = point_pcl.x() * point_pcl.x() +
                     point_pcl.y() * point_pcl.y() +
                     point_pcl.z() * point_pcl.z();

        if (d2 > min_sq && d2 < max_sq) {
            pcl::PointXYZ p;
            p.x = point_pcl.x();
            p.y = point_pcl.y();
            p.z = point_pcl.z();
            out.push_back(p);
        }
    }

    if (valid_points == 0) return false;
    double success_ratio = (valid_points - error_cnt) / valid_points;
    return success_ratio > 0.8;
}