#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>

#include <std_srvs/srv/trigger.hpp>
#include <dlo3d/tdf3d_64.hpp>

#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>
#include <cctype>
#include <thread>
#include <mutex>

struct Pose {
  double stamp;              // seconds
  double x, y, z;
  double qx, qy, qz, qw;     // quaternion (x,y,z,w)
};

class PclPoseSync : public rclcpp::Node
{
public:
  PclPoseSync() : Node("pcl_pose_sync_node")
  {
    // --- Parámetros ---
    declare_parameter<std::string>("poses_csv", "leica_pose.csv");
    declare_parameter<std::string>("cloud_topic", "/os_cloud_node/points");
    declare_parameter<std::string>("map_frame", "map");      // este será el frame del GRID
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<double>("max_sync_dt", 0.05); // 50 ms
    declare_parameter<int>("downsample_factor", 1);

    // Extrínsecos LiDAR->base_link (rotación en grados y traslación en metros)
    declare_parameter<double>("lidar_roll_deg", 0.0);
    declare_parameter<double>("lidar_pitch_deg", 0.0);
    declare_parameter<double>("lidar_yaw_deg", 0.0);
    declare_parameter<double>("lidar_tx", 0.0);
    declare_parameter<double>("lidar_ty", 0.0);
    declare_parameter<double>("lidar_tz", 0.0);

    // Grid params
    declare_parameter<bool>("grid_enabled", false);
    declare_parameter<double>("grid_min_x", -5.0);
    declare_parameter<double>("grid_max_x",  70.0);
    declare_parameter<double>("grid_min_y", -35.0);
    declare_parameter<double>("grid_max_y",  35.0);
    declare_parameter<double>("grid_min_z",  -5.0);
    declare_parameter<double>("grid_max_z",  30.0);
    declare_parameter<double>("grid_res",     0.05);

    // Cargar parámetros
    get_parameter("poses_csv", poses_csv_);
    get_parameter("map_frame", map_frame_);
    get_parameter("base_frame", base_frame_);
    get_parameter("max_sync_dt", max_sync_dt_);
    get_parameter("downsample_factor", downsample_factor_);

    double r_deg, p_deg, y_deg, tx, ty, tz;
    get_parameter("lidar_roll_deg", r_deg);
    get_parameter("lidar_pitch_deg", p_deg);
    get_parameter("lidar_yaw_deg", y_deg);
    get_parameter("lidar_tx", tx);
    get_parameter("lidar_ty", ty);
    get_parameter("lidar_tz", tz);

    get_parameter("grid_enabled", grid_enabled_);
    get_parameter("grid_min_x", grid_min_x_);
    get_parameter("grid_max_x", grid_max_x_);
    get_parameter("grid_min_y", grid_min_y_);
    get_parameter("grid_max_y", grid_max_y_);
    get_parameter("grid_min_z", grid_min_z_);
    get_parameter("grid_max_z", grid_max_z_);
    get_parameter("grid_res", grid_res_);

    // --- Extrínsecos T_base_lidar ---
    const float r = static_cast<float>(r_deg * M_PI / 180.0);
    const float p = static_cast<float>(p_deg * M_PI / 180.0);
    const float y = static_cast<float>(y_deg * M_PI / 180.0);

    T_base_lidar_ = Eigen::Affine3f::Identity();
    T_base_lidar_.translation() << static_cast<float>(tx),
                                   static_cast<float>(ty),
                                   static_cast<float>(tz);
    T_base_lidar_.rotate(
      Eigen::AngleAxisf(r, Eigen::Vector3f::UnitX()) *
      Eigen::AngleAxisf(p, Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf(y, Eigen::Vector3f::UnitZ())
    );

    // --- Grid setup ---
    if (grid_enabled_) {
      std::lock_guard<std::mutex> lk(grid_mutex_);
      grid_.setup(grid_min_x_, grid_max_x_,
                  grid_min_y_, grid_max_y_,
                  grid_min_z_, grid_max_z_,
                  grid_res_);
      grid_.clear();
      RCLCPP_INFO(get_logger(),
                  "Grid enabled: [%.2f..%.2f]x[%.2f..%.2f]x[%.2f..%.2f], res=%.3f",
                  grid_min_x_, grid_max_x_,
                  grid_min_y_, grid_max_y_,
                  grid_min_z_, grid_max_z_, grid_res_);
    } else {
      RCLCPP_INFO(get_logger(), "Grid disabled.");
    }

    // --- Cargar CSV ---
    RCLCPP_INFO(get_logger(), "Loading poses from '%s'...", poses_csv_.c_str());
    loadPoses(poses_csv_);
    RCLCPP_INFO(get_logger(), "Loaded %zu poses.", poses_.size());

    rclcpp::SensorDataQoS sensor_qos;
    sensor_qos.keep_last(2000);              // cola de 300 mensajes (KEEP_LAST)
                                            // (SensorDataQoS ya es best_effort + volatile)

    std::string cloud_topic;
    get_parameter("cloud_topic", cloud_topic);

    pcl_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_topic, sensor_qos,
    std::bind(&PclPoseSync::onCloud, this, std::placeholders::_1));
    local_pub_  = create_publisher<sensor_msgs::msg::PointCloud2>("/local_cloud", 10);
    global_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/global_cloud", 10);
    odom_pub_   = create_publisher<nav_msgs::msg::Odometry>("/odom_from_csv", 10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Servicio para exportar el grid
    save_srv_ = create_service<std_srvs::srv::Trigger>(
      "/save_grid_pcd",
      std::bind(&PclPoseSync::saveGridPCD, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "Node ready.");
  }

private:
  // ---------- CSV ----------
  static bool isNumberStart(const std::string &s)
  {
    if (s.empty()) return false;
    unsigned i = 0;
    if (s[0] == '+' || s[0] == '-') i = 1;
    return i < s.size() && std::isdigit(static_cast<unsigned char>(s[i]));
  }

  void loadPoses(const std::string &filename)
  {
    std::ifstream file(filename);
    if (!file.is_open()) {
      RCLCPP_ERROR(get_logger(), "Cannot open CSV: %s", filename.c_str());
      return;
    }

    std::string line;
    if (!std::getline(file, line)) {
      RCLCPP_ERROR(get_logger(), "Empty CSV: %s", filename.c_str());
      return;
    }

    // Primera línea: cabecera o datos
    if (!isNumberStart(line)) {
      // header: sec,nsec,x,y,z,qx,qy,qz,qw
    } else {
      parseCsvLine(line);
    }

    while (std::getline(file, line)) parseCsvLine(line);
    file.close();

    if (poses_.empty()) {
      RCLCPP_ERROR(get_logger(), "No poses parsed from CSV.");
      return;
    }

    // Orden por tiempo
    std::sort(poses_.begin(), poses_.end(),
              [](const Pose &a, const Pose &b){ return a.stamp < b.stamp; });

    // stamps para lower_bound
    stamps_.resize(poses_.size());
    for (size_t i = 0; i < poses_.size(); ++i) stamps_[i] = poses_[i].stamp;
  }

  void parseCsvLine(const std::string &line)
  {
    if (line.empty()) return;
    std::stringstream ss(line);
    std::string fld;
    std::vector<std::string> tok;
    while (std::getline(ss, fld, ',')) tok.push_back(fld);
    if (tok.size() < 9) return; // sec,nsec,x,y,z,qx,qy,qz,qw

    try {
      const int64_t sec  = std::stoll(tok[0]);
      const int64_t nsec = std::stoll(tok[1]);

      Pose p;
      p.stamp = static_cast<double>(sec) + static_cast<double>(nsec) * 1e-9;

      p.x  = std::stod(tok[2]);
      p.y  = std::stod(tok[3]);
      p.z  = std::stod(tok[4]);

      p.qx = std::stod(tok[5]);
      p.qy = std::stod(tok[6]);
      p.qz = std::stod(tok[7]);
      p.qw = std::stod(tok[8]);

      // Normaliza el cuaternión
      const double nq = std::sqrt(p.qx*p.qx + p.qy*p.qy + p.qz*p.qz + p.qw*p.qw);
      if (nq > 1e-12) {
        p.qx /= nq; p.qy /= nq; p.qz /= nq; p.qw /= nq;
      }

      poses_.push_back(p);
    } catch (...) {
      // línea inválida -> ignorar
    }
  }

  // ---------- Búsqueda de la pose más cercana ----------
  std::pair<int, double> nearestPose(double t_sec) const
  {
    if (stamps_.empty()) return {-1, 1e12};

    auto it = std::lower_bound(stamps_.begin(), stamps_.end(), t_sec);
    if (it == stamps_.begin()) {
      int idx = 0;
      return {idx, std::fabs(stamps_[idx] - t_sec)};
    } else if (it == stamps_.end()) {
      int idx = static_cast<int>(stamps_.size()) - 1;
      return {idx, std::fabs(stamps_[idx] - t_sec)};
    } else {
      int idx_hi = static_cast<int>(std::distance(stamps_.begin(), it));
      int idx_lo = idx_hi - 1;
      double d_hi = std::fabs(stamps_[idx_hi] - t_sec);
      double d_lo = std::fabs(stamps_[idx_lo] - t_sec);
      if (d_lo <= d_hi) return {idx_lo, d_lo};
      return {idx_hi, d_hi};
    }
  }

  // ---------- Callback de nubes ----------
  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (poses_.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No poses loaded.");
      return;
    }

    const double t_cloud = rclcpp::Time(msg->header.stamp).seconds();

    // Encuentra la pose más cercana
    auto [idx, dt] = nearestPose(t_cloud);
    if (idx < 0) {
      RCLCPP_WARN(get_logger(), "No pose found for cloud.");
      return;
    }
    const Pose &best = poses_[static_cast<size_t>(idx)];

    // Siempre reporta Δt y decide
    const bool accept = (dt <= max_sync_dt_);
    if (!accept) {
      RCLCPP_WARN(get_logger(),
                  "Δt = %.6f s > max_sync_dt = %.6f s → REJECTED (cloud @ %.6f, pose @ %.6f).",
                  dt, max_sync_dt_, t_cloud, best.stamp);
      publishLocal(*msg); // solo local para visualizar
      return;
    } else {
      RCLCPP_INFO(get_logger(),
                  "Δt = %.6f s ≤ max_sync_dt = %.6f s → ASSOCIATED (cloud @ %.6f, pose @ %.6f).",
                  dt, max_sync_dt_, t_cloud, best.stamp);
    }

    // Convertir a PCL (frame LiDAR)
    pcl::PointCloud<pcl::PointXYZ> cloud_lidar;
    pcl::fromROSMsg(*msg, cloud_lidar);

    // Local: LiDAR -> base_link (para /local_cloud)
    pcl::PointCloud<pcl::PointXYZ> cloud_base;
    pcl::transformPointCloud(cloud_lidar, cloud_base, T_base_lidar_.matrix());
    sensor_msgs::msg::PointCloud2 local_msg;
    pcl::toROSMsg(cloud_base, local_msg);
    local_msg.header = msg->header;
    local_msg.header.frame_id = base_frame_;
    local_pub_->publish(local_msg);

    // Pose base en "map" (GT)
    Eigen::Affine3f T_map_base = Eigen::Affine3f::Identity();
    T_map_base.translation() << static_cast<float>(best.x),
                                 static_cast<float>(best.y),
                                 static_cast<float>(best.z);
    Eigen::Quaternionf q_map_base(static_cast<float>(best.qw),
                                  static_cast<float>(best.qx),
                                  static_cast<float>(best.qy),
                                  static_cast<float>(best.qz));
    q_map_base.normalize();
    T_map_base.rotate(q_map_base);

    // Si aún no hemos fijado el origen del GRID, usar esta primera pose asociada
    if (!grid_origin_set_) {
      T_grid_map_ = T_map_base.inverse(); // grid <- map
      grid_origin_set_ = true;
      RCLCPP_INFO(get_logger(), "GRID origin fixed at first associated pose.");
    }

    // Transformación completa LiDAR -> GRID
    Eigen::Affine3f T_grid_lidar = T_grid_map_ * T_map_base * T_base_lidar_;

    // Global (GRID): transformar puntos y publicar
    pcl::PointCloud<pcl::PointXYZ> cloud_grid;
    pcl::transformPointCloud(cloud_lidar, cloud_grid, T_grid_lidar.matrix());

    // Downsample por stride
    pcl::PointCloud<pcl::PointXYZ> cloud_grid_ds;
    const int stride = std::max(1, downsample_factor_);
    cloud_grid_ds.points.reserve((cloud_grid.size() + stride - 1) / stride);
    for (size_t i = 0; i < cloud_grid.points.size(); i += stride)
      cloud_grid_ds.points.push_back(cloud_grid.points[i]);
    cloud_grid_ds.width = static_cast<uint32_t>(cloud_grid_ds.points.size());
    cloud_grid_ds.height = 1;
    cloud_grid_ds.is_dense = true;

    sensor_msgs::msg::PointCloud2 global_msg;
    pcl::toROSMsg(cloud_grid_ds, global_msg);
    global_msg.header = msg->header;
    global_msg.header.frame_id = map_frame_; // el frame del GRID
    global_pub_->publish(global_msg);

    // Odom (en GRID)
    Eigen::Affine3f T_grid_base = T_grid_map_ * T_map_base;
    Eigen::Quaternionf q_grid_base(T_grid_base.rotation());
    Eigen::Vector3f t_grid_base = T_grid_base.translation();

    nav_msgs::msg::Odometry odom;
    odom.header = msg->header;
    odom.header.frame_id = map_frame_;
    odom.child_frame_id  = base_frame_;
    odom.pose.pose.position.x = t_grid_base.x();
    odom.pose.pose.position.y = t_grid_base.y();
    odom.pose.pose.position.z = t_grid_base.z();
    odom.pose.pose.orientation.x = q_grid_base.x();
    odom.pose.pose.orientation.y = q_grid_base.y();
    odom.pose.pose.orientation.z = q_grid_base.z();
    odom.pose.pose.orientation.w = q_grid_base.w();
    odom_pub_->publish(odom);

    // TF GRID(map_frame_) -> base_link
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header = msg->header;
    tf_msg.header.frame_id = map_frame_;
    tf_msg.child_frame_id  = base_frame_;
    tf_msg.transform.translation.x = t_grid_base.x();
    tf_msg.transform.translation.y = t_grid_base.y();
    tf_msg.transform.translation.z = t_grid_base.z();
    tf_msg.transform.rotation.x = q_grid_base.x();
    tf_msg.transform.rotation.y = q_grid_base.y();
    tf_msg.transform.rotation.z = q_grid_base.z();
    tf_msg.transform.rotation.w = q_grid_base.w();
    tf_broadcaster_->sendTransform(tf_msg);

    // -------- Insertar en GRID (si está habilitado) --------
    if (grid_enabled_) {
      // Los puntos YA están en el frame del GRID → NO pasar pose a loadCloud
      std::vector<pcl::PointXYZ> cloud_vec(cloud_grid.points.begin(), cloud_grid.points.end());
      std::lock_guard<std::mutex> lk(grid_mutex_);
      grid_.loadCloud(cloud_vec);
    }
  }

  void publishLocal(const sensor_msgs::msg::PointCloud2 &in)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud_lidar;
    pcl::fromROSMsg(in, cloud_lidar);

    pcl::PointCloud<pcl::PointXYZ> cloud_base;
    pcl::transformPointCloud(cloud_lidar, cloud_base, T_base_lidar_.matrix());

    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(cloud_base, out);
    out.header = in.header;
    out.header.frame_id = base_frame_;
    local_pub_->publish(out);
  }

  // ---------- Servicio: exportar grid a PCD ----------
  void saveGridPCD(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    if (!grid_enabled_) {
      res->success = false;
      res->message = "Grid disabled.";
      return;
    }

    const std::string filename = "grid_data_" + std::to_string(pcd_save_counter_++) + ".pcd";
    RCLCPP_INFO(get_logger(), "[saveGridPCD] exporting to '%s' ...", filename.c_str());

    std::thread([this, filename]() {
      std::lock_guard<std::mutex> lk(grid_mutex_);
      grid_.exportGridToPCD(filename, 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      RCLCPP_INFO(this->get_logger(), "[saveGridPCD] done: %s", filename.c_str());
    }).detach();

    res->success = true;
    res->message = "Export started: " + filename;
  }

private:
  // Parámetros / estado
  std::string poses_csv_;
  std::string map_frame_, base_frame_;
  double max_sync_dt_{0.05};
  int downsample_factor_{1};

  // Poses
  std::vector<Pose> poses_;
  std::vector<double> stamps_; // para lower_bound

  // Extrínsecos LiDAR->base_link
  Eigen::Affine3f T_base_lidar_{Eigen::Affine3f::Identity()};

  // Rebase online al primer MATCH (GRID origin)
  bool grid_origin_set_{false};
  Eigen::Affine3f T_grid_map_{Eigen::Affine3f::Identity()}; // grid <- map

  // Grid
  bool grid_enabled_{false};
  double grid_min_x_{-5.0}, grid_max_x_{70.0};
  double grid_min_y_{-35.0}, grid_max_y_{35.0};
  double grid_min_z_{-5.0},  grid_max_z_{30.0};
  double grid_res_{0.05};
  TDF3D64 grid_;
  std::mutex grid_mutex_;
  size_t pcd_save_counter_{0};

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_pub_, global_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_srv_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PclPoseSync>());
  rclcpp::shutdown();
  return 0;
}
