// pcl_dt_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class PclDtNode : public rclcpp::Node {
public:
  PclDtNode()
  : Node("pcl_dt_node"),
    topic_(declare_parameter<std::string>("topic", "/points")),
    have_prev_(false)
  {
    // QoS con cola 5000 (igual que antes: best_effort + volatile)
    rclcpp::QoS qos(rclcpp::KeepLast(5000));
    qos.best_effort();
    qos.durability_volatile();

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic_, qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        const rclcpp::Time stamp(msg->header.stamp);
        const double t_sec = stamp.seconds();

        if (have_prev_) {
          const double dt = (stamp - prev_stamp_).seconds();
          const double hz = (dt > 0.0) ? (1.0 / dt) : std::numeric_limits<double>::infinity();

          RCLCPP_INFO(this->get_logger(),
            "PCL stamp: %.9f  |  dt: %.6f s  |  ~freq: %.2f Hz",
            t_sec, dt, hz);
        } else {
          RCLCPP_INFO(this->get_logger(),
            "PCL stamp: %.9f  |  dt: N/A (primer mensaje)",
            t_sec);
          have_prev_ = true;
        }

        prev_stamp_ = stamp;
      }
    );

    RCLCPP_INFO(get_logger(), "Suscrito a '%s' (PointCloud2) con cola 5000.", topic_.c_str());
  }

private:
  std::string topic_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Time prev_stamp_;
  bool have_prev_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PclDtNode>());
  rclcpp::shutdown();
  return 0;
}

