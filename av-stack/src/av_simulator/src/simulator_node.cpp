// av_simulator/simulator_node.cpp
// Minimal closed-loop simulator so the whole stack can run without a vehicle.
//   in : av_msgs/ControlCommand            on ~/input/command
//   out: sensor_msgs/PointCloud2 (LiDAR)   on ~/output/points
//        sensor_msgs/Imu                    on ~/output/imu
//        geometry_msgs/PoseStamped (GPS)    on ~/output/gps
//        std_msgs/Float64 (wheel speed)     on ~/output/wheel_speed
//        nav_msgs/Odometry (ground truth)   on ~/output/ground_truth
//
// It integrates a kinematic-bicycle model and renders a single parked obstacle
// into a synthetic LiDAR scan.
#include <cmath>
#include <memory>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "av_msgs/msg/control_command.hpp"
#include "av_common/math.hpp"

using namespace std::chrono_literals;

class SimulatorNode : public rclcpp::Node {
 public:
  SimulatorNode() : rclcpp::Node("simulator_node"), gen_(42) {
    wheelbase_ = declare_parameter("wheelbase", 2.7);
    max_accel_ = declare_parameter("max_accel", 2.0);
    max_decel_ = declare_parameter("max_decel", 4.0);
    obstacle_x_ = declare_parameter("obstacle_x", 100.0);
    obstacle_y_ = declare_parameter("obstacle_y", 0.0);

    cmd_sub_ = create_subscription<av_msgs::msg::ControlCommand>(
        "input/command", 10,
        [this](av_msgs::msg::ControlCommand::SharedPtr m) { cmd_ = *m; });

    points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("output/points", 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("output/imu", 50);
    gps_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("output/gps", 10);
    spd_pub_ = create_publisher<std_msgs::msg::Float64>("output/wheel_speed", 10);
    gt_pub_ = create_publisher<nav_msgs::msg::Odometry>("output/ground_truth", 10);

    dt_ = 0.02;  // 50 Hz
    timer_ = create_wall_timer(20ms, std::bind(&SimulatorNode::onTimer, this));
    RCLCPP_INFO(get_logger(), "simulator_node ready");
  }

 private:
  void onTimer() {
    // --- Integrate kinematic bicycle model. ---
    const double accel = cmd_.throttle * max_accel_ - cmd_.brake * max_decel_;
    v_ = std::max(0.0, v_ + accel * dt_);
    yaw_rate_ = (v_ / wheelbase_) * std::tan(cmd_.steer);
    yaw_ = av::normalizeAngle(yaw_ + yaw_rate_ * dt_);
    x_ += v_ * std::cos(yaw_) * dt_;
    y_ += v_ * std::sin(yaw_) * dt_;

    const rclcpp::Time stamp = now();

    // --- IMU (every tick). ---
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = "base_link";
    imu.angular_velocity.z = yaw_rate_ + noise(0.002);
    setQuat(imu.orientation, yaw_ + noise(0.01));
    imu_pub_->publish(imu);

    // --- Wheel speed (every tick). ---
    std_msgs::msg::Float64 spd;
    spd.data = v_ + noise(0.1);
    spd_pub_->publish(spd);

    // --- Ground truth. ---
    nav_msgs::msg::Odometry gt;
    gt.header.stamp = stamp;
    gt.header.frame_id = "map";
    gt.pose.pose.position.x = x_;
    gt.pose.pose.position.y = y_;
    setQuat(gt.pose.pose.orientation, yaw_);
    gt.twist.twist.linear.x = v_;
    gt_pub_->publish(gt);

    // --- GPS at ~2 Hz. ---
    if (++tick_ % 25 == 0) {
      geometry_msgs::msg::PoseStamped gps;
      gps.header.stamp = stamp;
      gps.header.frame_id = "map";
      gps.pose.position.x = x_ + noise(0.3);
      gps.pose.position.y = y_ + noise(0.3);
      setQuat(gps.pose.orientation, yaw_ + noise(0.02));
      gps_pub_->publish(gps);
    }

    // --- LiDAR at ~10 Hz. ---
    if (tick_ % 5 == 0) publishLidar(stamp);
  }

  void publishLidar(const rclcpp::Time& stamp) {
    // Build obstacle surface points in the ego frame.
    std::vector<std::array<float, 3>> pts;
    const double c = std::cos(-yaw_), s = std::sin(-yaw_);
    for (double ox = -1.0; ox <= 1.0; ox += 0.25)
      for (double oy = -1.0; oy <= 1.0; oy += 0.25)
        for (double oz = -1.0; oz <= 0.5; oz += 0.5) {
          const double rx = (obstacle_x_ + ox) - x_;
          const double ry = (obstacle_y_ + oy) - y_;
          pts.push_back({static_cast<float>(rx * c - ry * s),
                         static_cast<float>(rx * s + ry * c),
                         static_cast<float>(oz)});
        }

    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "base_link";
    sensor_msgs::PointCloud2Modifier mod(msg);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(pts.size());
    sensor_msgs::PointCloud2Iterator<float> ix(msg, "x"), iy(msg, "y"), iz(msg, "z");
    for (const auto& p : pts) {
      *ix = p[0]; *iy = p[1]; *iz = p[2];
      ++ix; ++iy; ++iz;
    }
    points_pub_->publish(msg);
  }

  static void setQuat(geometry_msgs::msg::Quaternion& q, double yaw) {
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
  }
  double noise(double sigma) {
    std::normal_distribution<double> d(0.0, sigma);
    return d(gen_);
  }

  double wheelbase_, max_accel_, max_decel_, obstacle_x_, obstacle_y_;
  double x_{0}, y_{0}, yaw_{0}, v_{0}, yaw_rate_{0}, dt_{0.02};
  std::uint64_t tick_{0};
  av_msgs::msg::ControlCommand cmd_;
  std::mt19937 gen_;
  rclcpp::Subscription<av_msgs::msg::ControlCommand>::SharedPtr cmd_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr gps_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr spd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gt_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimulatorNode>());
  rclcpp::shutdown();
  return 0;
}
