#include <Eigen/Dense>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>

using namespace std::chrono_literals;

// Class declaration
class EightTrajectory : public rclcpp::Node {
public:
  EightTrajectory();
  ~EightTrajectory();

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void publish_wheel_speed();

  std::vector<double> velocity2twist(double dphi, double dx, double dy);
  std::vector<double> twist2wheels(double wz, double vx, double vy);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr timer_cb_grp_;
  rclcpp::CallbackGroup::SharedPtr odom_cb_grp_;

  std::vector<std::vector<double>> waypoints_;

  geometry_msgs::msg::Point current_position_;
  double phi; // current_yaw_

  // Robot parameters
  double l;
  double r;
  double w;

  // Transformation matrix H_4x3
  // Pseudo-inverse of H
  Eigen::MatrixXd H, H_pseudo_inverse;

  // Wheel speeds
  Eigen::Vector4d u;
};

// Constructor definition
EightTrajectory::EightTrajectory()
    : Node("eight_trajectory"), H(4, 3), H_pseudo_inverse(3, 4) {
  timer_cb_grp_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  odom_cb_grp_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions options;
  options.callback_group = odom_cb_grp_;

  cmd_vel_publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  // Create a subscriber for the /odom topic
  odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", 10,
      std::bind(&EightTrajectory::odom_callback, this, std::placeholders::_1),
      options);

  // Initialize waypoints
  waypoints_ = {
      {0.0, 1.0, -1.0},      // w1
      {0.0, 1.0, 1.0},       // w2
      {0.0, 1.0, 1.0},       // w3
      {-1.5708, 1.0, -1.0},  // w4
      {-1.5708, -1.0, -1.0}, // w5
      {0.0, -1.0, 1.0},      // w6
      {0.0, -1.0, 1.0},      // w7
      {0.0, -1.0, -1.0},     // w8
      {0.0, 0.0, 0.0}        // Stop
  };
  //   waypoints_ = {{0.0, 1.0, -1.0}, {0.0, 1.0, 1.0}};

  // Robot parameters
  l = 0.170 / 2; // Half of the wheel base distance
  r = 0.100 / 2; // Radius of the wheels
  w = 0.270 / 2; // Half of track width

  // Initialize the transformation matrix H_4x3 with the correct size
  H = Eigen::MatrixXd(4, 3);                   // Explicitly size the matrix
  H.row(0) << (-l - w) / r, 1.0 / r, -1.0 / r; // Row 0
  H.row(1) << (l + w) / r, 1.0 / r, 1.0 / r;   // Row 1
  H.row(2) << (l + w) / r, 1.0 / r, -1.0 / r;  // Row 2
  H.row(3) << (-l - w) / r, 1.0 / r, 1.0 / r;  // Row 3
  // Compute the pseudo-inverse of H
  H_pseudo_inverse = H.completeOrthogonalDecomposition().pseudoInverse();

  timer_ = this->create_wall_timer(
      1s, std::bind(&EightTrajectory::publish_wheel_speed, this),
      timer_cb_grp_);
  RCLCPP_INFO(this->get_logger(), "eight_trajectory node created.");
}

// Destructor definition
EightTrajectory::~EightTrajectory() {
  RCLCPP_INFO(this->get_logger(), "Shutting down eight_trajectory node.");
}

void EightTrajectory::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  // Extract position
  current_position_ = msg->pose.pose.position;

  // Extract orientation (quaternion)
  tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

  // Convert quaternion to Euler angles (roll, pitch, yaw)
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  phi = yaw;

  // Log the position and orientation for debugging
  RCLCPP_DEBUG(this->get_logger(), "Position: [x: %f, y: %f, z: %f]",
               current_position_.x, current_position_.y, current_position_.z);
  RCLCPP_DEBUG(this->get_logger(), "Orientation (yaw): %f", phi);
}

void EightTrajectory::publish_wheel_speed() {
  std_msgs::msg::Float32MultiArray msg;
  std::vector<double> wheel_speeds;
  const int iterations = 18;
  RCLCPP_INFO(this->get_logger(), "Trajectory started.");

  for (const auto &waypoint : waypoints_) {
    double dphi = waypoint[0];
    double dx = waypoint[1];
    double dy = waypoint[2];
    // Log the waypoint
    RCLCPP_INFO(this->get_logger(), "dp: %.2f, dx: %.2f, dy: %.2f", dphi, dx,
                dy);
    // Publish the Twist message for 3 seconds
    for (int i = 0; i < iterations; ++i) {
      // Compute the twist
      auto twist_v = velocity2twist(dphi, dx, dy);

      // Create and publish Twist message
      geometry_msgs::msg::Twist twist;
      twist.linear.x = twist_v[1];
      twist.linear.y = twist_v[2];
      twist.angular.z = twist_v[0];
      RCLCPP_INFO(this->get_logger(), "twist vx: %.2f, vy: %.2f, wz: %.2f",
                  twist_v[1], twist_v[2], twist_v[0]);
      cmd_vel_publisher_->publish(twist);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
  }

  RCLCPP_INFO(this->get_logger(), "Trajectory completed.");
  timer_->cancel(); // Stop the timer
  rclcpp::shutdown();
}

std::vector<double> EightTrajectory::velocity2twist(double dphi, double dx,
                                                    double dy) {
  std::vector<double> twist = {};

  twist.push_back(dphi);
  twist.push_back(dx * cos(phi) + dy * sin(phi));
  twist.push_back(-dx * sin(phi) + dy * cos(phi));

  return twist;
}

// Main function
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EightTrajectory>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}