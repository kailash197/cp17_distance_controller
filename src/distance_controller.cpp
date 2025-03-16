#include <Eigen/Dense>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>

using namespace std::chrono_literals;

class DistanceController : public rclcpp::Node {
private:
  int scene_number_;
  double max_velocity_;
  std::vector<std::vector<double>> waypoints_; //{dx, dy, dphi}

  void SelectWaypoints();
  void pid_controller();
  std::vector<double> velocity2twist(double vx, double vy, double avz);

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr timer_cb_grp_;
  rclcpp::CallbackGroup::SharedPtr odom_cb_grp_;

  geometry_msgs::msg::Point current_position_;
  double phi; // current_yaw_

  // Robot parameters
  double l;
  double r;
  double w;

  // Transformation matrix H_4x3 & Pseudo-inverse of H
  Eigen::MatrixXd H, H_pseudo_inverse;

  // Wheel speeds
  Eigen::Vector4d u;

public:
  DistanceController(int scene_number);
  ~DistanceController();
};

DistanceController::~DistanceController() {
  RCLCPP_INFO(this->get_logger(), "Distance Controller Terminated.");
}

void DistanceController::odom_callback(
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

DistanceController::DistanceController(int scene_number)
    : Node("distance_controller_node"), scene_number_(scene_number) {
  timer_cb_grp_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  odom_cb_grp_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions options;
  options.callback_group = odom_cb_grp_;

  cmd_vel_publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", 10,
      std::bind(&DistanceController::odom_callback, this,
                std::placeholders::_1),
      options);

  SelectWaypoints();
  max_velocity_ = 0.5;
  RCLCPP_INFO(this->get_logger(), "Distance Controller Initialized.");

  timer_ = this->create_wall_timer(
      1s, std::bind(&DistanceController::pid_controller, this), timer_cb_grp_);
}

void DistanceController::pid_controller() {
  double time_;
  geometry_msgs::msg::Twist twist;
  RCLCPP_INFO(this->get_logger(), "Trajectory started.");

  // Loop through each waypoint
  for (const auto &waypoint : waypoints_) {

    double dx = waypoint[0];
    double dy = waypoint[1];
    double dphi = waypoint[2];

    // Log the waypoint
    RCLCPP_INFO(this->get_logger(), "dx: %.2f, dy: %.2f, dp: %.2f", dx, dy,
                dphi);
    time_ = 1.0 / max_velocity_;

    twist.linear.x = max_velocity_ * dx;    // v_x
    twist.linear.y = max_velocity_ * dy;    // v_y
    twist.angular.z = max_velocity_ * dphi; // ω_z
    RCLCPP_INFO(this->get_logger(), "twist vx: %.2f, vy: %.2f, wz: %.2f",
                twist.linear.x, twist.linear.y, twist.angular.z);

    // Publish the Twist message for t second
    auto start_time = this->now(); // Get the current time
    while ((this->now() - start_time).seconds() < time_) { // Run for 1 second
      cmd_vel_publisher_->publish(twist);                  // Publish the twist
      std::this_thread::sleep_for(
          std::chrono::milliseconds(250)); // delay to avoid overloading
    }
  }

  RCLCPP_INFO(this->get_logger(), "Trajectory completed.");
  timer_->cancel();         // Stop the timer
  odom_subscriber_.reset(); // Kill the odometry subscription
  rclcpp::shutdown();
}

std::vector<double> DistanceController::velocity2twist(double vx, double vy,
                                                       double avz) {
  // Create input vector
  Eigen::Vector3d velocity(vx, vy, avz);

  // Define the transformation matrix R row-wise
  Eigen::MatrixXd R(3, 3);                      // 3x3 matrix
  R.row(0) << 1, 0, 0;                          // Row 0
  R.row(1) << 0, std::cos(phi), std::sin(phi);  // Row 1
  R.row(2) << 0, -std::sin(phi), std::cos(phi); // Row 2

  // Perform matrix-vector multiplication
  Eigen::Vector3d twist = R * velocity;

  // Convert Eigen::Vector3d to std::vector<double>
  return std::vector<double>{twist(0), twist(1), twist(2)};
}

void DistanceController::SelectWaypoints() {
  switch (scene_number_) {
  case 1: // Simulation
    // Assign waypoints for Simulation
    RCLCPP_INFO(this->get_logger(), "Welcome to Simulation!");
    // Waypoints: {dx,dy,dphi}
    waypoints_ = {
        {0.0, 1.0, 0.0},   // w1
        {0.0, -1.0, 0.0},  // w2
        {0.0, -1.0, 0.0},  // w3
        {0.0, 1.0, 0.0},   // w4
        {1.0, 1.0, 0.0},   // w5
        {-1.0, -1.0, 0.0}, // w6
        {1.0, -1.0, 0.0},  // w7
        {-1.0, 1.0, 0.0},  // w8
        {1.0, 0.0, 0.0},   // w9
        {-1.0, 0.0, 0.0},  // w10
        {0.0, 0.0, 0.0}    // Stop
    };
    break;

  case 2: // CyberWorld
    // Assign waypoints for CyberWorld
    RCLCPP_INFO(this->get_logger(), "Welcome to CyberWorld!");
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
    break;

  default:
    RCLCPP_ERROR(this->get_logger(), "Invalid Scene Number: %d", scene_number_);
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // Check if a scene number argument is provided
  int scene_number = 1; // Default scene number to simulation
  if (argc > 1) {
    scene_number = std::atoi(argv[1]);
  }
  // Check if the scene number is valid before creating the node
  if (scene_number != 1 && scene_number != 2) {
    std::cerr << "Error: Invalid Scene Number -- " << scene_number << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  // Add as input variable the scene number
  auto node = std::make_shared<DistanceController>(scene_number);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}