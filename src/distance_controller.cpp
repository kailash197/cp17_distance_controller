#include <Eigen/Dense>
#include <geometry_msgs/msg/twist.hpp>
#include <limits>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>

class PID {
public:
  PID() : kp_(0.0), ki_(0.0), kd_(0.0), dt_(0.0) {}

  PID(double kp, double ki, double kd, double dt)
      : kp_(kp), ki_(ki), kd_(kd), dt_(dt), integral_(0.0), prev_error_(0.0) {}

  double compute(double error) {
    integral_ += error * dt_;
    integral_ = std::clamp(integral_, -0.5, 0.5); // Anti-windup
    double derivative = (error - prev_error_) / dt_;
    double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    prev_error_ = error;
    return output;
  }

  void reset_() {
    integral_ = 0.0;
    prev_error_ = 0.0;
  }

private:
  double kp_, ki_, kd_, dt_;
  double integral_;
  double prev_error_;
};

using namespace std::chrono_literals;

class DistanceController : public rclcpp::Node {
private:
  int scene_number_;
  double max_velocity_;
  double max_ang_velocity_;
  std::vector<std::vector<double>> waypoints_; //{dx, dy, dphi}

  void SelectWaypoints();
  void pid_controller();
  std::array<double, 3> local2globalframe(double vx, double vy, double avz);
  std::array<double, 3> global2localframe(double vx, double vy, double avz);

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr timer_cb_grp_;
  rclcpp::CallbackGroup::SharedPtr odom_cb_grp_;

  geometry_msgs::msg::Point current_position_;
  double phi; // current_yaw_

  PID pid_;
  double kp_sim, ki_sim, kd_sim;
  double kp_real, ki_real, kd_real;
  double time_step; // in milliseconds

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
  RCLCPP_DEBUG(this->get_logger(),
               "Position: [x: %f, y: %f, z: %f], Orientation (yaw): %f",
               current_position_.x, current_position_.y, current_position_.z,
               phi);
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

  // PID Parameters
  kp_sim = 1.5, ki_sim = 0.01, kd_sim = 0.1;
  kp_real = 1.5, ki_real = 0.01, kd_real = 0.10;
  time_step = 0.01; // in milliseconds
  SelectWaypoints();

  RCLCPP_INFO(this->get_logger(), "Distance Controller Initialized.");

  timer_ = this->create_wall_timer(
      1s, std::bind(&DistanceController::pid_controller, this), timer_cb_grp_);
}

void DistanceController::pid_controller() {
  geometry_msgs::msg::Twist cmd_vel;
  RCLCPP_INFO(this->get_logger(), "Trajectory started.");

  // Loop through each waypoint
  int counter = 0;
  for (const auto &waypoint : waypoints_) {
    pid_.reset_();
    // Transform waypoint to global frame
    auto [dx, dy, dphi] =
        local2globalframe(waypoint[0], waypoint[1], waypoint[2]);
    double target_x = current_position_.x + dx;
    double target_y = current_position_.y + dy;

    // Log the waypoint
    RCLCPP_INFO(this->get_logger(), "WP%u:[dx: %.2f, dy: %.2f, dphi: %.2f]",
                ++counter, waypoint[0], waypoint[1], waypoint[2]);
    RCLCPP_DEBUG(this->get_logger(), "Global dx: %.2f, dy: %.2f, dphi: %.2f",
                 dx, dy, dphi);

    rclcpp::Rate rate(int(1 / time_step)); // Control loop frequency
    double distance = std::numeric_limits<double>::max();
    // Run until distance is within tolerance
    while (distance > 0.01) {
      if (!rclcpp::ok()) { // Check if ROS is still running
        RCLCPP_WARN(this->get_logger(), "Trajectory Canceled.");
        timer_->cancel();         // Stop the timer
        odom_subscriber_.reset(); // Kill the odometry subscription
        rclcpp::shutdown();
        return;
      }

      // Calculate error in global frame
      double error_x = target_x - current_position_.x;
      double error_y = target_y - current_position_.y;
      distance = std::hypot(error_x, error_y);

      // Transform error to robot frame
      auto [robot_frame_x, robot_frame_y, robot_frame_z] =
          global2localframe(error_x, error_y, 0.0);
      //   double robot_frame_x = error_x * cos(phi) + error_y * sin(phi);
      //   double robot_frame_y = -error_x * sin(phi) + error_y * cos(phi);

      // Normalize direction
      double direction_x = robot_frame_x / distance;
      double direction_y = robot_frame_y / distance;
      RCLCPP_DEBUG(this->get_logger(), "Distance to target: %.2f", distance);

      // PID control
      double velocity = pid_.compute(distance);
      velocity = std::clamp(velocity, -max_velocity_, max_velocity_);

      cmd_vel.linear.x = direction_x * velocity;
      cmd_vel.linear.y = direction_y * velocity;
      cmd_vel_publisher_->publish(cmd_vel);

      rate.sleep();
    }

    // Now stop the bot
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    cmd_vel_publisher_->publish(cmd_vel);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  RCLCPP_INFO(this->get_logger(), "Trajectory completed.");
  timer_->cancel();         // Stop the timer
  odom_subscriber_.reset(); // Kill the odometry subscription
  rclcpp::shutdown();
}

std::array<double, 3>
DistanceController::global2localframe(double vx, double vy, double avz) {
  Eigen::Vector3d velocity(avz, vx, vy);

  Eigen::Matrix3d R;
  R << 1, 0, 0, 0, std::cos(phi), std::sin(phi), 0, -std::sin(phi),
      std::cos(phi);

  Eigen::Vector3d twist = R * velocity;

  return {twist(1), twist(2), twist(0)}; // dx, dy, dphi
}

std::array<double, 3>
DistanceController::local2globalframe(double vx, double vy, double avz) {
  Eigen::Vector3d velocity(avz, vx, vy);

  Eigen::MatrixXd R(3, 3);                      // 3x3 matrix
  R.row(0) << 1, 0, 0;                          // Row 0
  R.row(1) << 0, std::cos(phi), -std::sin(phi); // Row 1
  R.row(2) << 0, std::sin(phi), std::cos(phi);  // Row 2

  Eigen::Vector3d twist = R * velocity;

  return {twist(1), twist(2), twist(0)};
}

void DistanceController::SelectWaypoints() {
  switch (scene_number_) {
  case 1: // Simulation
    // Assign waypoints for Simulation
    RCLCPP_INFO(this->get_logger(), "Welcome to Simulation!");

    /* https://husarion.com/manuals/rosbot-xl/
    Maximum translational velocity = 0.8 m/s
    Maximum rotational velocity = 180 deg/s (3.14 rad/s)
    */
    max_velocity_ = 1.0;
    max_ang_velocity_ = 0.5;

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
    };

    pid_ = PID(kp_sim, ki_sim, kd_sim, time_step);
    break;

  case 2: // CyberWorld
    // Assign waypoints for CyberWorld
    RCLCPP_INFO(this->get_logger(), "Welcome to CyberWorld!");

    /* https://husarion.com/manuals/rosbot-xl/
    Maximum translational velocity = 0.8 m/s
    Maximum rotational velocity = 180 deg/s (3.14 rad/s)
    */
    max_velocity_ = 0.25;
    max_ang_velocity_ = 0.25;
    waypoints_ = {
        // check for angles
        {0.940, 0.0, 0.0},  // w1
        {0.0, -0.550, 0.0}, // w2
        {0.0, +0.550, 0.0}, // w3
        {-0.940, 0.0, 0.0}, // w4
    };

    pid_ = PID(kp_real, ki_real, kd_real, time_step);
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
