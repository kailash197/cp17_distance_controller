#include <rclcpp/rclcpp.hpp>
#include <vector>

class DistanceController : public rclcpp::Node {
private:
  int scene_number_;
  std::vector<std::vector<double>> waypoints_;
  void SelectWaypoints();

public:
  DistanceController(int scene_number);
  ~DistanceController();
};

DistanceController::~DistanceController() {
  RCLCPP_INFO(this->get_logger(), "Distance Controller Terminated.");
}

DistanceController::DistanceController(int scene_number)
    : Node("distance_controller_node"), scene_number_(scene_number) {
  SelectWaypoints();
  RCLCPP_INFO(this->get_logger(), "Distance Controller Initialized.");
  // Whatever you did before
}

void DistanceController::SelectWaypoints() {
  switch (scene_number_) {
  case 1: // Simulation
    // Assign waypoints for Simulation
    RCLCPP_INFO(this->get_logger(), "Welcome to Simulation!");
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