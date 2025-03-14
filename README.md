# PID Distance Controller for ROSBot XL

This project involves designing and implementing a PID (Proportional-Integral-Derivative) controller to enable the ROSBot XL mobile robot to move precisely to predefined waypoints in the XY plane. The controller ensures the robot moves at high speed while avoiding overshooting or falling short of the target distance.

---

## **Overview**

The **Distance Controller** is responsible for:
1. Moving the robot to predefined waypoints in a straight line.
2. Using a PID control loop to calculate the required linear velocity.
3. Ensuring the robot stops precisely at each waypoint without overshooting or colliding with obstacles.

The controller is tested in both simulation (Gazebo) and the real-world environment (Cyberworld lab).

---

## **Tasks**

### **Task 1: PID Distance Controller (Simulation)**
- Develop a PID controller to move the robot to predefined waypoints in simulation.
- Test the controller in an empty Gazebo world.

### **Task 2: PID Distance Controller (Real Robot)**
- Adapt the controller for the real ROSBot XL in the Cyberworld lab.
- Test the controller with real-world waypoints.

---

## **Requirements**
- **ROS2 (Robot Operating System 2)**
- **C++** for implementing the PID controller.
- **Gazebo** for simulation testing.
- **Git** for version control (mandatory for the project).

---

## **Setup Instructions**

1. **Build the ROS2 Workspace**
   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

3. **Run Simulation**
   - Launch the empty Gazebo world:
     ```bash
     ros2 launch rosbot_xl_gazebo empty_simulation.launch.py
     ```

4. **Run the Distance Controller**
   - Execute the distance controller:
     ```bash
     ros2 run distance_controller distance_controller
     ```

5. **Real Robot Testing**
   - Book a session in the Cyberworld real robot lab.
   - Modify the waypoints for the real-world environment.
   - Run the controller with the real robot:
     ```bash
     ros2 run distance_controller distance_controller 2
     ```

---

## **Implementation Details**

### **Key Components**
1. **Waypoints:**
   - Predefined distances for the robot to move.
   - Example: `(+1.000, -1.000, +0.00000)` means move 1.0 meters forward and 1.0 meters to the right.

2. **PID Control Loop:**
   - **Proportional (P):** Adjusts velocity based on the current error.
   - **Integral (I):** Corrects for accumulated error over time.
   - **Derivative (D):** Predicts future error based on the rate of change.

3. **Odometry Subscriber:**
   - Reads the robot's current position from the `/rosbot_xl_base_controller/odom` or `/odometry/filtered` topic.

4. **Velocity Publisher:**
   - Publishes velocity commands to the `/cmd_vel` topic to move the robot.

---

## **License**
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
```
