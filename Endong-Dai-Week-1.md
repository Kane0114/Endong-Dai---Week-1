
# Endong-Dai---Week-1
Weekly Report 1 for FURP23/24 - Omni-directional-Robot-Collision-Awareness

## Setting Up the Environment
- **Operating System**: Ubuntu 20.04 LTS
- **Framework Platform**: ROS1 Noetic
- **Simulation Platform**: Gazebo

## Following ROS 21 Talk Video Tutorials

### 1. Testing ROS Installation
- **Start ROS master**: 
  ```bash
  $ roscore
  ```
- **Launch TurtleSim simulator**:
  ```bash
  $ rosrun turtlesim turtlesim_node
  ```
- **Launch TurtleSim control node**:
  ```bash
  $ rosrun turtlesim turtle_teleop_key
  ```
- **Verification**: Successfully control the turtle using arrow keys, confirming ROS installation.

### 2. Understanding Core ROS Concepts
- **Node**: A process that performs computation.
- **Topic**: A communication channel used by nodes to exchange messages.
- **Message**: A data structure used in ROS for communication between nodes.
- **Service**: A synchronous communication mechanism in ROS.

### 3. Using ROS Command Line Tools
- **Example with TurtleSim**:
  - Run the three commands mentioned in section 1.
  - **View computation graph**: 
    ```bash
    $ rqt_graph
    ```
  - **List active nodes**:
    ```bash
    $ rosnode list
    ```

### Publishing Topic Messages
- **Publish a topic message**:
  ```bash
  $ rostopic pub -r 10 /turtle1/cmd_vel geometry_msgs/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
  ```
- **Verification**: The turtle keeps moving to the right. Use `Ctrl + C` to stop the script.

### Publishing Service Requests
- **Publish a service request**:
  ```bash
  $ rosservice call /spawn "x: 5.0
  y: 5.0
  theta: 0.0
  name: 'turtle2'"
  ```
- **Verification**: A new turtle spawns at the specified coordinates.

### Recording Topics
- **Record topics**:
  ```bash
  $ rosbag record -a -O cmd_record
  ```
- **Verification**: Control the turtle's movements freely. Use `Ctrl + C` to stop and save.

### Replaying Topics
- **Replay topics**:
  ```bash
  $ rosbag play cmd_record.bag
  ```
- **Verification**: Open the previously recorded file to replay the turtle's movements.

