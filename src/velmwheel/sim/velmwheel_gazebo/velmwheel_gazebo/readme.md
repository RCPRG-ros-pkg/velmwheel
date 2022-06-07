# Nodes

Package provides implementation of three Gazebo plugins simulating IMU sensor, LIDAR scanners and robot's driveline (which comes in the
two variants - realistic and idealistic)

## 1. **imu_gazebo**

Node providing access to measurement data from the simulated IMU sensor

### 1.1 Published Topics

#### ~/out (`sensor_msgs::msg::Imu`)

simulated IMU measurements

## 2. **lidar_l_gazebo** and **lidar_r_gazebo**

Nodes providing access to measurement data from the simulated LIDAR sensors

### 2.1 Published Topics

#### ~/scan(`sensor_msgs::msg::LaserScan`)

simulated LIDAR measurements

#### /tf(`geometry_msgs::msg::TransformStamped`)

transformation between `lidar_x_core` and `lidar_x_scan` (reference frame for the sensorical data)

## 3. **base_gazebo**

Node providing access to simulated and 'real' data concerning driveline of the robot

### 3.1 Published Topics

#### ~/joint_states(`sensor_msgs::msg::JointState`)

current state of the wheel's joints

#### ~/encoders(`velmwheel_msgs::msg::EncodersStamped`)

current angle and angular velocity measurements from encoders

#### ~/sim_pose(`geometry_msgs::msg::PoseStamped`)

pose of the robot in the simulated world

#### ~/sim_velocity(`geometry_msgs::msg::TwistStamped`)

velocity of the robot in the simulated world

#### /tf_static(`geometry_msgs::msg::TransformStamped`)

static transformation between `velmwheel` frame `base_link` (identity)

#### /tf(`geometry_msgs::msg::TransformStamped`)

transformation between `world` frame and `sim_velmwheel` representing pose of the robot in the world

### 3.2 Subscribed Topics

#### ~/controls(`velmwheel_msgs::msg::Wheels`)

current angular velocity setpoints for robot's wheels

### 3.2 Provided Services

#### ~/sim_set_rollers_friction(`velmwheel_msgs::srv::FrictionConfig`)

sets *mu1* and *mu2* friction coefficitents of wheel's rollers

#### ~/sim_set_inertia(`velmwheel_msgs::srv::InertiaConfig`)

sets inertia tensors of main robot's elements


# Models

At the moment package provides `.xacro` files extending URDF models provided by the `velmehweel_model` package with `<gazebo>` tags
required for simulation purpose. The `urdf/velmwheel.urdf.xacro` composes these with `velmehweel_model` models and provides a single
model description for both `rviz` and `gazebo` (there are some minor issues with the URDF format that need to be resolved at launchtime;
for more informations see `launch/sim.launch.py`)

## SDF model

Package provides also an old SDF model of the robot residing in `gazebo/models`. This is currently out-of-date and has been left for future
use due to issues described in `launch/sim.launch.py`
