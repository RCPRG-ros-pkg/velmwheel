/* ============================================================================================================================ *//**
 * @file       robot_gazebo.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Tuesday, 8th March 2022 5:16:22 pm
 * @modified   Wednesday, 25th May 2022 11:47:13 pm
 * @project    engineering-thesis
 * @brief      Implementation of the WUT Velmwheel robot
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

#include <ignition/math/Vector3.hh>
#include "velmwheel/gazebo/robot_gazebo.hpp"
#include "node_common/communication.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace gazebo { 

/* ==================================================== Initialization routines =================================================== */

void Robot::Load(physics::ModelPtr parent, sdf::ElementPtr sdf) {

    /* ------------------------- Initialize ROS interface ------------------------ */

    // Create ROS node
    node = gazebo_ros::Node::Get(sdf);

    // [Publisher] Create ROS publisher interface for broadcasting actual position of the robot in the simulation
    *node_common::communication::make_publisher_builder(pose_pub)
        .node(*node)
        .name(SIM_POSE_TOPIC_NAME)
        .qos(TOPIC_QUEUE_SIZE);

    // [Publisher] Create ROS publisher interface for broadcasting actual velocity of the robot in the simulation
    *node_common::communication::make_publisher_builder(velocity_pub)
        .node(*node)
        .name(SIM_VELOCITY_TOPIC_NAME)
        .qos(TOPIC_QUEUE_SIZE);
    
    // [Service] Create ROS service interface for setting fricition coefficients of wheel_'s rolls at runtime
    *node_common::communication::make_service_builder(set_rolls_frictions_srv)
        .node(*node)
        .name(SET_ROLLS_FRICTIONS_TOPIC_NAME)
        .callback(*this, &Robot::SetFriction);

    // [Service] Create ROS service interface for setting inertia of robot's components at runtime
    *node_common::communication::make_service_builder(set_inertia_srv)
        .node(*node)
        .name(SET_INERTIA_TOPIC_NAME)
        .callback(*this, &Robot::SetInertia);

    // Initialize TF2 broadcaster
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*node);

    /* ------------------------ Initialize Gazebo interface ---------------------- */

    // Keep a handle to the model
    model = parent;

    // Bind 'on-world-upate-begin' Gazebo event with the plugin's implementation
    update_connection = event::Events::ConnectWorldUpdateBegin(std::bind(&Robot::OnUpdate, this));

    // Get common prefix of the model's elements in the SDF description
    link_prefix = std::string(model->GetName()).append("::");

    // Get handles to Gazebo models of the robot's wheels
    auto wheel_rr = model->GetLink(link_prefix + "wheel_rr");
    auto wheel_rl = model->GetLink(link_prefix + "wheel_rl");
    auto wheel_fr = model->GetLink(link_prefix + "wheel_fr");
    auto wheel_fl = model->GetLink(link_prefix + "wheel_fl");
    // If failed to parse wheel_s, report error and exit
    if(not wheel_rr or not wheel_rl or not wheel_fr or not wheel_fl) {
        auto msg = "[robot_gazebo] Failed to parse 'wheel_*' links from the robot descriptio";
        RCLCPP_FATAL(node->get_logger(), msg);
        throw std::runtime_error{ msg };
    }

    // Get handles to Gazebo models of the _collision engines of the robot's wheels
    wheel_rr_collision = wheel_rr->GetCollision("wheel_rr_collision");
    wheel_rl_collision = wheel_rl->GetCollision("wheel_rl_collision");
    wheel_fr_collision = wheel_fr->GetCollision("wheel_fr_collision");
    wheel_fl_collision = wheel_fl->GetCollision("wheel_fl_collision");
    // If failed to parse wheel_s, report error and exit
    if(not wheel_rr_collision or not wheel_rl_collision or not wheel_fr_collision or not wheel_fl_collision)
        RCLCPP_FATAL(node->get_logger(),  "[robot_gazebo] Failed to parse 'wheel_*_collision' models from the robot descriptio");

    // Log info message
    RCLCPP_INFO(node->get_logger(), "Running 'robot_gazebo' plugin...");
    
    /* --------------------------------------------------------------------------- */
}

/* ======================================================= Cyclical routines ====================================================== */

void Robot::OnUpdate() {
    
    // Get current system time-point
    auto now = node->get_clock()->now();
    // Get pose of the robot in the simulation with respct to the world's frame
    const ignition::math::Pose3d model_pose = model->WorldPose();
    // Get velocities of the robot in the simulation with respct to the world's frame
    const ignition::math::Vector3d model_lin_vel = model->WorldLinearVel();
    const ignition::math::Vector3d model_rot_vel = model->WorldAngularVel();

    /* --------------------- Update ROS topics (actual state) -------------------- */

    geometry_msgs::msg::PoseStamped sim_pose_msg;

    // Prepare header of the robot's position message
    sim_pose_msg.header.stamp    = now;
    sim_pose_msg.header.frame_id = "world";
    // Fill the body of the robot's position message
    sim_pose_msg.pose.position.x    = model_pose.Pos().X();
    sim_pose_msg.pose.position.y    = model_pose.Pos().Y();
    sim_pose_msg.pose.position.z    = model_pose.Pos().Z();
    sim_pose_msg.pose.orientation.x = model_pose.Rot().X();
    sim_pose_msg.pose.orientation.y = model_pose.Rot().Y();
    sim_pose_msg.pose.orientation.z = model_pose.Rot().Z();
    sim_pose_msg.pose.orientation.w = model_pose.Rot().W();
    // Update robot's position
    pose_pub->publish(sim_pose_msg);

    geometry_msgs::msg::TwistStamped sim_vel_msg;

    // Prepare header of the robot's velocity message
    sim_vel_msg.header.stamp    = now;
    sim_vel_msg.header.frame_id = tf_frame_name;
    // Fill the body of the robot's velocity message
    sim_vel_msg.twist.linear.x  = model_lin_vel.X();
    sim_vel_msg.twist.linear.y  = model_lin_vel.Y();
    sim_vel_msg.twist.linear.z  = model_lin_vel.Z();
    sim_vel_msg.twist.angular.x = model_rot_vel.X();
    sim_vel_msg.twist.angular.y = model_rot_vel.Y();
    sim_vel_msg.twist.angular.z = model_rot_vel.Z();
    // Update robot's velocity
    velocity_pub->publish(sim_vel_msg);
    
    /* ------------------ Update TF transforms (actual state) -------------------- */

    geometry_msgs::msg::TransformStamped sim_tf_model_transform_msg;

    // Prepare header of the robot's world-position's transformation message
    sim_tf_model_transform_msg.header.stamp    = now;
    sim_tf_model_transform_msg.header.frame_id = "world";
    sim_tf_model_transform_msg.child_frame_id  = tf_frame_name;
    // Fill the body of the robot's world-position's transformation message
    sim_tf_model_transform_msg.transform.translation.x = model_pose.Pos().X();
    sim_tf_model_transform_msg.transform.translation.y = model_pose.Pos().Y();
    sim_tf_model_transform_msg.transform.translation.z = model_pose.Pos().Z();
    sim_tf_model_transform_msg.transform.rotation.x    = model_pose.Rot().X();
    sim_tf_model_transform_msg.transform.rotation.y    = model_pose.Rot().Y();
    sim_tf_model_transform_msg.transform.rotation.z    = model_pose.Rot().Z();
    sim_tf_model_transform_msg.transform.rotation.w    = model_pose.Rot().W();
    // Update robot's world-position's transformation
    tf_broadcaster->sendTransform(sim_tf_model_transform_msg);
}


void Robot::SetFriction(
    const velmwheel_gazebo_msgs::srv::FrictionConfig::Request::SharedPtr req,
    velmwheel_gazebo_msgs::srv::FrictionConfig::Response::SharedPtr res
) {
    // Configure friction coefficients in the model
    wheel_rr_collision->GetSurface()->FrictionPyramid()->SetMuPrimary   (req->mu1);
    wheel_rr_collision->GetSurface()->FrictionPyramid()->SetMuSecondary (req->mu2);
    wheel_rl_collision->GetSurface()->FrictionPyramid()->SetMuPrimary   (req->mu1);
    wheel_rl_collision->GetSurface()->FrictionPyramid()->SetMuSecondary (req->mu2);
    wheel_fr_collision->GetSurface()->FrictionPyramid()->SetMuPrimary   (req->mu1);
    wheel_fr_collision->GetSurface()->FrictionPyramid()->SetMuSecondary (req->mu2);
    wheel_fl_collision->GetSurface()->FrictionPyramid()->SetMuPrimary   (req->mu1);
    wheel_fl_collision->GetSurface()->FrictionPyramid()->SetMuSecondary (req->mu2);

    // Log confirmation message
    RCLCPP_DEBUG(node->get_logger(), "Model's frictions set to: %lf (mu1), %lf (mu2)", req->mu1, req->mu2);
}


void Robot::SetInertia(
    const velmwheel_gazebo_msgs::srv::InertiaConfig::Request::SharedPtr req,
    velmwheel_gazebo_msgs::srv::InertiaConfig::Response::SharedPtr res
) {
    // Get pointers to the main links of the model
    physics::LinkPtr base_ptr     = model->GetLink(link_prefix + "base");
    physics::LinkPtr frontPtr     = model->GetLink(link_prefix + "front");
    physics::LinkPtr wheel_rr_ptr = model->GetLink(link_prefix + "wheel_rr");
    physics::LinkPtr wheel_rl_ptr = model->GetLink(link_prefix + "wheel_rl");
    physics::LinkPtr wheel_fr_ptr = model->GetLink(link_prefix + "wheel_fr");
    physics::LinkPtr wheel_fl_ptr = model->GetLink(link_prefix + "wheel_fl");
    // Get pointers to inertia models of these links
    physics::InertialPtr base_ine     = base_ptr->GetInertial();
    physics::InertialPtr front_ine    = frontPtr->GetInertial();
    physics::InertialPtr wheel_rr_ine = wheel_rr_ptr->GetInertial();
    physics::InertialPtr wheel_rl_ine = wheel_rl_ptr->GetInertial();
    physics::InertialPtr wheel_fr_ine = wheel_fr_ptr->GetInertial();
    physics::InertialPtr wheel_fl_ine = wheel_fl_ptr->GetInertial();

    // Configure inertia of the `base` link
    base_ine -> SetMass(req->base.m);
    base_ine -> SetInertiaMatrix(req->base.ixx, req->base.iyy, req->base.izz, req->base.ixy, req->base.ixz, req->base.iyz);
    base_ine -> SetCoG(req->base.com.x, req->base.com.y, req->base.com.z);
    // Configure inertia of the `front` link
    front_ine -> SetMass(req->front.m);
    front_ine -> SetInertiaMatrix(req->front.ixx, req->front.iyy, req->front.izz, req->front.ixy, req->front.ixz, req->front.iyz);
    front_ine -> SetCoG(req->front.com.x, req->front.com.y, req->front.com.z);

    // Configure inertia of the rear-right wheel
    wheel_rr_ine -> SetMass(req->wheel.m);
    wheel_rr_ine -> SetInertiaMatrix(req->wheel.ixx, req->wheel.iyy, req->wheel.izz, req->wheel.ixy, req->wheel.ixz, req->wheel.iyz);
    wheel_rr_ine -> SetCoG(req->wheel.com.x, req->wheel.com.y, req->wheel.com.z);
    // Configure inertia of the rear-left wheel
    wheel_rl_ine -> SetMass(req->wheel.m);
    wheel_rl_ine -> SetInertiaMatrix(req->wheel.ixx, req->wheel.iyy, req->wheel.izz, req->wheel.ixy, req->wheel.ixz, req->wheel.iyz);
    wheel_rl_ine -> SetCoG(req->wheel.com.x, req->wheel.com.y, req->wheel.com.z);
    // Configure inertia of the front-right wheel
    wheel_fr_ine -> SetMass(req->wheel.m);
    wheel_fr_ine -> SetInertiaMatrix(req->wheel.ixx, req->wheel.iyy, req->wheel.izz, req->wheel.ixy, req->wheel.ixz, req->wheel.iyz);
    wheel_fr_ine -> SetCoG(req->wheel.com.x, req->wheel.com.y, req->wheel.com.z);
    // Configure inertia of the front-left wheel
    wheel_fl_ine -> SetMass(req->wheel.m);
    wheel_fl_ine -> SetInertiaMatrix(req->wheel.ixx, req->wheel.iyy, req->wheel.izz, req->wheel.ixy, req->wheel.ixz, req->wheel.iyz);
    wheel_fl_ine -> SetCoG(req->wheel.com.x, req->wheel.com.y, req->wheel.com.z);

    // Recalculate masses of the links
    try {
        frontPtr->UpdateMass();
        base_ptr->UpdateMass();
        wheel_rr_ptr->UpdateMass();
        wheel_rl_ptr->UpdateMass();
        wheel_fr_ptr->UpdateMass();
        wheel_fl_ptr->UpdateMass();
    // On error print the message
    } catch(common::Exception &err) {
        RCLCPP_ERROR_STREAM(node->get_logger(), "Failed to reconfigure inertia of the model(" << err.GetErrorStr() << ")");
    }

    // Log success
    RCLCPP_DEBUG(node->get_logger(), "Model's inertia has been succesfully reconfigured");
}

/* ================================================================================================================================ */

// Register class as Gazebo plugin
GZ_REGISTER_MODEL_PLUGIN(Robot)

/* ================================================================================================================================ */

} // End namespace gazebo
