/* ============================================================================================================================ *//**
 * @file       node.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 31st March 2022 10:16:02 pm
 * @modified   Friday, 1st April 2022 6:52:45 pm
 * @project    engineering-thesis
 * @brief      Definitions of API implementing common routines related to ROS2 nodes
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __NODE_COMMON_NODE_NODE_H__
#define __NODE_COMMON_NODE_NODE_H__

/* =========================================================== Includes =========================================================== */

#include "node_common/node.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace node_common {
namespace node {

/* ====================================================== Wall timer builder ====================================================== */

WallTimerBuilder::WallTimerBuilder(rclcpp::TimerBase::SharedPtr &timer) :
    timer_ { timer }
{ }


WallTimerBuilder &WallTimerBuilder::node(rclcpp::Node &node) {
    node_ = &node;
    return *this;
}


template<typename Rep, typename Ratio>
WallTimerBuilder &WallTimerBuilder::period(const std::chrono::duration<Rep, Ratio> &period) {
    period_ = rclcpp::Duration{ period };
    return *this;
}


WallTimerBuilder &WallTimerBuilder::period(const rclcpp::Duration &period) {
    period_ = period;
    return *this;
}


WallTimerBuilder &WallTimerBuilder::rate(double rate) {
    if(rate > 0.0)
        period_ = rclcpp::Duration::from_seconds(1.0 / rate);
    return *this;
}


WallTimerBuilder &WallTimerBuilder::callback(void (*callback)()) {
    callback_ = callback;
    return *this;
}


template<typename T>
WallTimerBuilder &WallTimerBuilder::callback(T &obj, void (T::*callback)()) {
    callback_ = std::bind(callback, &obj);
    return *this;
}


WallTimerBuilder &WallTimerBuilder::callback(const std::function<void(void)> &callback) {
    callback_ = callback;
    return *this;
}


void WallTimerBuilder::operator*() {

    // Check if node has been configured
    if(not this->node_)
        throw std::runtime_error{ "[WallTimerBuilder] Node has not been configured" };
        
    // Initialize the server
    this->timer_ = this->node_->create_wall_timer(
        this->period_.to_chrono<std::chrono::duration<double>>(),
        this->callback_
    );
    
    // If initialization failed, print message and throw
    if(not this->timer_) {
        
        std::stringstream sstream;

        // Get fully qualified name of the node
        auto node_fqn = std::string(this->node_->get_fully_qualified_name());
        // Construct message
        sstream << "[" << node_fqn << "] Failed to create the wall timer for the '" << node_fqn << "' node";
        // Print message
        RCLCPP_FATAL_STREAM(this->node_->get_logger(), sstream.str());
        // Throw error
        throw std::runtime_error{ sstream.str() };
    }
}

/* ====================================================== Interface functions ===================================================== */


WallTimerBuilder make_wall_timer_builder(rclcpp::TimerBase::SharedPtr &timer) {
    return WallTimerBuilder(timer);
}

/* ====================================================== Auxiliary functions ===================================================== */

void print_hello(rclcpp::Node &node) {
    RCLCPP_INFO_STREAM(node.get_logger(), "The '" << node.get_fully_qualified_name() << "' node has been started");
}


void print_goodbye(rclcpp::Node &node) {
    RCLCPP_INFO_STREAM(node.get_logger(), "The '" << node.get_fully_qualified_name() << "' node has been stoppeed");
}

/* ================================================================================================================================ */

} // End namespace node
} // End namespace node_common

#endif
