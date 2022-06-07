/* ============================================================================================================================ *//**
 * @file       node.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 16th March 2022 4:37:20 pm
 * @modified   Wednesday, 25th May 2022 1:19:07 am
 * @project    engineering-thesis
 * @brief      Declarations of API implementing common routines related to ROS2 nodes
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __NODE_COMMON_NODE_H__
#define __NODE_COMMON_NODE_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <functional>
// ROS includes
#include "rclcpp/rclcpp.hpp"

/* =========================================================== Namespace ========================================================== */

namespace node_common {

/**
 * @brief Set of wrappers related to ROS node management
 */
namespace node {
    
/* ============================================================ Builder =========================================================== */

/**
 * @brief Builder class for the ROS2 wall timer
 */
class WallTimerBuilder {
public:

    /**
     * @brief Construct a new WallTimerBuilder object used to build a ROS2 wall timer
     * 
     * @param timer 
     *    handle to which the wall timer be attached
     */
    inline WallTimerBuilder(rclcpp::TimerBase::SharedPtr &timer);

public:

    /**
     * @brief Sets timer's associated node
     * @param node 
     *    node to be set
     * @returns 
     *    reference to *this
     */
    inline WallTimerBuilder &node(rclcpp::Node &node);

    /**
     * @brief Sets timer's period
     * @param period 
     *    period to be set
     * @returns 
     *    reference to *this
     */
    template<typename Rep, typename Ratio>
    inline WallTimerBuilder &period(const std::chrono::duration<Rep, Ratio> &period);

    /**
     * @overload WallTimerBuilder &period(const std::chrono::duration<Rep, Ratio> &)
     * @brief Sets timer's period
     * @param period 
     *    period to be set
     * @returns 
     *    reference to *this
     */
    inline WallTimerBuilder &period(const rclcpp::Duration &period);

    /**
     * @brief Sets timer's frequency
     * @param rate 
     *    rate to be set in [Hz]
     * @returns 
     *    reference to *this
     */
    inline WallTimerBuilder &rate(double rate);

    /**
     * @brief Sets timer's callback to the free function
     * @param callback 
     *    callback to be set
     * @returns 
     *    reference to *this
     */
    inline WallTimerBuilder &callback(void (*callback)());

    /**
     * @overload WallTimerBuilder &callback(void (*)())
     * @brief Sets server's callback to the member function
     * 
     * @tparam T 
     *    type of the method's class
     * @param obj 
     *    pointer to the object ot be bound with callback
     * @param callback 
     *    callback to be set
     * @returns 
     *    reference to *this
     */
    template<typename T>
    inline WallTimerBuilder &callback(T &obj, void (T::*callback)());

    /**
     * @overload WallTimerBuilder &callback(void (*)())
     * @brief Sets server's callback to the std::function
     * 
     * @param callback 
     *    callback to be set
     * @returns 
     *    reference to *this
     */
    inline WallTimerBuilder &callback(const std::function<void()> &callback);

public:

    /**
     * @brief Initializes wall timer with the current configruation
     */
    inline void operator*();
    
protected:

    /// Associated timer interface
    rclcpp::TimerBase::SharedPtr &timer_;
    /// Associated node
    rclcpp::Node *node_ { nullptr };
    /// Timer's period
    rclcpp::Duration period_ { rclcpp::Duration::from_seconds(0.0) };
    /// Server's callback
    std::function<void(void)> callback_;

};

/* ====================================================== Interface functions ===================================================== */

/**
 * @brief Creates ROS2 wall timer builder
 * 
 * @param timer 
 *    handle to which the wall timer be attached
 * @returns 
 *    builder object
 * 
 * @code
 * 
 *    // Register publisher interface for the given node with some parameters
 *    *node_common::communication::make_wall_timer_builder(timer)
 *       .node(some_node)
 *       .rate(some_rate_hz)
 *       .callback(some_callback)
 * 
 * @endcode
 */
inline WallTimerBuilder make_wall_timer_builder(rclcpp::TimerBase::SharedPtr &timer);

/* ====================================================== Auxiliary functions ===================================================== */

/**
 * @brief Prints standard message at node's start
 */
inline void print_hello(rclcpp::Node &node);

/**
 * @brief Prints standard message at node's stop
 */
inline void print_goodbye(rclcpp::Node &node);

/* ================================================================================================================================ */

} // End namespace node
} // End namespace node_common

#endif

/* ==================================================== Implementation includes =================================================== */

#include "node_common/node/node.hpp"

/* ================================================================================================================================ */
