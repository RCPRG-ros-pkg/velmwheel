/* ============================================================================================================================ *//**
 * @file       communication.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 16th March 2022 4:37:20 pm
 * @modified   Wednesday, 25th May 2022 12:38:30 am
 * @project    engineering-thesis
 * @brief      Definitions of private builder API of the module implementing common routines related to ROS2 communication system
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __NODE_COMMON_COMMUNICATION_COMMUNICATION_H__
#define __NODE_COMMON_COMMUNICATION_COMMUNICATION_H__

/* =========================================================== Includes =========================================================== */

#include "node_common/communication.hpp"

/* =========================================================== Namespace ========================================================== */

namespace node_common {
namespace communication {

/* ========================================================= Base classes ========================================================= */

template<typename Derived>
Derived &InterfaceBuilderBase<Derived>::node(rclcpp::Node &node) {
    this->node_ = &node;
    return this->impl();
}


template<typename Derived>
Derived &InterfaceBuilderBase<Derived>::name(const std::string &name) {
    this->name_ = name;
    return this->impl();
}


template<typename Derived>
Derived &InterfaceBuilderBase<Derived>::impl() {
    return static_cast<Derived&>(*this);
}


template<typename Derived>
Derived &TopicInterfaceBuilderBase<Derived>::qos(const rclcpp::QoS &qos) {
    this->qos_ = qos;
    return this->impl();
}


template<typename Derived>
Derived &ServiceInterfaceBuilderBase<Derived>::qos_profile(const rmw_qos_profile_t &qos_profile) {
    this->qos_profile_ = qos_profile;
    return this->impl();
}


template<typename Derived>
Derived &ServiceInterfaceBuilderBase<Derived>::group(rclcpp::CallbackGroup::SharedPtr group) {
    this->group_ = group;
    return this->impl();
}

/* =========================================================== Publisher ========================================================== */

template<
    typename MessageT,
    typename AllocatorT
> PublisherInterfaceBuilder<MessageT, AllocatorT>::PublisherInterfaceBuilder(
    std::shared_ptr<rclcpp::Publisher<MessageT, AllocatorT>> &interface
) :
    interface_{interface}
{ }

template<
    typename MessageT,
    typename AllocatorT
> PublisherInterfaceBuilder<MessageT, AllocatorT>&
PublisherInterfaceBuilder<MessageT, AllocatorT>::options(const rclcpp::PublisherOptionsWithAllocator<AllocatorT> &options) {
    this->options_ = options;
    return *this;
}

template<
    typename MessageT,
    typename AllocatorT
> void PublisherInterfaceBuilder<MessageT, AllocatorT>::operator*() {

    // Check if node has been configured
    if(not this->node_)
        throw std::runtime_error{ "[PublisherInterfaceBuilder] Node has not been configured" };
        
    // Initialize the publisher
    this->interface_ = this->node_->template create_publisher<MessageT, AllocatorT>(
        this->name_,
        this->qos_,
        this->options_
    );
    // If initialization failed, print message and throw
    if(not this->interface_) {
        
        std::stringstream sstream;

        // Get fully qualified name of the node
        auto node_fqn = std::string(this->node_->get_fully_qualified_name());
        // Construct message
        sstream << "[" << node_fqn << "] Failed to create the '" << node_fqn + this->name_ << "' topic publisher";
        // Print message
        RCLCPP_FATAL_STREAM(this->node_->get_logger(), sstream.str());
        // Throw error
        throw std::runtime_error{ sstream.str() };
    }
    
}

/* ========================================================== Subscriber ========================================================== */

template<
    typename MessageT,
    typename AllocatorT
> SubscriberInterfaceBuilder<MessageT, AllocatorT>::SubscriberInterfaceBuilder(
    std::shared_ptr<rclcpp::Subscription<MessageT, AllocatorT>> &interface
) :
    interface_{interface}
{ }


template<
    typename MessageT,
    typename AllocatorT
> SubscriberInterfaceBuilder<MessageT, AllocatorT>&
SubscriberInterfaceBuilder<MessageT, AllocatorT>::options(
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> &options
) {
    this->options_ = options;
    return *this;
}


template<
    typename MessageT,
    typename AllocatorT
> SubscriberInterfaceBuilder<MessageT, AllocatorT>&
SubscriberInterfaceBuilder<MessageT, AllocatorT>::memory_alloc_strategy(
    typename MessageMemoryStrategy::SharedPtr msg_mem_strat
) {
    this->msg_mem_strat_ = msg_mem_strat;
    return *this;
}


template<
    typename MessageT,
    typename AllocatorT
> SubscriberInterfaceBuilder<MessageT, AllocatorT>&
SubscriberInterfaceBuilder<MessageT, AllocatorT>::callback(CallbackFreeType callback) {
    this->callback_ = callback;
    return *this;
}


template<typename MessageT,typename AllocatorT> 
template<typename T>
SubscriberInterfaceBuilder<MessageT, AllocatorT>&SubscriberInterfaceBuilder<MessageT, AllocatorT>::callback(T &obj, CallbackMemberType<T> callback) {
    this->callback_ = std::bind(callback, &obj, std::placeholders::_1);
    return *this;
}


template<
    typename MessageT,
    typename AllocatorT
> void SubscriberInterfaceBuilder<MessageT, AllocatorT>::operator*() {

    // Check if node has been configured
    if(not this->node_)
        throw std::runtime_error{ "[SubscriberInterfaceBuilder] Node has not been configured" };

    // Initialize the subscriber
    this->interface_ = this->node_->template create_subscription<MessageT, CallbackType, AllocatorT>(
        this->name_,
        this->qos_,
        std::move(this->callback_),
        this->options_,
        this->msg_mem_strat_
    );
    // If initialization failed, print message and throw
    if(not this->interface_) {
        
        std::stringstream sstream;

        // Get fully qualified name of the node
        auto node_fqn = std::string(this->node_->get_fully_qualified_name());
        // Construct message
        sstream << "[" << node_fqn << "] Failed to create the '" << node_fqn + this->name_ << "' topic subscriber";
        // Print message
        RCLCPP_FATAL_STREAM(this->node_->get_logger(), sstream.str());
        // Throw error
        throw std::runtime_error{ sstream.str() };
    }
}

/* ============================================================ Client ============================================================ */

template<typename ServiceT>
ClientInterfaceBuilder<ServiceT>::ClientInterfaceBuilder(
    std::shared_ptr<rclcpp::Client<ServiceT>> &interface
) :
    interface_{interface}
{ }


template<typename ServiceT>
void ClientInterfaceBuilder<ServiceT>::operator*() {

    // Check if node has been configured
    if(not this->node_)
        throw std::runtime_error{ "[ClientInterfaceBuilder] Node has not been configured" };

    // Initialize the client
    this->interface_ = this->node_->template create_client<ServiceT>(
        this->name_,
        this->qos_profile_,
        this->group_
    );
    // If initialization failed, print message and throw
    if(not this->interface_) {
        
        std::stringstream sstream;

        // Get fully qualified name of the node
        auto node_fqn = std::string(this->node_->get_fully_qualified_name());
        // Construct message
        sstream << "[" << node_fqn << "] Failed to create the '" << node_fqn + this->name_ << "' service client";
        // Print message
        RCLCPP_FATAL_STREAM(this->node_->get_logger(), sstream.str());
        // Throw error
        throw std::runtime_error{ sstream.str() };
    }
}

/* ============================================================ Client ============================================================ */

template<typename ServiceT>
ServiceInterfaceBuilder<ServiceT>::ServiceInterfaceBuilder(
    std::shared_ptr<rclcpp::Service<ServiceT>> &interface
) :
    interface_{interface}
{ }


template<typename ServiceT>
ServiceInterfaceBuilder<ServiceT>&
ServiceInterfaceBuilder<ServiceT>::callback(CallbackFreeType callback) {
    this->callback_ = callback;
    return *this;
}


template<typename ServiceT>
template<typename T>
ServiceInterfaceBuilder<ServiceT>&
ServiceInterfaceBuilder<ServiceT>::callback(
    T &obj,
    ServiceInterfaceBuilder<ServiceT>::CallbackMemberType<T> callback
) {
    this->callback_ = std::bind(callback, &obj, std::placeholders::_1, std::placeholders::_2);
    return *this;
}



template<typename ServiceT>
void ServiceInterfaceBuilder<ServiceT>::operator*() {

    // Check if node has been configured
    if(not this->node_)
        throw std::runtime_error{ "[ServiceInterfaceBuilder] Node has not been configured" };

    // Initialize the server
    this->interface_ = this->node_->template create_service<ServiceT, CallbackType>(
        this->name_,
        std::move(this->callback_),
        this->qos_profile_,
        this->group_
    );
    // If initialization failed, print message and throw
    if(not this->interface_) {
        
        std::stringstream sstream;

        // Get fully qualified name of the node
        auto node_fqn = std::string(this->node_->get_fully_qualified_name());
        // Construct message
        sstream << "[" << node_fqn << "] Failed to create the '" << node_fqn + this->name_ << "' service server";
        // Print message
        RCLCPP_FATAL_STREAM(this->node_->get_logger(), sstream.str());
        // Throw error
        throw std::runtime_error{ sstream.str() };
    }
}

/* ===================================================== I*nterface functions ===================================================== */

template<
    typename MessageT,
    typename AllocatorT
> PublisherInterfaceBuilder<MessageT, AllocatorT> make_publisher_builder(
    std::shared_ptr<rclcpp::Publisher<MessageT, AllocatorT>> &interface
) {
    return PublisherInterfaceBuilder<MessageT, AllocatorT>(interface);
}


template<
    typename MessageT,
    typename AllocatorT
> SubscriberInterfaceBuilder<MessageT, AllocatorT> make_subscriber_builder(
    std::shared_ptr<rclcpp::Subscription<MessageT, AllocatorT>> &interface
) {
    return SubscriberInterfaceBuilder<MessageT, AllocatorT>(interface);
}


template<typename ServiceT>
ClientInterfaceBuilder<ServiceT> make_client_builder(
    std::shared_ptr<rclcpp::Client<ServiceT>> &interface
) {
    return ClientInterfaceBuilder<ServiceT>(interface);
}


template<typename ServiceT>
ServiceInterfaceBuilder<ServiceT> make_service_builder(
    std::shared_ptr<rclcpp::Service<ServiceT>> &interface
) {
    return ServiceInterfaceBuilder<ServiceT>(interface);
}

/* ================================================================================================================================ */

} // End namespace communication
} // End namespace node_common

/* ================================================================================================================================ */

#endif
