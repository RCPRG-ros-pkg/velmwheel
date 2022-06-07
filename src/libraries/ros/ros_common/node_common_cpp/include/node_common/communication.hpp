/* ============================================================================================================================ *//**
 * @file       communication.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 19th May 2022 9:53:09 am
 * @modified   Wednesday, 25th May 2022 1:17:47 am
 * @project    engineering-thesis
 * @brief      
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* ============================================================================================================================ *//**
 * @file       communication.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 16th March 2022 4:37:20 pm
 * @modified   Wednesday, 25th May 2022 12:46:27 am
 * @project    engineering-thesis
 * @brief      Declarations of API implementing common routines related to ROS2 communication system
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __NODE_COMMON_COMMUNICATION_H__
#define __NODE_COMMON_COMMUNICATION_H__

/* =========================================================== Includes =========================================================== */

#include <functional>
#include "rclcpp/rclcpp.hpp"

/* =========================================================== Namespace ========================================================== */

namespace node_common {

/**
 * @brief Set of wrappers related to ROS communication interfaces
 */
namespace communication {

/* ========================================================= Base classes ========================================================= */

/**
 * @brief Base class of te builders hierarchy creating ROS2 communciation interfaces
 * 
 * @tparam Derived
 *    class deriving from this class
 */
template<typename Derived>
class InterfaceBuilderBase {
public:

    /**
     * @brief Construct a new InterfaceBuilderBase object
     */
    InterfaceBuilderBase() = default;

public:

    /**
     * @brief Sets node of the interface
     * @param node 
     *    node to be set
     * @returns 
     *    reference to *this casted to @tparam Derived
     */
    inline Derived &node(rclcpp::Node &node);

    /**
     * @brief Sets name of the interface
     * @param name 
     *    name to be set
     * @returns 
     *    reference to *this casted to @tparam Derived
     */
    inline Derived &name(const std::string &name);

protected:

    /**
     * @returns 
     *    reference to *this casted to the @tparam Derived
     */
    inline Derived &impl();

protected:

    /// Reference to the ROS node
    rclcpp::Node *node_;
    /// Name of the topic
    std::string name_;

};

/**
 * @brief Base class of te builders hierarchy creating ROS2 communciation interfaces
 *    for topic (i.e. publisher/subscriber)
 * 
 * @tparam Derived
 *    class deriving from this class
 */
template<typename Derived>
class TopicInterfaceBuilderBase : public InterfaceBuilderBase<Derived> {
public:

    /// Depth of the topic's history used to initialize default QoS
    static constexpr std::size_t DEFAULT_QOS_HISTORY_DEPTH = 10;

public:

    /**
     * @brief Construct a new TopicInterfaceBuilderBase object
     */
    TopicInterfaceBuilderBase() = default;

public:

    /**
     * @brief Sets QoS of the interface
     * @param qos 
     *    QoS to be set
     * @returns 
     *    reference to *this casted to @tparam Derived
     */
    inline Derived &qos(const rclcpp::QoS &qos);

protected:

    /// QoS of the topic interface
    rclcpp::QoS qos_{ DEFAULT_QOS_HISTORY_DEPTH };

};

/**
 * @brief Base class of te builders hierarchy creating ROS2 communciation interfaces
 *    for services (i.e. server/client)
 * 
 * @tparam Derived
 *    class deriving from this class
 */
template<typename Derived>
class ServiceInterfaceBuilderBase : public InterfaceBuilderBase<Derived> {
public:

    /**
     * @brief Construct a new ServiceInterfaceBuilderBase object
     */
    ServiceInterfaceBuilderBase() = default;

public:

    /**
     * @brief Sets QoS of the interface
     * @param qos_profile 
     *    rmw_qos_profile_t Quality of service profile for client
     * @returns 
     *    reference to *this casted to @tparam Derived
     */
    inline Derived &qos_profile(const rmw_qos_profile_t &qos_profile);

    /**
     * @brief Sets callback group of the service
     * @param group 
     *    callback group to be set
     * @returns 
     *    reference to *this
     */
    inline Derived &group(rclcpp::CallbackGroup::SharedPtr group);
    
protected:

    /// QoS of the topic interface
    rmw_qos_profile_t qos_profile_ { rmw_qos_profile_services_default };
    /// Callback group of the service
    rclcpp::CallbackGroup::SharedPtr group_ { nullptr };
    
};

/* =========================================================== Publisher ========================================================== */

/**
 * @brief Builder class for ROS publisher interfaces
 * 
 * @tparam MessageT
 *    type of the messages published on the topic
 * @tparam AllocatorT
 *    type of the allocator to be used
 */
template<
    typename MessageT,
    typename AllocatorT = std::allocator<void>
> class PublisherInterfaceBuilder : public TopicInterfaceBuilderBase<PublisherInterfaceBuilder<MessageT, AllocatorT>> {
public:

    /**
     * @brief Construct a new PublisherInterfaceBuilder object used to build publisher interface
     *    that will be handled by the given @p interface
     * 
     * @param interface 
     *    reference to the shared pointer that will store the built interface
     */
    inline PublisherInterfaceBuilder(std::shared_ptr<rclcpp::Publisher<MessageT, AllocatorT>> &interface);

public:

    /**
     * @brief Sets publisher's options of the interface
     * @param options 
     *    name to be set
     * @returns 
     *    reference to *this
     */
    inline PublisherInterfaceBuilder &options(const rclcpp::PublisherOptionsWithAllocator<AllocatorT> &options);

public:

    /**
     * @brief Initializes publisher with the current configruation
     */
    void operator*();
    
protected:

    /// Interface to be initialized
    std::shared_ptr<rclcpp::Publisher<MessageT, AllocatorT>> &interface_;
    /// Publisher's options
    rclcpp::PublisherOptionsWithAllocator<AllocatorT> options_ { rclcpp::PublisherOptionsWithAllocator<AllocatorT>() };

};

/* ========================================================== Subscriber ========================================================== */

/**
 * @brief Builder class for ROS subscriber interfaces
 * 
 * @tparam MessageT
 *    type of the messages published on the topic
 * @tparam AllocatorT
 *    type of the allocator to be used
 */
template<
    typename MessageT,
    typename AllocatorT = std::allocator<void>
> class SubscriberInterfaceBuilder : public TopicInterfaceBuilderBase<SubscriberInterfaceBuilder<MessageT, AllocatorT>> {
public:

    /// Signature of the callback to the subscriber
    using CallbackSignature = void(const MessageT&);
    /// Type of the free function callback to the subscriber
    using CallbackFreeType = void (*)(const MessageT&);
    /// Type of the member function callback to the subscriber
    template<typename T>
    using CallbackMemberType = void (T::*)(const MessageT&);
    /// Type of the callback used by the builder
    using CallbackType = std::function<CallbackSignature>;

    /// Memory allocation strategy
    using MessageMemoryStrategy = rclcpp::message_memory_strategy::MessageMemoryStrategy<
        typename rclcpp::subscription_traits::has_message_type<CallbackType>::type,
        AllocatorT
    >;

public:

    /**
     * @brief Construct a new SubscriberInterfaceBuilder object used to build subscriber interface
     *    that will be handled by the given @p interface
     * 
     * @param interface 
     *    reference to the shared pointer that will store the built interface
     */
    inline SubscriberInterfaceBuilder(std::shared_ptr<rclcpp::Subscription<MessageT, AllocatorT>> &interface);

public:

    /**
     * @brief Sets subscriber's options of the interface
     * @param options 
     *    name to be set
     * @returns 
     *    reference to *this
     */
    inline SubscriberInterfaceBuilder &options(const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> &options);

    /**
     * @brief Sets subscriber's memory allocation strategy
     * @param msg_mem_strat 
     *    memory allocation strategy to be set
     * @returns 
     *    reference to *this
     */
    inline SubscriberInterfaceBuilder &memory_alloc_strategy(typename MessageMemoryStrategy::SharedPtr msg_mem_strat);

    /**
     * @brief Sets subscriber's callback to the free function
     * @param callback 
     *    callback to be set
     * @returns 
     *    reference to *this
     */
    inline SubscriberInterfaceBuilder &callback(CallbackFreeType callback);

    /**
     * @overload SubscriberInterfaceBuilder &callback(CallbackFreeType)
     * @brief Sets subscriber's callback to the member function
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
    inline SubscriberInterfaceBuilder &callback(T &obj, CallbackMemberType<T> callback);

public:

    /**
     * @brief Initializes subscriber with the current configruation
     */
    void operator*();

protected:

    /// Interface to be initialized
    std::shared_ptr<rclcpp::Subscription<MessageT, AllocatorT>> &interface_;
    /// Subscriber's options
    rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> options_ { rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>() };
    /// Memory allocation strategy
    typename MessageMemoryStrategy::SharedPtr msg_mem_strat_ { ( MessageMemoryStrategy::create_default() ) };
    /// Subscriber's callback
    CallbackType callback_;

};

/* ============================================================ Client ============================================================ */

/**
 * @brief Builder class for ROS service client interfaces
 * 
 * @tparam ServiceT
 *    type of the service
 */
template<typename ServiceT>
class ClientInterfaceBuilder : public ServiceInterfaceBuilderBase<ClientInterfaceBuilder<ServiceT>> {
public:

    /**
     * @brief Construct a new ClientInterfaceBuilder object used to build publisher interface
     *    that will be handled by the given @p interface
     * 
     * @param interface 
     *    reference to the shared pointer that will store the built interface
     */
    inline ClientInterfaceBuilder(std::shared_ptr<rclcpp::Client<ServiceT>> &interface);

public:

    /**
     * @brief Initializes client with the current configruation
     */
    void operator*();
    
protected:

    /// Interface to be initialized
    std::shared_ptr<rclcpp::Client<ServiceT>> &interface_;

};

/* ============================================================ Client ============================================================ */

/**
 * @brief Builder class for ROS service server interfaces
 * 
 * @tparam ServiceT
 *    type of the service
 */
template<typename ServiceT>
class ServiceInterfaceBuilder : public ServiceInterfaceBuilderBase<ServiceInterfaceBuilder<ServiceT>> {
public:

    /// Signature of the callback to the service
    using CallbackSignature = void(const typename ServiceT::Request::SharedPtr, typename ServiceT::Response::SharedPtr);
    /// Type of the free function callback to the service
    using CallbackFreeType = void (*)(const typename ServiceT::Request::SharedPtr, typename ServiceT::Response::SharedPtr);
    /// Type of the member function callback to the service
    template<typename T>
    using CallbackMemberType = void (T::*)(const typename ServiceT::Request::SharedPtr, typename ServiceT::Response::SharedPtr);
    /// Type of the callback used by the builder
    using CallbackType = std::function<CallbackSignature>;

public:

    /**
     * @brief Construct a new ServiceInterfaceBuilder object used to build publisher interface
     *    that will be handled by the given @p interface
     * 
     * @param interface 
     *    reference to the shared pointer that will store the built interface
     */
    inline ServiceInterfaceBuilder(std::shared_ptr<rclcpp::Service<ServiceT>> &interface);

public:

    /**
     * @brief Sets server's callback to the free function
     * @param callback 
     *    callback to be set
     * @returns 
     *    reference to *this
     */
    inline ServiceInterfaceBuilder &callback(CallbackFreeType callback);

    /**
     * @overload ServiceInterfaceBuilder &callback(CallbackFreeType)
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
    inline ServiceInterfaceBuilder &callback(T &obj, CallbackMemberType<T> callback);

public:

    /**
     * @brief Initializes server with the current configruation
     */
    void operator*();
    
protected:

    /// Interface to be initialized
    std::shared_ptr<rclcpp::Service<ServiceT>> &interface_;
    /// Server's callback
    CallbackType callback_;

};

/* ====================================================== Interface functions ===================================================== */

/**
 * @brief Creates ROS2 publisher builder
 * 
 * @tparam MessageT 
 *    type of the message
 * @tparam AllocatorT 
 *    allocator type for the publisher
 * @param interface 
 *    handle to which the interface will be attached
 * @returns 
 *    builder object
 * 
 * @par Example
 * @code
 * 
 *    // Register publisher interface for the given node with some parameters
 *    *node_common::communication::make_publisher_builder(pub)
 *       .node(some_node)
 *       .name(some_name)
 *       .qos(some_qos);
 * 
 * @endcode
 */
template<
    typename MessageT,
    typename AllocatorT
> inline PublisherInterfaceBuilder<MessageT, AllocatorT> make_publisher_builder(
    std::shared_ptr<rclcpp::Publisher<MessageT, AllocatorT>> &interface
);

/**
 * @brief Creates ROS2 subscriber builder
 * 
 * @tparam MessageT 
 *    type of the message
 * @tparam AllocatorT 
 *    allocator type for the publisher
 * @param interface 
 *    handle to which the interface will be attached
 * @returns 
 *    builder object
 * 
 * @par Example
 * @code
 * 
 *    // Register publisher interface for the given node with some parameters
 *    *node_common::communication::make_subscriber_builder(subs)
 *       .node(some_node)
 *       .name(some_name)
 *       .qos(some_qos)
 *       .callback(some_callback)
 *       .options(some_options)
 *       .memory_alloc_strategy(some_memory_alloc_strategy);
 * 
 * @endcode
 */
template<
    typename MessageT,
    typename AllocatorT
> inline SubscriberInterfaceBuilder<MessageT, AllocatorT> make_subscriber_builder(
    std::shared_ptr<rclcpp::Subscription<MessageT, AllocatorT>> &interface
);

/**
 * @brief Creates ROS2 service client builder
 * 
 * @tparam ServiceT 
 *    type of the service
 * @param interface 
 *    handle to which the interface will be attached
 * @returns 
 *    builder object
 * 
 * @par Example
 * @code
 * 
 *    // Register client interface for the given node with some parameters
 *    *node_common::communication::make_client_builder(client)
 *       .node(some_node)
 *       .name(some_name)
 *       .qos_profile(some_qos_profile)
 *       .group(some_group);
 * 
 * @endcode
 */
template<typename ServiceT>
inline ClientInterfaceBuilder<ServiceT> make_client_builder(
    std::shared_ptr<rclcpp::Client<ServiceT>> &interface
);

/**
 * @brief Creates ROS2 service server builder
 * 
 * @tparam ServiceT 
 *    type of the service
 * @tparam AllocatorT 
 *    allocator type for the publisher
 * @param interface 
 *    handle to which the interface will be attached
 * @returns 
 *    builder object
 * 
 * @par Example
 * @code
 * 
 *    // Register publisher interface for the given node with some parameters
 *    *node_common::communication::make_service_builder(subs)
 *       .node(some_node)
 *       .name(some_name)
 *       .callback(some_callback)
 *       .qos_profile(some_qos_profile)
 *       .group(some_group);
 * 
 * @endcode
 */
template<typename ServiceT>
inline ServiceInterfaceBuilder<ServiceT> make_service_builder(
    std::shared_ptr<rclcpp::Service<ServiceT>> &interface
);

/* ================================================================================================================================ */

} // End namespace communication
} // End namespace node_common

/* ==================================================== Implementation includes =================================================== */

#include "node_common/communication/communication.hpp"

/* ================================================================================================================================ */

#endif