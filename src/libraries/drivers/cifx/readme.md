# Note

When running any programm using the CIFX driver make sure that you have `root proviledges`, as it is required to access files in the /sys/class/uio/ folder. Moreover, root status can be required when setting specifix scheduling politic/priority.

# All sins made by the driver (in context of realtime workload)

1. Memory
    - heap instead of stack (mainly)
    - dynamic memory allocation :white_check_mark:
    - small allocations (std::string) :white_check_mark:
    - non-local allocation :white_check_mark:
2. Dynamic dispatch
    - dynamic polymorphism :white_check_mark:
    - dynamic type erasure (std::function) :white_check_mark:
3. Exceptions
    - exceptions :white_check_mark:
4. Synchronistion
    - heavy synchronisation (std::mutex, partially due to *futex*) :white_check_mark:

# Sins that left to be made

1. RTTI
2. Polymorphic inheritance

# Reasons

1. Desigining convinience
2. Implementation safety
3. Performance limitations due to dependencie simplementation (ROS2, CIFX Toolkit)
4. Implementation effort (template-based code - very performant, hard to write and read)

# Fixes

1. Memory
    - [memory locking](https://design.ros2.org/articles/realtime_background.html)
    - [preallocation](https://design.ros2.org/articles/realtime_background.html)
    - std::string_view
    - std::static_vector (future...)
    - custom allocators (e.g. ROS-provided TLSF implementation)
    * still non-optimal caching misses
2. Dynamic dispatch
    - [none]
    - usage of std::variant for cache-friendly dynamic dispatch
    - potential: optimisation of the binary layout in the linker
    * required for non-templated designs
    * mainly cache-related performance cost
3. Exceptions
    - used only on non-critical or fatal-error paths
    - in the university world most of errors are ascalated to system shutdown
4. Synchronistion
    - *futex*
    - atomics in short-lock scenarios

# Pros (in the ROS context)

- strong isolation of the hardware-specific elements from the rest of the system
- flexible, extensible drivers interfaces
- flexible, extensible ROS interface
- strong usage of ROS2 composition
    * possible zero-copy messages passing but current implementation of IPC (Intra process communication comes with a large CPU overhead, [source](https://github.com/ros2/rclcpp/issues/1642))

