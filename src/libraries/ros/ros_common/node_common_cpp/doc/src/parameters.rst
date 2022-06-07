Parameters
==========

Reference
---------

.. doxygenstruct:: node_common::parameters::Range
   :members:

.. doxygentypedef:: node_common::parameters::IntegerRange

.. doxygentypedef:: node_common::parameters::FloatingPointRange

.. doxygenstruct:: node_common::parameters::ParamDescriptor
   :members:

.. doxygenfunction:: node_common::parameters::declare_parameter(rclcpp::Node &, const ParamDescriptor<T, N>&)
.. doxygenfunction:: node_common::parameters::declare_parameter(rclcpp::Node &, std::string_view, const ParamDescriptor<T, N>&)

.. doxygenfunction:: node_common::parameters::get_param(rclcpp::Node &, std::string_view)
.. doxygenfunction:: node_common::parameters::get_param(rclcpp::Node &, std::string_view, std::string_view)
.. doxygenfunction:: node_common::parameters::get_param(rclcpp::Node &, const ParamDescriptor<T, N>&);
.. doxygenfunction:: node_common::parameters::get_param(rclcpp::Node &, std::string_view, const ParamDescriptor<T, N>&);

.. doxygenfunction:: node_common::parameters::declare_parameter_and_get(rclcpp::Node &, const ParamDescriptor<T, N>&)
.. doxygenfunction:: node_common::parameters::declare_parameter_and_get(rclcpp::Node &, std::string_view, const ParamDescriptor<T, N>&)