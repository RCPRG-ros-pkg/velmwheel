Device registers
================

Predefined registers
--------------------

.. doxygenvariable:: ethercat::devices::elmo::registers::DEVICE_TYPE
.. doxygenvariable:: ethercat::devices::elmo::registers::ERROR_REGISTER

Common entries
--------------

.. doxygenvariable:: ethercat::devices::elmo::registers::ABORT_CONNECTION_OPTION_CODE
.. doxygenvariable:: ethercat::devices::elmo::registers::ERROR_CODE
.. doxygenvariable:: ethercat::devices::elmo::registers::MOTOR_TYPE
.. doxygenvariable:: ethercat::devices::elmo::registers::SUPPORTED_DRIVE_MODES
.. doxygenvariable:: ethercat::devices::elmo::registers::DIGITAL_INPUTS

Device control
--------------

.. doxygenvariable:: ethercat::devices::elmo::registers::CONTROLWORD
.. doxygenvariable:: ethercat::devices::elmo::registers::STATUSWORD
.. doxygenvariable:: ethercat::devices::elmo::registers::QUICK_STOP_OPTION_CODE
.. doxygenvariable:: ethercat::devices::elmo::registers::SHUTDOWN_OPTION_CODE
.. doxygenvariable:: ethercat::devices::elmo::registers::DISABLE_OPERATION_MODE
.. doxygenvariable:: ethercat::devices::elmo::registers::HALT_OPTION_CODE
.. doxygenvariable:: ethercat::devices::elmo::registers::FAULT_REACTION_CODE

Modes of operation
------------------

.. doxygenvariable:: ethercat::devices::elmo::registers::MODES_OF_OPERATION
.. doxygenvariable:: ethercat::devices::elmo::registers::MODES_OF_OPERATION_DISPLAY

Factors
-------

.. doxygenvariable:: ethercat::devices::elmo::registers::POLARITY
.. doxygenvariable:: ethercat::devices::elmo::registers::POSITION_ENCODER_RESOLUTION
.. doxygenvariable:: ethercat::devices::elmo::registers::VELOCITY_ENCODER_RESOLUTION
.. doxygenvariable:: ethercat::devices::elmo::registers::POSITION_FACTOR
.. doxygenvariable:: ethercat::devices::elmo::registers::VELOCITY_ENCODER_FACTOR
.. doxygenvariable:: ethercat::devices::elmo::registers::VELOCITY_FACTOR_1
.. doxygenvariable:: ethercat::devices::elmo::registers::VELOCITY_FACTOR_2
.. doxygenvariable:: ethercat::devices::elmo::registers::ACCELERATION_FACTOR

Homing
------

.. doxygenvariable:: ethercat::devices::elmo::registers::HOME_OFFSET
.. doxygenvariable:: ethercat::devices::elmo::registers::HOMING_MODE
.. doxygenvariable:: ethercat::devices::elmo::registers::HOMING_SPEEDS
.. doxygenvariable:: ethercat::devices::elmo::registers::HOMING_ACCELERATION

Position control
----------------

.. doxygenvariable:: ethercat::devices::elmo::registers::POSITION_DEMAND_VALUE_IN_POSITION_UNITS
.. doxygenvariable:: ethercat::devices::elmo::registers::POSITION_ACTUAL_VALUE_IN_INCREMENTS
.. doxygenvariable:: ethercat::devices::elmo::registers::POSITION_ACTUAL_VALUE
.. doxygenvariable:: ethercat::devices::elmo::registers::FOLLOWING_ERROR_WINDOW
.. doxygenvariable:: ethercat::devices::elmo::registers::POSITION_WINDOW
.. doxygenvariable:: ethercat::devices::elmo::registers::POSITION_WINDOW_TIME_OUT
.. doxygenvariable:: ethercat::devices::elmo::registers::FOLLOWING_ERROR_ACTUAL_VALUE
.. doxygenvariable:: ethercat::devices::elmo::registers::POSITION_CONTROL_EFFORT
.. doxygenvariable:: ethercat::devices::elmo::registers::POSITION_DEMAND_VALUE_IN_INCREMENTS

Profiled Position
-----------------

.. doxygenvariable:: ethercat::devices::elmo::registers::TARGET_POSITION
.. doxygenvariable:: ethercat::devices::elmo::registers::POSITION_RANGE_LIMIT
.. doxygenvariable:: ethercat::devices::elmo::registers::SOFTWARE_POSITION_LIMIT
.. doxygenvariable:: ethercat::devices::elmo::registers::MAXIMUM_PROFILE_VELOCITY
.. doxygenvariable:: ethercat::devices::elmo::registers::PROFILED_VELOCITY
.. doxygenvariable:: ethercat::devices::elmo::registers::END_VELOCITY
.. doxygenvariable:: ethercat::devices::elmo::registers::PROFILED_ACCELERATION
.. doxygenvariable:: ethercat::devices::elmo::registers::PROFILED_DECELERATION
.. doxygenvariable:: ethercat::devices::elmo::registers::QUICK_STOP_DECELERATION
.. doxygenvariable:: ethercat::devices::elmo::registers::MOTION_PROFILE_TYPE
.. doxygenvariable:: ethercat::devices::elmo::registers::MAXIMUM_ACCELERATION
.. doxygenvariable:: ethercat::devices::elmo::registers::MAXIMUM_DECELERATION

Interpolated Position
---------------------

.. doxygenvariable:: ethercat::devices::elmo::registers::INTERPOLATION_SUB_MODE_SELECT
.. doxygenvariable:: ethercat::devices::elmo::registers::INTERPOLATION_DATA_RECORD
.. doxygenvariable:: ethercat::devices::elmo::registers::INTERPOLATION_TIME_PERIOD
.. doxygenvariable:: ethercat::devices::elmo::registers::INTERPOLATION_SYNC_DEFINITION
.. doxygenvariable:: ethercat::devices::elmo::registers::INTERPOLATION_DATA_CONFIGURATION

Profile velocity
----------------

.. doxygenvariable:: ethercat::devices::elmo::registers::VELOCITY_SENSOR_ACTUAL_VALUE
.. doxygenvariable:: ethercat::devices::elmo::registers::VELOCITY_WINDOW
.. doxygenvariable:: ethercat::devices::elmo::registers::SENSOR_SELECTION_CODE
.. doxygenvariable:: ethercat::devices::elmo::registers::VELOCITY_DEMAND_VALUE
.. doxygenvariable:: ethercat::devices::elmo::registers::VELOCITY_ACTUAL_VALUE
.. doxygenvariable:: ethercat::devices::elmo::registers::VELOCITY_WINDOW
.. doxygenvariable:: ethercat::devices::elmo::registers::VELOCITY_WINDOW_TIME
.. doxygenvariable:: ethercat::devices::elmo::registers::VELOCITY_THRESHOLD
.. doxygenvariable:: ethercat::devices::elmo::registers::VELOCITY_THRESHOLD_TIME
.. doxygenvariable:: ethercat::devices::elmo::registers::TARGET_VELOCITY

Profiled Torque
---------------

.. doxygenvariable:: ethercat::devices::elmo::registers::Target_torque
.. doxygenvariable:: ethercat::devices::elmo::registers::Max_torque
.. doxygenvariable:: ethercat::devices::elmo::registers::Max_current
.. doxygenvariable:: ethercat::devices::elmo::registers::Torque_demand_value
.. doxygenvariable:: ethercat::devices::elmo::registers::Motor_rated_current
.. doxygenvariable:: ethercat::devices::elmo::registers::Motor_rated_torque
.. doxygenvariable:: ethercat::devices::elmo::registers::Torque_actual_value
.. doxygenvariable:: ethercat::devices::elmo::registers::Current_actual_value
.. doxygenvariable:: ethercat::devices::elmo::registers::Torque_slope
.. doxygenvariable:: ethercat::devices::elmo::registers::Torque_profile_type
