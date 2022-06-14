#!/usr/bin/env python3
# ====================================================================================================================================
# @file       common.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Thursday, 28th April 2022 12:46:17 pm
# @modified   Friday, 27th May 2022 5:33:34 pm
# @project    engineering-thesis
# @brief      Common definitions for nodes of the `velmwheel_ethercat_driver` package
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# Unqualified name of the memory-locking service
LOCK_MEMORY_SERVICE_NAME='lock_memory'

# Unqualified name of the bus-state-setter service
SET_BUS_STATE_SERVICE_NAME='set_bus_state'
# Unqualified name of the bus-state-getter service
GET_BUS_STATE_SERVICE_NAME='get_bus_state'
# Unqualified name of the master-state-setter service
SET_MASTER_STATE_SERVICE_NAME='set_master_state'
# Unqualified name of the master-state-getter service
GET_MASTER_STATE_SERVICE_NAME='get_master_state'
# Unqualified name of the driver-loading service
LOAD_DRIVER_SERVICE_NAME='load_driver'

# Unqualified name of the driver-unloading service
UNLOAD_DRIVER_SERVICE_NAME='unload_driver'
# Unqualified name of the drivers-listing service
LIST_DRIVERS_SERVICE_NAME='list_drivers'
# Unqualified name of the bus-timings-getter service
GET_BUS_TIMING_SERVICE_NAME='get_bus_timing'
