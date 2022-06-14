/* ============================================================================================================================ *//**
 * @file       process_data.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 4th May 2022 12:03:11 pm
 * @modified   Wednesday, 25th May 2022 9:45:11 pm
 * @project    engineering-thesis
 * @brief      Definition of the class wrapping description and providing related API for the 'Process Data' concept of the CIFX 
 *             Toolkit Framework
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// Private includes
#include "cifx/channel.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace cifx {

/* ================================================ Public methods (Notifications) ================================================ */

namespace details {

    void cifx_process_data_callback(
        uint32_t notification_event,
        [[maybe_unused]] uint32_t data_len,
        [[maybe_unused]] void* data,
        void* context
    ) {

        // Get reference to the @ref Mailbox object
        ProcessData &process_data = *static_cast<ProcessData*>(context);

        // Dispatch callback
        switch(notification_event) {
                
            // Dispatch handler for the 'Input' event
            case CIFX_NOTIFY_PD0_IN:

                if(process_data.callbacks.input)
                    process_data.callbacks.input();
            
                break;
                
            // Dispatch handler for the 'Input' event
            case CIFX_NOTIFY_PD1_IN:

                if(process_data.callbacks.input)
                    process_data.callbacks.input();
            
                break;
                
            // Dispatch handler for the 'Output' event
            case CIFX_NOTIFY_PD0_OUT:

                if(process_data.callbacks.output)
                    process_data.callbacks.output();
            
                break;
                
            // Dispatch handler for the 'Output' event
            case CIFX_NOTIFY_PD1_OUT:

                if(process_data.callbacks.output)
                    process_data.callbacks.output();
            
                break;
                
            // Non-supported event
            default: break;
        }

        return;
    }

} // End namespace details

/* ================================================================================================================================ */

} // End namespace cifx
