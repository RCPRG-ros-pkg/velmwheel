/* ============================================================================================================================ *//**
 * @file       error.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Monday, 14th February 2022 2:33:20 pm
 * @modified   Thursday, 30th June 2022 2:00:19 pm
 * @project    engineering-thesis
 * @brief      Definitions of helper functions and types associated with CIFX errors system
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __CIFX_ERROR_ERROR_H__
#define __CIFX_ERROR_ERROR_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <sstream>
// Private includes
#include "cifx/error.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace cifx {

/* ======================================================== Free functions ======================================================== */

std::string error_to_str(int32_t error) {

    // Maximal length of the eror's description
    constexpr std::size_t DESCRIPTION_BUF_SIZE_MAX = 200;

    std::array<char, DESCRIPTION_BUF_SIZE_MAX> description_buffer;

    // Convert the error
    auto ret = xDriverGetErrorDescription(error, description_buffer.data(), description_buffer.size());
    // If failed to convert, try to find error in custom errors' table
    if(ret != CIFX_NO_ERROR) {

        // Convert the error
        ret = xDriverGetCustomErrorDescription(error, description_buffer.data(), description_buffer.size());
        // If error again, throw
        if(ret != CIFX_NO_ERROR)
            throw std::out_of_range{ "[cifx::error_to_str] Failed to conver error code to string" };

    }

    return std::string(description_buffer.data());
}

/* ======================================================= Helper functions ======================================================= */

namespace details {

    static inline std::string to_msg(int32_t code, const std::string &what) {
        std::stringstream ss;
        ss << what << " (" << error_to_str(code) << ")";
        return ss.str();
    }


    static inline std::string to_msg(int32_t code, std::string_view context, const std::string &what) {
        std::stringstream ss;
        ss << "[" << context << "] " << what << " (" << error_to_str(code) << ")";
        return ss.str();
    }

} // End namespace details

/* =========================================================== Subtypes =========================================================== */

Error::Error(int32_t code, const std::string &what) :
    error_code{ code },
    msg{ what },
    what_msg{ details::to_msg(code, what) }
{ }


Error::Error(int32_t code, std::string_view context, const std::string &what) :
    error_code{ code },
    context{ context },
    msg{ what },
    what_msg{ details::to_msg(code, context, what) }
{ }


int32_t Error::get_code() const {
    return error_code;
}


const std::string &Error::get_context() const {
    return context;
}


const std::string &Error::get_msg() const {
    return msg;
}


const char *Error::what() const noexcept {
    return what_msg.c_str();
}


void Error::change_context(std::string_view ctx) {
    context = ctx;
    what_msg = ctx.empty() ?
        details::to_msg(error_code, msg) :
        details::to_msg(error_code, context, msg);
}


void Error::rethrow_with_context(std::string_view ctx) {
    change_context(ctx);
    throw (*this);
}

/* ================================================================================================================================ */

} // End namespace cifx

/* ================================================================================================================================ */

#endif
