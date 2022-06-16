/* ============================================================================================================================ *//**
 * @file       OS_Includes.c
 * @author     Adam Kowalewski
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 19th May 2022 9:53:09 am
 * @modified   Wednesday, 15th June 2022 1:39:54 pm
 * @project    engineering-thesis
 * @brief      CIFX'es files-related OS functions [implementation]
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

#include <stdio.h>
#include <stdarg.h>
#include <assert.h>
#include "OS_Includes.h"

/* ======================================================= Static functions ======================================================= */

/**
 * @brief Returns the trace level description
 * 
 * @param trace_level
 *    trace level used
 * @returns
 *    human-readable name of the trace level
 */
static const char* get_trace_level_description(uint32_t trace_level) {
	switch(trace_level)	{
		case TRACE_LEVEL_DEBUG:
			return LOG_DEBUG_COL "DEBUG" LOG_END_COL;
		case TRACE_LEVEL_INFO:
			return LOG_INFO_COL "INFO" LOG_END_COL;
		case TRACE_LEVEL_WARNING:
			return LOG_WARNING_COL "WARNING" LOG_END_COL;
		case TRACE_LEVEL_ERROR:
			return LOG_ERROR_COL "ERROR" LOG_END_COL;
		default:
			assert(0);
			__builtin_unreachable();
	}
}

/**
 * @brief Prints log message to the stdout using USER_Trace
 * 
 * @param trace_level 
 *    requested trace leve
 * @param colour 
 *    message's colour escape code (not used when @p context is NULL)
 * @param context 
 *    message's context; not used when NULL
 * @param format 
 *    message's format (printf-like)
 * @param arg 
 *    format-dependent parameters
 */
static void message(int trace_level, const char *colour, const char *context, const char *format, va_list arg) {

    char buf[TRACE_BUF_SIZE];

    // Format message
    if(context == NULL)
        snprintf(buf, TRACE_BUF_SIZE, "%s", format);
    else 
        snprintf(buf, TRACE_BUF_SIZE, "[%s%s" LOG_END_COL "] %s", colour, context, format);

    // Print message
	xTraceVa(trace_level, buf, arg);
}

/* ======================================================= Public functions ======================================================= */

void xTraceVa(uint32_t trace_level, const char* format, va_list arg) {
    
    assert(format != NULL);

	// Check whether global and local copy of trace level match
	if(g_ulTraceLevel > trace_level)
		return;

	// Get human-readable trace level's description
	const char* const description = get_trace_level_description(trace_level);
	printf("[%s]", description);

	// Printf additional space if format string begins with some predefined character
	if(format[0] != '[')
		putchar(' ');	

	// Print formatted message
	vprintf(format, arg);

	putchar('\n');
}


void xTrace(uint32_t trace_level, const char* format, ...) {

    // Initialize variable arguments
    va_list arg;
	va_start(arg, format);

    // Print message
	xTraceVa(trace_level, format, arg);

    // Remove variable list
	va_end(arg);

}


void xTraceDebugVa(const char *context, const char *format, va_list arg) {

	// Print message
	message(TRACE_LEVEL_DEBUG, LOG_DEBUG_COL, context, format, arg);

}


void xTraceDebug(const char *context, const char *format, ...) {
    
	// Initialize variable arguments
    va_list arg;
	va_start(arg, format);

    xTraceDebugVa(context, format, arg);

    // Remove variable list
	va_end(arg);

}


void xTraceInfoVa(const char *context, const char *format, va_list arg) {
    
    // Print message
	message(TRACE_LEVEL_INFO, LOG_INFO_COL, context, format, arg);

}


void xTraceInfo(const char *context, const char *format, ...) {

    // Initialize variable arguments
    va_list arg;
	va_start(arg, format);
    
	xTraceInfoVa(context, format, arg);

    // Remove variable list
	va_end(arg);
}


void xTraceWarnVa(const char *context, const char *format, va_list arg) {

	// Print message
	message(TRACE_LEVEL_WARNING, LOG_WARNING_COL, context, format, arg);

}


void xTraceWarn(const char *context, const char *format, ...) {

    // Initialize variable arguments
    va_list arg;
	va_start(arg, format);

    xTraceWarnVa(context, format, arg);

    // Remove variable list
	va_end(arg);
}


void xTraceErrorVa(const char *context, const char *format, va_list arg) {


    // Print message
	message(TRACE_LEVEL_ERROR, LOG_ERROR_COL, context, format, arg);
}


void xTraceError(const char *context, const char *format, ...) {

    // Initialize variable arguments
    va_list arg;
	va_start(arg, format);

	xTraceErrorVa(context, format, arg);

    // Remove variable list
	va_end(arg);
}


void xTraceSystemErrorVa(const char *context, int ec, const char *format, va_list arg) {

    char message_buf[TRACE_BUF_SIZE];

    // Format message
    snprintf(message_buf, TRACE_BUF_SIZE, "(%s) %s", strerror(ec), format);

    // Print message
	message(TRACE_LEVEL_ERROR, LOG_ERROR_COL, context, message_buf, arg);
	
}


void xTraceSystemError(const char *context, int ec, const char *format, ...) {
    
    // Initialize variable arguments
    va_list arg;
	va_start(arg, format);

	xTraceSystemErrorVa(context, ec, format, arg);

    // Remove variable list
	va_end(arg);
}


void xTraceGlobalSystemErrorVa(const char *context, const char *format, va_list arg) {

    char message_buf[TRACE_BUF_SIZE] = "";

    // Format message
    snprintf(message_buf, TRACE_BUF_SIZE, "(%s) ", strerror(errno));
    strncat(message_buf, format, TRACE_BUF_SIZE - strlen(message_buf));

    // Print message
	message(TRACE_LEVEL_ERROR, LOG_ERROR_COL, context, message_buf, arg);
}


void xTraceGlobalSystemError(const char *context, const char *format, ...) {

    // Initialize variable arguments
    va_list arg;
	va_start(arg, format);

	xTraceGlobalSystemErrorVa(context, format, arg);

    // Remove variable list
	va_end(arg);
}

/* ================================================================================================================================ */
