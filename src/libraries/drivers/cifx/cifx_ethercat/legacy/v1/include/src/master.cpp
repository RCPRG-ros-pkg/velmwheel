/* ============================================================================================================================ *//**
 * @file       master.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Monday, 14th February 2022 2:33:20 pm
 * @modified   Wednesday, 20th April 2022 8:06:27 pm
 * @project    engineering-thesis
 * @brief      CIFX API-based EtherCAT Master [implementation]
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// System includes
#include <sstream>
#include <exception>
// CIFX includes
#include "Hil_ApplicationCmd.h"
#include "Hil_Results.h"
// Private includes
#include "cifx/ethercat/legacy/master.hpp"

/* ============================================================ Macros ============================================================ */

// Helper macro used to returned from function if master's not configured
#define CHECK_IF_CONFIGURED() \
do {                          \
    if(!configured)           \
        return Unconfigured;  \
} while(0);

/* ========================================================== Namespaces ========================================================== */

namespace cifx {
namespace ethercat {

/* ========================================================== Static data ========================================================= */

// File's context
static const char *context = "EthercatMaster";

/* ================================================================================================================================ */
/* ----------------------------------------------- Constructors and configuration ------------------------------------------------- */
/* ================================================================================================================================ */

Master::Master(CifxConfig cifx, bool verbose) {

    int32_t code;

    // Save verbosity mode
    this->verbose = verbose;

    // Open the channel
    traceInfo("Opening channel");
    channel = NULL;
    code = xChannelOpen(cifx.driver, const_cast<char*>(cifx.device_name.c_str()), cifx.channel_num, &channel);
    if(code != CIFX_NO_ERROR) {
        traceError("Channel opening failed");
        throw std::exception();
    }

    uint32_t state;

    // Message firmware that the application is active on the opened channel
    code = xChannelHostState(channel, CIFX_HOST_STATE_READY, &state, cifx.timeout_ms);
    if(code != CIFX_NO_ERROR) {
        traceError("Firmware could not be messaged about user's app activity");
        throw std::exception();
    }

    // Keep 'host state change' timeout
    hostStateChangeTimeoutMs = cifx.timeout_ms;
}

Master::~Master() {
    
    int32_t code;
    uint32_t state;

    // Message firmware that the application is active on the opened channel
    code = xChannelHostState(channel, CIFX_HOST_STATE_NOT_READY, &state, hostStateChangeTimeoutMs);
    if(code != CIFX_NO_ERROR)
        traceError("Firmware could not be messaged about user's app activity");

    // Close the channel
    traceInfo("Closing channel");
    code = xChannelClose(channel);
    if(code != CIFX_NO_ERROR)
        traceError("Channel closing failed");

}

/* ================================================================================================================================ */
/* --------------------------------------------------- General system settings ---------------------------------------------------- */
/* ================================================================================================================================ */

void Master::setVerbosity(bool verbosity) {
    verbose = verbosity;
}


Master::ErrorCode Master::getFirmwareInfo(FirmwareInfo &info) {

    int32_t code;

    // Prepare informations structure for request
    CHANNEL_INFORMATION chinfo;

    // Get firmware informations from the device
    code = xChannelInfo(channel, sizeof(chinfo), (void *) &chinfo);
    if(code != CIFX_NO_ERROR) {
        traceError("Could not retrive firmware informatinos from the device");
        return Err;
    }

    // Put informations to the output structure
    info.major      = chinfo.usFWMajor;
    info.minor      = chinfo.usFWMinor;
    info.build      = chinfo.usFWBuild;
    info.revision   = chinfo.usFWRevision;
    info.year       = chinfo.usFWYear;
    info.month      = chinfo.bFWMonth;
    info.day        = chinfo.bFWDay;
    // Turn name to the string
    std::stringstream nameStream;
    for (int i = 0; i < chinfo.bFWNameLength; ++i)
        nameStream << (char) chinfo.abFWName[i];
    info.name = nameStream.str();

    return Ok;
}

/* ================================================================================================================================= */
/* ---------------------------------------------------- Configuration functions ---------------------------------------------------- */
/* ================================================================================================================================= */

Master::ErrorCode Master::setSyncMode(SyncMode mode) {

    CHECK_IF_CONFIGURED();

    // Check acutal sync mode
    if(mode == syncMode)
        return Ok;

    // Prepare request
    HIL_SET_HANDSHAKE_CONFIG_REQ_T req = {0};
    req.tHead.ulDest = HIL_PACKET_DEST_DEFAULT_CHANNEL;
    req.tHead.ulLen = sizeof(HIL_SET_HANDSHAKE_CONFIG_REQ_DATA_T);
    req.tHead.ulCmd = HIL_SET_HANDSHAKE_CONFIG_REQ;
    // Prepare configuration
    req.tData.bPDOutHskMode = 4;
    switch(mode) {
        case syncFreeRun: req.tData.bPDInHskMode = 4; break;
        case syncMode1:   req.tData.bPDInHskMode = 5; break;
        case syncMode2:   req.tData.bPDInHskMode = 6; break;
    }

    int32_t code;

    // Send request to the device
    code = xChannelPutPacket(channel, (CIFX_PACKET *) &req, packetTimeoutMs);
    if(code != CIFX_NO_ERROR) {
        traceError("Could not send 'SyncMode' request packet");
        return Err;
    }

    // Get confirmation
    HIL_SET_HANDSHAKE_CONFIG_CNF_T conf = {0};
    code = xChannelGetPacket(channel, sizeof(conf), (CIFX_PACKET*) &conf, packetTimeoutMs);
    if(code != CIFX_NO_ERROR) {
        // Timeoutcase
        if(code == CIFX_DEV_GET_NO_PACKET) {
            traceError("Timeout when waiting for 'SyncMode' confirmation packet");
            return Timeout;
        }
        // Error case
        else {
            traceError("Could not send 'SyncMode' confirmation packet");
            return Err;
        }
    }

    // Check status code
    if(conf.tHead.ulSta != SUCCESS_HIL_OK) {
        tracePacketConfirmationError("SyncMode", conf.tHead.ulSta);
        return Err;
    }

    // Keep actual sync mode
    syncMode = mode;

    return Ok;
}


/* ================================================================================================================================ */
/* -------------------------------------------------------- Regular control ------------------------------------------------------- */
/* ================================================================================================================================ */

bool Master::isConfigured() {
    return configured;
}


bool Master::isBusRunning() {
    return busRunning;
}


Master::ErrorCode Master::busStart() {

    CHECK_IF_CONFIGURED();

    // Check whether bus is running
    if(busRunning)
        return BusAlreadyRunning;

    uint32_t state;

    // Change bus'es state
    int32_t code = xChannelBusState(channel, CIFX_BUS_STATE_ON, &state, busStateChangeTimeoutMs);
    if(code != CIFX_NO_ERROR) {

        // Bus state changed, but communication could not be started
        if(code == CIFX_DEV_NO_COM_FLAG) {
            traceWarning("Firmware could not establish communication to the bus");
            return BusNoCommunication;
        }

        // Bus state could not be changed
        traceError("Bus could not be started");
        return Err;
    }

    // Mark bus as running
    busRunning = true;

    return Ok;
}


Master::ErrorCode Master::busStop() {

    CHECK_IF_CONFIGURED();

    // Check whether bus is running
    if(!busRunning)
        return BusAlreadyStopped;

    uint32_t state;

    // Change bus'es state
    int32_t code = xChannelBusState(channel, CIFX_BUS_STATE_OFF, &state, busStateChangeTimeoutMs);
    if(code == CIFX_NO_ERROR) {
        traceError("Bus could not be stopped");
        return Err;
    }

    // Mark bus as running
    busRunning = false;

    return Ok;
}



/* ================================================================================================================================ */
/* ----------------------------------------------- Master configuration (private) ------------------------------------------------- */
/* ================================================================================================================================ */

Master::ErrorCode Master::loadEni(const std::string &path) {

    if(configured)
        return AlreadyConfigured;

    int32_t code;

    // Open configuration file
    uint32_t file_len;
    void *file = OS_FileOpen(const_cast<char*>(path.c_str()), &file_len);
    if(file == NULL) {
        traceError("Could not open configuration file (%s)", path.c_str());
        return FileOpenErr;
    }

    // Allocate memory for file's data
    uint8_t* file_buf = (uint8_t*) OS_Memalloc(file_len);

    // Read file's data
    if(file_len != OS_FileRead(file, 0, file_len, file_buf)) {
        traceError("Could not read configuration file");
        return FileAccessErr;        
    }

    // Load configuration file to the device
    code = xChannelDownload(channel,
        DOWNLOAD_MODE_CONFIG,
        const_cast<char*>(path.c_str()),
        file_buf, file_len,
        NULL, NULL, NULL);
    if(code != CIFX_NO_ERROR) {
        traceError("Could not download configuration file to the device");
        return Err;           
    }

    // Request Master stack to load new configuration
    ErrorCode ec = loadEniSend();
    if(ec != Ok) {
        traceError("Could not load configuration file to the stack");
        return Err;       
    }

    return Ok;
}

/* ================================================================================================================================ */
/* ------------------------------------------ Master configuration (senders) (private) -------------------------------------------- */
/* ================================================================================================================================ */

Master::ErrorCode Master::loadEniSend() {

    // Prepare request
    ECM_IF_LOAD_ENI_REQ_T req = {0};
    req.tHead.ulDest = HIL_PACKET_DEST_DEFAULT_CHANNEL;
    req.tHead.ulLen  = sizeof(ECM_IF_LOAD_ENI_REQ_TYPE_Ttag);
    req.tHead.ulCmd  = ECM_IF_CMD_LOAD_ENI_REQ;

    // For ENI file "Uncompressed XML" config file's type should be sued
    req.tData.ulLoadType = ECM_IF_LOAD_ENI_XML_UNCOMPRESSED;

    int32_t code;

    // Send package
    code = xChannelPutPacket(channel, (CIFX_PACKET *) &req, packetTimeoutMs);
    if(code != CIFX_NO_ERROR) {
        traceError("Failed to send 'Load ENI' service request to the device");
        return Err;
    }

    // Get response from the device
    ECM_IF_LOAD_ENI_CNF_T conf = {0};
    code = xChannelGetPacket(channel, sizeof(conf), (CIFX_PACKET *) &conf, packetTimeoutMs);
    if(code != CIFX_NO_ERROR) {
        // Timeoutcase
        if(code == CIFX_DEV_GET_NO_PACKET) {
            traceError("Timeout when waiting for 'Load ENI' confirmation packet");
            return Timeout;
        }
        // Error case
        else {
            traceError("Unknown error during reception of 'Load ENI' confirmation packet");
            return Err;
        }
    }

    // Check return code
    if(conf.tHead.ulSta != SUCCESS_HIL_OK) {
        tracePacketConfirmationError("Load ENI", conf.tHead.ulSta);
        return Err;
    }

    return Ok;
}

/* ================================================================================================================================ */
/* -------------------------------------------------- Some fucntions (private) ---------------------------------------------------- */
/* ================================================================================================================================ */

void Master::traceDebug(std::string format, ...) {

    if(verbose) {

        // Initialize variable arguments
        va_list arg;
        va_start(arg, format);
        
        xTraceDebugVa(context, const_cast<char*>(format.c_str()), arg);

        // Remove variable list
        va_end(arg);
    }
}


void Master::traceInfo(std::string format, ...) {

    if(verbose) {
        // Initialize variable arguments
        va_list arg;
        va_start(arg, format);
        
        xTraceInfoVa(context, const_cast<char*>(format.c_str()), arg);

        // Remove variable list
        va_end(arg);
    }
}


void Master::traceWarning(std::string format, ...) {

    if(verbose) {
        // Initialize variable arguments
        va_list arg;
        va_start(arg, format);
        
        xTraceWarnVa(context, const_cast<char*>(format.c_str()), arg);

        // Remove variable list
        va_end(arg);
    }
}


void Master::traceError(std::string format, ...) {

    if(verbose) {
        // Initialize variable arguments
        va_list arg;
        va_start(arg, format);
        
        xTraceErrorVa(context, const_cast<char*>(format.c_str()), arg);

        // Remove variable list
        va_end(arg);
    }
}


void Master::tracePacketConfirmationError(std::string function, uint32_t code) {
    traceError("'%s' confirmation returned with (%lx) code", function.c_str(), code);
}

/* ================================================================================================================================ */

} // End namespace ethercat
} // End namespace cifx
