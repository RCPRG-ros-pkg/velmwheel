/** ==================================================================================================================================
 * @file       master.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Monday, 14th February 2022 2:33:20 pm
 * @modified   Wednesday, 20th April 2022 4:21:23 pm
 * @project    engineering-thesis
 * @brief
 *    
 *    Interface of the CIFX-based EtherCAT Master Stack (V4.5.0)
 *    
 * @note This file contains legacy interface of the cifx::ethercat::Master class that has never been finalized. This 
 *    implementation aimed for providing a clean C++ interface over the CIFX ECM API enabling full in-code configuration
 *    of the CIFX EtherCAT interface device without usage of the ENI file as an alternative for configuring the system
 *    via ENI using Visual Studio TwinCAT framework. The class could not be finalized due to lack of the official support
 *    from Hilscher for non-ENI coinfigruation API (and so no clear documentation covering the topic). 
 * 
 *    This implementation may be finalized in the future when the documentation for CIFX ECM API will be completed
 * 
 * @copyright Krzysztof Pierczyk © 2022
 * ================================================================================================================================ */

#ifndef __CIFX_ETHERCAT_MASTER_H__
#define __CIFX_ETHERCAT_MASTER_H__

/* =========================================================== Includes =========================================================== */

#include <vector>
#include <memory>
#include <string>
#include "cifxDriver.h"
#include "EcmIF_Public.h"
#include "cifx/ethercat/legacy/slave.hpp"

/* =========================================================== Namespace ========================================================== */

namespace cifx {
namespace ethercat {

/* ========================================================= Declarations ========================================================= */

/**
 * @brief CIFX API-based EtherCAT Master. Class wraps functions provided by the ECM API to make interactions
 *    with the CIFX driver more straightforward. Nonetheless knowledge of the EtherCAT Master Stack documentation
 *    as well as general working principles of the netX hardware is strongly recommended
 * 
 * @note CIFX cards use two ways to communicate with the Host (i.e. PC): Mailbox Communication (asynchronous,
 *    packets-based) and Cyclical Communication (although it is very similar to the EtherCAT's CoE communication 
 *    model, it is not the same). Using the Ethercat::Master class user can register for receiving various 
 *    informations about EtherCAT Master Stack's state from the device without explicitly requesting for it 
 *    ( @see  resgisterFor*Indications() functions). Unfortunetely Mailbox can hold only a single packet at 
 *    the time. Because many other methods provided by the class also expect receiving a response packets from 
 *    the CIFX, it is crucial to regularly check if new indications arrived ( @see checkIndications() method) 
 *    to avoid situation where some service cannot be performed due to non-empty Mailbox. The best practice is
 *    to perform all required Mailbox-based actions before registering for indications and using them only during
 *    cyclical data exchange.
 */
class Master {

    /* ============================================================================================================= */
    /* ------------------------------------------------ Public types ----------------------------------------------- */
    /* ============================================================================================================= */

public: /* Public Typedefs */

    /**
     * @note Toolkit's types redefinitions are made in case Hilscher chanes
     *    naming in the future
     */

    // Driver handle
    typedef CIFXHANDLE DriverHandle;

    // Channel handle
    typedef CIFXHANDLE ChannelHandle;

    // EtherCAT DC deviation informations
    typedef ECM_IF_GET_DC_DEVIATION_CNF_DATA_T DcDeviation;

    // EtherCAT DC Slave informations
    typedef ECM_IF_GET_SLAVE_DC_INFO_CNF_DATA_T DcSlaveInfo;

    // EtherCAT Master timing informations
    typedef ECM_IF_GET_TIMING_INFO_CNF_DATA_T TimingInfo;

    // EtherCAT Master WcState (Working Counter State) informations
    typedef ECM_IF_WCSTATE_INFO_ENTRY_T WcStateInfo;

    // EtherCAT Master process image mapping informations
    typedef ECM_IF_CYCLIC_CMD_MAPPING_ENTRY_T MappingInfo;

    // EtherCAT Master slave mapping informations
    typedef ECM_IF_CYCLIC_SLAVE_MAPPING_ENTRY_T SlaveMappingInfo;

    // EtherCAT Master slave's topology information
    typedef ECM_IF_GET_TOPOLOGY_INFO_CNF_DATA_ENTRY_T TopologyInfo;

    // EtherCAT Master frame's eror counter information
    typedef ECM_IF_GET_ERROR_COUNTERS_CNF_DATA_T FrameErrorInfo;

public: /* Error codes */

    // Class-specific error codes
    typedef enum {
        Num = -13,
        // General Error codes
        Err,
        NulArg,
        FileOpenErr,
        FileAccessErr,
        Timeout,
        // Indications-related codes
        IndicationAwaits,
        PackageAwaits,
        // Configuration-related codes
        AlreadyConfigured,
        Unconfigured,
        // Bus-related codes
        BusAlreadyRunning,
        BusAlreadyStopped,
        BusNoCommunication,
        // Success code
        Ok
    } ErrorCode;

    /* ============================================================================================================= */
    /* --------------------------------------- Constructors and destructors ---------------------------------------- */
    /* ============================================================================================================= */

public: /* Construction structures */

    /**
     * @brief Informations related to the CIFX driver
     */
    typedef struct {

        // Handle to the opened driver
        DriverHandle driver;
        // Name of the used device in the toolkit
        std::string device_name;
        // Device's associated with the EtherCAT Master communication stack
        unsigned channel_num;
        // Initialization/deinitialization stack's timeout
        uint32_t timeout_ms;

    } CifxConfig;

public: /* Constructors and destructors */

    /**
     * @brief: Default constructor. Prepares Master driver to operate on the given
     *    communication channel
     * 
     * @param cifx
     *    CIFX-specific configuration of the EtherCAT Master
     * @param verbose
     *    if @c true Master is configured in verbose logging mode (will be printing different
     *    logs using xTrace*() functions depending on the case).
     * 
     * @note When this code is created, the EtherCAT Master firmware for CIFX cards
     *    (ECM V4.5) is provided as standalone binary file containing RTX RTOS (RTOS that 
     *    Hilscher's communication stacks are based on) as well as EtherCAT Master stack.
     *    It means that device with the firmawer loaded into always expects communication 
     *    from Host (i.e. PC) to be performed via the Communication Channel 0. Technically
     *    class'es constructor could create device's instance in the Toolkit on its own and
     *    hold it exclusively.
     * 
     *    However, RTX RTOS provides possibility to load protocol's stacks as detached modules.
     *    It is possible that in the future the Master class will be used with the board that 
     *    utilizes not only the EtherCAT Master stack but also other communication stacks. In
     *    such case it is crucial for user's application to have access to the device's other
     *    channels.
     * 
     * @throws std::exception on failure
     */
    Master(CifxConfig cifx, bool verbose = true);

    /**
     * @brief Default destructor. When the object is deleted, bus communication is turned off
     *    and the channel is closed
     */
    ~Master();

    /* ============================================================================================================= */
    /* ------------------------------------------ General system settings ------------------------------------------ */
    /* ============================================================================================================= */

public: /* Device info */

    /**
     * @brief General infromations about EtherCAT Master firmware loaded to the 
     *    CIFX device
     */
    typedef struct {

        // Firmware's version
        uint16_t major;
        uint16_t minor;
        uint16_t build;
        uint16_t revision;
        // Firmware's name
        std::string name;
        // Firmware's date
        uint16_t year;
        uint8_t  month;
        uint8_t  day;

    } FirmwareInfo;

    /**
     * @brief Sets driver's verbosity state
     * 
     * @param verbosity 
     *    @c true if verbose logging should be enabled
     *    @c false otherwise
     */
    void setVerbosity(bool verbosity);

    /**
     * @brief Retrives information about the EtherCAT Master stack's running in
     *    the CIFX device
     * 
     * @param info [out]
     *    retrived info on success
     * @returns
     *    @c Ok on success
     *    @c Err on error
     */
    ErrorCode getFirmwareInfo(FirmwareInfo &info);

    /* ============================================================================================================= */
    /* ------------------------------------------ Configuration functions ------------------------------------------ */
    /* ============================================================================================================= */

public: /* Configuration constants */

    // Type of bus configruation
    typedef enum {
        configEni,
        configManual
    } ConfigMode;

    // Synchronisation mode
    typedef enum {
        syncFreeRun,
        syncMode1,
        syncMode2
    } SyncMode;

public: /* Configuration structures */

    /**
     * @brief Master configuration (used when manual configuration is performed)
     */
    typedef struct {

        // System timeouts
        struct {

            // Timeout for bus'es state change
            uint32_t busStateChangeTimeoutMs;
            // Timeout for Mailbox-base services of the CIFX device
            uint32_t packetTimeoutMs;

        } timeouts;

        // Synchronisation mode of the cyclical data exchange ( @see readBus(), writeBus() )
        SyncMode syncMode;

        // Configuration mode
        ConfigMode mode;

        // Bus configuration
        union {

            // Manual configuration structure
            struct {

            } manual;

            // ENI-based configuration structure
            struct {

                // Path to the ENI file
                std::string path;

            } eni;

        } bus;

    } Config;

public: /* Configuation methods */

    /**
     * @brief: Adds slave device to the Master's list. The list cannot be modified when the
     *    bus communication is running. If the list is changed, reconfigruation
     *    is required.
     * 
     * @param slave
     *    slave to be added 
     * @returns 
     *    @c Ok on success
     *    error code on error
     */
    ErrorCode addSlave(const Slave &slave);

    /**
     * @brief Deletes slave device from the Master's list. The list cannot be modified when the
     *    bus communication is running. If the list is changed, reconfigruation
     *    is required
     * 
     * @param name
     *    name of the slave device to be deleted
     * @returns 
     *    @c Ok on success
     *    error code on error
     */
    ErrorCode deleteSlave(std::string name);

    /**
     * @brief Configures master device with the given parameters
     * 
     * @param config
     *    master's configuration
     * @returns 
     *   @c Ok on success
     *    error code on error
     */
    ErrorCode configure(const Config &config);

    /**
     * @brief Configures synchronisation mode ( @see EtherCAT Master documentation )
     * 
     * @param mode
     *    synchronisation mode to be set
     * @returns 
     *    @c Ok on success
     *    @c Unconfigured if master is not configrued
     *    @c Timeout when waiting for the confimation packet timeouts 
     *    @c Err on error
     * 
     * @see readBus(), writeBus()
     */
    ErrorCode setSyncMode(SyncMode mode);

    /* ============================================================================================================= */
    /* ---------------------------------------- Normal operation functions ----------------------------------------- */
    /* ============================================================================================================= */

public: /* State control */

    /**
     * @returns 
     *    @c true if Master is configured
     *    @c false otherwise
     */
    bool isConfigured();

    /**
     * @returns 
     *    @c true if bus communication is running
     *    @c false if bus communication is stopped
     */
    bool isBusRunning();

    /**
     * @brief Starts the bus communication
     * @returns 
     *    @c Ok on success
     *    @c Unconfigured is master is nto configured
     *    @c BusAlreadyRunning if bus already runs
     *    @c Err on error
     */
    ErrorCode busStart();

    /**
     * @brief Stops the bus communication
     * @returns 
     *    @c Ok on success
     *    @c Unconfigured is master is nto configured
     *    @c BusAlreadyStopped if bus already runs
     *    @c Err on error
     */
    ErrorCode busStop();

public: /* Cyclic data exchange */

    /**
     * @brief Commands Master to read data from the bus and passes it to 
     *    registered slaves
     * 
     * @returns 
     *    @c Ok on success
     *    error code on error
     */
    ErrorCode readBus();

    /**
     * @brief Reads output data from registred slaves and commands Master 
     *    to write data to the bus
     * 
     * @returns 
     *    @c Ok on success
     *    error code on error
     */
    ErrorCode writeBus();

    /* ============================================================================================================= */
    /* ------------------------------------------ Asynchronous indications ----------------------------------------- */
    /* ============================================================================================================= */

    /**
     * @note not-implemented yet
     * 
     * @brief Indications are a way to asynchronous get informations from the CIFX device by the Mailbox 
     *   system when the related event happens. Presence of these packets in the mailbox can be polled with 
     *   checkIndications() function. 
     * 
     * @note When indications are used, there is chance that some service requested by the user (that also
     *   uses Mailbox) cannot be completed because of the indication awaiting. In such cases the service
     *   mathod will always return @c IndicationAwaits error that indicates the checkIndications() function
     *   should be called repeatedly until the Mailbox is empty. No other services can be called (i.e. will
     *   always return with the upwritten code) until it happens.
     * 
     * @note When indications are used, there is chance that call to th checkIndications() discovers a service
     *   confirmation in the Mailbox (because it also has to read the packet from mailbox). In such case, it
     *   stores in the internal buffer and returns eror code indicating what service should be called to retrive
     *   this message. No other services (including indications check) can be performed until the required function
     *   is called.
     */

public: /* Indication-related constants */

    // Type of the indication send by Master device
    typedef enum {
        masterCurrentStateInd,
        slavesCurrentStateInd,
        newDiagLogEntryInd,
    } IndicationType;

    // Type of the mailbox pakage waiting to be read
    typedef enum {
        None
    } PacketAwaiting;

public: /* Indications-related structures */

    // Specific indication data
    typedef struct {

        // Data type
        IndicationType type;

        // Indication-specific data
        union {
            ECM_IF_MASTER_CURRENT_STATE_IND_DATA_T masterCurrentState;
            ECM_IF_SLAVE_CURRENT_STATE_IND_DATA_T slavesCurrentState;
            ECM_IF_NEW_DIAG_LOG_ENTRIES_IND_DATA_T newDiagLogEntry;
        } data;

        // Number of data in @a data (used when appliccable)
        uint32_t data_num;

    } IndicationData;

public: /* Indications-specific functions */

    /**
     * @brief Checks Master device's mailbox for indication messages. If indications're found, they are
     *    returned via parameters. If user's application is registered for indications it should
     *    regularly check the mailbox to avoid it's overflow. 
     * 
     * @note If any other package awaits in the Mailbox and user calls @f checkIndications() the 
     *    @c PackageAwaits error code will be returned. Call to the packageAwaiting() will point
     *    type of the packet awaiting (and so the method that should be called to retrive package)
     * 
     * @note If indication awaits in the mailbox, and user calls a method that uses mailbox-base
     *    service (e.g. setSlaveTargetState()) the message will be read but the error code 
     *    @c IndicationAwaits will be returned. It meands that the user should call checkIndications()
     *    to read awaiting indication befor performing any other access to Mailbox.
     * 
     * @param data [out]
     *    indication's data (if @c Ok retrned)
     * @param entries [in/out]
     *    number of entries in @p data bfufer on input; number of entries read on output
     * @returns 
     *    @c Ok on success
     *    error code on error
     */
    ErrorCode checkIndications(IndicationData *data, uint32_t &entries);

    /**
     * @brief Returns type of the package waiting to be received by the user
     * 
     * @note If package is awaiting only call to the related method will success. All other calls shall
     *    fail.
     * 
     * @param package [out]
     *    awaiting package
     * @returns 
     *    @c Ok on success
     *    error code on error
     */
    ErrorCode packageAwaiting(PacketAwaiting &package);
    
    /* ============================================================================================================= */
    /* ------------------------------------------- EtherCAT state machine ------------------------------------------ */
    /* ============================================================================================================= */

public: /* EtherCAT State Machine functions */

    /**
     * @brief Sets master's target state ( @see ECM_IF_STATE_* )
     * 
     * @param target_state
     *    target state
     * @returns 
     *    @c Ok on success
     *    error code on error
     */
    ErrorCode setMasterTargetState(uint8_t target_state);

    /**
     * @brief Gets master's current state ( @see ECM_IF_STATE_* )
     * 
     * @param current_state [out]
     *    current state
     * @returns 
     *    @c Ok on success
     *    error code on error
     */
    ErrorCode getMasterCurrentState(uint8_t &current_state);

    /**
     * @brief Sets master's target state ( @see ECM_IF_STATE_* )
     * 
     * @param slave_address
     *    fixed address of the slave (configured one, no topological-dependent)
     * @param target_state
     *    target state
     * @returns 
     *    @c Ok on success
     *    error code on error
     */
    ErrorCode setSlaveTargetState(uint16_t slave_address, uint8_t target_state);

    /**
     * @brief Gets master's current state ( @see ECM_IF_STATE_* )
     * 
     * @param slave_address
     *    fixed address of the slave (configured one, no topological-dependent)
     * @param current_state [out]
     *    current state
     * @returns 
     *    @c Ok on success
     *    error code on error
     */
    ErrorCode getSlaveCurrentState(uint16_t slave_address, uint8_t &current_state);

    /**
     * @brief Registers host for receiving indications packets when the Master/Slaves state (i.e. state
     *    in the EtherCAT State Machine) is changed. 
     * 
     * @returns 
     *    @c Ok on success
     *    error code on error
     */
    ErrorCode resgisterForStatusIndications();

    /**
     * @brief Unregisters host from receiving indications packets when the Master/Slaves state (i.e. state
     *    in the EtherCAT State Machine) is changed. 
     * 
     * @returns 
     *    @c Ok on success
     *    error code on error
     */
    ErrorCode unresgisterFromStatusIndications();

    /* ============================================================================================================= */
    /* ----------------------------------------------- Diagnostic log ---------------------------------------------- */
    /* ============================================================================================================= */


public: /* Diagnostic log functions */

    /**
     * @brief Gets the oldest diagnostic log entry form the Master
     * 
     * @param lost_entries 
     *    number of lost entries since last read ( @see Master V4.5 documentation)
     * @param entry 
     *    diagnostic entry read from the master
     * @returns 
     *    @c Ok on success
     *    error code on error
     */
    ErrorCode getDiagnosticLogEntry(uint32_t &lost_entries, ECM_DIAG_ENTRY_T &entry);

    /**
     * @brief Clears diagnostic logs in the Master
     * 
     * @returns 
     *    @c Ok on success
     *    error code on error
     */
    ErrorCode clearDiagnosticLog();

    /**
     * @brief Registers host for receiving indications packets when new diagnostic log entries emerge
     * 
     * @note 'New diagnostic log entry' indications should be handled in the specific way by the user
     *    application. If the indication is received, the next called service on the device's Mailbox
     *    should be reading dignostic logs ( @see getDiagnosticLogEntry() ) as long as the log is empty
     *    (indications are sent by the CIFX hardware as header for the list of log entries). For details
     *    @see 'Diagnostic log indication handlin' chapter in the EtherCAT Master V4.5.0 doc
     * 
     * @returns 
     *    @c Ok on success
     *    error code on error
     */
    ErrorCode resgisterForDiagLogIndications();

    /**
     * @brief Unegisters host for receiving indications packets when new diagnostic log entries emerge
     * 
     * @returns 
     *    @c Ok on success
     *    error code on error
     */
    ErrorCode unresgisterForDiagLogIndications();

    /* ============================================================================================================= */
    /* --------------------------------------------- CoE Communication --------------------------------------------- */
    /* ============================================================================================================= */


public: /* CoE Communication structures */

    /**
     * @brief CoE SDO request structure
     */
    typedef struct {

        // Slave's configured address (topological address during generic bus scan)
        uint16_t slaveAddress;
        // CoE object's index
        uint16_t objIndex;
        // CoE object's subindex
        uint8_t subIndex;
        // Access type (@c true for complete access, @c false for single subindex)
        bool completeAccess;
        // Requeste's timeout [ms]
        uint32_t timeoutMs;

    } CoeSdoRequest;

    /**
     * @brief CoE (SDOINFO) Object Dictionary List request structure
     */
    typedef struct {

        // Slave's configured address (topological address during generic bus scan)
        uint16_t slaveAddress;
        // Requeste's timeout [ms]
        uint32_t timeoutMs;
        // Type of the list ot be read ( @see ECM_IF_COE_SDOINFO_GETODLIST_LIST_TYPE_* )
        uint16_t listType;

    } CoeOdListRequest;

    /**
     * @brief CoE (SDOINFO) Object Description request structure
     */
    typedef struct {

        // Slave's configured address (topological address during generic bus scan)
        uint16_t slaveAddress;
        // CoE object's index
        uint16_t objIndex;
        // CoE object's subindex
        uint8_t subIndex;
        // Requeste's timeout [ms]
        uint32_t timeoutMs;

    } CoeOdDescRequest;

    /**
     * @brief CoE (SDOINFO) Object Entry request structure
     */
    typedef struct {

        // Slave's configured address (topological address during generic bus scan)
        uint16_t slaveAddress;
        // CoE object's index
        uint16_t objIndex;
        // CoE object's subindex
        uint8_t subIndex;
        // Request value info ( @see MSK_ECM_IF_COE_SDOINFO_GETENTRYDESC_VALUE_INFO_FLAGS_* ) [in/out]
        uint8_t requestedValueInfo;
        // Requeste's timeout [ms]
        uint32_t timeoutMs;

    } CoeEntryDescRequest;
    
public: /* CoE functions */

    /**
     * @note CoE access is possible in the PREOP, SAFEOP and OP slave's states only if supported by
     *    the slave at all)
     */

    /**
     * @brief Downloads (writes) data to the slave via CoE SDO
     * 
     * @param req 
     *    request structure
     * @param data 
     *    data to be downloaded
     * @param data_len
     *    number of bytes to be downloaded
     * @returns 
     *    @c Ok on success
     *    error code on error 
     */
    ErrorCode coeDownload(const CoeSdoRequest &req, uint8_t *data, uint32_t data_len);

    /**
     * @brief Uploads (reads) data from the slave via CoE SDO
     * 
     * @param req 
     *    request structure
     * @param data [out]
     *    data buffer
     * @param data_len [in/out]
     *    max number of bytes to be read; set to actual number of bytes read on success
     * @returns 
     *    @c Ok on success
     *    error code on error
     */
    ErrorCode coeUpload(const CoeSdoRequest &req, uint8_t *data, uint32_t &data_len);

    /**
     * @brief Reads Object Dictionary List from the slave via CoE SDOINFO
     * 
     * @param req 
     *    request structure
     * @param data [out]
     *    data buffer
     * @param data_len [in/out]
     *    max number of bytes to be read; set to actual number of bytes read on success
     * @returns 
     *    @c Ok on success
     *    error code on error
     */
    ErrorCode coeGetOdList(const CoeOdListRequest &req, uint8_t *data, uint32_t &data_len);

    /**
     * @brief Reads Object's Description from the slave via CoE SDOINFO
     * 
     * @param req 
     *    request structure
     * @param data [out]
     *    data buffer
     * @param data_len [in/out]
     *    max number of bytes to be read; set to actual number of bytes read on success
     * @returns 
     *    @c Ok on success
     *    error code on error
     */
    ErrorCode coeGetObjectDescription(const CoeOdDescRequest &req, uint8_t *data, uint32_t &data_len);

    /**
     * @brief Reads Entry's Description from the slave via CoE SDOINFO
     * 
     * @param req [in/out]
     *    request structure
     * @param data [out]
     *    data buffer
     * @param data_len [in/out]
     *    max number of bytes to be read; set to actual number of bytes read on success
     * @returns 
     *    @c Ok on success
     *    error code on error
     */
    ErrorCode coeGetEntryDescription(CoeOdDescRequest &req, uint8_t *data, uint32_t &data_len);

    /* ============================================================================================================= */
    /* --------------------------------------------- Distributed Clocks -------------------------------------------- */
    /* ============================================================================================================= */

public: /* Distributed Clocks diagnostic functions  */

    /**
     * @brief Reads DC (Distributed Clocks) deviation informations from the device
     * 
     * @param deviation [out]
     *    DC deviation informations
     * @returns 
     *    @c Ok success
     *    error code on error
     */
    ErrorCode getDcDeviation(DcDeviation &deviation);

    /**
     * @brief Resets DC (Distributed Clocks) max deviation informations from the device
     * 
     * @returns 
     *    @c Ok success
     *    error code on error
     */
    ErrorCode resetDcMaxDeviation();

    /**
     * @brief Reads DC (Distributed Clocks) informations concerning from the slave device
     * 
     * @param slave_address
     *    slave device's configured/fixed address
     * @param info [out]
     *    read informations on success
     * @returns 
     *    @c Ok success
     *    error code on error
     */
    ErrorCode getDcDeviation(uint16_t slave_address, DcSlaveInfo &info);

    /* ============================================================================================================= */
    /* ----------------------------------------------- Bus statistics ---------------------------------------------- */
    /* ============================================================================================================= */

public: /* Bus statistics functions */

    /**
     * @brief Reads timing informations from the device
     * 
     * @param info [out]
     *    timing informations
     * @returns 
     *    @c Ok success
     *    error code on error
     */
    ErrorCode getBusTimingInfo(TimingInfo &info);

    /**
     * @brief Retrieves informations about error frames' statics
     * 
     * @param info [out]
     *    retrived informations
     * @param reset 
     *    if ste to @c true, the error counter logs will be reset after read
     * @returns 
     *    @c Ok success
     *    error code on error
     */
    ErrorCode getFrameErrorCounters(FrameErrorInfo &info, bool reset);

    /* ============================================================================================================= */
    /* --------------------------------------- Configuration Readout (private) ------------------------------------- */
    /* ============================================================================================================= */

private: /* Configuration readout functions (private) */

    /**
     * @brief Gets WcState (Working Counter State) informations from the master device. Every
     *    area in the process image related to the cyclical telegram has it's WcState assigned
     *    as the boolean flag. This flag is updated by the Master device every cycle. If expected
     *    (i.e. configured, e.g. <Cnt> tag in ENI file) Working Counter for the given telegam matches
     *    the actual one, the WcState is valid and indicates that the data in the given process data's
     *    are are also valid.
     * 
     * @note The entries are in order of their appearance in the cyclic frames.
     * 
     * @param state [out]
     *    states of the Working counters associated with requested data areas in the process data image
     * @param entries [in/out]
     *    number of entries in @p state on input; number of entries read on output (if @c Ok returned)
     * @param offset 
     *    offset of the first entry to be read 
     * @returns 
     *    @c Ok success
     *    error code on error
     */
    ErrorCode getWcStateInfo(WcStateInfo *state, uint32_t &entries, uint32_t offset);

    /**
     * @brief Retrieves informations about regions mapped into the process data image (i.e. data read/written
     *    by the cyclical communication to the CIFX card; @note this is not the same as EtherCAT's logical
     *    address space). Informations about subsequent regions are returned in entries.
     * 
     * @note The entries are in order of their appearance in the cyclic frames.
     * 
     * @param info [out]
     *    informations about data areas in the process data image
     * @param entries [in/out]
     *    number of entries in @p state on input; number of entries read on output (if @c Ok returned)
     * @param offset 
     *    offset of the first entry to be read 
     * @returns 
     *    @c Ok success
     *    error code on error
     */
    ErrorCode getCyclicCommandMapping(MappingInfo *info, uint32_t &entries, uint32_t offset);

    /**
     * @brief Retrieves informations about slave's mapping
     * 
     * @param info [out]
     *    informations about slaves mapping
     * @param entries [in/out]
     *    number of entries in @p state on input; number of entries read on output (if @c Ok returned)
     * @param offset 
     *    offset of the first entry to be read 
     * @returns 
     *    @c Ok success
     *    error code on error
     */
    ErrorCode getCyclicSlaveMapping(SlaveMappingInfo *info, uint32_t &entries, uint32_t offset);

    /* ============================================================================================================= */
    /* --------------------------------------------- Topology (private) -------------------------------------------- */
    /* ============================================================================================================= */

private: /* Topology informations (private) */

    /**
     * @brief Retrieves lsit of entries describing topology of the slaves' connections
     * 
     * @param data [out]
     *    data entries
     * @param entries [in/out]
     *    number of entries in @p data on input; number of entries retrived on output if @c Ok returned
     * @param first_unconnected [out]
     *    index of the first entry describing an unconnected slave (all further entries refer to such slaves)
     * @returns 
     *    @c Ok success
     *    error code on error
     */
    ErrorCode getTopologyInfo(TopologyInfo *data, uint32_t &entries, uint32_t &first_unconnected);

    /**
     * @todo Implement Hilscher's generic and ECM-specific functions responsible for bus scan
     * @see Hilscher's "Application noteNetwork Revision 5 2017-01"
     * @see Hilscher's "Protocol API EtherCAT Master V4.5.0 Revision 6 2020-09" (page 196)
     */

    /* ============================================================================================================= */
    /* ----------------------------------------- ESC/SII Access (private) ------------------------------------------ */
    /* ============================================================================================================= */

private: /* ESC access (private) */

    /**
     * @brief Reads register from the slave
     *
     * @param slaveAddress
     *    Address of the slave to be accessed. During bus scan use the topology position. During normal
     *    operation use configured address.
     * @param addr
     *    physical address to be read
     * @param data [out]
     *    data buffer
     * @param bytes
     *    number of bytes to be read
     * @returns 
     *    @c Ok success
     *    error code on error
     */
    ErrorCode readSlaveRegister(uint16_t slaveAddress, uint16_t addr, uint8_t *data, uint16_t bytes);

    /**
     * @brief Writes register to the slave
     *
     * @param slaveAddress
     *    Address of the slave to be accessed. During bus scan use the topology position. During normal
     *    operation use configured address.
     * @param addr
     *    physical address to be read
     * @param data [out]
     *    data to be written
     * @param bytes
     *    number of bytes to be read
     * @returns 
     *    @c Ok success
     *    error code on error
     */
    ErrorCode writeSlaveRegister(uint16_t slaveAddress, uint16_t addr, uint8_t *data, uint16_t bytes);

    /**
     * @brief Reads slaves SII/EEPROM data
     *
     * @param slaveAddress
     *    Address of the slave to be accessed. During bus scan use the topology position. During normal
     *    operation use configured address.
     * @param word_off
     *    word offset to be read
     * @param data [out]
     *    data buffer
     * @param words
     *    number of words (16-bit) to be read
     * @returns 
     *    @c Ok success
     *    error code on error
     */
    ErrorCode readSlaveSiiEeprom(uint16_t slaveAddress, uint32_t word_off, uint8_t *data, uint32_t words);

    /**
     * @brief Writes register to the slave
     *
     * @param slaveAddress
     *    Address of the slave to be accessed. During bus scan use the topology position. During normal
     *    operation use configured address.
     * @param word_off
     *    word offset to be read
     * @param data
     *    data to be written
     * @param words
     *    number of words (16-bit) to be written
     * @returns 
     *    @c Ok success
     *    error code on error
     */
    ErrorCode writeSlaveSiiEeprom(uint16_t slaveAddress, uint32_t word_off, uint8_t *data, uint32_t words);

    /* ============================================================================================================= */
    /* --------------------------------------- Master configuration (private) -------------------------------------- */
    /* ============================================================================================================= */

private: /* Private configuration typedefs */

    /**
     * @note Toolkit's types redefinitions are made in case Hilscher chanes
     *    naming in the future
     */

    typedef ECM_IF_BEGIN_CONFIGURATION_REQ_DATA_T             BeginConfigData;
    typedef ECM_IF_END_CONFIGURATION_CNF_DATA_T               EndConfigData;
    typedef ECM_IF_ADD_SLAVE_REQ_DATA_T                       SlaveConfig;
    typedef ECM_IF_ADD_SLAVE_MAILBOX_REQ_DATA_T               SlaveMailboxConfig;
    typedef ECM_IF_ADD_SLAVE_SMCFG_REQ_DATA_T                 SlaveSmConfig;
    typedef ECM_IF_ADD_SLAVE_FMMUCFG_REQ_DATA_T               SlaveFmmuConfig;
    typedef ECM_IF_ADD_SLAVE_COE_INITCMD_REQ_DATA_T           SlaveCoeInitCmd;
    typedef ECM_IF_ADD_SLAVE_REG_INITCMD_REQ_DATA_T           SlaveRegInitCmd;
    typedef ECM_IF_ADD_SLAVE_DC_PARAMS_REQ_DATA_T             SlaveDcparams;
    typedef ECM_IF_MANDATORY_SLAVE_LIST_ENTRY_T               SlaveMandatoryConfig;
    typedef ECM_IF_ADD_SLAVE_ESM_TIMEOUTS_REQ_DATA_T          SlaveEsmTimeouts;
    typedef ECM_IF_ADD_SLAVE_ESC_TIMEOUTS_REQ_DATA_T          SlaveEscTimeouts;
    typedef ECM_IF_ADD_CYCLIC_FRAME_REQ_DATA_T                CyclicFrame;
    typedef ECM_IF_ADD_CYCLIC_TELEGRAM_REQ_DATA_T             CyclicTelegram;
    typedef ECM_IF_SET_DEFAULT_TARGET_STATE_REQ_DATA_T        DefaultTargetState;
    typedef ECM_IF_SET_BASE_SYNC_OFFSET_PERCENTAGE_REQ_DATA_T BaseSyncOffsetConfig;

public: /* Configuration functions */

    /**
     * @brief Loads EtherCAT Master's configuration from downloaded ENI file
     * 
     * @param path
     *    path to the ENI file
     * @returns 
     *    @c Ok success
     *    @c AlreadyConfigured if master is already configured
     *    @c FileOpenErr if file could not be opened
     *    @c FileAccessErr if file could not be accessed
     *    @c Err on other failure
     */
    ErrorCode loadEni(const std::string &path);

private: /* Configuration functions (packet senders) */

    /**
     * @brief Begins EtherCAT Master's manual configuration
     * 
     * @param config 
     *    configuration's generl parameters
     * @returns 
     *    @c Ok success
     *    error code on error
     */
    ErrorCode beginConfigurationSend(const BeginConfigData &config);

    /**
     * @brief Ends EtherCAT Master's manual configuration
     * 
     * @param config [out]
     *    configuration's summary
     * @returns 
     *    @c Ok success
     *    error code on error
     */
    ErrorCode endConfigurationSend(EndConfigData &config);

    /**
     * @brief Aborts EtherCAT Master's manual configuration
     * 
     * @returns 
     *    @c Ok success
     *    error code on error
     */
    ErrorCode abortConfigurationSend();

    /**
     * @brief Unloads EtherCAT Master's configuration and turns it to the unconfigured state
     * 
     * @returns 
     *    @c Ok success
     *    error code on error
     */
    ErrorCode unloadConfigurationSend();

    /**
     * @brief Loads EtherCAT Master's configuration from downloaded ENI file
     * 
     * @returns 
     *    @c Ok success
     *    @c Timeout on timeout
     *    @c Err on failure
     */
    ErrorCode loadEniSend();

    /**
     * @brief Adds slave to the Master's configuration
     * 
     * @param config
     *    slave's configuration
     * @returns 
     *    @c Ok success
     *    error code on error
     */
    ErrorCode addSlaveSend(const SlaveConfig &config);

    /**
     * @brief Adds slave's mailbox configuration
     * 
     * @param config 
     *    slave's mailbox configuration
     * @returns 
     *    @c Ok success
     *    error code on error 
     */
    ErrorCode addSlaveMailboxConfigSend(const SlaveMailboxConfig &config);

    /**
     * @brief Adds slave's SM (Synchronisation Manager) configuration
     * 
     * @param config 
     *    slave's SM configuration
     * @returns 
     *    @c Ok success
     *    error code on error 
     */
    ErrorCode addSlaveSmConfigSend(const SlaveSmConfig &config);

    /**
     * @brief Adds slave's FMMU (Fieldbus Memory Management Unit) configuration
     * 
     * @param config 
     *    slave's FMMU configuration
     * @returns 
     *    @c Ok success
     *    error code on error 
     */
    ErrorCode addSlaveFmmuConfigSend(const SlaveFmmuConfig &config);

    /**
     * @brief Adds slave's register initial command 
     * 
     * @param cmd
     *    slave's register init command
     * @returns 
     *    @c Ok success
     *    error code on error 
     */
    ErrorCode addSlaveRegInitCmdSend(const SlaveRegInitCmd& cmd);

    /**
     * @brief Adds slave's CoE initial command 
     * 
     * @param cmd
     *    slave's CoE init command
     * @returns 
     *    @c Ok success
     *    error code on error 
     */
    ErrorCode addSlaveCoeInitCmdSend(const SlaveCoeInitCmd& cmd);

    /**
     * @brief Adds slave's DC parameters 
     * 
     * @param params
     *    slave's DC (Distibuted Clocks) params
     * @returns 
     *    @c Ok success
     *    error code on error 
     */
    ErrorCode addSlaveCoeInitCmdSend(const SlaveDcparams& params);
    
    /**
     * @brief Adds slave's ESC (EtherCAT Slave Chip) timeouts (i.e. timeouts that apply
     *   when slave's registers are read/written)
     * 
     * @param timeouts
     *    slave's timeouts
     * @returns 
     *    @c Ok success
     *    error code on error 
     */
    ErrorCode addSlaveEscTimeoutsSend(const SlaveEscTimeouts& timeouts);

    /**
     * @brief Adds list of mandatory slave's in configuration
     * 
     * @param config
     *    slave's configuration
     * @param entries
     *    number of entries in @p config
     * @returns 
     *    @c Ok success
     *    error code on error 
     */
    ErrorCode addMandatorySlaveListSend(SlaveMandatoryConfig* config, uint32_t entries);

    /**
     * @brief Adds cyclical frame to the master's configuration
     * 
     * @param frame
     *    frame's parameters
     * @returns 
     *    @c Ok success
     *    error code on error 
     */
    ErrorCode addCyclicFrameSend(const CyclicFrame& frame);

    /**
     * @brief Adds cyclical telegram to the master's configuration
     * 
     * @param telegram
     *    telegram's parameters
     * @returns 
     *    @c Ok success
     *    error code on error 
     */
    ErrorCode addCyclictelegramSend(const CyclicFrame& frame);

    /**
     * @brief Sets Master's default target state during normal oepration
     * 
     * @param state
     *    default target state
     * @returns 
     *    @c Ok success
     *    error code on error 
     */
    ErrorCode setDefaultTargetStateSend(const DefaultTargetState& state);

    /**
     * @brief Sets base SYNC offset in the bus cycle
     * 
     * @param offset
     *    SYNC offset
     * @returns 
     *    @c Ok success
     *    error code on error 
     */
    ErrorCode setBaseSyncOffsetSend(const BaseSyncOffsetConfig& offset);    

    /* ============================================================================================================= */
    /* ----------------------------------------- Loggin fucntions (private) ---------------------------------------- */
    /* ============================================================================================================= */

    /**
     * @brief Debug message trace function
     * 
     * @param format 
     *    message function
     * @param ... 
     *    @p format depentent parameters (printf-style)
     */
    void traceDebug(std::string format, ...);

    /**
     * @brief Info message trace function
     * 
     * @param format 
     *    message function
     * @param ... 
     *    @p format depentent parameters (printf-style)
     */
    void traceInfo(std::string format, ...);

    /**
     * @brief Warning message trace function
     * 
     * @param format 
     *    message function
     * @param ... 
     *    @p format depentent parameters (printf-style)
     */
    void traceWarning(std::string format, ...);

    /**
     * @brief Error message trace function
     * 
     * @param format 
     *    message function
     * @param ... 
     *    @p format depentent parameters (printf-style)
     */
    void traceError(std::string format, ...);

    /**
     * @brief Prints error message about non-zero status code returned with the packet
     *    confirmation
     * 
     * @param function
     *    text representing function related to the error confirmation 
     * @param code 
     *    error status code
     */
    void tracePacketConfirmationError(std::string function, uint32_t code);

    /* ============================================================================================================= */
    /* ----------------------------------------- Member variables (private) ---------------------------------------- */
    /* ============================================================================================================= */

private: /* CIFX-specific variables */

    // Handle to the communication channel instance (representation of DPM's channel)
    ChannelHandle channel;

private: /* Master's state variables */

    // Set to true if driver is configured
    bool configured;

    // Set to true if bus communication is running
    bool busRunning;

    // Set to true if application registered for ANY indications
    bool registeredForIndications;

    // Actual sync mode
    SyncMode syncMode;

private: /* Master's configuration variables */

    // Master's log verbosity
    bool verbose;

    // Host state timeout
    uint32_t hostStateChangeTimeoutMs;
    // Timeout for bus'es state change
    uint32_t busStateChangeTimeoutMs;
    // Timeout for packet's response
    uint32_t packetTimeoutMs;

private: /* Slave's configuration */

    // Registered slave devices
    std::vector<std::shared_ptr<Slave>> slaves; 

private: /* Data buffers */

    // Input process data image (from slaves to master)
    void *inputBuf; 
    // Size of the input process data image
    size_t inputBufSize;

    // Output process data image (from master to slaves)
    void *outputBuf;
    // Size of the output process data image
    size_t outputBufSize;

};

/* ================================================================================================================================ */

} // End namespace ethercat
} // End namespace cifx

#endif
