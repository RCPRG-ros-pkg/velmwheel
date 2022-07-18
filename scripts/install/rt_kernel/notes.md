# RT Kernel

Project has been tested with the RT kernel configured via setting following options on top of the default Ubuntu 22.04LTS kernel configuration:

    # Enable CONFIG_PREEMPT_RT
    -> General Setup
    -> Preemption Model (Fully Preemptible Kernel (Real-Time))
    (X) Fully Preemptible Kernel (Real-Time)

    # Enable CONFIG_HIGH_RES_TIMERS
    -> General setup
    -> Timers subsystem
    [*] High Resolution Timer Support

    # Enable CONFIG_NO_HZ_FULL
    -> General setup
    -> Timers subsystem
    -> Timer tick handling (Full dynticks system (tickless))
        (X) Full dynticks system (tickless)

    # Set CONFIG_HZ_1000 (note: this is no longer in the General Setup menu, go back twice)
    -> Processor type and features
    -> Timer frequency (1000 HZ)
    (X) 1000 HZ

    # Set CPU_FREQ_DEFAULT_GOV_PERFORMANCE [=y]
    ->  Power management and ACPI options
    -> CPU Frequency scaling
    -> CPU Frequency scaling (CPU_FREQ [=y])
        -> Default CPUFreq governor (<choice> [=y])
        (X) performance

According to this [question](https://askubuntu.com/questions/1329538/compiling-the-kernel-5-11-11) also the following configuration has been supplied:

    # Unset TRUSTED_KEYS list
    -> Cryptographic API
        -> Certificates for signature checking
            -> Additional X.509 keys for default system keyring
            ("")
            -> X.509 certificates to be preloaded into the system blacklist keyring
            ("")

This should probably be replaced with a properly generated, self-signed x509 certificate, although this way is simpler for testing purposes.
