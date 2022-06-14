Welcome to `elmo-ethercat` library !
====================================

**elmo-ethercat** provides abstract implementation of the <a href=https://www.elmomc.com/product/gold-dc-whistle/>Elmo Gold DC Whistle</a> 
servomotor drivers (product: 0x00030924, rev: 0x00010420) in form of the C++ library based on the :ethercat:`ethercat<>` abstract EtherCAT
driver. Library aims to provide full control over capabilities of the device including automatic deduction of preconfigured control mode
based on the content of ENI file.

.. toctree:: 
    :maxdepth: 1
    :caption: Modules:

    src/driver
    src/device_registers
