# Informations about toolkit

To build toolkit properly the `cifx_os` and `cifx_user` user-defined packages should be provided during compilation,
as Toolkit is OS- and platform-agnostic. To get informations about general rules of interacting with netX systems 
(powering CIFX and COMX devices) refer to the Hilscher's documents in the given order:

1. `cifx netX Application Programmers Guide PRG xx EN.pdf` - general description of the workflow concerning netX devices
and their communication protocol (so called Dual Port Memory). Describes birdseye perspective of the coding principles
and refers reader to other documents.

2. `netx Dual-Port Memory Interface DPM xx EN.pdf` - comprehensive description of the DPM (Dual Port Memory) interface.
This document has some extensions describing details of host-netX communication via DPM:
    - `netX Dual-Port Memory - Programming reference guide PRG xx EN.pdf`
    - `netX Dual-Port Memory packet-based services netX 10 50 51 52 100 500 API xx EN.pdf`
    - `netX Dual-Port Memory packet-based services netX 90 4000 4100 API 05 EN.pdf` - this documment can
      be skipped, as it describes elements analogous to the previous one but when other netX chips are
      used (CIFX cards use netx 10/5x/100/500 chips)

3. `cifX netX Toolkit - DPM TK xx EN.pdf` - detailed description of the CIFX Tookit which is set of OS- and platform-
independent source code implementing DPM's configuration and communication routines. FIle describes how to acomodate
Toolkit to own platform and OS.

4. `cifX API PR xx EN.pdf` - describes Hilscher standard application programming interface which offers all necessary
functions and information needed to handle netX-based device.

5. `PC Cards CIFX *card-type* UM xx EN.pdf` and `PC Cards cifX Software Installation UM xx EN.pdf` - describe formal
description of choosen Hilscher's cards and software packs

# Setup

The toolkit was build and tested on `Ubuntu 20.04.2 LTS` with `GCC 9.3.0` compiler and `CIFX/netX Toolkit V2.6.0`.
