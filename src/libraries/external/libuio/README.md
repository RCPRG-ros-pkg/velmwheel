# Original header

libuio - UserspaceIO helper library

Copyright (C) 2011 Benedikt Spranger
based on libUIO by Hans J. Koch

libuio is free software; you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License version 2.1
as published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the Lesser GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

Comments are welcome.

	- Benedikt Spranger <b.spranger@linutronix.de>

# Afterword

These sources was cloned from original Benedikt Spranger's [repository](https://github.com/missinglinkelectronics/libuio). They was organised to compose ROS2-compatible package that can be used in various projects. `libuio` implements not only dynamically linked library but also two executables - `lsuio` and `readuio` that are installe along with package's binaries.

# Hotfixes

- Missing declaration of `uio_free_info()` added
- Function `uio_get_path()` added
