
PhysX Plug-in

Copyright 2015 University of Central Florida



SUMMARY
=======

This plug-in uses NVIDIA's PhysX library to handle physics for OpenSim.
Its goal is to provide all the functions that the Bullet plug-in does.

This code has been released under the Apache License Version 2.0 (see 
licensing information below) and is provided by the University of
Central Florida and the U.S. Army's Army Research Laboratory Simulation
and Training Technology Center (under the Military Open Simulator
Enterprise Strategy, or MOSES, program).  Individual contributors can
be found in the CONTRIBUTORS.txt file included in this archive.


COMPILING
=========

This wrapper has a number of dependencies:

- NVIDIA's PhysX (version 3.3.3 or later)
- NVIDIA's Cuda library (for GPU support)
- The PhysX Wrapper (also created by UCF and available separately)

This plug-in is built with OpenSim.  Specifically, the version of OpenSim
used can be obtained as follows:

git clone https://github.com/opensim/opensim.git
git checkout c4f86309683bc8db0bdeabddcefd78bb72b178b6

It may work for other versions but your mileage may vary.  Note that this
plug-in has *not* adopted the new API introduced for OpenSim so it will
almost not certainly work for the latest OpenSim.

The dependencies above need to be copied into the OpenSim/bin/lib64 directory,
and the OpenSim/bin/lib64 directory must be in your path (Windows) or 
LD_LIBRARY_PATH (Linux).

In addition, for Linux a link within the OpenSim/bin directory is required
as follows (this addresses a path issue with this DSO):

ln -s lib64/libPhysX3Gpu_x64.so libPhysX3Gpu_x64.so


LICENSE
=======

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

