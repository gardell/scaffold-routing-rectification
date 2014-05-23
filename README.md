scaffold-routing-rectification
==============================

Rectification of a calculated scaffold routing using NVIDIA PhysX.

To run the program, the following files must be placed in the same directory:
*scaffold-routing-rectification.exe found in x64/Release/
*PhysX3_x64.dll, PhysX3Common_x64.dll found in scaffold-routing-rectification/

The program takes the following input arguments:

Usage: scaffold-routing-rectification.exe
        --input=<filename>
        --output=<filename>
        [ --scaling=<decimal> ]
	[ --discretize_lengths=<true|false> ]
        [ --density=<decimal> ]
        [ --spring_stiffness=<decimal> ]
        [ --fixed_spring_stiffness=<decimal> ]
        [ --spring_damping=<decimal> ]
        [ --attach_fixed=<true|false> ]
        [ --static_friction=<decimal> ]
        [ --dynamic_friction=<decimal> ]
        [ --restitution=<decimal> ]
        [ --rigid_body_sleep_threshold=<decimal> ]
        [ --visual_debugger=<true|false> ]

Usually, the rectification is run as:


scaffold-routing-rectification.exe --input=inputfile.rmsh --output=outputfile.vhelix

The visual_debugger is only available when doing a debug build. For this to work, use the scaffold-routing-rectification.exe from x64/Debug/ and PhysX3CHECKED_x64.dll, PhysX3CommonCHECKED_x64.dll from scaffold-routing-rectification/. These require Visual Studio 2013 to be installed for debug builds of the Visual C++ Runtime.

To use the visual_debugger, start NVIDIA PhysX Visual Debugger *before* the scaffold-routing-rectification.exe.

Currently, lengths are discretized taking into account their routing. To disable this, use --discretize_lengths=false