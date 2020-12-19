This code accompanies a series of articles, which appear here:
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-2/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-3/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-4/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-5/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-6/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-7/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-8/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-9/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-10/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-11/

        http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        http://www.mijagourlay.com/

    \author Copyright 2009-2011 Dr. Michael Jason Gourlay; All rights reserved.

This demonstration project uses Intel's Threading Building Blocks, which you
can obtain here: http://www.threadingbuildingblocks.org/

After you obtain the libraries, you have to unpack them manually and place them
where you want them to reside.  There is no installer for the free version.  For
example, I placed the unpacked folder, which was named "tbb22_013oss_win",
in C:\Program Files\Intel\ThreadingBuildingBlocks.

Note that, starting with version 3, TBB no longer supports Visual Studio .NET
(a.k.a. 2003, a.k.a. version 7.1).  This project is therefore set up to use
The tbb22_013oss_win version of TBB.

You will also have to set up your environment, manually.  You have at least two
options:  Modify "autoexec.bat" or enter each environment variable by hand.

The TBB distribution comes with batch files named "tbbvars.bat" to assign
environment variables.  Those TBB batch files reside in the "bin" directory of
each of the subdirectories for the various version of Visual Studio.  But
running that batch file by itself (e.g. via a Startup command) will not affect
your "global" environment.  You will probably have to execute those commands
from your system's "autoexec.bat" in order for the change to take affect
uniformly.  Usually, "autoexec.bat" resides in "C:\" and, since it is a system
file, is usually hidden.  To find it in Explorer, you will have to change View
settings to disable hiding hidden files, and to disable hiding system files.
Then "autoexec.bat" will appear in the listing for "C:\" or wherever it resides.
Then you can edit that file to contain the commands to set the TBB environment
variables.

Also, you might find that you need to set your executable path such that the
TBB "bin" directory precedes other subdirectories.

For example, the following set of commands could go into your autoexec.bat,
assuming you placed your TBB distribution in the location indicated below:

SET TBB22_INSTALL_DIR=C:\Program Files\Intel\ThreadingBuildingBlocks\tbb22_013oss_win
SET TBB_ARCH_PLATFORM=ia32\vc7.1
SET PATH=%TBB22_INSTALL_DIR%\%TBB_ARCH_PLATFORM%\bin;%PATH%
SET LIB=%TBB22_INSTALL_DIR%\%TBB_ARCH_PLATFORM%\lib;%LIB%
SET INCLUDE=%TBB22_INSTALL_DIR%\include;%INCLUDE%
IF "%TBB_ARCH_PLATFORM%"=="em64t\vc7.1" SET TBB_SECURITY_SWITCH=/GS-
IF "%TBB_ARCH_PLATFORM%"=="intel64\vc7.1" SET TBB_SECURITY_SWITCH=/GS-

After you modify "autoexec.bat" you will have to reboot your machine for the
changes to take effect.

If you do not want to modify "autoexec.bat" or if you do not have permission
to do so, then you would have to set environment variables manually using
Control Panel --> System --> Advanced --> Environment Variables.  After you
enter each of the variables listed in the "tbbvars.bat" file, you will be
able to build and run all of the configurations included in this project.

Alternately, you can modify the tbbvars.bat script to use "setx" instead of "set".
The syntax is a little different.  Here is an example:

SETX TBB22_INSTALL_DIR "C:\Program Files\Intel\ThreadingBuildingBlocks\tbb22_013oss_win"
SETX PATH "C:\Program Files\Intel\ThreadingBuildingBlocks\tbb22_013oss_win\ia32\vc7.1\bin;%PATH%"
SETX LIB "C:\Program Files\Intel\ThreadingBuildingBlocks\tbb22_013oss_win\ia32\vc7.1\lib;%LIB%"
SETX INCLUDE "C:\Program Files\Intel\ThreadingBuildingBlocks\tbb22_013oss_win\include;%INCLUDE%"

This project also includes a configuration that does not use TBB, so that you
can compare timings.  Try ProfileWithoutTbb.  Incidentally, that configuration
should build and run even if you do not have TBB installed.  For any build
configuration, you can also change USE_TBB=1 to USE_TBB=0 within Project -->
Properties --> C++ --> Preprocessor --> Preprocessor Definitions; it will just
run slower than the corresponding version with TBB.

-=-

After you get it all running, once the window appears, you can press various
keys to make changes.  The function keys change the initial conditions.

You can also mouse-click-drag in the window to change the view. Left mouse
button controls rotation, middle mouse button controls translation and right
mouse button controls distance from the view target. Also, the "," and "." keys
zoom in and out.

Pressing "Esc" will exit the program.

Pressing "g" cycles through various modes of rendering a grid.

Presing "c" or "C" toggles continuous camera motion.

In profile builds, pressing "?" toggles spewing profile data in the console window.

-=-

This file and all that accompany it are copyright 2009-2011 by the author,
Dr. Michael Jason Gourlay; All rights reserved.  Contact the author at
mijagourlay.com for licensing.

Licensing terms are negotiable but generally entail the following:
* The author receives credit in any product that uses this code, usually by listing in the credits for the product.
* The author receives a free copy of the product for each SKU."
* If the website lists other middleware used by the product, then VorteGrid should also be listed, along with its logo, which is the blue-and-cyan swirly icon that comes with this distribution, with the word "VorteGrid" superimposed over that image.
