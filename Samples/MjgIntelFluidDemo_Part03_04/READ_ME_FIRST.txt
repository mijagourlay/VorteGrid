This code accompanies a series of articles, the first of which appears here:
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/

This demonstration project uses Intel's Threading Building Blocks, which you
can obtain here: http://www.threadingbuildingblocks.org/

After you obtain the libraries, you have to unpack then manually and place them
where you want them to reside.  There is no installer for the free version.  For
example, I placed the unpacked folder, which was named "tbb21_017oss_win",
in C:\Program Files\Intel\ThreadingBuildingBlocks.

You will also have to set up your environment, manually.  You have at least two
options:  Modify "autoexec.bat" or enter each environment variable by hand.

The TBB distribution comes with batch files named "tbbvars.bat" to assign environment
variables.  Those TBB batch files reside in the "bin" directory of each of the
subdirectories for the various version of Visual Studio.  But running that batch file
by itself (e.g. via a Startup command) will not affect your "global" environment.
You will probably have to execute those commands that from your system's "autoexec.bat"
in order for the change to take affect uniformly.  Usually, "autoexec.bat" resides
in "C:\" and, since it is a system file, is usually hidden.  To find it in Explorer,
you will have to change View settings to disable hiding hidden files, and to disable
hiding system files.  Then "autoexec.bat" will appear in the listing for "c:\" or
wherever it resides.  Then you can edit that file to contain the commands to set
the TBB environment variables.

Also, you might find that you need to set your executable path such that the
TBB "bin" directory precedes other subdirectories.

For example, the following set of commands could go into your autoexec.bat,
assuming you placed your TBB distribution in the location indicated below:

SET TBB21_INSTALL_DIR=C:\Program Files\Intel\ThreadingBuildingBlocks\tbb21_017oss_win
SET TBB_ARCH_PLATFORM=ia32\vc7.1
SET PATH=%TBB21_INSTALL_DIR%\%TBB_ARCH_PLATFORM%\bin;%PATH%
SET LIB=%TBB21_INSTALL_DIR%\%TBB_ARCH_PLATFORM%\lib;%LIB%
SET INCLUDE=%TBB21_INSTALL_DIR%\include;%INCLUDE%
IF "%TBB_ARCH_PLATFORM%"=="em64t\vc7.1" SET TBB_SECURITY_SWITCH=/GS-
IF "%TBB_ARCH_PLATFORM%"=="intel64\vc7.1" SET TBB_SECURITY_SWITCH=/GS-

After you modify "autoexec.bat" you will have to reboot your machine for the
changes to take effect.

If you do not want to modify "autoexec.bat" or if you do not have permission
to do so, then you would have to set environment variables manually using
Control Panel --> System --> Advanced --> Environment Variables.  After you
enter each of the variables listed in the "tbbvars.bat" file, you will be
able to build and run all of the configurations included in this project.

This project also includes a configuration that does not use TBB, so that you
can compare timings.  Try ProfileWithoutTbb.  Incidentally, that project
should build and run even if you do not have TBB installed; it will just
run slower than the corresponding version with TBB.

-=-

After you get it all running, once the window appears, you can press
various keys to make changes.  The function keys change the initial
conditions.  In particular, "F1", "F2" and "F3" will have general interest.

You can also mouse-click-drag in the window to change the view.
Left mouse button controls rotation, middle mouse button controls
translation and right mouse button controls distance from the view target.
Also, the "," and "." keys zoom in and out.

In profile builds, the "?" key toggles spewing profile data in the console window.

Pressing "Esc" will exit the program.
