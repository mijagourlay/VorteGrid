rem @echo off
REM
REM Copyright 2005-2010 Intel Corporation.  All Rights Reserved.
REM
REM This file is part of Threading Building Blocks.
REM
REM Threading Building Blocks is free software; you can redistribute it
REM and/or modify it under the terms of the GNU General Public License
REM version 2 as published by the Free Software Foundation.
REM
REM Threading Building Blocks is distributed in the hope that it will be
REM useful, but WITHOUT ANY WARRANTY; without even the implied warranty
REM of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
REM GNU General Public License for more details.
REM
REM You should have received a copy of the GNU General Public License
REM along with Threading Building Blocks; if not, write to the Free Software
REM Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
REM
REM As a special exception, you may use this file as part of a free software
REM library without restriction.  Specifically, if other files instantiate
REM templates or use macros or inline functions from this file, or you compile
REM this file and link it with other files to produce an executable, this
REM file does not by itself cause the resulting executable to be covered by
REM the GNU General Public License.  This exception does not however
REM invalidate any other reasons why the executable file might be covered by
REM the GNU General Public License.
REM
SETX TBB22_INSTALL_DIR "C:\Program Files\Intel\tbb22_013oss_win"
SET TBB_ARCH_PLATFORM ia32\vc7.1
SETX PATH "C:\Program Files\Intel\tbb22_013oss_win\ia32\vc7.1\bin;%PATH%"
SETX LIB "C:\Program Files\Intel\tbb22_013oss_win\ia32\vc7.1\lib;%LIB%"
SETX INCLUDE "C:\Program Files\Intel\tbb22_013oss_win\include;%INCLUDE%"
IF "%TBB_ARCH_PLATFORM%"=="em64t\vc7.1" SETX TBB_SECURITY_SWITCH=/GS-
IF "%TBB_ARCH_PLATFORM%"=="intel64\vc7.1" SETX TBB_SECURITY_SWITCH=/GS-
pause -1