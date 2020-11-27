# Copyright 2005-2010 Intel Corporation.  All Rights Reserved.
#
# This file is part of Threading Building Blocks.
#
# Threading Building Blocks is free software; you can redistribute it
# and/or modify it under the terms of the GNU General Public License
# version 2 as published by the Free Software Foundation.
#
# Threading Building Blocks is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with Threading Building Blocks; if not, write to the Free Software
# Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
#
# As a special exception, you may use this file as part of a free software
# library without restriction.  Specifically, if other files instantiate
# templates or use macros or inline functions from this file, or you compile
# this file and link it with other files to produce an executable, this
# file does not by itself cause the resulting executable to be covered by
# the GNU General Public License.  This exception does not however
# invalidate any other reasons why the executable file might be covered by
# the GNU General Public License.

# Common Makefile that builds and runs example.

# Just specify your program basename
PROG=Pover
ARGS=

# The C++ compiler options
CXX = cl.exe

# Uncomment one of next lines to choose user interface type (console, gdiplus, direct draw)
#UI = con
UI = gdi
#UI = dd

# Machine architecture, auto-detected from TBB_ARCH_PLATFORM by default
# Use XARCH variable to change it. See index.html for more information
ARCH0 = $(TBB_ARCH_PLATFORM)-
ARCHA = $(ARCH0:\vc7.1-=)
ARCHB = $(ARCHA:\vc8-=)
ARCHC = $(ARCHB:\vc9-=)
ARCH3 = $(ARCHC:ia32=x86)
ARCH4 = $(ARCH3:intel64=AMD64)
XARCH = $(ARCH4:-=x86)

MAKEINC = ../../common/gui/Makefile.win

all: release test
release:
	@$(MAKE) -f $(MAKEINC) UI=$(UI) CXX="$(CXX)" CXXFLAGS="$(CXXFLAGS)" LFLAGS="$(LDFLAGS) tbb.lib tbbmalloc.lib $(LIBS)" XARCH=$(XARCH) RCNAME=pover SOURCE=*.cpp EXE=$(PROG).exe build_one
debug:
	@$(MAKE) -f $(MAKEINC) UI=$(UI) DEBUG=_debug CXX="$(CXX)" CXXFLAGS="$(CXXFLAGS) /D TBB_USE_DEBUG" LFLAGS="$(LDFLAGS) tbb_debug.lib tbbmalloc_debug.lib $(LIBS)" XARCH=$(XARCH) RCNAME=pover SOURCE=*.cpp EXE=$(PROG).exe build_one
clean:
	@cmd.exe /C del $(PROG).exe *.obj *.?db *.manifest vc7.1\pover.res
test:
	$(PROG) $(ARGS)
