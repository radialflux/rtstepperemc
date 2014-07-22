#
# configure.ac - autoconf input file for rtstepper_emc
# 
# (c) 2010-2011 Copyright Eckler Software
# 
# Author: David Suffield, dsuffiel@ecklersoft.com
#
# Notes on cross-compiling with mingw32.
#
#  1. Install mingw.
#
#     yum install mingw32-*
#
#  2. Now build binary for Windows.
#
#     ./bootstrap
#     ./configure --host i686-pc-mingw32 CFLAGS=-I/usr/i686-pc-mingw32/sys-root/mingw/include/ddk (Fedora 13)
#
#     or
#
#     ./configure --host i586-mingw32msvc (Ubuntu 9.04)
#     ./configure --host=i686-w64-mingw32 LDFLAGS="-L/usr/i686-w64-mingw32/lib" (Ubuntu 12.04 32-bit)
#     ./configure --host=x86_64-w64-mingw32 LDFLAGS="-L/usr/x86_64-w64-mingw32/lib" (Ubuntu 12.04 64-bit)
#
#     make
#     make zip
#
#  3. Copy the zip file to Windows XP and unzip. 
#
# Notes on compiling on OSX.
#
#  1. Download install xcode to get the GNU tools.
#  2. Download install darwinports at darwinport.org/getdp.
#  3. Install libusb support
#
#      sudo port install libusb-legacy
#
#  4. Build binary for OSX 10.6.
#
#      ./configure --prefix=/opt/local LDFLAGS="-L/opt/local/lib -L/opt/local/lib/libusb-legacy" CFLAGS="-I/opt/local/include -I/opt/local/include/libusb-legacy" CXXFLAGS="-I/opt/local/include -I/opt/local/include/libusb-legacy"
#     make
#     make dmg
#
# Notes on compiling on Ubuntu 10.04
#
#     ./configure --prefix=/usr CFLAGS=-I/usr/include/tcl8.4 CXXFLAGS=-I/usr/include/tcl8.4 
#     make
#     make install
#
# Handy C format command:
#     indent -bli0 -nbad -i3 -npcs -npsl -nut -l160 -c1 tclext.c
#
# History:
#
# Version 0.9, des 12/20/2010    Using emc2_2.4.4 created rtstepper_emc, a version for the rt-stepper dongle. Removed
#                                emcmot, emcio, hal, rt-linux and emcsvr. GUIs keystick, tkemc and mini will display.
#                                In order to run emc.so on Fedora 13 selinux must be set to passive - "setenforce 0".
# Version 1.7, des 03/3/2011     First public release.
# Version 1.7.9, des 08/29/11    Added abort support for input1 and input2. 
# Version 1.7.11, des 08/31/11   Support for input1 and input2 works. 
# Version 1.7.12, des 01/26/11   Fixed darwin dmg issue in rtstepper.list.
# Version 1.7.20, des 12/29/11   New initial work to remove rcslib dependency. Replace NML with MSG API.
# Version 1.7.21, des 02/02/12   Coding complete and compiles into one dll. No more milltask process. No rcslib dependencies.
# Version 1.7.35, des 02/18/12   Fixed old and new bugs. Will now run jog, mdi and auto commands in VirtualBox.
# Version 1.7.36, des 02/19/12   Fixed USB synchronization issue after pthread_cancel().
# Version 1.7.37, des 02/20/12   Tweaked tkmini for Mac OSX. The Aqua display is not ideal, but it works.
# Version 1.7.49, des 03/03/12   Finished drag/drop application bundle (*.dmg) for OSX. 
# Version 1.7.51, des 03/05/12   DLLEXPORT in tcl 8.4 (Ubuntu 10.04) has problems, replaced with local DLL_EXPORT.  
# Version 1.8, des 03/14/2012    First public release.
# Version 1.8.1, des 03/18/2012  Fixed broken Mac application bundle. 
# Version 1.8.2, des 03/22/2012  Fixed rt-test libusb link in Mac application bundle. 
# Version 1.8.3, des 06/18/2012  First cut at adding support for ACTIVE_HIGH step/direction logic. Previous support was low_true. 
# Version 1.8.5, des 06/20/2012  Good release for ACTIVE_HIGH step/direction logic.
# Version 1.8.6, des 06/23/2012  Fine tuned rtstepper.ini file for "A" axis (angular). Increased acceleration & velocity profile.
# Version 1.8.9, des 07/08/2012  Fixed bug in the DIRECTION_ACTIVE_HIGH logic. Fixed a forever loop when Stepping followed by a Resume.
# Version 1.8.13, des 07/30/2012  First cut with separate thread for trajectory planner.
# Version 1.8.14, des 08/12/2012  New thread ok, now up to two moves are buffered for USB IO.
# Version 1.8.15, des 08/30/2012  Fixed thread synchronization issue with previous control_cycle_thread().
# Version 1.8.16, des 09/09/2012  Public release.
# Version 1.8.17, des 10/01/2012  Fixed a dongle.mutex initialization issue which showed up in Windows.
# Version 1.8.18, des 02/28/2013  Minor changes for building on Ubuntu 12.04.
# Version 1.8.22, des 06/14/2013  Compiles with tinypy.
# Version 1.8.25, des 06/17/2013  Added pytest.c.
# Version 1.8.27, des 06/18/2013  Initial pytest ok, runs emc_ui_init()/emc_ui_exit().
# Version 1.8.35, des 06/24/2013  Tkmini runs.
# Version 1.8.37, des 06/25/2013  Step buffer regression tests ok.
# Version 1.8.47, des 07/04/2013  Pre-release ready.
# Version 1.8.48, des 07/07/2013  Updated udev rules for debian based distros (ie: Mint).
# Version 1.8.57, des 07/19/2013  Re-wrote ini code. All ini reads are now thread safe.
# Version 1.8.61, des 07/31/2013  Added support for coolant.
# Version 1.8.83, des 08/27/2013  Got m190.py working. A python script for homing a specified axis.
# Version 1.8.85, des 09/02/2013  Fixed tkmini hang in showMode(). Fixed mutex hang _wait_received(). 
# Version 1.8.86, des 09/03/2013  Fixed a hang when exiting tkmini. Moved dongle mutex and write_done_cond into ui.cc. 
# Version 2.0, des 09/10/2013  First public release with python script support for mcodes.

AC_INIT([rtstepperemc], [2.0])
AC_CANONICAL_HOST

# Checks for programs.
AC_PROG_CC
AC_PROG_CXX
AC_PROG_INSTALL
AC_GNU_SOURCE
#AC_C_BIGENDIAN

darwin_build="no"
mingw_build="no"
linux_build="no"

AC_MSG_CHECKING("for canonical host")
case "$host" in
   *-darwin*)
      darwin_build="yes"
      AC_MSG_RESULT("OS X")
      ;;

   *-mingw*)
      mingw_build="yes"
      AC_MSG_RESULT("Windows cross compile")
      ;;

     *)
      linux_build="yes"
      AC_MSG_RESULT("Linux/Unix")
      ;;
esac
AM_CONDITIONAL(DARWIN_BUILD, test x$darwin_build = xyes)

AC_MSG_CHECKING([for mini build])
AC_ARG_ENABLE(mini_build,
  [  --enable-mini-build    enable mini gui build (default=yes)],
  mini_build=$enableval, mini_build=yes)
if test "$mini_build" = "yes"; then
   AC_MSG_RESULT(yes)
else
   AC_MSG_RESULT(no)
fi
AM_CONDITIONAL(MINI_BUILD, test x$mini_build = xyes)

# Checks for required common libraries.
AC_CHECK_LIB([m], [pow],, [AC_MSG_ERROR([cannot find libm math support], 10)])

# Checks for required development packages.
AC_CHECK_HEADERS(pthread.h,, [AC_MSG_ERROR([cannot find libpthread-devel support], 11)])
#AC_CHECK_HEADERS(curses.h,, [AC_MSG_ERROR([cannot find libcurses-devel support], 12)])
if test "$mingw_build" = "yes"; then
   AC_CHECK_HEADERS(usb-winusb.h,, [AC_MSG_ERROR([cannot find libusb-winusb support], 13)])
else
   AC_CHECK_HEADERS(usb.h,, [AC_MSG_ERROR([cannot find libusb-devel support], 13)])
fi

AC_MSG_CHECKING(for host machine platform)
machine=`uname -m`
if test "$machine" = "x86_32" || test "$machine" = "i686"; then
    platform="x86_32"
elif test "$machine" = "x86_64"; then
    platform="x86_64"
elif test "$machine" = "ppc"; then
    platform="ppc"
else
    platform="unsupport"
fi
AC_MSG_RESULT($platform)

AC_CHECK_FUNCS([strcasestr])
AC_CHECK_FUNCS([sincos])

AC_ARG_WITH(relnum, AC_HELP_STRING([--with-relnum=N], [set release number (default=0)]),
   relnum=$withval, relnum=0)

AC_ARG_WITH(rulesdir, AC_HELP_STRING([--with-rulesdir=DIR], [set udev rules directory [default=/etc/udev/rules.d]]),
   rulesdir=$withval, rulesdir="/etc/udev/rules.d")

if test "$mingw_build" = "yes"; then
   dll_extension="dll"
   if test "$host" = "i686-pc-mingw32"; then
      mingw_path="/usr/i686-pc-mingw32/sys-root/mingw/bin"
      mingw_inc_path="/usr/i686-pc-mingw32/sys-root/mingw/include"
      mingw_lib_path="/usr/i686-pc-mingw32/sys-root/mingw/lib"
   elif test "$host" = "i686-w64-mingw32"; then
      mingw_path="/usr/${host}/lib"
      mingw_inc_path="/usr/${host}/include"
      mingw_lib_path="/usr/${host}/lib"  
   elif test "$host" = "x86_64-w64-mingw32"; then
      mingw_path="/usr/${host}/lib"
      mingw_inc_path="/usr/${host}/include"
      mingw_lib_path="/usr/${host}/lib"  
   else
      mingw_path="/usr/i586-mingw32msvc/lib"
      mingw_inc_path="/usr/i586-mingw32msvc/include" # ??
      mingw_lib_path="/usr/i586-mingw32msvc/lib"
   fi
elif test "$darwin_build" = "yes"; then
   dll_extension="dylib"
else
   dll_extension="so"
   CFLAGS="$CFLAGS -fPIC"
   CXXFLAGS="$CXXFLAGS -fPIC"
fi

AC_DEFUN([AC_DEFINE_DIR], [
  prefix_NONE=
  exec_prefix_NONE=
  test "x$prefix" = xNONE && prefix_NONE=yes && prefix=$ac_default_prefix
  test "x$exec_prefix" = xNONE && exec_prefix_NONE=yes && exec_prefix=$prefix
dnl In Autoconf 2.60, ${datadir} refers to ${datarootdir}, which in turn
dnl refers to ${prefix}.  Thus we have to use `eval' twice.
  eval ac_define_dir="\"[$]$2\""
  eval ac_define_dir="\"$ac_define_dir\""
  $1="$ac_define_dir"
  test "$prefix_NONE" && prefix=NONE
  test "$exec_prefix_NONE" && exec_prefix=NONE
])

AC_DEFINE_DIR([abs_bindir], [bindir])
AC_DEFINE_DIR([abs_datarootdir], [datarootdir])

EMC2_HOME=$abs_datarootdir/$PACKAGE_NAME
EMC2_TCL_DIR=$abs_datarootdir/$PACKAGE_NAME/mini/
EMC2_BIN_DIR=$abs_bindir
EMC2_TCL_LIB_DIR=$abs_datarootdir/$PACKAGE_NAME/
EMC2_CONFIG_PATH=/etc/$PACKAGE_NAME
EMC2_HELP_DIR=$abs_datarootdir/$PACKAGE_NAME
EMC2_RTLIB_DIR=$abs_datarootdir/$PACKAGE_NAME
EMC2_NCFILES_DIR=$abs_datarootdir/$PACKAGE_NAME
EMC2_LANG_DIR=$abs_datarootdir/$PACKAGE_NAME
EMC2_IMAGE_DIR=$abs_datarootdir/$PACKAGE_NAME
REALTIME=$abs_datarootdir/$PACKAGE_NAME
SIMULATOR="no"

if test "$mingw_build" = "yes"; then
   EMC2_HOME=
   EMC2_TCL_DIR=
   EMC2_TCL_LIB_DIR=
   EMC2_CONFIG_PATH=etc/$PACKAGE_NAME
elif test "$darwin_build" = "yes"; then
   EMC2_HOME=
   EMC2_TCL_DIR=
   EMC2_TCL_LIB_DIR=lib/
   EMC2_CONFIG_PATH=etc/$PACKAGE_NAME
fi

AC_SUBST(platform)
AC_SUBST(relnum)
AC_SUBST(rulesdir)
AC_SUBST(dll_extension)
AC_SUBST(mingw_path)
AC_SUBST(mingw_inc_path)
AC_SUBST(mingw_lib_path)
AC_SUBST(mingw_build)
AC_SUBST(mini_build)
AC_SUBST(darwin_build)
AC_SUBST(linux_build)

AC_SUBST(EMC2_HOME)
AC_SUBST(EMC2_BIN_DIR)
AC_SUBST(EMC2_TCL_DIR)
AC_SUBST(EMC2_TCL_LIB_DIR)
AC_SUBST(EMC2_CONFIG_PATH)
AC_SUBST(EMC2_HELP_DIR)
AC_SUBST(EMC2_RTLIB_DIR)
AC_SUBST(EMC2_NCFILES_DIR)
AC_SUBST(EMC2_LANG_DIR)
AC_SUBST(EMC2_IMAGE_DIR)
AC_SUBST(REALTIME)
AC_SUBST(SIMULATOR)

AC_CONFIG_FILES(Makefile mini/emc.tcl mini/mini.tcl rtstepper.list)
AC_OUTPUT
