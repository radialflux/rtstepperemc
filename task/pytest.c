/*****************************************************************************\

  pytest.c - tinypy test for rtstepperemc library

  (c) 2013 Copyright Eckler Software

  Author: David Suffield, dsuffiel@ecklersoft.com

  This program is free software; you can redistribute it and/or modify
  it under the terms of version 2 of the GNU General Public License as published by
  the Free Software Foundation.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA

  Upstream patches are welcome. Any patches submitted to the author must be 
  unencumbered (ie: no Copyright or License). Patches that are accepted will 
  be applied to the GPL version, but the author reserves the rights to 
  Copyright and License.  

\*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>
#include <ctype.h>
#include <unistd.h>
#include <errno.h>
#include <sys/time.h>
#if (defined(__WIN32__) || defined(_WINDOWS))
#include <usb-winusb.h>
#else
#include <dlfcn.h>
#include <usb.h>
#endif

#if (defined(__WIN32__) || defined(_WINDOWS))
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#define sleep(n) Sleep(1000 * n)
#endif

#include "emc.h"

#define _STRINGIZE(x) #x
#define STRINGIZE(x) _STRINGIZE(x)

#define BUG(args...) fprintf(stderr, __FILE__ " " STRINGIZE(__LINE__) ": " args)

typedef void * (*emc_ui_open_t)(const char *ini_file);
typedef enum EMC_RESULT (*emc_ui_close_t)(void *hd);
typedef enum EMC_RESULT (*emc_ui_mdi_cmd_t)(void *hd, const char *mdi);
typedef enum EMC_RESULT (*emc_ui_get_operator_message_t)(void *hd, char *buf, int buf_size);
typedef enum EMC_RESULT (*emc_ui_estop_t)(void *hd);
typedef enum EMC_RESULT (*emc_ui_estop_reset_t)(void *hd);
typedef enum EMC_RESULT (*emc_ui_machine_on_t)(void *hd);
typedef enum EMC_RESULT (*emc_ui_machine_off_t)(void *hd);
typedef enum EMC_RESULT (*emc_ui_home_t)(void *hd, int axis);
typedef enum EMC_RESULT (*emc_ui_get_ini_key_value_t)(void *hd, const char *section, const char *key, const char *value, int value_size);
typedef enum EMC_RESULT (*emc_ui_mdi_mode_t)(void *hd);
typedef enum EMC_RESULT (*emc_ui_get_version_t)(const char **ver);
typedef enum EMC_RESULT (*emc_ui_auto_mode_t)(void *hd);
typedef enum EMC_RESULT (*emc_ui_program_open_t)(void *hd, const char *program);
typedef enum EMC_RESULT (*emc_ui_program_run_t)(void *hd, int line);
typedef enum EMC_RESULT (*emc_ui_dout_t)(void *hd, int output_num, int value, int sync);
typedef enum EMC_RESULT (*emc_ui_wait_command_done_t)(void *hd, double timeout);

static emc_ui_open_t _emc_ui_open;
static emc_ui_close_t _emc_ui_close;
static emc_ui_mdi_cmd_t _emc_ui_mdi_cmd;
static emc_ui_get_operator_message_t _emc_ui_get_operator_message;
static emc_ui_estop_t _emc_ui_estop;
static emc_ui_estop_reset_t _emc_ui_estop_reset;
static emc_ui_machine_on_t _emc_ui_machine_on;
static emc_ui_machine_off_t _emc_ui_machine_off;
static emc_ui_home_t _emc_ui_home;
static emc_ui_get_ini_key_value_t _emc_ui_get_ini_key_value;
static emc_ui_mdi_mode_t _emc_ui_mdi_mode;
static emc_ui_get_version_t _emc_ui_get_version;
static emc_ui_auto_mode_t _emc_ui_auto_mode;
static emc_ui_program_open_t _emc_ui_program_open;
static emc_ui_program_run_t _emc_ui_program_run;
static emc_ui_dout_t _emc_ui_dout;
static emc_ui_wait_command_done_t _emc_ui_wait_command_done;

static int verbose;

static void *_dlopen(const char *file)
{
#if (defined(__WIN32__) || defined(_WINDOWS))
   return (void *)LoadLibrary(file);
#else
   return dlopen(file, RTLD_LAZY);
#endif
}

static void *_dlsym(void *handle, const char *symbol)
{
#if (defined(__WIN32__) || defined(_WINDOWS))
   return (void *)GetProcAddress((HINSTANCE)handle, symbol);
#else
   return dlsym(handle, symbol);
#endif
}

static int _dlclose(void *handle)
{
#if (defined(__WIN32__) || defined(_WINDOWS))
    return FreeLibrary((HINSTANCE)handle);
#else
    return dlclose(handle);
#endif
}

static void usage()
{
   fprintf(stdout, "pytest %s, tinypy rtstepperemc library test\n", PACKAGE_VERSION);
   fprintf(stdout, "(c) 2013 Copyright Eckler Software\n");
   fprintf(stdout, "David Suffield, dsuffiel@ecklersoft.com\n");
   fprintf(stdout, "usage: pytest [-m mdi_command] [-i ini_file] [-g gcode_file]\n");
}

int main(int argc, char *argv[])
{
   void *hd, *ps;
   int i, ret=1, num_axis;
   char mdi[LINELEN];
   char ini[LINELEN];
   char gfile[LINELEN];
   char buf[LINELEN];
   const char *ver;

   mdi[0] = ini[0] = gfile[0] = 0;

   while ((i = getopt(argc, argv, "vhm:i:g:")) != -1)
   {
      switch (i)
      {
      case 'v':
         verbose++;
         break;
      case 'm':
         strncpy(mdi, optarg, sizeof(mdi));
         break;
      case 'g':
         strncpy(gfile, optarg, sizeof(gfile));
         break;
      case 'i':
         strncpy(ini, optarg, sizeof(ini));
         break;
      case 'h':
         usage();
         exit(0);
      case '?':
         usage();
         fprintf(stderr, "unknown argument: %s\n", argv[1]);
         exit(-1);
      default:
         break;
      }
   }

   if ((hd = _dlopen("./rtstepperemc." DLL_EXTENSION)) == NULL)
   {
      fprintf(stderr, "unable to open %s\n", "./rtstepperemc." DLL_EXTENSION);
      goto bugout;
   }

   if ((_emc_ui_open = _dlsym(hd, "emc_ui_open")) == NULL)
      goto bugout;
   if ((_emc_ui_close = _dlsym(hd, "emc_ui_close")) == NULL)
      goto bugout;
   if ((_emc_ui_mdi_cmd = _dlsym(hd, "emc_ui_mdi_cmd")) == NULL)
      goto bugout;
   if ((_emc_ui_get_operator_message = _dlsym(hd, "emc_ui_get_operator_message")) == NULL)
      goto bugout;
   if ((_emc_ui_estop = _dlsym(hd, "emc_ui_estop")) == NULL)
      goto bugout;
   if ((_emc_ui_estop_reset = _dlsym(hd, "emc_ui_estop_reset")) == NULL)
      goto bugout;
   if ((_emc_ui_machine_on = _dlsym(hd, "emc_ui_machine_on")) == NULL)
      goto bugout;
   if ((_emc_ui_machine_off = _dlsym(hd, "emc_ui_machine_off")) == NULL)
      goto bugout;
   if ((_emc_ui_home = _dlsym(hd, "emc_ui_home")) == NULL)
      goto bugout;
   if ((_emc_ui_get_ini_key_value = _dlsym(hd, "emc_ui_get_ini_key_value")) == NULL)
      goto bugout;
   if ((_emc_ui_mdi_mode = _dlsym(hd, "emc_ui_mdi_mode")) == NULL)
      goto bugout;
   if ((_emc_ui_get_version = _dlsym(hd, "emc_ui_get_version")) == NULL)
      goto bugout;
   if ((_emc_ui_auto_mode = _dlsym(hd, "emc_ui_auto_mode")) == NULL)
      goto bugout;
   if ((_emc_ui_program_open = _dlsym(hd, "emc_ui_program_open")) == NULL)
      goto bugout;
   if ((_emc_ui_program_run = _dlsym(hd, "emc_ui_program_run")) == NULL)
      goto bugout;
   if ((_emc_ui_dout = _dlsym(hd, "emc_ui_dout")) == NULL)
      goto bugout;
   if ((_emc_ui_wait_command_done = _dlsym(hd, "emc_ui_wait_command_done")) == NULL)
      goto bugout;

   if ((ps = _emc_ui_open(ini)) == NULL)
   {
      fprintf(stdout, "error opening rtstepperemc library ini=%s\n", ini);
      goto bugout;
   }

   if (verbose)
   {
      _emc_ui_get_version(&ver);
      fprintf(stdout, "rtstepperemc library version=%s\n", ver);
   }

   if (mdi[0])
   {
      _emc_ui_get_ini_key_value(ps, "TRAJ", "AXES", buf, sizeof(buf));
      num_axis = strtod(buf, NULL);

      /* Estop green light. */
      _emc_ui_estop_reset(ps);
      _emc_ui_machine_on(ps);

      /* Zero all axis. */
      for (i=0; i<num_axis; i++)
         _emc_ui_home(ps, i);

      /* Set mdi mode. */
      _emc_ui_mdi_mode(ps);

      /* Execute mdi command. */
      _emc_ui_mdi_cmd(ps, mdi);
      _emc_ui_wait_command_done(ps, 0.0);

      /* Check for errors. */
      _emc_ui_get_operator_message(ps, buf, sizeof(buf));
      while (buf[0])
      {
         fprintf(stderr,"%s\n", buf);
         _emc_ui_get_operator_message(ps, buf, sizeof(buf));
      }

      /* Estop red light. */
      _emc_ui_estop(ps);
      _emc_ui_machine_off(ps);
   }

   if (gfile[0])
   {
      _emc_ui_get_ini_key_value(ps, "TRAJ", "AXES", buf, sizeof(buf));
      num_axis = strtod(buf, NULL);

      /* Estop green light. */
      _emc_ui_estop_reset(ps);
      _emc_ui_machine_on(ps);

      /* Zero all axis. */
      for (i=0; i<num_axis; i++)
         _emc_ui_home(ps, i);

      /* Set auto mode. */
      _emc_ui_auto_mode(ps);

      /* Execute gcode file. */
      _emc_ui_program_open(ps, gfile);
      _emc_ui_program_run(ps, 0);
      _emc_ui_wait_command_done(ps, 0.0);

      /* Check for errors. */
      _emc_ui_get_operator_message(ps, buf, sizeof(buf));
      while (buf[0])
      {
         fprintf(stderr,"%s\n", buf);
         _emc_ui_get_operator_message(ps, buf, sizeof(buf));
      }

      /* Estop red light. */
      _emc_ui_estop(ps);
      _emc_ui_machine_off(ps);
   }

   _emc_ui_close(ps);

   ret=0;

bugout:
   if (ret)
      fprintf(stderr, "pytest error: exiting\n");
   if (hd)
      _dlclose(hd);
   return ret;
}
