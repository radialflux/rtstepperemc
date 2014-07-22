/************************************************************************************\

  rtstepper_log.c - Unix syslog type compatibility routines
 
  (c) 2008-2011 Copyright Eckler Software

  Author: David Suffield, dsuffiel@ecklersoft.com

  This library is free software; you can redistribute it and/or
  modify it under the terms of version 2.1 of the GNU Lesser General Public
  License as published by the Free Software Foundation.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

  Upstream patches are welcome. Any patches submitted to the author must be 
  unencumbered (ie: no Copyright or License). Patches that are accepted will 
  be applied to the GPL version, but the author reserves the rights to 
  Copyright and License.  

\************************************************************************************/

#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/time.h>
#include "rtstepper.h"

static FILE *logfd;
static pthread_mutex_t syslog_mutex = PTHREAD_MUTEX_INITIALIZER;
static struct timeval start_time;

static char *month[12] = { "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" };

void rtstepper_close_log(void)
{
   if (logfd)
   {
      fclose(logfd);
      logfd = NULL;
   }
   pthread_mutex_destroy(&syslog_mutex);
}       /* rtstepper_close_log */

void rtstepper_open_log(const char *ident, int logopt)
{
   struct tm *pt;
   time_t t;
   char log_file[512], backup[512];
   struct stat sb;

   if (logfd)
      return;

   sprintf(log_file, "%s.log", ident);

   if (logopt == RTSTEPPER_LOG_BACKUP && stat(log_file, &sb) == 0)
   {
      /* Save any old log file. */
      sprintf(backup, "%s.bak", ident);
      remove(backup);
      rename(log_file, backup);
   }

   logfd = fopen(log_file, "a");

   gettimeofday(&start_time, NULL);
   t = time(NULL);
   pt = localtime(&t);
   rtstepper_syslog("started %s %s %d %d:%d:%d\n", log_file, month[pt->tm_mon], pt->tm_mday, pt->tm_hour, pt->tm_min, pt->tm_sec);
}       /* rtstepper_open_log */

void rtstepper_syslog(const char *fmt, ...)
{
   struct timeval now;
   va_list args;
   char tmp[512];
   int n;

   if (!logfd)
      return;

   pthread_mutex_lock(&syslog_mutex);

   va_start(args, fmt);

   if ((n = vsnprintf(tmp, sizeof(tmp), fmt, args)) == -1)
      tmp[sizeof(tmp) - 1] = 0; /* output was truncated */

   gettimeofday(&now, NULL);
   fprintf(logfd, "%lu.%.3lus %s", now.tv_sec & 0xfff, (unsigned long int)now.tv_usec % 1000000, tmp);
   fflush(logfd);

   va_end(args);

   pthread_mutex_unlock(&syslog_mutex);
}       /* rtstepper_syslog */
