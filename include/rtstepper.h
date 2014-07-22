/************************************************************************************\

  rtstepper.h - rt-stepper dongle support for EMC2

  (c) 2011 Copyright Eckler Software

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
  unencumbered (ie: no Copyright or License).

  See project revision history the "configure.ac" file.

\************************************************************************************/

#ifndef _RTSTEPPER_H
#define _RTSTEPPER_H

#include <stdint.h>
#include <pthread.h>

#if (defined(__WIN32__) || defined(_WINDOWS))
#include <usb-winusb.h>
#else
#include <usb.h>
#endif

#include "motion.h"

enum RTSTEPPER_RESULT
{
   RTSTEPPER_R_REQ_ERROR = -3,
   RTSTEPPER_R_MALLOC_ERROR = -2,
   RTSTEPPER_R_IO_ERROR = -1,
   RTSTEPPER_R_OK = 0,
   RTSTEPPER_R_INPUT_TRUE = 1,
   RTSTEPPER_R_INPUT_FALSE = 2,
};

enum RTSTEPPER_GPO_BIT
{
   RTSTEPPER_OUTPUT0,
   RTSTEPPER_OUTPUT1,
   RTSTEPPER_OUTPUT2,
   RTSTEPPER_OUTPUT3,
   RTSTEPPER_OUTPUT4,
   RTSTEPPER_OUTPUT5,
   RTSTEPPER_OUTPUT6,
   RTSTEPPER_OUTPUT7,
   RTSTEPPER_GPO_MAX,
};

typedef int (*rtstepper_io_error_cb)(int result);

struct rtstepper_app_session
{
   uint16_t state_bits;         /* dongle state bits */
   uint16_t old_state_bits;
   int input0_abort_enabled;    /* 0=false, 1=true */
   int input1_abort_enabled;    /* 0=false, 1=true */
   int input2_abort_enabled;    /* 0=false, 1=true */
   char snum[LINELEN];          /* dongle serial number */
   int step_pin[EMCMOT_MAX_AXIS];       /* DB25 pin number */
   int direction_pin[EMCMOT_MAX_AXIS];  /* DB25 pin number */
   int step_active_high[EMCMOT_MAX_AXIS];       /* DB25 pin polarity */
   int direction_active_high[EMCMOT_MAX_AXIS];  /* DB25 pin polarity */
   int id;                      /* gcode line number */
   int master_index[EMCMOT_MAX_AXIS];   /* running position in step counts */
   int clk_tail[EMCMOT_MAX_AXIS];       /* used calculate number cycles between pulses */
   int direction[EMCMOT_MAX_AXIS];      /* cycle time step direction */
   double steps_per_unit[EMCMOT_MAX_AXIS];      /* INPUT_SCALE */
   int gpo_pin[RTSTEPPER_GPO_MAX];            /* DB25 pin number */
   int gpo[RTSTEPPER_GPO_MAX];            /* general purpose output */
   unsigned char *buf;          /* staging step/direction buffer */
   int buf_size;                /* staging buffer size */
   int total;                   /* staging current buffer count */
   unsigned char *xfr_buf;     /* step/direction buffer */
   int xfr_total;           /* buffer count */
   int xfr_active;              /* 0=no, 1=yes */
   struct usb_device *libusb_device;    /* selected usb device */
   int usbfd;                   /* usb file descriptor */
   pthread_t bulk_write_tid;    /* thread handle */
   pthread_mutex_t mutex;
   pthread_cond_t write_done_cond;
   rtstepper_io_error_cb error_function;   /* client callback function */
};

#define RTSTEPPER_STEP_STATE_ABORT_BIT 0x01     /* active high, 1=yes, 0=no */
#define RTSTEPPER_STEP_STATE_EMPTY_BIT 0x02     /* active high, 1=yes, 0=no */
#define RTSTEPPER_STEP_STATE_STALL_BIT 0x04     /* active high, 1=yes, 0=no */
#define RTSTEPPER_STEP_STATE_INPUT0_BIT 0x08    /* active high, 1=yes, 0=no */
#define RTSTEPPER_STEP_STATE_INPUT1_BIT 0x10    /* active high, 1=yes, 0=no */
#define RTSTEPPER_STEP_STATE_INPUT2_BIT 0x20    /* active high, 1=yes, 0=no */

#define RTSTEPPER_LOG_BACKUP 0x01       /* create logfile backup */

#ifdef __cplusplus
extern "C"
{
#endif

/* Function prototypes */

   enum RTSTEPPER_RESULT rtstepper_init(struct rtstepper_app_session *ps, rtstepper_io_error_cb error_function);
   enum RTSTEPPER_RESULT rtstepper_exit(struct rtstepper_app_session *ps);
   enum RTSTEPPER_RESULT rtstepper_query_state(struct rtstepper_app_session *ps);
   enum RTSTEPPER_RESULT rtstepper_clear_abort(struct rtstepper_app_session *ps);
   enum RTSTEPPER_RESULT rtstepper_set_abort(struct rtstepper_app_session *ps);
   enum RTSTEPPER_RESULT rtstepper_set_abort_wait(struct rtstepper_app_session *ps);
   enum RTSTEPPER_RESULT rtstepper_encode(struct rtstepper_app_session *ps, int id, double *index, int num_axis);
   enum RTSTEPPER_RESULT rtstepper_start_xfr(struct rtstepper_app_session *ps, int id, int num_axis);
//   enum RTSTEPPER_RESULT rtstepper_clear_xfr_result(struct rtstepper_app_session *ps);
//    int rtstepper_is_xfr_done(struct rtstepper_app_session *ps, int *result);
   int rtstepper_is_connected(struct rtstepper_app_session *ps);
   enum RTSTEPPER_RESULT rtstepper_is_input0_triggered(struct rtstepper_app_session *ps);
   enum RTSTEPPER_RESULT rtstepper_is_input1_triggered(struct rtstepper_app_session *ps);
   enum RTSTEPPER_RESULT rtstepper_is_input2_triggered(struct rtstepper_app_session *ps);
   enum RTSTEPPER_RESULT rtstepper_input0_state(struct rtstepper_app_session *ps);
   enum RTSTEPPER_RESULT rtstepper_input1_state(struct rtstepper_app_session *ps);
   enum RTSTEPPER_RESULT rtstepper_input2_state(struct rtstepper_app_session *ps);
   enum RTSTEPPER_RESULT rtstepper_home(struct rtstepper_app_session *ps, int axis);
   enum RTSTEPPER_RESULT rtstepper_set_gpo(struct rtstepper_app_session *ps, enum RTSTEPPER_GPO_BIT num, int value);
   void rtstepper_close_log(void);
   void rtstepper_open_log(const char *ident, int logopt);
   void rtstepper_syslog(const char *fmt, ...);
#ifdef __cplusplus
}
#endif

#endif                          /* _RTSTEPPER_H */
