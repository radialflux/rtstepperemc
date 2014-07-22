/*****************************************************************************\

  rtstepper.c - rt-stepper dongle support for EMC2

  (c) 2008-2011 Copyright Eckler Software

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

\*****************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <ctype.h>
#include <errno.h>
#include <pthread.h>
#include <math.h>
#include "rtstepper.h"
#include "bug.h"

#define LIBUSB_TIMEOUT 30000    /* milliseconds */
#define LIBUSB_CONTROL_REQ_TIMEOUT 5000

/* EP0 Vendor Setup commands (bRequest). */
enum STEP_CMD
{
   STEP_SET,                    /* set step elements, clear state bits */
   STEP_QUERY,                  /* query current step and state info */
   STEP_ABORT_SET,              /* set un-synchronized stop */
   STEP_ABORT_CLEAR,            /* clear un-synchronized stop */
};

//#define STEP_BUF_CHUNK 4096
#define STEP_BUF_CHUNK 16384

struct __attribute__ ((packed)) step_state
{
   union
   {
      uint16_t _word;
   };
};

struct __attribute__ ((packed)) step_elements
{
   char reserved[8];
};

struct __attribute__ ((packed)) step_query
{
   struct step_state state_bits;
   unsigned char reserved;
   unsigned char trip_cnt;      /* stall trip count */
   uint32_t step;               /* running step count */
};

enum FD_ID
{
   FD_NA = 0,
   FD_ff_0_1,
   FD_MAX
};

extern const char *USER_HOME_DIR;

/*
 * The folloing fd arrays must match "enum FD_ID" definition.
 */

static const char *fd_name[FD_MAX] = {
   "na",
   "ff/0/1",
};

static int fd_class[FD_MAX] = {
   0, 0xff,
};

static int fd_subclass[FD_MAX] = {
   0, 0x0,
};

static int fd_protocol[FD_MAX] = {
   0, 0x1,
};

/* USB file descriptor, one for each USB protocol. */
struct file_descriptor
{
   usb_dev_handle *hd;
   enum FD_ID fd;
   int config;
   int interface;
   int alt_setting;
};

static struct file_descriptor fd_table[FD_MAX]; /* usb file descriptors */

static FILE *dump;
static int verbose;

/* 
 * DB25 pin to bitfield map.
 *                pin #       na, 1,  2,   3,   4,   5,    6,    7,   8,    9  
 */
static const int pin_map[] = { 0, 0, 0x1, 0x2, 0x4, 0x8, 0x10, 0x20, 0x40, 0x80 };

static int is_rt(struct usb_device *dev, const char *sn)
{
   usb_dev_handle *hd = NULL;
   char sz[128];
   int r, stat = 0;

   if ((hd = usb_open(dev)) == NULL)
   {
      BUG("invalid usb_open\n");
      goto bugout;
   }

   if (dev->descriptor.idVendor != 0x4d8)
      goto bugout;

   if ((r = usb_get_string_simple(hd, dev->descriptor.iProduct, sz, sizeof(sz))) < 0)
   {
      BUG("invalid product id string ret=%d\n", r);
      goto bugout;
   }

   if (strcasecmp(sz, "rt-stepper 1a") != 0)
      goto bugout;

   if (sn[0])
   {
      if ((r = usb_get_string_simple(hd, dev->descriptor.iSerialNumber, sz, sizeof(sz))) < 0)
         goto bugout;
      if (!sz[0])
         strcpy(sz, "0");
      if (strcmp(sz, sn) != 0)
         goto bugout;
   }

   stat = 1;    /* found usb device */

 bugout:
   if (hd != NULL)
      usb_close(hd);

   return stat;
}       /* is_rt() */

/* Get interface descriptor for specified xx/xx/xx protocol. */
static int get_interface(struct usb_device *dev, enum FD_ID index, struct file_descriptor *pfd)
{
   struct usb_interface_descriptor *pi;
   int i, j, k;

   for (i = 0; i < dev->descriptor.bNumConfigurations; i++)
   {
      if (dev->config == NULL)
         goto bugout;

      for (j = 0; j < dev->config[i].bNumInterfaces; j++)
      {
         if (dev->config[i].interface == NULL)
            goto bugout;

         for (k = 0; k < dev->config[i].interface[j].num_altsetting; k++)
         {
            if (dev->config[i].interface[j].altsetting == NULL)
               goto bugout;

            pi = &dev->config[i].interface[j].altsetting[k];
            if (pi->bInterfaceClass == fd_class[index] && pi->bInterfaceSubClass == fd_subclass[index] && pi->bInterfaceProtocol == fd_protocol[index])
            {
               pfd->config = i; /* found interface */
               pfd->interface = j;
               pfd->alt_setting = k;
               pfd->fd = index;
               return 0;
            }
         }
      }
   }

 bugout:
   return 1;    /* no interface found */
}       /* get_interface() */

static int claim_interface(struct usb_device *dev, struct file_descriptor *pfd)
{
   int stat = 1;

   if (pfd->hd != NULL)
      return 0; /* interface is already claimed */

   if ((pfd->hd = usb_open(dev)) == NULL)
   {
      BUG("invalid usb_open\n");
      goto bugout;
   }

   /*
    * Linux, OSX and winusb.sys will set configuration, but I have seen Ubuntu 10.04 fail,
    * so set configuration to make sure.
    */
   if (usb_set_configuration(pfd->hd, dev->config[pfd->config].bConfigurationValue))
   {
      BUG("unable to set configuration %d\n", pfd->config);
//      goto bugout;   ignor any error...
   }

   if (usb_claim_interface(pfd->hd, pfd->interface))
   {
      usb_close(pfd->hd);
      pfd->hd = NULL;
      BUG("invalid claim_interface %s\n", fd_name[pfd->fd]);
      goto bugout;
   }

   if (usb_set_altinterface(pfd->hd, pfd->alt_setting))
   {
      usb_release_interface(pfd->hd, pfd->interface);
      usb_close(pfd->hd);
      pfd->hd = NULL;
      BUG("invalid set_altinterface %s altset=%d\n", fd_name[pfd->fd], pfd->alt_setting);
      goto bugout;
   }

   DBG("claimed %s interface\n", fd_name[pfd->fd]);

   stat = 0;

 bugout:
   return stat;
}       /* claim_interface() */

static int release_interface(struct file_descriptor *pfd)
{
   if (pfd->hd == NULL)
      return 0;

   usb_release_interface(pfd->hd, pfd->interface);
   usb_close(pfd->hd);
   pfd->hd = NULL;

   DBG("released %s interface\n", fd_name[pfd->fd]);

   return 0;
}       /* release_interface() */

static struct usb_device *get_libusb_device(const char *sn)
{
   struct usb_bus *bus;
   struct usb_device *dev;

   for (bus = usb_get_busses(); bus; bus = bus->next)
   {
      for (dev = bus->devices; dev; dev = dev->next)
      {
         if (is_rt(dev, sn))
            return dev; /* found usb rt device */
      }
   }
   return NULL;
}       /* get_libusb_device() */

static int raw_write(struct rtstepper_app_session *ps, const void *buf, int size)
{
   struct file_descriptor *pfd;
   int len = -EIO, tmo;

   if (!ps->usbfd)
      goto bugout;

   pfd = &fd_table[ps->usbfd];

   tmo = (int) ((double) size * 0.021333);      /* timeout in ms = steps * period * 1000 */
   tmo += 5000; /* plus 5 seconds */

   len = usb_bulk_write(pfd->hd, 1, (char *) buf, size, tmo);

   if (len < 0)
   {
      BUG("raw_write failed buf=%p size=%d len=%d\n", buf, size, len);
      goto bugout;
   }

   if (len != size)
   {
      BUG("raw_write failed to complete buf=%p size=%d len=%d\n", buf, size, len);
      len = -EIO;
      goto bugout;
   }

   DBG("write len=%d size=%d msec=%d\n", len, size, tmo);

 bugout:
   return len;
}       /* raw_write() */

void bulk_write_thread(struct rtstepper_app_session *ps)
{
   int i, result, cnt = 0;
   char s[64];
   static int derr = 0;

   pthread_detach(pthread_self());

   if (verbose)
   {
      if (!derr && dump == NULL)
      {
         sprintf(s, "%s/.%s/raw.dat", USER_HOME_DIR, PACKAGE_NAME);
         if ((dump = fopen(s, "w+")) == NULL)   /* truncate any old file */
         {
            BUG("unable to create %s\n", s);
            derr = 1;
         }
         fprintf(dump, "byte: gcode_line_num: cnt:\n");
      }
      if (!derr)
      {
         /* Dump byte_value, ring buffer index, gcode line number. */
         for (i = 0; i < ps->xfr_total; i++)
            fprintf(dump, "%x %d %d\n", ps->xfr_buf[i], ps->id, ++cnt);
      }
   }

   result = raw_write(ps, ps->xfr_buf, ps->xfr_total);
   
   DBG("bulk_write_thread() done ret=%d\n", result);

   if (verbose && dump)
      fflush(dump);

   free(ps->xfr_buf);
   ps->xfr_buf = NULL;
   ps->xfr_total = 0;

   pthread_mutex_lock(&ps->mutex);
   ps->xfr_active = 0;
   pthread_cond_broadcast(&ps->write_done_cond);
   pthread_mutex_unlock(&ps->mutex);

   if (result < 0 && ps->error_function != NULL)
      ps->error_function(result);  /* call client error handler */

   return;
}       /* bulk_write_thread() */

enum RTSTEPPER_RESULT rtstepper_start_xfr(struct rtstepper_app_session *ps, int id, int num_axis)
{
   enum RTSTEPPER_RESULT stat = RTSTEPPER_R_IO_ERROR;
   int i, axis, mid;

   if (ps->total == 0)
      return RTSTEPPER_R_OK;

   if (ps->xfr_active)
   {
      BUG("unable start bulk_write_thread, already active\n");
      goto bugout;      /* bail */
   }

   DBG("start_xfr: x_index=%d y_index=%d z_index=%d a_index=%d c_index=%d buf=%p cnt=%d\n", ps->master_index[0],
       ps->master_index[1], ps->master_index[2], ps->master_index[3], ps->master_index[5], ps->buf, ps->total);

   /* Finish last pulse for this step buffer. */
   for (axis = 0; axis < num_axis; axis++)
   {
      if (ps->step_pin[axis] == 0)
         continue;   /* skip */

      if (ps->clk_tail[axis])
      {
         /* Stretch pulse to 50% duty cycle. */
         mid = (ps->total - ps->clk_tail[axis]) / 2;
         for (i = 0; i < mid; i++)
         {
            if (ps->step_active_high[axis])
               ps->buf[ps->clk_tail[axis] + i] |= pin_map[ps->step_pin[axis]]; /* set bit */
            else
               ps->buf[ps->clk_tail[axis] + i] &= ~pin_map[ps->step_pin[axis]]; /* set bit */
         }
      }
   }

   ps->id = id;
   ps->xfr_buf = ps->buf;
   ps->xfr_total = ps->total;
   ps->buf = NULL;
   ps->buf_size = 0;
   ps->total = 0;
   for (i = 0; i < EMCMOT_MAX_AXIS; i++)
   {
      ps->clk_tail[i] = 0;
      ps->direction[i] = 0;
   }

   ps->xfr_active = 1;
   if (pthread_create(&ps->bulk_write_tid, NULL, (void *(*)(void *)) bulk_write_thread, (void *) ps) != 0)
   {
      BUG("unable to creat bulk_write_thread\n");
      goto bugout;      /* bail */
   }

   stat = RTSTEPPER_R_OK;

 bugout:
   return stat;
}       /* rtstepper_start_xfr() */

/*
 * Given a command position in counts for each axis, encode each value into a single step/direction byte. 
 * Store the byte in buffer that is big enough to hold a complete stepper motor move.
 */
enum RTSTEPPER_RESULT rtstepper_encode(struct rtstepper_app_session *ps, int id, double index[], int num_axis)
{
   int i, axis, step, mid, new_size, stat = RTSTEPPER_R_MALLOC_ERROR;
   unsigned char *tmp;

   if (ps->buf == NULL || (ps->buf_size - ps->total) < 2)
   {
      new_size = (ps->buf_size < STEP_BUF_CHUNK) ? STEP_BUF_CHUNK : ps->buf_size * 2;
      if ((tmp = (unsigned char *) realloc(ps->buf, new_size)) == NULL)
      {
         free(ps->buf);
         ps->buf = NULL;
         BUG("unable to malloc step buffer size=%d\n", new_size);
         goto bugout;   /* bail */
      }
      ps->buf = tmp;
      ps->buf_size = new_size;
   }

   for (axis = 0; axis < num_axis; axis++)
   {
      /* Check DB25 pin assignments for this axis, if no pins are assigned skip this axis. Useful for XYZABC axes where AB are unused. */
      if (ps->step_pin[axis] == 0 || ps->direction_pin[axis] == 0)
         continue;   /* skip */

      /* Set step bit to default state, high if low_true logic or low if high_true logic. */
      if (ps->step_active_high[axis])
      {
         ps->buf[ps->total] &= ~pin_map[ps->step_pin[axis]];
         ps->buf[ps->total + 1] &= ~pin_map[ps->step_pin[axis]];
      }
      else
      {
         ps->buf[ps->total] |= pin_map[ps->step_pin[axis]];
         ps->buf[ps->total + 1] |= pin_map[ps->step_pin[axis]];
      }

      /* Set direction bit to default state, high if low_true logic or low if high_true logic. */
      if (ps->direction_active_high[axis])
      {
         ps->buf[ps->total] &= ~pin_map[ps->direction_pin[axis]];
         ps->buf[ps->total + 1] &= ~pin_map[ps->direction_pin[axis]];
      }
      else
      {
         ps->buf[ps->total] |= pin_map[ps->direction_pin[axis]];
         ps->buf[ps->total + 1] |= pin_map[ps->direction_pin[axis]];
      }

      /* Calculate the step pulse for this clock cycle */
      step = round(index[axis] * ps->steps_per_unit[axis]) - ps->master_index[axis];

      if (step < -1 || step > 1)
      {
         BUG("invalid step value: id=%d axis=%d cmd_pos=%0.8f master_index=%d input_scale=%0.2f step=%d\n",
             id, axis, index[axis], ps->master_index[axis], ps->steps_per_unit[axis], step);
         step = 0;
      }

      if (step)
      {
         /* Got a valid step pulse this cycle. */
         if (ps->clk_tail[axis])
         {
            /* Using the second pulse, stretch pulse to 50% duty cycle. */
            mid = (ps->total - ps->clk_tail[axis]) / 2;
            for (i = 0; i < mid; i++)
            {
               if (ps->step_active_high[axis])
                  ps->buf[ps->clk_tail[axis] + i] |= pin_map[ps->step_pin[axis]]; /* set bit */
               else
                  ps->buf[ps->clk_tail[axis] + i] &= ~pin_map[ps->step_pin[axis]]; /* set bit */
            }
         }

         /* save old step location */
         ps->clk_tail[axis] = ps->total;

         /* save step direction */
         ps->direction[axis] = step;
      }

      /* Set direction bit. */
      if (ps->direction[axis] < 0)
      {
         if (ps->direction_active_high[axis])
         {
            ps->buf[ps->total] |= pin_map[ps->direction_pin[axis]];    /* set bit */
            ps->buf[ps->total + 1] |= pin_map[ps->direction_pin[axis]];  /* set bit */
         }
         else
         {
            ps->buf[ps->total] &= ~pin_map[ps->direction_pin[axis]];       /* set bit */
            ps->buf[ps->total + 1] &= ~pin_map[ps->direction_pin[axis]];   /* set bit */
         }
      }

      ps->master_index[axis] += step;

//      DBG("axis=%d index=%0.6f master_index=%d\n", axis, index[axis] * ps->steps_per_unit[axis], ps->master_index[axis]);
   }    /* for (axis=0; axis < num_axis; axis++) */

   for (i=0; i<RTSTEPPER_GPO_MAX; i++)
   {
      if (ps->gpo_pin[i] == 0)
         continue;   /* skip */

      if (ps->gpo[i])
      {
         ps->buf[ps->total] |= pin_map[ps->gpo_pin[i]];    /* set bit */
         ps->buf[ps->total + 1] |= pin_map[ps->gpo_pin[i]];  /* set bit */
      }
      else
      {
         ps->buf[ps->total] &= ~pin_map[ps->gpo_pin[i]];       /* clear bit */
         ps->buf[ps->total + 1] &= ~pin_map[ps->gpo_pin[i]];   /* clear bit */
      }
   }

   ps->total += 2;

   stat = RTSTEPPER_R_OK;

 bugout:
   return stat;
}       /* rtstepper_encode() */

int rtstepper_is_connected(struct rtstepper_app_session *ps)
{
   return ps->usbfd;
}       /* rtstepper_is_connected() */

enum RTSTEPPER_RESULT rtstepper_home(struct rtstepper_app_session *ps, int axis)
{
   enum RTSTEPPER_RESULT stat;

   DBG("rtstepper_home() axis=%d\n", axis);

   if (axis < 0 || axis > EMCMOT_MAX_AXIS - 1)
   {
      BUG("invalid home axis=%d\n", axis);
      stat = RTSTEPPER_R_REQ_ERROR;
      goto bugout;
   }

   ps->master_index[axis] = 0.0;
   stat = RTSTEPPER_R_OK;
 bugout:
   return stat;
}       /* rtstepper_home() */

/* Set abort state in dongle. */
enum RTSTEPPER_RESULT rtstepper_set_abort(struct rtstepper_app_session *ps)
{
   enum RTSTEPPER_RESULT stat;
   int len;

   if (!ps->usbfd)
   {
      stat = RTSTEPPER_R_REQ_ERROR;
      goto bugout;
   }

   len = usb_control_msg(fd_table[ps->usbfd].hd, USB_ENDPOINT_OUT | USB_TYPE_VENDOR | USB_RECIP_INTERFACE,      /* bmRequestType */
                         STEP_ABORT_SET,        /* bRequest */
                         0x0,   /* wValue */
                         fd_table[ps->usbfd].interface, /* wIndex */
                         NULL, 0, LIBUSB_CONTROL_REQ_TIMEOUT);
   if (len < 0)
   {
      BUG("set_abort failed ret=%d\n", len);
      stat = RTSTEPPER_R_IO_ERROR;
      goto bugout;
   }
   stat = RTSTEPPER_R_OK;

 bugout:
   return stat;
}       /* rtstepper_set_abort() */

/* Clear abort state in dongle. */
enum RTSTEPPER_RESULT rtstepper_clear_abort(struct rtstepper_app_session *ps)
{
   enum RTSTEPPER_RESULT stat;
   int len;

   if (!ps->usbfd)
   {
      stat = RTSTEPPER_R_REQ_ERROR;
      goto bugout;
   }

   len = usb_control_msg(fd_table[ps->usbfd].hd, USB_ENDPOINT_OUT | USB_TYPE_VENDOR | USB_RECIP_INTERFACE,      /* bmRequestType */
                         STEP_ABORT_CLEAR,      /* bRequest */
                         0x0,   /* wValue */
                         fd_table[ps->usbfd].interface, /* wIndex */
                         NULL, 0, LIBUSB_CONTROL_REQ_TIMEOUT);
   if (len < 0)
   {
      BUG("clear_abort failed ret=%d\n", len);
      stat = RTSTEPPER_R_IO_ERROR;
      goto bugout;
   }

   /* Reset step_state_inputx_bit for rtstepper_is_inputx_triggered(). */
   ps->old_state_bits &= ~(RTSTEPPER_STEP_STATE_INPUT0_BIT | RTSTEPPER_STEP_STATE_INPUT1_BIT | RTSTEPPER_STEP_STATE_INPUT2_BIT);

   stat = RTSTEPPER_R_OK;

 bugout:
   return stat;
}       /* rtstepper_clear_abort() */

enum RTSTEPPER_RESULT rtstepper_set_abort_wait(struct rtstepper_app_session *ps)
{
   enum RTSTEPPER_RESULT stat;

   if (!ps->usbfd)
   {
      stat = RTSTEPPER_R_REQ_ERROR;
      goto bugout;
   }

   DBG("rtstepper_set_abort_wait()\n");
   rtstepper_set_abort(ps);

   /* Instead of waiting for all IO to finish, kill it now. */
   if (ps->xfr_active)
   {
      pthread_cancel(ps->bulk_write_tid);
      pthread_cond_broadcast(&ps->write_done_cond);
   }

   /* Wait for step buffer to drain. */
   while ((stat = rtstepper_query_state(ps)) == RTSTEPPER_R_OK)
   {
      if (ps->state_bits & RTSTEPPER_STEP_STATE_EMPTY_BIT)
         break;  /* done */
   }

   if (ps->xfr_active || stat != RTSTEPPER_R_OK)
   {
      rtstepper_exit(ps);
      stat = rtstepper_init(ps, ps->error_function);
   }

 bugout:
   return stat;
}       /* rtstepper_set_abort_wait() */

enum RTSTEPPER_RESULT rtstepper_set_gpo(struct rtstepper_app_session *ps, enum RTSTEPPER_GPO_BIT num, int value)
{
   enum RTSTEPPER_RESULT stat = RTSTEPPER_R_REQ_ERROR;

   if (!ps->usbfd)
      goto bugout;

   DBG("rtstepper_set_gpo() num=%d val=%d\n", num, value);

   if (num < 0 || num >= RTSTEPPER_GPO_MAX)
   {
      BUG("invalid output signal OUTPUT=%d\n", num);
      goto bugout;      
   }

   if (ps->gpo_pin[num] == 0)
   {
      DBG("OUTPUT=%d is not enabled\n", num);
      goto bugout;      
   }

   ps->gpo[num] = value;
   stat = RTSTEPPER_R_OK;
 bugout:
   return stat;
}       /* rtstepper_set_gpo() */

enum RTSTEPPER_RESULT rtstepper_is_input0_triggered(struct rtstepper_app_session *ps)
{
   enum RTSTEPPER_RESULT stat = RTSTEPPER_R_INPUT_FALSE;
   int new_bit, old_bit;

   if (!ps->usbfd)
      goto bugout;

   old_bit = ps->old_state_bits & RTSTEPPER_STEP_STATE_INPUT0_BIT;
   if (ps->input0_abort_enabled)
   {
      new_bit = ps->state_bits & RTSTEPPER_STEP_STATE_INPUT0_BIT;
      if ((new_bit == RTSTEPPER_STEP_STATE_INPUT0_BIT) && (old_bit == 0))
      {
         /* Found input0 low to high transition. */
         stat = RTSTEPPER_R_INPUT_TRUE;
      }
      ps->old_state_bits |= new_bit;
   }

 bugout:
   return stat;
}       /* rtstepper_is_input0_triggered() */

enum RTSTEPPER_RESULT rtstepper_is_input1_triggered(struct rtstepper_app_session *ps)
{
   enum RTSTEPPER_RESULT stat = RTSTEPPER_R_INPUT_FALSE;
   int new_bit, old_bit;

   if (!ps->usbfd)
      goto bugout;

   old_bit = ps->old_state_bits & RTSTEPPER_STEP_STATE_INPUT1_BIT;
   if (ps->input1_abort_enabled)
   {
      new_bit = ps->state_bits & RTSTEPPER_STEP_STATE_INPUT1_BIT;
      if ((new_bit == RTSTEPPER_STEP_STATE_INPUT1_BIT) && (old_bit == 0))
      {
         /* Found input1 low to high transition. */
         stat = RTSTEPPER_R_INPUT_TRUE;
      }
      ps->old_state_bits |= new_bit;
   }

 bugout:
   return stat;
}       /* rtstepper_is_input1_triggered() */

enum RTSTEPPER_RESULT rtstepper_is_input2_triggered(struct rtstepper_app_session *ps)
{
   enum RTSTEPPER_RESULT stat = RTSTEPPER_R_INPUT_FALSE;
   int new_bit, old_bit;

   if (!ps->usbfd)
      goto bugout;

   old_bit = ps->old_state_bits & RTSTEPPER_STEP_STATE_INPUT2_BIT;
   if (ps->input2_abort_enabled)
   {
      new_bit = ps->state_bits & RTSTEPPER_STEP_STATE_INPUT2_BIT;
      if ((new_bit == RTSTEPPER_STEP_STATE_INPUT2_BIT) && (old_bit == 0))
      {
         /* Found input2 low to high transition. */
         stat = RTSTEPPER_R_INPUT_TRUE;
      }
      ps->old_state_bits |= new_bit;
   }

 bugout:
   return stat;
}       /* rtstepper_is_input2_triggered() */

enum RTSTEPPER_RESULT rtstepper_input0_state(struct rtstepper_app_session *ps)
{
   enum RTSTEPPER_RESULT stat = RTSTEPPER_R_INPUT_FALSE;

   if (!ps->usbfd)
      goto bugout;

   if (ps->old_state_bits & RTSTEPPER_STEP_STATE_INPUT0_BIT)
      stat = RTSTEPPER_R_INPUT_TRUE;

 bugout:
   return stat;
}       /* rtstepper_input0_state() */

enum RTSTEPPER_RESULT rtstepper_input1_state(struct rtstepper_app_session *ps)
{
   enum RTSTEPPER_RESULT stat = RTSTEPPER_R_INPUT_FALSE;

   if (!ps->usbfd)
      goto bugout;

   if (ps->old_state_bits & RTSTEPPER_STEP_STATE_INPUT1_BIT)
      stat = RTSTEPPER_R_INPUT_TRUE;

 bugout:
   return stat;
}       /* rtstepper_input1_state() */

enum RTSTEPPER_RESULT rtstepper_input2_state(struct rtstepper_app_session *ps)
{
   enum RTSTEPPER_RESULT stat = RTSTEPPER_R_INPUT_FALSE;

   if (!ps->usbfd)
      goto bugout;

   if (ps->old_state_bits & RTSTEPPER_STEP_STATE_INPUT2_BIT)
      stat = RTSTEPPER_R_INPUT_TRUE;

 bugout:
   return stat;
}       /* rtstepper_input2_state() */

/* 
 * Read dongle state bits. Blocks on 1ms intervals. Returns following bitfield definitions.
 *    abort_bit = 0x1
 *    empty_bit = 0x2 
 *    input0_bit = 0x8 
 *    input2_bit = 0x10 
 *    input3_bit = 0x20
 * All bits are active high: 1=true 0=false
 */
enum RTSTEPPER_RESULT rtstepper_query_state(struct rtstepper_app_session *ps)
{
   struct step_query query_response;
   enum RTSTEPPER_RESULT stat;
   int len;
   static unsigned int cnt = 0;
   static int good_query = 0;

   ps->state_bits = 0;

   if (!ps->usbfd)
   {
      stat = RTSTEPPER_R_REQ_ERROR;
      goto bugout;
   }

   len = usb_control_msg(fd_table[ps->usbfd].hd, USB_ENDPOINT_IN | USB_TYPE_VENDOR | USB_RECIP_INTERFACE,       /* bmRequestType */
                         STEP_QUERY,    /* bRequest */
                         0x0,   /* wValue */
                         fd_table[ps->usbfd].interface, /* wIndex */
                         (char *) &query_response, sizeof(query_response), LIBUSB_CONTROL_REQ_TIMEOUT);
   if (len != sizeof(query_response))
   {
      if (cnt++ < 30)
         BUG("invalid usb query_response len=%d good_query_cnt=%d\n", len, good_query);
      stat = RTSTEPPER_R_IO_ERROR;
      goto bugout;
   }
   else
      good_query++;

   ps->state_bits = query_response.state_bits._word;
   stat = RTSTEPPER_R_OK;

 bugout:
   return stat;
}       /* rtstepper_query_state() */

enum RTSTEPPER_RESULT rtstepper_init(struct rtstepper_app_session *ps, rtstepper_io_error_cb error_function)
{
   struct step_elements elements;
   enum RTSTEPPER_RESULT stat = RTSTEPPER_R_IO_ERROR;
   int i, len;

   DBG("rtstepper_init() ps=%p\n", ps);

   ps->old_state_bits = 0;
   ps->usbfd = 0;
   ps->buf_size = 0;
   for (i = 0; i < EMCMOT_MAX_AXIS; i++)
   {
      ps->clk_tail[i] = 0;
      ps->direction[i] = 0;
   }
   ps->total = 0;
   ps->xfr_total = 0;
   ps->xfr_active = 0;
   ps->error_function = error_function;

//   pthread_mutex_init(&ps->mutex, NULL);
//   pthread_cond_init(&ps->write_done_cond, NULL);

   usb_init();
   usb_find_busses();
   usb_find_devices();

   /* Find first usb device or usb device matching specified serial number. */
   if ((ps->libusb_device = get_libusb_device(ps->snum)) == NULL)
   {
      BUG("unable to find usb %s dongle snum=%s\n", PACKAGE_NAME, ps->snum);
      goto bugout;
   }

   if (get_interface(ps->libusb_device, FD_ff_0_1, &fd_table[FD_ff_0_1]))
   {
      BUG("unable to find usb interface %s\n", fd_name[FD_ff_0_1]);
      goto bugout;
   }

   if (claim_interface(ps->libusb_device, &fd_table[FD_ff_0_1]))
   {
      BUG("unable to claim usb interface fd=%d snum=%s\n", FD_ff_0_1, ps->snum);
      goto bugout;
   }

   ps->usbfd = FD_ff_0_1;

   /* Clear any outstanding Abort and step count. */
   memset(&elements, 0, sizeof(elements));
   len = usb_control_msg(fd_table[ps->usbfd].hd, USB_ENDPOINT_OUT | USB_TYPE_VENDOR | USB_RECIP_INTERFACE,      /* bmRequestType */
                         STEP_SET,      /* bRequest */
                         0x0,   /* wValue */
                         fd_table[ps->usbfd].interface, /* wIndex */
                         (char *) &elements, sizeof(elements), LIBUSB_CONTROL_REQ_TIMEOUT);

   if (len != sizeof(elements))
   {
      BUG("unable to initialize dongle\n");
      goto bugout;
   }

   stat = RTSTEPPER_R_OK;

 bugout:
   return stat;
}       /* rtstepper_init() */

enum RTSTEPPER_RESULT rtstepper_exit(struct rtstepper_app_session *ps)
{
   DBG("rtstepper_exit() ps=%p\n", ps);

   if (ps == NULL)
      return RTSTEPPER_R_IO_ERROR;

   if (ps->buf)
   {
      free(ps->buf);
      ps->buf = NULL;
   }

   if (ps->xfr_buf)
   {
      free(ps->xfr_buf);
      ps->xfr_buf = NULL;
   }

   if (ps->usbfd)
   {
      release_interface(&fd_table[ps->usbfd]);
      ps->usbfd = 0;
   }

//   pthread_mutex_destroy(&ps->mutex);
//   pthread_cond_destroy(&ps->write_done_cond);

   return RTSTEPPER_R_OK;
}       /* rtstepper_exit() */
