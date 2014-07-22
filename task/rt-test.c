/*****************************************************************************\

  rt-test.c - test for pic firmware running on rt-stepper dongle

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
#include <usb.h>
#endif

#if (defined(__WIN32__) || defined(_WINDOWS))
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#define sleep(n) Sleep(1000 * n)
#endif

#define _STRINGIZE(x) #x
#define STRINGIZE(x) _STRINGIZE(x)

#define BUG(args...) fprintf(stderr, __FILE__ " " STRINGIZE(__LINE__) ": " args)

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

static struct step_elements elements;
static struct step_query query_response;

enum FD_ID
{
   FD_NA = 0,
   FD_ff_0_1,
   FD_MAX
};

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

struct usb_device *libusb_device;       /* selected usb device */
struct file_descriptor fd_table[FD_MAX];        /* usb file descriptors */

static void usage()
{
   fprintf(stdout, "rt-test %s, USB system test\n", PACKAGE_VERSION);
   fprintf(stdout, "(c) 2008-2011 Copyright Eckler Software\n");
   fprintf(stdout, "David Suffield, dsuffiel@ecklersoft.com\n");
   fprintf(stdout, "usage: rt-test [-s serial_number]\n");
   fprintf(stdout, "WARNING!!!! Do *NOT* run with parallel port connected!\n");
}

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

static int bulk_write(int fd, const void *buf, int size, int usec)
{
   int len = -EIO;

   if (fd_table[fd].hd == NULL)
   {
      BUG("invalid bulk_write state\n");
      goto bugout;
   }

   len = usb_bulk_write(fd_table[fd].hd, 1, (char *) buf, size, usec);

   if (len < 0)
   {
      BUG("bulk_write failed buf=%p size=%d len=%d: %m\n", buf, size, len);
      goto bugout;
   }

 bugout:
   return len;
}

int main(int argc, char *argv[])
{
   char snum[128];              /* serial number */
   char buf[65536];             /* for testing pick multiple 64-byte writes otherwise there could be a stall */
   unsigned char toggle;
   int i, ret = 10, fd, len, tmo;

   snum[0] = 0;
   while ((i = getopt(argc, argv, "vhps:")) != -1)
   {
      switch (i)
      {
      case 's':
         strncpy(snum, optarg, sizeof(snum));
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

   fprintf(stdout, "WARNING!!!! Do *NOT* run with parallel port connected!\n");

   usb_init();
   usb_find_busses();
   usb_find_devices();

   /* Find first usb device or usb device matching specified serial number. */
   if ((libusb_device = get_libusb_device(snum)) == NULL)
   {
      BUG("unable to open usb device snum=%s\n", snum);
      goto bugout;
   }

   fd = FD_ff_0_1;

   if (get_interface(libusb_device, fd, &fd_table[fd]))
   {
      BUG("unable to find interface %s\n", fd_name[fd]);
      goto bugout;
   }

   if (claim_interface(libusb_device, &fd_table[fd]))
   {
      BUG("unable to claim usb device interface=%s snum=%s\n", fd_name[fd], snum);
      goto bugout;
   }

   /* Clear device state bits and running step count. */
   len = usb_control_msg(fd_table[fd].hd, USB_ENDPOINT_OUT | USB_TYPE_VENDOR | USB_RECIP_INTERFACE,     /* bmRequestType */
                         STEP_SET,      /* bRequest */
                         0x0,   /* wValue */
                         fd_table[fd].interface,        /* wIndex */
                         (char *) &elements, sizeof(elements), LIBUSB_CONTROL_REQ_TIMEOUT);

   if (len != sizeof(elements))
   {
      BUG("invalid set_response: %m\n");
      goto bugout;
   }

   /* Verify state. */
   len = usb_control_msg(fd_table[fd].hd, USB_ENDPOINT_IN | USB_TYPE_VENDOR | USB_RECIP_INTERFACE,      /* bmRequestType */
                         STEP_QUERY,    /* bRequest */
                         0x0,   /* wValue */
                         fd_table[fd].interface,        /* wIndex */
                         (char *) &query_response, sizeof(query_response), LIBUSB_CONTROL_REQ_TIMEOUT);

   if (len != sizeof(query_response))
   {
      BUG("invalid query_response: %m\n");
      goto bugout;
   }

   fprintf(stdout, "query_response (Begin) len=%d trip=%d state=%x steps=%d\n", len,
           query_response.trip_cnt, query_response.state_bits._word, query_response.step);

   toggle = 0x55;
   for (i = 0; i < sizeof(buf); i++)
   {
      buf[i] = toggle;
      toggle = ~toggle;
   }

   if ((len = bulk_write(fd, buf, sizeof(buf), LIBUSB_TIMEOUT)) < 0)
   {
      BUG("unable to write data len=%d fd=%d snum=%s\n", len, fd, snum);
      goto bugout;
   }
   fprintf(stdout, "wrote %d bytes\n", len);

   /* Wait for write to finish. */
   tmo = (double) sizeof(buf) * 0.000021333;    /* timeout in seconds = steps * period */
   tmo += 1;    /* plus 1 second */
   sleep(tmo);

   len = usb_control_msg(fd_table[fd].hd, USB_ENDPOINT_IN | USB_TYPE_VENDOR | USB_RECIP_INTERFACE,      /* bmRequestType */
                         STEP_QUERY,    /* bRequest */
                         0x0,   /* wValue */
                         fd_table[fd].interface,        /* wIndex */
                         (char *) &query_response, sizeof(query_response), LIBUSB_CONTROL_REQ_TIMEOUT);

   if (len != sizeof(query_response))
   {
      BUG("invalid query_response: %m\n");
      goto bugout;
   }

   fprintf(stdout, "query_response (End) len=%d trip=%d state=%x steps=%d\n", len,
           query_response.trip_cnt, query_response.state_bits._word, query_response.step);

   release_interface(&fd_table[fd]);

//   if (query_response.step != sizeof(buf) || query_response.state_bits._word & STEP_STATE_STALL_BIT)
   if (query_response.step != sizeof(buf))
   {
      fprintf(stderr, "USB test failed.\n");
      fprintf(stderr, "  Suggestions in order:\n");
      fprintf(stderr, "     1. Remove any hubs, plug directly into PC.\n");
      fprintf(stderr, "     2. Try a certified HIGH-SPEED or USB 2.0 cable.\n");
      fprintf(stderr, "     3. Try a different USB port directly into PC.\n");
      fprintf(stderr, "     4. Kill all extraneous resource intensive Applications.\n");
      fprintf(stderr, "     5. Try running in a TTY terminal without Gnome or KDE.\n");
      fprintf(stderr, "     6. Try a different PC. Your USB subsytem may not support rt-stepper\n");
      ret = 1;
   }
   else
   {
      fprintf(stdout, "USB test passed.\n");
      ret = 0;
   }

 bugout:
   return ret;
}
