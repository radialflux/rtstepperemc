/************************************************************************************\

  msg.h - message queue processor for rt-stepper

  (c) 2008 Copyright Eckler Software

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

\************************************************************************************/

#ifndef _MSG_H
#define _MSG_H

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

//#include "emc.h"

#ifdef __cplusplus
extern "C"
{
#endif

   int get_message(struct emc_session *ps, struct _emc_command_msg_t **msg, unsigned int *last, int *lock, const char *tag);
   int remove_message(struct emc_session *ps, struct _emc_command_msg_t *msg, int *lock, const char *tag);
   unsigned int post_message(struct emc_session *ps, struct _emc_command_msg_t *msg, const char *tag);
   unsigned int send_message(struct emc_session *ps, struct _emc_command_msg_t *msg, const char *tag);
   int dump_message(struct emc_session *ps);
   int peek_message(struct emc_session *ps, struct _emc_command_msg_t **msg, unsigned int *last, int *lock, const char *tag);
   const char *lookup_message(int type);
   const char *lookup_rcs_status(int type);
   const char *lookup_task_exec_state(int type);
   const char *lookup_task_interp_state(int type);

#ifdef __cplusplus
}
#endif

#endif                          /* _MSG_H */
