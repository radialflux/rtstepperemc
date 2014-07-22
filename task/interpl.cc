/********************************************************************
* Description: interpl.cc
*   Mechanism for queueing NML messages, used by the interpreter and
*   canonical interface to report list of NML statements from program
*   files to HME.
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author:
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2004 All rights reserved.
*
* Last change:
********************************************************************/

#include <string.h>     /* memcpy() */
#include "emc.h"
#include "interpl.h"    // these decls
#include "bug.h"

MSG_INTERP_LIST::MSG_INTERP_LIST()
{
//    linked_list_ptr = new LinkedList;
   linked_list_ptr = new RCS_LINKED_LIST;

   next_line_number = 0;
   line_number = 0;
}

MSG_INTERP_LIST::~MSG_INTERP_LIST()
{
   if (NULL != linked_list_ptr)
   {
      delete linked_list_ptr;
      linked_list_ptr = NULL;
   }
}

// sets the line number used for subsequent appends
int MSG_INTERP_LIST::set_line_number(int line)
{
   next_line_number = line;

   return 0;
}

int MSG_INTERP_LIST::append(emc_command_msg_t * cmd_ptr)
{
   /* check for invalid data */
   if (NULL == cmd_ptr)
   {
      BUG("MSG_INTERP_LIST::append : attempt to append NULL msg\n");
      return -1;
   }

   if (cmd_ptr->msg.type == 0)
   {
      BUG("MSG_INTERP_LIST::append : attempt to append 0 type\n");
      return -1;
   }

   if (NULL == linked_list_ptr)
   {
      return -1;
   }
   // fill in the MSG_INTERP_LIST_NODE
   temp_node.line_number = next_line_number;
   memcpy(temp_node.command.commandbuf, cmd_ptr, sizeof(emc_command_msg_t));

   // stick it on the list
   linked_list_ptr->store_at_tail(&temp_node,
                                  sizeof(emc_command_msg_t) + sizeof(temp_node.line_number) +
                                  sizeof(temp_node.dummy) + 32 + (32 - sizeof(emc_command_msg_t) % 32), 1);

   DBG("MSG_INTERP_LIST::append() type=%d cmd=%s list_size=%d, line_number=%d\n",
       cmd_ptr->msg.type, lookup_message(cmd_ptr->msg.type), linked_list_ptr->list_size, temp_node.line_number);

   return 0;
}

emc_command_msg_t *MSG_INTERP_LIST::get()
{
   emc_command_msg_t *ret;
   MSG_INTERP_LIST_NODE *node_ptr;

   if (NULL == linked_list_ptr)
   {
      line_number = 0;
      return NULL;
   }

   node_ptr = (MSG_INTERP_LIST_NODE *) linked_list_ptr->retrieve_head();

   if (NULL == node_ptr)
   {
      line_number = 0;
      return NULL;
   }
   // save line number of this one, for use by get_line_number
   line_number = node_ptr->line_number;

   // get it off the front
   ret = (emc_command_msg_t *) ((char *) node_ptr->command.commandbuf);

   DBG("MSG_INTERP_LIST::get() id=%d  cmd=%s\n", node_ptr->line_number, lookup_message(ret->msg.type));

   return ret;
}

void MSG_INTERP_LIST::clear()
{
   if (NULL != linked_list_ptr)
   {
      linked_list_ptr->delete_members();
   }
}

void MSG_INTERP_LIST::print()
{
   emc_command_msg_t *cmd;

   if (NULL == linked_list_ptr)
   {
      return;
   }

   cmd = (emc_command_msg_t *) linked_list_ptr->get_head();

   while (NULL != cmd)
   {
      DBG("%d ", cmd->msg.type);
      cmd = (emc_command_msg_t *) linked_list_ptr->get_next();
   }

   DBG("\n");
}

int MSG_INTERP_LIST::len()
{
   if (NULL == linked_list_ptr)
   {
      return 0;
   }

   return ((int) linked_list_ptr->list_size);
}

int MSG_INTERP_LIST::get_line_number()
{
   return line_number;
}
