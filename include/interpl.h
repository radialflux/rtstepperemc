/********************************************************************
* Description: interpl.h
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
#ifndef _INTERPL_H
#define _INTERPL_H

#include "linklist.h"

#define MAX_MSG_COMMAND_SIZE 1000

// these go on the interp list
struct MSG_INTERP_LIST_NODE
{
   int line_number;             // line number it was on
   union _dummy_union
   {
      int i;
      long l;
      double d;
      float f;
      long long ll;
      long double ld;
   } dummy;                     // paranoid alignment variable.

   union _command_union
   {
      char commandbuf[MAX_MSG_COMMAND_SIZE];    // the MSG command;
      int i;
      long l;
      double d;
      float f;
      long long ll;
      long double ld;
   } command;
};

// here's the interp list itself
class MSG_INTERP_LIST
{
 public:
   MSG_INTERP_LIST();
   ~MSG_INTERP_LIST();

   int set_line_number(int line);
   int get_line_number();
   int append(emc_command_msg_t *);
   emc_command_msg_t *get();
   void clear();
   void print();
   int len();

 private:
//    class LinkedList * linked_list_ptr;
     class RCS_LINKED_LIST * linked_list_ptr;
   MSG_INTERP_LIST_NODE temp_node;      // filled in and put on the list
   int next_line_number;        // line number used to fill temp_node
   int line_number;             // line number of node from get()
};

extern MSG_INTERP_LIST interp_list;     /* MSG Union, for interpreter */

#endif /* _INTERPL_H */
