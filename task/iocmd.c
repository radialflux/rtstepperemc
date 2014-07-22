/********************************************************************
*
* iocmd.c - IO support commands for EMC2
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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <ctype.h>
#include "emc.h"
#include "emctool.h"
#include "tool_parse.h"
#include "bug.h"

static char *ttcomments[CANON_POCKETS_MAX];
static int fms[CANON_POCKETS_MAX];
static int random_toolchanger = 0;

/********************************************************************
*
* Description: saveToolTable(const char *filename, CANON_TOOL_TABLE toolTable[])
*               Saves the tool table from toolTable[] array into file filename.
*                 Array is CANON_TOOL_MAX + 1 entries, since 0 is included.
*
* Return Value: Zero on success or -1 if file not found.
*
* Side Effects: Default setting used if the parameter not found in
*               the ini file.
*
********************************************************************/
static int saveToolTable(const char *filename, struct CANON_TOOL_TABLE toolTable[])
{
   int pocket;
   FILE *fp;
   const char *name;
   int start_pocket;

   // check filename
   if (filename[0] == 0)
   {
      name = TOOL_TABLE_FILE;
   }
   else
   {
      // point to name provided
      name = filename;
   }

   // open tool table file
   if ((fp = fopen(name, "w")) == NULL)
   {
      BUG("unable to save tool table %s\n", name);
      emcOperatorMessage(0, EMC_I18N("unable to save tool table %s"), name);
      return -1;
   }

   if (random_toolchanger)
   {
      start_pocket = 0;
   }
   else
   {
      start_pocket = 1;
   }
   for (pocket = start_pocket; pocket < CANON_POCKETS_MAX; pocket++)
   {
      if (toolTable[pocket].toolno != -1)
      {
         fprintf(fp, "T%d P%d", toolTable[pocket].toolno, random_toolchanger ? pocket : fms[pocket]);
         if (toolTable[pocket].diameter)
            fprintf(fp, " D%f", toolTable[pocket].diameter);
         if (toolTable[pocket].offset.tran.x)
            fprintf(fp, " X%+f", toolTable[pocket].offset.tran.x);
         if (toolTable[pocket].offset.tran.y)
            fprintf(fp, " Y%+f", toolTable[pocket].offset.tran.y);
         if (toolTable[pocket].offset.tran.z)
            fprintf(fp, " Z%+f", toolTable[pocket].offset.tran.z);
         if (toolTable[pocket].offset.a)
            fprintf(fp, " A%+f", toolTable[pocket].offset.a);
         if (toolTable[pocket].offset.b)
            fprintf(fp, " B%+f", toolTable[pocket].offset.b);
         if (toolTable[pocket].offset.c)
            fprintf(fp, " C%+f", toolTable[pocket].offset.c);
         if (toolTable[pocket].offset.u)
            fprintf(fp, " U%+f", toolTable[pocket].offset.u);
         if (toolTable[pocket].offset.v)
            fprintf(fp, " V%+f", toolTable[pocket].offset.v);
         if (toolTable[pocket].offset.w)
            fprintf(fp, " W%+f", toolTable[pocket].offset.w);
         if (toolTable[pocket].frontangle)
            fprintf(fp, " I%+f", toolTable[pocket].frontangle);
         if (toolTable[pocket].backangle)
            fprintf(fp, " J%+f", toolTable[pocket].backangle);
         if (toolTable[pocket].orientation)
            fprintf(fp, " Q%d", toolTable[pocket].orientation);
         fprintf(fp, " ;%s\n", ttcomments[pocket]);
      }
   }

   fclose(fp);
   return 0;
}

static void load_tool(int pocket)
{
   if (random_toolchanger)
   {
      // swap the tools between the desired pocket and the spindle pocket
      struct CANON_TOOL_TABLE temp;
      char *comment_temp;

      temp = emcioStatus.tool.toolTable[0];
      emcioStatus.tool.toolTable[0] = emcioStatus.tool.toolTable[pocket];
      emcioStatus.tool.toolTable[pocket] = temp;

      comment_temp = ttcomments[0];
      ttcomments[0] = ttcomments[pocket];
      ttcomments[pocket] = comment_temp;

      if (saveToolTable(TOOL_TABLE_FILE, emcioStatus.tool.toolTable) != 0)
      {
//         emcioStatus.status = RCS_ERROR;
      }
   }
   else if (pocket == 0)
   {
      // magic T0 = pocket 0 = no tool
      emcioStatus.tool.toolTable[0].toolno = -1;
      ZERO_EMC_POSE(emcioStatus.tool.toolTable[0].offset);
      emcioStatus.tool.toolTable[0].diameter = 0.0;
      emcioStatus.tool.toolTable[0].frontangle = 0.0;
      emcioStatus.tool.toolTable[0].backangle = 0.0;
      emcioStatus.tool.toolTable[0].orientation = 0;
   }
   else
   {
      // just copy the desired tool to the spindle
      emcioStatus.tool.toolTable[0] = emcioStatus.tool.toolTable[pocket];
   }
}

static void reload_tool_number(int toolno)
{
   int i;
   for (i = 0; i < CANON_POCKETS_MAX; i++)
   {
      if (emcioStatus.tool.toolTable[i].toolno == toolno)
      {
         load_tool(i);
         break;
      }
   }
}

void emciocommandHandler(emcio_command_t * emcioCommand)
{
   int i;
   int type;

   if (emcioCommand == NULL || emcioCommand->type == 0 || emcioCommand->serial_number == emcioStatus.echo_serial_number)
   {    // command already finished
      return;
   }

   type = emcioCommand->type;
   emcioStatus.status = RCS_DONE;

   switch (type)
   {
   case 0:
      break;

   case EMCIO_IO_INIT_COMMAND:
      break;

   case EMCIO_TOOL_INIT_COMMAND:
      for (i = 0; i < CANON_POCKETS_MAX; i++)
      {
         if (ttcomments[i] == NULL)
            ttcomments[i] = (char *) malloc(CANON_TOOL_ENTRY_LEN);
      }

      // on nonrandom machines, always start by assuming the spindle is empty
      if (!random_toolchanger)
      {
         emcioStatus.tool.toolTable[0].toolno = -1;
         ZERO_EMC_POSE(emcioStatus.tool.toolTable[0].offset);
         emcioStatus.tool.toolTable[0].diameter = 0.0;
         emcioStatus.tool.toolTable[0].frontangle = 0.0;
         emcioStatus.tool.toolTable[0].backangle = 0.0;
         emcioStatus.tool.toolTable[0].orientation = 0;
         fms[0] = 0;
         ttcomments[0][0] = '\0';
      }

      loadToolTable(TOOL_TABLE_FILE, emcioStatus.tool.toolTable, fms, ttcomments, random_toolchanger);
      reload_tool_number(emcioStatus.tool.toolInSpindle);
      break;

   case EMCIO_TOOL_HALT_COMMAND:
      // Cleanup tooltable.
      for (i = 0; i < CANON_POCKETS_MAX; i++)
      {
         if (ttcomments[i] != NULL)
         {
            free(ttcomments[i]);
            ttcomments[i] = NULL;
         }
      }
      break;

   case EMCIO_TOOL_ABORT_COMMAND:
      // this gets sent on any Task Abort, so it might be safer to stop
      // the spindle  and coolant
      emcioStatus.coolant.mist = 0;
      emcioStatus.coolant.flood = 0;
//            *(iocontrol_data->coolant_mist)=0;          /* coolant mist output pin */
//            *(iocontrol_data->coolant_flood)=0;         /* coolant flood output pin */
      break;

   case EMCIO_TOOL_PREPARE_COMMAND:
      {
         int p = emcioCommand->tool;

         // it doesn't make sense to prep the spindle pocket
         if (random_toolchanger && p == 0)
            break;

         /* set tool number first */
//                *(iocontrol_data->tool_prep_pocket) = p;
         if (!random_toolchanger && p == 0)
         {
//                    *(iocontrol_data->tool_prep_number) = 0;
         }
         else
         {
//                    *(iocontrol_data->tool_prep_number) = emcioStatus.tool.toolTable[p].toolno;
         }
         /* then set the prepare pin to tell external logic to get started */
//                *(iocontrol_data->tool_prepare) = 1;
         // the feedback logic is done inside read_hal_inputs()
         // we only need to set RCS_EXEC if RCS_DONE is not already set by the above logic
//                if (tool_status != 10) //set above to 10 in case PREP already finished (HAL loopback machine)
//                    emcioStatus.status = RCS_EXEC;
      }
      break;

   case EMCIO_TOOL_LOAD_COMMAND:
      // it doesn't make sense to load a tool from the spindle pocket
      if (random_toolchanger && emcioStatus.tool.pocketPrepped == 0)
      {
         break;
      }

      // it's not necessary to load the tool already in the spindle
      if (!random_toolchanger && emcioStatus.tool.pocketPrepped > 0 &&
          emcioStatus.tool.toolInSpindle == emcioStatus.tool.toolTable[emcioStatus.tool.pocketPrepped].toolno)
      {
         break;
      }

      if (emcioStatus.tool.pocketPrepped != -1)
      {
         //notify HW for toolchange
//                *(iocontrol_data->tool_change) = 1;
         // the feedback logic is done inside read_hal_inputs() we only
         // need to set RCS_EXEC if RCS_DONE is not already set by the
         // above logic
//                if (tool_status != 11)
         // set above to 11 in case LOAD already finished (HAL
         // loopback machine)
//                    emcioStatus.status = RCS_EXEC;
      }
      break;

   case EMCIO_TOOL_UNLOAD_COMMAND:
      emcioStatus.tool.toolInSpindle = 0;
      break;

   case EMCIO_TOOL_LOAD_TOOL_TABLE_COMMAND:
      {
         const char *filename = emcioCommand->file;
         if (!strlen(filename))
            filename = TOOL_TABLE_FILE;
         if (loadToolTable(filename, emcioStatus.tool.toolTable, fms, ttcomments, random_toolchanger) == 0)
            reload_tool_number(emcioStatus.tool.toolInSpindle);
      }
      break;

   case EMCIO_TOOL_SET_OFFSET_COMMAND:
      {
         int p, t, o;
         double d, f, b;
         EmcPose offs;

         p = emcioCommand->pocket;
         t = emcioCommand->toolno;
         offs = emcioCommand->offset;
         d = emcioCommand->diameter;
         f = emcioCommand->frontangle;
         b = emcioCommand->backangle;
         o = emcioCommand->orientation;

         DBG("EMC_TOOL_SET_OFFSET pocket=%d toolno=%d zoffset=%lf, xoffset=%lf, diameter=%lf,"
             " frontangle=%lf, backangle=%lf, orientation=%d\n", p, t, offs.tran.z, offs.tran.x, d, f, b, o);

         emcioStatus.tool.toolTable[p].toolno = t;
         emcioStatus.tool.toolTable[p].offset = offs;
         emcioStatus.tool.toolTable[p].diameter = d;
         emcioStatus.tool.toolTable[p].frontangle = f;
         emcioStatus.tool.toolTable[p].backangle = b;
         emcioStatus.tool.toolTable[p].orientation = o;

         if (emcioStatus.tool.toolInSpindle == t)
         {
            emcioStatus.tool.toolTable[0] = emcioStatus.tool.toolTable[p];
         }
      }
      if (saveToolTable(TOOL_TABLE_FILE, emcioStatus.tool.toolTable) != 0)
      {
//         emcioStatus.status = RCS_ERROR;
      }
      break;

   case EMCIO_TOOL_SET_NUMBER_COMMAND:
      {
         int number;

         number = emcioCommand->tool;
         emcioStatus.tool.toolInSpindle = number;
//                *(iocontrol_data->tool_number) = emcioStatus.tool.toolInSpindle; //likewise in HAL
      }
      break;

   case EMCIO_COOLANT_MIST_ON_COMMAND:
      emcioStatus.coolant.mist = 1;
      break;

   case EMCIO_COOLANT_MIST_OFF_COMMAND:
      emcioStatus.coolant.mist = 0;
      break;

   case EMCIO_COOLANT_FLOOD_ON_COMMAND:
      emcioStatus.coolant.flood = 1;
      break;

   case EMCIO_COOLANT_FLOOD_OFF_COMMAND:
      emcioStatus.coolant.flood = 0;
      break;

   case EMCIO_AUX_ESTOP_ON_COMMAND:
      /* assert an ESTOP to the outside world (thru HAL) */
//            *(iocontrol_data->user_enable_out) = 0; //disable on ESTOP_ON
//            hal_init_pins(); //resets all HAL pins to safe value
      break;

   case EMCIO_AUX_ESTOP_OFF_COMMAND:
      /* remove ESTOP */
//            *(iocontrol_data->user_enable_out) = 1; //we're good to enable on ESTOP_OFF
      /* generate a rising edge to reset optional HAL latch */
//            *(iocontrol_data->user_request_enable) = 1;
      break;

   case EMCIO_AUX_ESTOP_RESET_COMMAND:
      // doesn't do anything right now, this will need to come from GUI
      // but that means task needs to be rewritten/rethinked
      break;

   case EMCIO_LUBE_ON_COMMAND:
      emcioStatus.lube.on = 1;
//            *(iocontrol_data->lube) = 1;
      break;

   case EMCIO_LUBE_OFF_COMMAND:
      emcioStatus.lube.on = 0;
//            *(iocontrol_data->lube) = 0;
      break;

   case EMCIO_SET_DEBUG_COMMAND:
//            EMC_DEBUG = ((EMC_SET_DEBUG *) emcioCommand)->debug;
      break;

   default:
      BUG("IO: unknown command %d\n", emcioCommand->type);
      break;
   }    /* switch (type) */

   // ack for the received command
   emcioStatus.command_type = type;
   emcioStatus.echo_serial_number = emcioCommand->serial_number;
   //set above, to allow some commands to fail this
   //emcioStatus.status = RCS_DONE;
//    emcioStatus.heartbeat++;

   /* clear reset line to allow for a later rising edge */
//        *(iocontrol_data->user_request_enable) = 0;

   return;
}
