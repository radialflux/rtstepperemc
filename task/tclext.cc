/********************************************************************
* tclext.cc - Tcl (Tool Command Language) extension for EMC2
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
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
#include <math.h>
#include <tcl.h>
#include "emc.h"
#include "canon.h"      // CANON_UNITS, CANON_UNITS_INCHES,MM,CM
#include "bug.h"

/*
  Some commands take 0 or more arguments. 0 arguments means they return
  the associated value; the argument would be to set the value.

  Commands are sent to the EMC, and control resumes immediately. You can
  call a timed wait until the command got there, or a timed wait until the
  command completed, or not wait at all.

  EMC commands:

  EMC_INIFILE
  Exported values of the EMC global of the same name

  emc_plat
  Returns the platform for which this was compiled, e.g., linux_2_0_36

  emc_ini <var> <section>
  Returns the string value of <var> in section <section>, in EMC_INIFILE

  emc_wait received | done
  Force a wait for the previous command to be received, or done. This lets
  you wait in the event that "emc_set_wait none" is in effect.

  emc_update (none) | none | auto
  With no arg, forces an update of the EMC status. With "none", doesn't
  cause an automatic update of status with other emc_ words. With "auto",
  makes emc_ words automatically update status before they return values.

  emc_operator_message
  Returns the current EMC operator message string, or "ok" if none.

  emc_estop (none) | on | off
  With no arg, returns the estop setting as "on" or "off". Otherwise,
  sends an estop on or off command.

  emc_machine (none) | on | off
  With no arg, returns the machine setting as "on" or "off". Otherwise,
  sends a machine on or off command.

  emc_mode (none) | manual | auto | mdi
  With no arg, returns the mode setting as "manual", "auto", or "mdi".
  Otherwise, sends a mode manual, auto, or mdi command.

  emc_mist (none) | on | off
  With no arg, returns the mist setting as "on" or "off". Otherwise,
  sends a mist on or off command.

  emc_flood (none) | on | off
  With no arg, returns the flood setting as "on" or "off". Otherwise,
  sends a flood on or off command.

  emc_lube (none) | on | off
  With no arg, returns the lubricant pump setting as "on" or "off".
  Otherwise, sends a lube on or off command.

  emc_lube_level
  Returns the lubricant level sensor reading as "ok" or "low".

  emc_spindle (none) | forward | reverse | increase | decrease | constant | off
  With no arg, returns the value of the spindle state as "forward",
  "reverse", "increase", "decrease", or "off". With arg, sends the spindle
  command. Note that "increase" and "decrease" will cause a speed change in
  the corresponding direction until a "constant" command is sent.

  emc_tool
  Returns the id of the currently loaded tool

  emc_tool_offset
  Returns the currently applied tool length offset

  emc_load_tool_table <file>
  Loads the tool table specified by <file>

  emc_home 0 | 1 | 2 | ...
  Homes the indicated axis.

  emc_unhome 0 | 1 | 2 | ...
  Unhomes the indicated axis.

  emc_jog_stop 0 | 1 | 2 | ...
  Stop the axis jog

  emc_jog_incr 0 | 1 | 2 | ... <speed> <incr>
  Jog the indicated axis by increment <incr> at the <speed>; sign of
  speed is direction

  emc_feed_override {<percent>}
  With no args, returns the current feed override, as a percent. With
  argument, set the feed override to be the percent value

  emc_abs_cmd_pos 0 | 1 | ...
  Returns double obj containing the XYZ-SXYZ commanded pos in abs coords,
  at given index, 0 = X, etc.

  emc_abs_act_pos
  Returns double objs containing the XYZ-SXYZ actual pos in abs coords

  emc_rel_cmd_pos 0 | 1 | ...
  Returns double obj containing the XYZ-SXYZ commanded pos in rel coords,
  at given index, 0 = X, etc., including tool length offset

  emc_rel_act_pos
  Returns double objs containing the XYZ-SXYZ actual pos in rel coords,
  including tool length offset

  emc_joint_pos
  Returns double objs containing the actual pos in absolute coords of individual
  joint/slider positions, excludes tool length offset

  emc_pos_offset X | Y | Z | R | P | W
  Returns the position offset associated with the world coordinate provided

  emc_joint_limit 0 | 1 | ...
  Returns "ok", "minsoft", "minhard", "maxsoft", "maxhard"

  emc_joint_fault 0 | 1 | ...
  Returns "ok" or "fault"

  emc_joint_homed 0 | 1 | ...
  Returns "homed", "not"

  emc_mdi <string>
  Sends the <string> as an MDI command

  emc_task_plan_init
  Initializes the program interpreter

  emc_open <filename>
  Opens the named file

  emc_run {<start line>}
  Without start line, runs the opened program from the beginning. With
  start line, runs from that line. A start line of -1 runs in verify mode.

  emc_pause
  Pause program execution

  emc_resume
  Resume program execution

  emc_step
  Step the program one line

  emc_program
  Returns the name of the currently opened program, or "none"

  emc_program_line
  Returns the currently executing line of the program

  emc_program_status
  Returns "idle", "running", or "paused"

  emc_program_codes
  Returns the string for the currently active program codes

  emc_override_limit none | 0 | 1
  returns state of override, sets it or deactivates it (used to jog off hardware limit switches)
  
  emc_program_codes
  Returns the string for the currently active program codes

  emc_joint_type <joint>
  Returns "linear", "angular", or "custom" for the type of the specified joint

  emc_joint_units <joint>
  Returns "inch", "mm", "cm", or "deg", "rad", "grad", or "custom",
  for the corresponding native units of the specified axis. The type
  of the axis (linear or angular) is used to resolve which type of units
  are returned. The units are obtained heuristically, based on the
  EMC_AXIS_STAT::units numerical value of user units per mm or deg.
  For linear joints, something close to 0.03937 is deemed "inch",
  1.000 is "mm", 0.1 is "cm", otherwise it's "custom".
  For angular joints, something close to 1.000 is deemed "deg",
  PI/180 is "rad", 100/90 is "grad", otherwise it's "custom".
 
  emc_program_units
  emc_program_linear_units
  Returns "inch", "mm", "cm", or "none", for the corresponding linear 
  units that are active in the program interpreter.

  emc_program_angular_units
  Returns "deg", "rad", "grad", or "none" for the corresponding angular
  units that are active in the program interpreter.

  emc_user_linear_units
  Returns "inch", "mm", "cm", or "custom", for the
  corresponding native user linear units of the EMC trajectory
  level. This is obtained heuristically, based on the
  EMC_TRAJ_STAT::linearUnits numerical value of user units per mm.
  Something close to 0.03937 is deemed "inch", 1.000 is "mm", 0.1 is
  "cm", otherwise it's "custom".

  emc_user_angular_units
  Returns "deg", "rad", "grad", or "custom" for the corresponding native
  user angular units of the EMC trajectory level. Like with linear units,
  this is obtained heuristically.

  emc_display_linear_units
  emc_display_angular_units
  Returns "inch", "mm", "cm", or "deg", "rad", "grad", or "custom",
  for the linear or angular units that are active in the display. 
  This is effectively the value of linearUnitConversion or
  angularUnitConversion, resp.

  emc_linear_unit_conversion {inch | mm | cm | auto}
  With no args, returns the unit conversion active. With arg, sets the
  units to be displayed. If it's "auto", the units to be displayed match
  the program units.
 
  emc_angular_unit_conversion {deg | rad | grad | auto}
  With no args, returns the unit conversion active. With arg, sets the
  units to be displayed. If it's "auto", the units to be displayed match
  the program units.

  emc_teleop_enable
  Should motion run in teleop mode? (No args
  gets it, one arg sets it.)

  emc_kinematics_type
  returns the type of kinematics functions used identity=1, serial=2,
  parallel=3, custom=4
*/

#ifdef DEBUG_TCL_EXTRA
#define DBG9 DBG
#else
#define DBG9(args...)
#endif

#define CLOSE(a,b,eps) ((a)-(b) < +(eps) && (a)-(b) > -(eps))
#define LINEAR_CLOSENESS 0.0001
#define ANGULAR_CLOSENESS 0.0001
#define INCH_PER_MM (1.0/25.4)
#define CM_PER_MM 0.1
#define GRAD_PER_DEG (100.0/90.0)
#define RAD_PER_DEG TO_RAD      // from posemath.h

enum LINEAR_UNIT_CONVERSION
{
   LINEAR_UNITS_CUSTOM = 1,
   LINEAR_UNITS_AUTO,
   LINEAR_UNITS_MM,
   LINEAR_UNITS_INCH,
   LINEAR_UNITS_CM
};

enum ANGULAR_UNIT_CONVERSION
{
   ANGULAR_UNITS_CUSTOM = 1,
   ANGULAR_UNITS_AUTO,
   ANGULAR_UNITS_DEG,
   ANGULAR_UNITS_RAD,
   ANGULAR_UNITS_GRAD
};

static enum LINEAR_UNIT_CONVERSION linearUnitConversion;
static enum ANGULAR_UNIT_CONVERSION angularUnitConversion;

/* Values are converted to mm, then to desired units. */
static double convertLinearUnits(double u)
{
   double in_mm;

   /* convert u to mm */
   in_mm = u / emcStatus->motion.traj.linearUnits;

   /* convert u to display units */
   switch (linearUnitConversion)
   {
   case LINEAR_UNITS_MM:
      return in_mm;
      break;
   case LINEAR_UNITS_INCH:
      return in_mm * INCH_PER_MM;
      break;
   case LINEAR_UNITS_CM:
      return in_mm * CM_PER_MM;
      break;
   case LINEAR_UNITS_AUTO:
      switch (emcStatus->task.programUnits)
      {
      case CANON_UNITS_MM:
         return in_mm;
         break;
      case CANON_UNITS_INCHES:
         return in_mm * INCH_PER_MM;
         break;
      case CANON_UNITS_CM:
         return in_mm * CM_PER_MM;
         break;
      }
      break;

   case LINEAR_UNITS_CUSTOM:
      return u;
      break;
   }

   // If it ever gets here we have an error.
   return u;
}

static double convertAngularUnits(double u)
{
   // Angular units are always degrees
   return u;
}

/* EMC command functions */

static int emc_plat(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   DBG("emc_plat()\n");
   if (objc == 1)
   {
//      Tcl_SetResult(interp, "Linux", TCL_VOLATILE);
      Tcl_SetObjResult(interp, Tcl_NewStringObj("Linux", -1));
      return TCL_OK;
   }

   Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_plat: need no args", -1));
   return TCL_ERROR;
}

static int emc_ini(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   char buf[LINELEN];
   char *inistring, *defaultstr=NULL;
   const char *varstr = NULL, *secstr = NULL;
   int stat = TCL_ERROR;

   if (objc != 3 && objc != 4)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_ini: need 'var' and 'section'", -1));
      goto bugout;
   }

   varstr = Tcl_GetStringFromObj(objv[1], 0);
   secstr = Tcl_GetStringFromObj(objv[2], 0);

   if (objc == 4)
      defaultstr = Tcl_GetStringFromObj(objv[3], 0);

   inistring = buf;
   if (emc_ui_get_ini_key_value(cd, secstr, varstr, buf, sizeof(buf)) != EMC_R_OK)
   {
      if (defaultstr != NULL)
         inistring = defaultstr;
   }

   DBG("emc_ini() sect=%s key=%s val=%s default=%s\n", secstr, varstr, inistring, defaultstr);

   Tcl_SetObjResult(interp, Tcl_NewStringObj((char *)inistring, -1));

   stat = TCL_OK;

 bugout:
   return stat;
}

static int emc_wait(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   char *objstr;
   double timeout;

   if (objc == 3)
   {
      objstr = Tcl_GetStringFromObj(objv[1], 0);
      Tcl_GetDoubleFromObj(0, objv[2], &timeout);
      DBG("emc_wait() cmd val=%s timeout=%0.5f\n", objstr, timeout);
      if (!strcmp(objstr, "received"))
      {
         if (emc_ui_wait_command_received(cd, timeout) != EMC_R_OK)
         {
            Tcl_SetObjResult(interp, Tcl_NewStringObj("timeout or error", -1));
         }
         return TCL_OK;
      }
      if (!strcmp(objstr, "done"))
      {
         if (emc_ui_wait_command_done(cd, timeout) != EMC_R_OK)
         {
            Tcl_SetObjResult(interp, Tcl_NewStringObj("timeout or error", -1));
         }
         return TCL_OK;
      }
   }

   Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_wait: need 'received' or 'done' and 'timeout'", -1));
   return TCL_ERROR;
}

static int emc_update(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   char * DUMMY_VARIABLE objstr = NULL;

   if (objc == 1)
   {
      // no arg-- return status
      emc_ui_update_status(cd);
   }
   else if (objc == 2)
   {
      objstr = Tcl_GetStringFromObj(objv[1], 0);
      DBG("emc_update() val=%s\n", objstr);
   }

   return TCL_OK;
}

static int emc_operator_message(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   char operator_text_string[LINELEN];

   if (objc == 1)
   {
      // get any new string, it's saved in global operator_text_string[]
      if (emc_ui_get_operator_message(cd,operator_text_string, sizeof(operator_text_string)) != EMC_R_OK)
      {
         Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_operator_message: bad status from EMC", -1));
         return TCL_ERROR;
      }
      // put error on result list
      if (operator_text_string[0] == 0)
      {
         Tcl_SetObjResult(interp, Tcl_NewStringObj("ok", -1));
         operator_text_string[0] = 0;
      }
      else
      {
         Tcl_SetObjResult(interp, Tcl_NewStringObj(operator_text_string, -1));
         DBG("emc_operator_message() str=%s", operator_text_string);  /* no lf needed */
      }
      return TCL_OK;
   }

   Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_operator_message: need no args", -1));
   return TCL_ERROR;
}

static int emc_estop(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   const char *objstr = NULL;
   int new_estop, stat = TCL_OK;
   static int old_estop = -1;

   if (objc == 1)
   {
      if (emcStatus->task.state == EMC_TASK_STATE_ESTOP)
      {
         objstr = "on";
         new_estop = 1;
      }
      else
      {
         objstr = "off";
         new_estop = 0;
      }
      if (new_estop != old_estop)
      {
         DBG("emc_estop() QUERY result=%s\n", objstr);
         old_estop = new_estop;
      }
      Tcl_SetObjResult(interp, Tcl_NewStringObj(objstr, -1));
   }
   else if (objc == 2)
   {
      objstr = Tcl_GetStringFromObj(objv[1], 0);
      DBG("emc_estop() WRITE val=%s\n", objstr);
      if (!strcmp(objstr, "on"))
      {
         emc_ui_estop(cd);
      }
      else if (!strcmp(objstr, "off"))
      {
         emc_ui_estop_reset(cd);
      }
      old_estop = -1;
   }
   else
   {
      BUG("emc_estop() error objc=%d val=%s\n", objc, objstr);
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_estop: need 'on', 'off', or no args", -1));
      stat = TCL_ERROR;
   }
   return stat;
}

static int emc_machine(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   const char *objstr = NULL;
   int new_state;
   static int old_state = -1;

   if (objc == 1)
   {
      if (emcStatus->task.state == EMC_TASK_STATE_ON)
      {
         objstr = "on";
         new_state = 1;
      }
      else
      {
         objstr = "off";
         new_state = 0;
      }
      if (new_state != old_state)
      {
         DBG("emc_machine() QUERY result=%s\n", objstr);
         old_state = new_state;
      }
      Tcl_SetObjResult(interp, Tcl_NewStringObj(objstr, -1));
      return TCL_OK;
   }

   if (objc == 2)
   {
      objstr = Tcl_GetStringFromObj(objv[1], 0);
      DBG("emc_machine() WRITE val=%s\n", objstr);
      old_state = -1;
      if (!strcmp(objstr, "on"))
      {
         emc_ui_machine_on(cd);
         return TCL_OK;
      }
      if (!strcmp(objstr, "off"))
      {
         emc_ui_machine_off(cd);
         return TCL_OK;
      }
   }

   Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_machine: need 'on', 'off', or no args", -1));
   return TCL_ERROR;
}

static int emc_mode(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   const char *objstr;
   static int old_mode = -1;

   if (objc == 1)
   {
      switch (emcStatus->task.mode)
      {
      case EMC_TASK_MODE_MANUAL:
         objstr = "manual";
         break;
      case EMC_TASK_MODE_AUTO:
         objstr = "auto";
         break;
      case EMC_TASK_MODE_MDI:
         objstr = "mdi";
         break;
      default:
         objstr = "?";
         break;
      }
      if (old_mode != emcStatus->task.mode)
      {
         old_mode = emcStatus->task.mode;
         DBG("emc_mode() QUERY val=%s\n", objstr);
      }
      Tcl_SetObjResult(interp, Tcl_NewStringObj(objstr, -1));
      return TCL_OK;
   }

   if (objc == 2)
   {
      objstr = Tcl_GetStringFromObj(objv[1], 0);
      DBG("emc_mode() WRITE val=%s\n", objstr);
      old_mode = -1;
      if (!strcmp(objstr, "manual"))
      {
         emc_ui_manual_mode(cd);
         return TCL_OK;
      }
      if (!strcmp(objstr, "auto"))
      {
         emc_ui_auto_mode(cd);
         return TCL_OK;
      }
      if (!strcmp(objstr, "mdi"))
      {
         emc_ui_mdi_mode(cd);
         return TCL_OK;
      }
   }

   Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_mode: need 'manual', 'auto', 'mdi', or no args", -1));
   return TCL_ERROR;
}

static int emc_mist(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   char *objstr;

   if (objc == 1)
   {
      if (emcStatus->io.coolant.mist == 1)
      {
         Tcl_SetObjResult(interp, Tcl_NewStringObj("on", -1));
      }
      else
      {
         Tcl_SetObjResult(interp, Tcl_NewStringObj("off", -1));
      }
      return TCL_OK;
   }

   if (objc == 2)
   {
      objstr = Tcl_GetStringFromObj(objv[1], 0);
      DBG("emc_mist() WRITE val=%s\n", objstr);
      if (!strcmp(objstr, "on"))
      {
         emc_ui_mist_on(cd);
         return TCL_OK;
      }
      if (!strcmp(objstr, "off"))
      {
         emc_ui_mist_off(cd);
         return TCL_OK;
      }
   }

   Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_mist: need 'on', 'off', or no args", -1));
   return TCL_ERROR;
}

static int emc_flood(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   char *objstr;

   if (objc == 1)
   {
      if (emcStatus->io.coolant.flood == 1)
      {
         Tcl_SetObjResult(interp, Tcl_NewStringObj("on", -1));
      }
      else
      {
         Tcl_SetObjResult(interp, Tcl_NewStringObj("off", -1));
      }
      return TCL_OK;
   }

   if (objc == 2)
   {
      objstr = Tcl_GetStringFromObj(objv[1], 0);
      DBG("emc_flood() WRITE val=%s\n", objstr);
      if (!strcmp(objstr, "on"))
      {
         emc_ui_flood_on(cd);
         return TCL_OK;
      }
      if (!strcmp(objstr, "off"))
      {
         emc_ui_flood_off(cd);
         return TCL_OK;
      }
   }

   Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_flood: need 'on', 'off', or no args", -1));
   return TCL_ERROR;
}

static int emc_lube(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   char *objstr;

   if (objc == 1)
   {
      if (emcStatus->io.lube.on == 0)
      {
         Tcl_SetObjResult(interp, Tcl_NewStringObj("off", -1));
      }
      else
      {
         Tcl_SetObjResult(interp, Tcl_NewStringObj("on", -1));
      }
      return TCL_OK;
   }

   if (objc == 2)
   {
      objstr = Tcl_GetStringFromObj(objv[1], 0);
      DBG("emc_lube() WRITE val=%s\n", objstr);
      if (!strcmp(objstr, "on"))
      {
         emc_ui_lube_on(cd);
         return TCL_OK;
      }
      if (!strcmp(objstr, "off"))
      {
         emc_ui_lube_off(cd);
         return TCL_OK;
      }
   }

   Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_lube: need 'on', 'off', or no args", -1));
   return TCL_ERROR;
}

static int emc_lube_level(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   DBG("emc_lube_level()\n");
   if (objc == 1)
   {
      if (emcStatus->io.lube.level == 0)
      {
         Tcl_SetObjResult(interp, Tcl_NewStringObj("low", -1));
      }
      else
      {
         Tcl_SetObjResult(interp, Tcl_NewStringObj("ok", -1));
      }
      return TCL_OK;
   }

   Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_lube_level: need no args", -1));
   return TCL_ERROR;
}

static int emc_spindle(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   char *objstr;

   if (objc == 1)
   {
      if (emcStatus->motion.spindle.increasing > 0)
      {
         Tcl_SetObjResult(interp, Tcl_NewStringObj("increase", -1));
      }
      else if (emcStatus->motion.spindle.increasing < 0)
      {
         Tcl_SetObjResult(interp, Tcl_NewStringObj("decrease", -1));
      }
      else if (emcStatus->motion.spindle.direction > 0)
      {
         Tcl_SetObjResult(interp, Tcl_NewStringObj("forward", -1));
      }
      else if (emcStatus->motion.spindle.direction < 0)
      {
         Tcl_SetObjResult(interp, Tcl_NewStringObj("reverse", -1));
      }
      else
      {
         Tcl_SetObjResult(interp, Tcl_NewStringObj("off", -1));
      }
      return TCL_OK;
   }

   if (objc == 2)
   {
      objstr = Tcl_GetStringFromObj(objv[1], 0);
      DBG("emc_spindle() WRITE val=%s\n", objstr);
      if (!strcmp(objstr, "forward"))
      {
         emc_ui_spindle_forward(cd);
         return TCL_OK;
      }
      if (!strcmp(objstr, "reverse"))
      {
         emc_ui_spindle_reverse(cd);
         return TCL_OK;
      }
#if 0
      if (!strcmp(objstr, "increase"))
      {
         emc_ui_spindle_increase();
         return TCL_OK;
      }
      if (!strcmp(objstr, "decrease"))
      {
         emc_ui_spindle_decrease();
         return TCL_OK;
      }
      if (!strcmp(objstr, "constant"))
      {
         emc_ui_spindle_constant();
         return TCL_OK;
      }
#endif
      if (!strcmp(objstr, "off"))
      {
         emc_ui_spindle_off(cd);
         return TCL_OK;
      }
   }

   Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_spindle: need 'on', 'off', or no args", -1));
   return TCL_ERROR;
}

#if 0
static int emc_brake(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   char *objstr;

   if (objc == 1)
   {
      if (emcStatus->motion.spindle.brake == 1)
      {
         Tcl_SetObjResult(interp, Tcl_NewStringObj("on", -1));
      }
      else
      {
         Tcl_SetObjResult(interp, Tcl_NewStringObj("off", -1));
      }
      return TCL_OK;
   }

   if (objc == 2)
   {
      objstr = Tcl_GetStringFromObj(objv[1], 0);
      DBG("emc_brake() WRITE val=%s\n", objstr);
      if (!strcmp(objstr, "on"))
      {
         emc_ui_brake_engage();
         return TCL_OK;
      }
      if (!strcmp(objstr, "off"))
      {
         emc_ui_brake_release();
         return TCL_OK;
      }
   }

   Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_brake: need 'on', 'off', or no args", -1));
   return TCL_ERROR;
}
#endif

static int emc_tool(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   Tcl_Obj *toolobj;

   DBG9("emc_tool()\n");
   if (objc != 1)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_tool: need no args", -1));
      return TCL_ERROR;
   }

   toolobj = Tcl_NewIntObj(emcStatus->io.tool.toolInSpindle);

   Tcl_SetObjResult(interp, toolobj);
   return TCL_OK;
}

static int emc_tool_offset(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   Tcl_Obj *tlobj;
   int axis = 2;

   DBG9("emc_tool_offset()\n");
   if (objc > 2)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_tool_offset: need 0 or 1 args", -1));
      return TCL_ERROR;
   }

   if (objc == 2)
   {
      if (Tcl_GetIntFromObj(0, objv[1], &axis) != TCL_OK)
      {
         return TCL_ERROR;
      }
   }

   if (axis == 0)
   {
      tlobj = Tcl_NewDoubleObj(convertLinearUnits(emcStatus->task.toolOffset.tran.x));
   }
   else if (axis == 1)
   {
      tlobj = Tcl_NewDoubleObj(convertLinearUnits(emcStatus->task.toolOffset.tran.y));
   }
   else if (axis == 2)
   {
      tlobj = Tcl_NewDoubleObj(convertLinearUnits(emcStatus->task.toolOffset.tran.z));
   }
   else if (axis == 3)
   {
      tlobj = Tcl_NewDoubleObj(convertAngularUnits(emcStatus->task.toolOffset.a));
   }
   else if (axis == 4)
   {
      tlobj = Tcl_NewDoubleObj(convertAngularUnits(emcStatus->task.toolOffset.b));
   }
   else if (axis == 5)
   {
      tlobj = Tcl_NewDoubleObj(convertAngularUnits(emcStatus->task.toolOffset.c));
   }
   else if (axis == 6)
   {
      tlobj = Tcl_NewDoubleObj(convertLinearUnits(emcStatus->task.toolOffset.u));
   }
   else if (axis == 7)
   {
      tlobj = Tcl_NewDoubleObj(convertLinearUnits(emcStatus->task.toolOffset.v));
   }
   else if (axis == 8)
   {
      tlobj = Tcl_NewDoubleObj(convertLinearUnits(emcStatus->task.toolOffset.w));
   }
   else
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_tool_offset: axis must be from 0..8", -1));
      return TCL_ERROR;
   }
   Tcl_SetObjResult(interp, tlobj);
   return TCL_OK;
}

static int emc_load_tool_table(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   DBG("emc_load_tool_table()\n");
   if (objc != 2)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_load_tool_table: need file", -1));
      return TCL_ERROR;
   }

   if (emc_ui_tool_table(cd, (Tcl_GetStringFromObj(objv[1], 0))) != EMC_R_OK)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_load_tool_table: can't open file", -1));
      return TCL_OK;
   }

   return TCL_OK;
}

static int emc_set_tool_offset(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   int tool;
   double length;
   double diameter;

   DBG("emc_set_tool_offset()\n");
   if (objc != 4)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_set_tool_offset: need <tool> <length> <diameter>", -1));
      return TCL_ERROR;
   }

   if (Tcl_GetIntFromObj(0, objv[1], &tool) != TCL_OK)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_set_tool_offset: need tool as integer, 0..", -1));
      return TCL_ERROR;
   }
   if (Tcl_GetDoubleFromObj(0, objv[2], &length) != TCL_OK)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_set_tool_offset: need length as real number", -1));
      return TCL_ERROR;
   }
   if (Tcl_GetDoubleFromObj(0, objv[3], &diameter) != TCL_OK)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_set_tool_offset: need diameter as real number", -1));
      return TCL_ERROR;
   }

   if (emc_ui_tool_offset(cd, tool, length, diameter) != EMC_R_OK)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_set_tool_offset: can't set it", -1));
      return TCL_OK;
   }

   return TCL_OK;
}

static int emc_abs_cmd_pos(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   int axis;
   Tcl_Obj *posobj;

//    DBG("emc_abs_cmd_pos()\n");
   if (objc != 2)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_abs_cmd_pos: need exactly 1 non-negative integer", -1));
      return TCL_ERROR;
   }

   if (Tcl_GetIntFromObj(0, objv[1], &axis) == TCL_OK)
   {
      if (axis == 0)
      {
         posobj = Tcl_NewDoubleObj(convertLinearUnits(emcStatus->motion.traj.position.tran.x));
      }
      else if (axis == 1)
      {
         posobj = Tcl_NewDoubleObj(convertLinearUnits(emcStatus->motion.traj.position.tran.y));
      }
      else if (axis == 2)
      {
         posobj = Tcl_NewDoubleObj(convertLinearUnits(emcStatus->motion.traj.position.tran.z));
      }
      else
      {
         if (axis == 3)
         {
            posobj = Tcl_NewDoubleObj(convertAngularUnits(emcStatus->motion.traj.position.a));
         }
         else if (axis == 4)
         {
            posobj = Tcl_NewDoubleObj(convertAngularUnits(emcStatus->motion.traj.position.b));
         }
         else if (axis == 5)
         {
            posobj = Tcl_NewDoubleObj(convertAngularUnits(emcStatus->motion.traj.position.c));
         }
         else if (axis == 6)
         {
            posobj = Tcl_NewDoubleObj(convertLinearUnits(emcStatus->motion.traj.position.u));
         }
         else if (axis == 7)
         {
            posobj = Tcl_NewDoubleObj(convertLinearUnits(emcStatus->motion.traj.position.v));
         }
         else if (axis == 8)
         {
            posobj = Tcl_NewDoubleObj(convertLinearUnits(emcStatus->motion.traj.position.w));
         }
         else
         {
            posobj = Tcl_NewDoubleObj(0.0);
         }
      }
   }
   else
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_abs_cmd_pos: bad integer argument", -1));
      return TCL_ERROR;
   }

   Tcl_SetObjResult(interp, posobj);
   return TCL_OK;
}       /* emc_abs_cmd_pos() */

static int emc_abs_act_pos(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   int axis;
   Tcl_Obj *posobj;

//    DBG("emc_abs_act_pos()\n");
   if (objc != 2)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_abs_act_pos: need exactly 1 non-negative integer", -1));
      return TCL_ERROR;
   }

   if (Tcl_GetIntFromObj(0, objv[1], &axis) == TCL_OK)
   {
      if (axis == 0)
      {
         posobj = Tcl_NewDoubleObj(convertLinearUnits(emcStatus->motion.traj.actualPosition.tran.x));
      }
      else if (axis == 1)
      {
         posobj = Tcl_NewDoubleObj(convertLinearUnits(emcStatus->motion.traj.actualPosition.tran.y));
      }
      else if (axis == 2)
      {
         posobj = Tcl_NewDoubleObj(convertLinearUnits(emcStatus->motion.traj.actualPosition.tran.z));
      }
      else
      {
         if (axis == 3)
         {
            posobj = Tcl_NewDoubleObj(convertAngularUnits(emcStatus->motion.traj.actualPosition.a));
         }
         else if (axis == 4)
         {
            posobj = Tcl_NewDoubleObj(convertAngularUnits(emcStatus->motion.traj.actualPosition.b));
         }
         else if (axis == 5)
         {
            posobj = Tcl_NewDoubleObj(convertAngularUnits(emcStatus->motion.traj.actualPosition.c));
         }
         else if (axis == 6)
         {
            posobj = Tcl_NewDoubleObj(convertLinearUnits(emcStatus->motion.traj.actualPosition.u));
         }
         else if (axis == 7)
         {
            posobj = Tcl_NewDoubleObj(convertLinearUnits(emcStatus->motion.traj.actualPosition.v));
         }
         else if (axis == 8)
         {
            posobj = Tcl_NewDoubleObj(convertLinearUnits(emcStatus->motion.traj.actualPosition.w));
         }
         else
         {
            posobj = Tcl_NewDoubleObj(0.0);
         }
      }
   }
   else
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_abs_act_pos: bad integer argument", -1));
      return TCL_ERROR;
   }

   Tcl_SetObjResult(interp, posobj);
   return TCL_OK;
}       /* emc_abs_act_pos() */

static int emc_rel_cmd_pos(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   int axis;
   Tcl_Obj *posobj;

//    DBG("emc_rel_cmd_pos()\n");
   if (objc != 2)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_rel_cmd_pos: need exactly 1 non-negative integer", -1));
      return TCL_ERROR;
   }

   if (Tcl_GetIntFromObj(0, objv[1], &axis) == TCL_OK)
   {
      if (axis == 0)
      {
         posobj =
            Tcl_NewDoubleObj(convertLinearUnits(emcStatus->motion.traj.position.tran.x - emcStatus->task.origin.tran.x - emcStatus->task.toolOffset.tran.x));
      }
      else if (axis == 1)
      {
         posobj =
            Tcl_NewDoubleObj(convertLinearUnits(emcStatus->motion.traj.position.tran.y - emcStatus->task.origin.tran.y - emcStatus->task.toolOffset.tran.y));
      }
      else if (axis == 2)
      {
         posobj =
            Tcl_NewDoubleObj(convertLinearUnits(emcStatus->motion.traj.position.tran.z - emcStatus->task.origin.tran.z - emcStatus->task.toolOffset.tran.z));
      }
      else
      {
         if (axis == 3)
         {
            posobj = Tcl_NewDoubleObj(convertAngularUnits(emcStatus->motion.traj.position.a - emcStatus->task.origin.a - emcStatus->task.toolOffset.a));
         }
         else if (axis == 4)
         {
            posobj = Tcl_NewDoubleObj(convertAngularUnits(emcStatus->motion.traj.position.b - emcStatus->task.origin.b - emcStatus->task.toolOffset.b));
         }
         else if (axis == 5)
         {
            posobj = Tcl_NewDoubleObj(convertAngularUnits(emcStatus->motion.traj.position.c - emcStatus->task.origin.c - emcStatus->task.toolOffset.c));
         }
         else if (axis == 6)
         {
            posobj = Tcl_NewDoubleObj(convertLinearUnits(emcStatus->motion.traj.position.u - emcStatus->task.origin.u - emcStatus->task.toolOffset.u));
         }
         else if (axis == 7)
         {
            posobj = Tcl_NewDoubleObj(convertLinearUnits(emcStatus->motion.traj.position.v - emcStatus->task.origin.v - emcStatus->task.toolOffset.v));
         }
         else if (axis == 8)
         {
            posobj = Tcl_NewDoubleObj(convertLinearUnits(emcStatus->motion.traj.position.w - emcStatus->task.origin.w - emcStatus->task.toolOffset.w));
         }
         else
         {
            posobj = Tcl_NewDoubleObj(0.0);
         }
      }
   }
   else
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_rel_cmd_pos: bad integer argument", -1));
      return TCL_ERROR;
   }

   Tcl_SetObjResult(interp, posobj);
   return TCL_OK;
}       /* emc_rel_cmd_pos() */

static int emc_rel_act_pos(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   int axis;
   Tcl_Obj *posobj;

//    DBG("emc_rel_act_pos()\n");
   if (objc != 2)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_rel_act_pos: need exactly 1 non-negative integer", -1));
      return TCL_ERROR;
   }

   if (Tcl_GetIntFromObj(0, objv[1], &axis) == TCL_OK)
   {
      if (axis == 0)
      {
         posobj =
            Tcl_NewDoubleObj(convertLinearUnits(emcStatus->motion.traj.actualPosition.tran.x -
                                                emcStatus->task.origin.tran.x - emcStatus->task.toolOffset.tran.x));
      }
      else if (axis == 1)
      {
         posobj =
            Tcl_NewDoubleObj(convertLinearUnits(emcStatus->motion.traj.actualPosition.tran.y -
                                                emcStatus->task.origin.tran.y - emcStatus->task.toolOffset.tran.y));
      }
      else if (axis == 2)
      {
         posobj =
            Tcl_NewDoubleObj(convertLinearUnits(emcStatus->motion.traj.actualPosition.tran.z -
                                                emcStatus->task.origin.tran.z - emcStatus->task.toolOffset.tran.z));
      }
      else
      {
         if (axis == 3)
         {
            posobj = Tcl_NewDoubleObj(convertAngularUnits(emcStatus->motion.traj.actualPosition.a - emcStatus->task.origin.a - emcStatus->task.toolOffset.a));
         }
         else if (axis == 4)
         {
            posobj = Tcl_NewDoubleObj(convertAngularUnits(emcStatus->motion.traj.actualPosition.b - emcStatus->task.origin.b - emcStatus->task.toolOffset.b));
         }
         else if (axis == 5)
         {
            posobj = Tcl_NewDoubleObj(convertAngularUnits(emcStatus->motion.traj.actualPosition.c - emcStatus->task.origin.c - emcStatus->task.toolOffset.c));
         }
         else if (axis == 6)
         {
            posobj = Tcl_NewDoubleObj(convertLinearUnits(emcStatus->motion.traj.actualPosition.u - emcStatus->task.origin.u - emcStatus->task.toolOffset.u));
         }
         else if (axis == 7)
         {
            posobj = Tcl_NewDoubleObj(convertLinearUnits(emcStatus->motion.traj.actualPosition.v - emcStatus->task.origin.v - emcStatus->task.toolOffset.v));
         }
         else if (axis == 8)
         {
            posobj = Tcl_NewDoubleObj(convertLinearUnits(emcStatus->motion.traj.actualPosition.w - emcStatus->task.origin.w - emcStatus->task.toolOffset.w));
         }
         else
         {
            posobj = Tcl_NewDoubleObj(0.0);
         }
      }
   }
   else
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_rel_act_pos: bad integer argument", -1));
      return TCL_ERROR;
   }

   Tcl_SetObjResult(interp, posobj);
   return TCL_OK;
}       /* emc_rel_act_pose() */

static int emc_joint_pos(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   int axis;
   Tcl_Obj *posobj;

   DBG("emc_joint_pos()\n");
   if (objc != 2)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_joint_pos: need exactly 1 non-negative integer", -1));
      return TCL_ERROR;
   }

   if (Tcl_GetIntFromObj(0, objv[1], &axis) == TCL_OK)
   {
      posobj = Tcl_NewDoubleObj(emcStatus->motion.axis[axis].input);
   }
   else
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_joint_pos: bad integer argument", -1));
      return TCL_ERROR;
   }

   Tcl_SetObjResult(interp, posobj);
   return TCL_OK;
}

static int emc_pos_offset(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   char string[256];
   Tcl_Obj *posobj;

   DBG9("emc_pos_offset()\n");
   if (objc != 2)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_pos_offset: need exactly 1 non-negative integer", -1));
      return TCL_ERROR;
   }

   strcpy(string, Tcl_GetStringFromObj(objv[1], 0));

   if (string[0] == 'X')
   {
      posobj = Tcl_NewDoubleObj(convertLinearUnits(emcStatus->task.origin.tran.x));
   }
   else if (string[0] == 'Y')
   {
      posobj = Tcl_NewDoubleObj(convertLinearUnits(emcStatus->task.origin.tran.y));
   }
   else if (string[0] == 'Z')
   {
      posobj = Tcl_NewDoubleObj(convertLinearUnits(emcStatus->task.origin.tran.z));
   }
   else if (string[0] == 'A')
   {
      posobj = Tcl_NewDoubleObj(convertAngularUnits(emcStatus->task.origin.a));
   }
   else if (string[0] == 'B')
   {
      posobj = Tcl_NewDoubleObj(convertAngularUnits(emcStatus->task.origin.b));
   }
   else if (string[0] == 'C')
   {
      posobj = Tcl_NewDoubleObj(convertAngularUnits(emcStatus->task.origin.c));
   }
   else if (string[0] == 'U')
   {
      posobj = Tcl_NewDoubleObj(convertLinearUnits(emcStatus->task.origin.u));
   }
   else if (string[0] == 'V')
   {
      posobj = Tcl_NewDoubleObj(convertLinearUnits(emcStatus->task.origin.v));
   }
   else if (string[0] == 'W')
   {
      posobj = Tcl_NewDoubleObj(convertLinearUnits(emcStatus->task.origin.w));
   }
   else
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_pos_offset: bad integer argument", -1));
      return TCL_ERROR;
   }

   Tcl_SetObjResult(interp, posobj);
   return TCL_OK;
}

static int emc_joint_limit(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   int joint;

   DBG9("emc_joint_limit()\n");
   if (objc != 2)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_joint_limit: need exactly 1 non-negative integer", -1));
      return TCL_ERROR;
   }

   if (Tcl_GetIntFromObj(0, objv[1], &joint) == TCL_OK)
   {
      if (joint < 0 || joint >= EMC_AXIS_MAX)
      {
         Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_joint_limit: joint out of bounds", -1));
         return TCL_ERROR;
      }

      if (emcStatus->motion.axis[joint].minHardLimit)
      {
         Tcl_SetObjResult(interp, Tcl_NewStringObj("minhard", -1));
         return TCL_OK;
      }
      else if (emcStatus->motion.axis[joint].minSoftLimit)
      {
         Tcl_SetObjResult(interp, Tcl_NewStringObj("minsoft", -1));
         return TCL_OK;
      }
      else if (emcStatus->motion.axis[joint].maxSoftLimit)
      {
         Tcl_SetObjResult(interp, Tcl_NewStringObj("maxsoft", -1));
         return TCL_OK;
      }
      else if (emcStatus->motion.axis[joint].maxHardLimit)
      {
         Tcl_SetObjResult(interp, Tcl_NewStringObj("maxsoft", -1));
         return TCL_OK;
      }
      else
      {
         Tcl_SetObjResult(interp, Tcl_NewStringObj("ok", -1));
         return TCL_OK;
      }
   }

   Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_joint_limit: joint out of bounds", -1));
   return TCL_ERROR;
}

static int emc_joint_fault(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   int joint;

   DBG("emc_joint_fault()\n");
   if (objc != 2)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_joint_fault: need exactly 1 non-negative integer", -1));
      return TCL_ERROR;
   }

   if (Tcl_GetIntFromObj(0, objv[1], &joint) == TCL_OK)
   {
      if (joint < 0 || joint >= EMC_AXIS_MAX)
      {
         Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_joint_fault: joint out of bounds", -1));
         return TCL_ERROR;
      }

      if (emcStatus->motion.axis[joint].fault)
      {
         Tcl_SetObjResult(interp, Tcl_NewStringObj("fault", -1));
         return TCL_OK;
      }
      else
      {
         Tcl_SetObjResult(interp, Tcl_NewStringObj("ok", -1));
         return TCL_OK;
      }
   }

   Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_joint_fault: joint out of bounds", -1));
   return TCL_ERROR;
}

static int emc_override_limit(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   Tcl_Obj *obj;
   int on;

   DBG("emc_override_limit()\n");
   if (objc == 1)
   {
      // motion overrides all axes at same time, so just reference index 0
      obj = Tcl_NewIntObj(emcStatus->motion.axis[0].overrideLimits);
      Tcl_SetObjResult(interp, obj);
      return TCL_OK;
   }

   if (objc == 2)
   {
      if (Tcl_GetIntFromObj(0, objv[1], &on) == TCL_OK)
      {
         if (on)
         {
            if (emc_ui_override_limits(cd, 0) != EMC_R_OK)
            {
               Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_override_limit: can't send command", -1));
               return TCL_OK;
            }
         }
         else
         {
            if (emc_ui_override_limits(cd, -1) != EMC_R_OK)
            {
               Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_override_limit: can't send command", -1));
               return TCL_OK;
            }
         }
         return TCL_OK;
      }
      else
      {
         Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_override_limit: need 0 or 1", -1));
         return TCL_ERROR;
      }
   }

   Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_override_limit: need no args, 0 or 1", -1));
   return TCL_ERROR;
}

static int emc_joint_homed(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   int joint;

   DBG9("emc_joint_homed()\n");
   if (objc != 2)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_joint_homed: need exactly 1 non-negative integer", -1));
      return TCL_ERROR;
   }

   if (Tcl_GetIntFromObj(0, objv[1], &joint) == TCL_OK)
   {
      if (joint < 0 || joint >= EMC_AXIS_MAX)
      {
         Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_joint_homed: joint out of bounds", -1));
         return TCL_ERROR;
      }

      if (emcStatus->motion.axis[joint].homed)
      {
         Tcl_SetObjResult(interp, Tcl_NewStringObj("homed", -1));
         return TCL_OK;
      }
      else
      {
         Tcl_SetObjResult(interp, Tcl_NewStringObj("not", -1));
         return TCL_OK;
      }
   }

   Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_joint_homed: joint out of bounds", -1));
   return TCL_ERROR;
}

static int emc_mdi(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   char string[256];
   int t;

   if (objc < 2)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_mdi: need command", -1));
      return TCL_ERROR;
   }
   // bug-- check for string overflow
   strcpy(string, Tcl_GetStringFromObj(objv[1], 0));
   for (t = 2; t < objc; t++)
   {
      strcat(string, " ");
      strcat(string, Tcl_GetStringFromObj(objv[t], 0));
   }
   DBG("emc_mdi() WRITE val=%s\n", string);

   if (emc_ui_mdi_cmd(cd, string) != EMC_R_OK)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_mdi: error executing command", -1));
      return TCL_OK;
   }

   return TCL_OK;
}

static int emc_home(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   int axis;

   if (objc != 2)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_home: need axis", -1));
      return TCL_ERROR;
   }

   if (Tcl_GetIntFromObj(0, objv[1], &axis) == TCL_OK)
   {
      DBG("emc_home() axis=%d\n", axis);
      emc_ui_home(cd, axis);
      return TCL_OK;
   }

   Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_home: need axis as integer, 0..", -1));
   return TCL_ERROR;
}

static int emc_unhome(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   int axis;

   if (objc != 2)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_unhome: need axis", -1));
      return TCL_ERROR;
   }

   if (Tcl_GetIntFromObj(0, objv[1], &axis) == TCL_OK)
   {
      DBG("emc_unhome() axis=%d\n", axis);
      emc_ui_unhome(cd, axis);
      return TCL_OK;
   }

   Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_unhome: need axis as integer, 0..", -1));
   return TCL_ERROR;
}

static int emc_jog_stop(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   int axis;

   DBG("emc_jog_stop()\n");
   if (objc != 2)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_jog_stop: need axis", -1));
      return TCL_ERROR;
   }

   if (Tcl_GetIntFromObj(0, objv[1], &axis) != TCL_OK)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_jog_stop: need axis as integer, 0..", -1));
      return TCL_ERROR;
   }

   if (emc_ui_jog_stop(cd, axis) != EMC_R_OK)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_jog_stop: can't send jog stop msg", -1));
      return TCL_OK;
   }

   return TCL_OK;
}

#if 0
static int emc_jog(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   int axis;
   double speed;

   DBG("emc_jog()\n");
   if (objc != 3)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_jog: need axis and speed", -1));
      return TCL_ERROR;
   }

   if (Tcl_GetIntFromObj(0, objv[1], &axis) != TCL_OK)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_jog: need axis as integer, 0..", -1));
      return TCL_ERROR;
   }
   if (Tcl_GetDoubleFromObj(0, objv[2], &speed) != TCL_OK)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_jog: need speed as real number", -1));
      return TCL_ERROR;
   }

   if (emc_ui_jog_cont(axis, speed) != EMC_R_OK)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_jog: can't jog", -1));
      return TCL_OK;
   }

   return TCL_OK;
}
#endif

static int emc_jog_incr(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   int axis;
   double speed;
   double incr;

   if (objc != 4)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_jog_incr: need axis, speed, and increment", -1));
      return TCL_ERROR;
   }

   if (Tcl_GetIntFromObj(0, objv[1], &axis) != TCL_OK)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_jog_incr: need axis as integer, 0..", -1));
      return TCL_ERROR;
   }
   if (Tcl_GetDoubleFromObj(0, objv[2], &speed) != TCL_OK)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_jog_incr: need speed as real number", -1));
      return TCL_ERROR;
   }
   if (Tcl_GetDoubleFromObj(0, objv[3], &incr) != TCL_OK)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_jog_incr: need increment as real number", -1));
      return TCL_ERROR;
   }

   DBG("emc_jog_incr() axis=%d speed=%0.8f incr=%0.8f\n", axis, speed, incr);
   if (emc_ui_jog_incr(cd, axis, speed, incr) != EMC_R_OK)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_jog_incr: can't jog", -1));
      return TCL_OK;
   }

   return TCL_OK;
}

static int emc_feed_override(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   Tcl_Obj *feedobj;
   int percent;
   static unsigned int cnt;

   if (objc == 1)
   {
      percent = (int)(emcStatus->motion.traj.scale * 100.0 + 0.5);
      if (cnt++ < 4)
         DBG("emc_feed_override() READ percent=%d\n", percent);
      feedobj = Tcl_NewIntObj(percent);
      Tcl_SetObjResult(interp, feedobj);
      return TCL_OK;
   }

   if (objc != 2)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_feed_override: need percent", -1));
      return TCL_ERROR;
   }

   if (Tcl_GetIntFromObj(0, objv[1], &percent) == TCL_OK)
   {
      DBG("emc_feed_override() WRITE percent=%0.6f\n", percent / 100.0);
      emc_ui_feed_override(cd, ((double) percent) / 100.0);
      return TCL_OK;
   }

   Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_feed_override: need percent", -1));
   return TCL_ERROR;
}

static int emc_task_plan_init(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   DBG("emc_task_plan_init()\n");
   if (emc_ui_plan_init(cd) != EMC_R_OK)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_task_plan_init: can't init interpreter", -1));
      return TCL_OK;
   }

   return TCL_OK;
}

static int emc_open(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   char *gfile;

   if (objc != 2)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_open: need file", -1));
      return TCL_ERROR;
   }

   gfile = Tcl_GetStringFromObj(objv[1], 0);
   DBG("emc_open() file=%s\n", gfile);

   if (emc_ui_program_open(cd, gfile) != EMC_R_OK)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_open: can't open file", -1));
      return TCL_OK;
   }

   return TCL_OK;
}

static int emc_run(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   int line;

   if (objc == 1)
   {
      DBG("emc_run()\n");
      if (emc_ui_program_run(cd, 0) != EMC_R_OK)
      {
         Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_run: can't execute program", -1));
         return TCL_OK;
      }
   }

   if (objc == 2)
   {
      if (Tcl_GetIntFromObj(0, objv[1], &line) != TCL_OK)
      {
         Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_run: need integer start line", -1));
         return TCL_ERROR;
      }
      DBG("emc_run() from line=%d\n", line);
      if (emc_ui_program_run(cd, line) != EMC_R_OK)
      {
         Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_run: can't execute program", -1));
         return TCL_OK;
      }
   }

   return TCL_OK;
}

static int emc_pause(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   DBG("emc_pause()\n");
   if (emc_ui_program_pause(cd) != EMC_R_OK)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_pause: can't pause program", -1));
      return TCL_OK;
   }

   return TCL_OK;
}

static int emc_resume(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   DBG("emc_resume()\n");
   if (emc_ui_program_resume(cd) != EMC_R_OK)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_resume: can't resume program", -1));
      return TCL_OK;
   }

   return TCL_OK;
}

static int emc_step(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   DBG("emc_step()\n");
   if (emc_ui_program_step(cd) != EMC_R_OK)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_step: can't step program", -1));
      return TCL_OK;
   }

   return TCL_OK;
}

static int emc_abort(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   DBG("emc_abort()\n");
   if (emc_ui_abort(cd) != EMC_R_OK)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_abort: can't execute program", -1));
      return TCL_OK;
   }

   return TCL_OK;
}

static int emc_program(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   DBG("emc_program()\n");
   if (objc != 1)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_program: need no args", -1));
      return TCL_ERROR;
   }

   if (emcStatus->task.file[0] != 0)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj(emcStatus->task.file, -1));
      return TCL_OK;
   }

   Tcl_SetObjResult(interp, Tcl_NewStringObj("none", -1));
   return TCL_OK;
}

static int emc_program_status(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   DBG9("emc_program_status()\n");
   if (objc != 1)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_program_status: need no args", -1));
      return TCL_ERROR;
   }

   switch (emcStatus->task.interpState)
   {
   case EMC_TASK_INTERP_READING:
   case EMC_TASK_INTERP_WAITING:
      Tcl_SetObjResult(interp, Tcl_NewStringObj("running", -1));
      return TCL_OK;
      break;

   case EMC_TASK_INTERP_PAUSED:
      Tcl_SetObjResult(interp, Tcl_NewStringObj("paused", -1));
      return TCL_OK;
      break;

   default:
      Tcl_SetObjResult(interp, Tcl_NewStringObj("idle", -1));
      return TCL_OK;
   }

   Tcl_SetObjResult(interp, Tcl_NewStringObj("idle", -1));
   return TCL_OK;
}

static int emc_program_line(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   Tcl_Obj *lineobj;
   int programActiveLine = 0;

   if (objc != 1)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_program_line: need no args", -1));
      return TCL_ERROR;
   }

//   if (programStartLine < 0 || emcStatus->task.readLine < programStartLine)
//   {
   // controller is skipping lines
//      programActiveLine = emcStatus->task.readLine;
//   }
//   else
//   {   // controller is not skipping lines
   if (emcStatus->task.currentLine > 0)
   {
      if (emcStatus->task.motionLine > 0 && emcStatus->task.motionLine < emcStatus->task.currentLine)
      {
         // active line is the motion line, which lags
         programActiveLine = emcStatus->task.motionLine;
      }
      else
      {
         // active line is the current line-- no motion lag
         programActiveLine = emcStatus->task.currentLine;
      }
   }
   else
   {
      // no active line at all
      programActiveLine = 0;
   }
//   }                            // end of else controller is not skipping
   // lines

//    DBG("emc_program_line() QUERY line=%d\n", programActiveLine);

   lineobj = Tcl_NewIntObj(programActiveLine);

   Tcl_SetObjResult(interp, lineobj);
   return TCL_OK;
}

static int emc_program_codes(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   char codes_string[256];
   char string[256];
   int i, code;

   DBG9("emc_program_codes()\n");
   if (objc != 1)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_program_codes: need no args", -1));
      return TCL_ERROR;
   }

   // fill in the active G codes
   codes_string[0] = 0;
   for (i = 1; i < ACTIVE_G_CODES_; i++)
   {
      code = emcStatus->task.activeGCodes[i];
      if (code == -1)
      {
         continue;
      }
      if (code % 10)
      {
         sprintf(string, "G%.1f ", (double) code / 10.0);
      }
      else
      {
         sprintf(string, "G%d ", code / 10);
      }
      strcat(codes_string, string);
   }

   // fill in the active M codes, settings too
   for (i = 1; i < ACTIVE_M_CODES_; i++)
   {
      code = emcStatus->task.activeMCodes[i];
      if (code == -1)
      {
         continue;
      }
      sprintf(string, "M%d ", code);
      strcat(codes_string, string);
   }

   // fill in F and S codes also
   sprintf(string, "F%.0f ", emcStatus->task.activeSettings[1]);
   strcat(codes_string, string);
   sprintf(string, "S%.0f", fabs(emcStatus->task.activeSettings[2]));
   strcat(codes_string, string);

   Tcl_SetObjResult(interp, Tcl_NewStringObj(codes_string, -1));
   return TCL_OK;
}

static int emc_joint_type(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   int joint;

   DBG("emc_joint_type()\n");
   if (objc != 2)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_joint_type: need exactly 1 non-negative integer", -1));
      return TCL_ERROR;
   }

   if (Tcl_GetIntFromObj(0, objv[1], &joint) == TCL_OK)
   {
      if (joint < 0 || joint >= EMC_AXIS_MAX)
      {
         Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_joint_type: joint out of bounds", -1));
         return TCL_ERROR;
      }

      switch (emcStatus->motion.axis[joint].axisType)
      {
      case EMC_AXIS_LINEAR:
         Tcl_SetObjResult(interp, Tcl_NewStringObj("linear", -1));
         break;
      case EMC_AXIS_ANGULAR:
         Tcl_SetObjResult(interp, Tcl_NewStringObj("angular", -1));
         break;
      default:
         Tcl_SetObjResult(interp, Tcl_NewStringObj("custom", -1));
         break;
      }

      return TCL_OK;
   }

   Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_joint_type: invalid joint number", -1));
   return TCL_ERROR;
}

static int emc_joint_units(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   int joint;

   DBG("emc_joint_units()\n");
   if (objc != 2)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_joint_units: need exactly 1 non-negative integer", -1));
      return TCL_ERROR;
   }

   if (Tcl_GetIntFromObj(0, objv[1], &joint) == TCL_OK)
   {
      if (joint < 0 || joint >= EMC_AXIS_MAX)
      {
         Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_joint_units: joint out of bounds", -1));
         return TCL_ERROR;
      }

      switch (emcStatus->motion.axis[joint].axisType)
      {
      case EMC_AXIS_LINEAR:
         /* try mm */
         if (CLOSE(emcStatus->motion.axis[joint].units, 1.0, LINEAR_CLOSENESS))
         {
            Tcl_SetObjResult(interp, Tcl_NewStringObj("mm", -1));
            return TCL_OK;
         }
         /* now try inch */
         else if (CLOSE(emcStatus->motion.axis[joint].units, INCH_PER_MM, LINEAR_CLOSENESS))
         {
            Tcl_SetObjResult(interp, Tcl_NewStringObj("inch", -1));
            return TCL_OK;
         }
         /* now try cm */
         else if (CLOSE(emcStatus->motion.axis[joint].units, CM_PER_MM, LINEAR_CLOSENESS))
         {
            Tcl_SetObjResult(interp, Tcl_NewStringObj("cm", -1));
            return TCL_OK;
         }
         /* else it's custom */
         Tcl_SetObjResult(interp, Tcl_NewStringObj("custom", -1));
         return TCL_OK;
         break;

      case EMC_AXIS_ANGULAR:
         /* try degrees */
         if (CLOSE(emcStatus->motion.axis[joint].units, 1.0, ANGULAR_CLOSENESS))
         {
            Tcl_SetObjResult(interp, Tcl_NewStringObj("deg", -1));
            return TCL_OK;
         }
         /* now try radians */
         else if (CLOSE(emcStatus->motion.axis[joint].units, RAD_PER_DEG, ANGULAR_CLOSENESS))
         {
            Tcl_SetObjResult(interp, Tcl_NewStringObj("rad", -1));
            return TCL_OK;
         }
         /* now try grads */
         else if (CLOSE(emcStatus->motion.axis[joint].units, GRAD_PER_DEG, ANGULAR_CLOSENESS))
         {
            Tcl_SetObjResult(interp, Tcl_NewStringObj("grad", -1));
            return TCL_OK;
         }
         /* else it's custom */
         Tcl_SetObjResult(interp, Tcl_NewStringObj("custom", -1));
         return TCL_OK;
         break;

      default:
         Tcl_SetObjResult(interp, Tcl_NewStringObj("custom", -1));
         return TCL_OK;
         break;
      }
   }

   Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_joint_units: invalid joint number", -1));
   return TCL_ERROR;
}

static int emc_program_linear_units(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   const char *val;

   if (objc != 1)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_program_linear_units: need no args", -1));
      return TCL_ERROR;
   }

   switch (emcStatus->task.programUnits)
   {
   case CANON_UNITS_INCHES:
      val = "inch";
      break;
   case CANON_UNITS_MM:
      val = "mm";
      break;
   case CANON_UNITS_CM:
      val = "cm";
      break;
   default:
      val = "custom";
      break;
   }

   DBG("emc_program_linear_units() QUERY val=%s\n", val);
   Tcl_SetObjResult(interp, Tcl_NewStringObj(val, -1));
   return TCL_OK;
}

static int emc_program_angular_units(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   DBG("emc_program_angular_units()\n");
   if (objc != 1)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_program_angular_units: need no args", -1));
      return TCL_ERROR;
   }

   // currently the EMC doesn't have separate program angular units, so
   // these are simply "deg"
   Tcl_SetObjResult(interp, Tcl_NewStringObj("deg", -1));
   return TCL_OK;
}

static int emc_user_linear_units(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   DBG("emc_user_linear_units()\n");
   if (objc != 1)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_user_linear_units: need no args", -1));
      return TCL_ERROR;
   }

   /* try mm */
   if (CLOSE(emcStatus->motion.traj.linearUnits, 1.0, LINEAR_CLOSENESS))
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("mm", -1));
      return TCL_OK;
   }
   /* now try inch */
   else if (CLOSE(emcStatus->motion.traj.linearUnits, INCH_PER_MM, LINEAR_CLOSENESS))
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("inch", -1));
      return TCL_OK;
   }
   /* now try cm */
   else if (CLOSE(emcStatus->motion.traj.linearUnits, CM_PER_MM, LINEAR_CLOSENESS))
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("cm", -1));
      return TCL_OK;
   }

   /* else it's custom */
   Tcl_SetObjResult(interp, Tcl_NewStringObj("custom", -1));
   return TCL_OK;
}

static int emc_user_angular_units(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   DBG("emc_user_angular_units()\n");
   if (objc != 1)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_user_angular_units: need no args", -1));
      return TCL_ERROR;
   }

   /* try degrees */
   if (CLOSE(emcStatus->motion.traj.angularUnits, 1.0, ANGULAR_CLOSENESS))
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("deg", -1));
      return TCL_OK;
   }
   /* now try radians */
   else if (CLOSE(emcStatus->motion.traj.angularUnits, RAD_PER_DEG, ANGULAR_CLOSENESS))
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("rad", -1));
      return TCL_OK;
   }
   /* now try grads */
   else if (CLOSE(emcStatus->motion.traj.angularUnits, GRAD_PER_DEG, ANGULAR_CLOSENESS))
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("grad", -1));
      return TCL_OK;
   }

   /* else it's an abitrary number, so just return it */
   Tcl_SetObjResult(interp, Tcl_NewStringObj("custom", -1));
   return TCL_OK;
}

static int emc_display_linear_units(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   DBG9("emc_display_linear_units()\n");
   if (objc != 1)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_display_linear_units: need no args", -1));
      return TCL_ERROR;
   }

   switch (linearUnitConversion)
   {
   case LINEAR_UNITS_INCH:
      Tcl_SetObjResult(interp, Tcl_NewStringObj("inch", -1));
      break;
   case LINEAR_UNITS_MM:
      Tcl_SetObjResult(interp, Tcl_NewStringObj("mm", -1));
      break;
   case LINEAR_UNITS_CM:
      Tcl_SetObjResult(interp, Tcl_NewStringObj("cm", -1));
      break;
   case LINEAR_UNITS_AUTO:
      switch (emcStatus->task.programUnits)
      {
      case CANON_UNITS_MM:
         Tcl_SetObjResult(interp, Tcl_NewStringObj("(mm)", -1));
         break;
      case CANON_UNITS_INCHES:
         Tcl_SetObjResult(interp, Tcl_NewStringObj("(inch)", -1));
         break;
      case CANON_UNITS_CM:
         Tcl_SetObjResult(interp, Tcl_NewStringObj("(cm)", -1));
         break;
      }
      break;
   default:
      Tcl_SetObjResult(interp, Tcl_NewStringObj("custom", -1));
      break;
   }

   return TCL_OK;
}

static int emc_display_angular_units(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   DBG("emc_display_angular_units()\n");
   if (objc != 1)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_display_angular_units: need no args", -1));
      return TCL_ERROR;
   }

   switch (angularUnitConversion)
   {
   case ANGULAR_UNITS_DEG:
      Tcl_SetObjResult(interp, Tcl_NewStringObj("deg", -1));
      break;
   case ANGULAR_UNITS_RAD:
      Tcl_SetObjResult(interp, Tcl_NewStringObj("rad", -1));
      break;
   case ANGULAR_UNITS_GRAD:
      Tcl_SetObjResult(interp, Tcl_NewStringObj("grad", -1));
      break;
   case ANGULAR_UNITS_AUTO:
      Tcl_SetObjResult(interp, Tcl_NewStringObj("(deg)", -1));  /*! \todo FIXME-- always deg? */
      break;
   default:
      Tcl_SetObjResult(interp, Tcl_NewStringObj("custom", -1));
      break;
   }

   return TCL_OK;
}

static int emc_linear_unit_conversion(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   char *objstr;

   DBG("emc_linear_unit_conversion()\n");
   if (objc == 1)
   {
      // no arg-- return unit setting
      switch (linearUnitConversion)
      {
      case LINEAR_UNITS_INCH:
         Tcl_SetObjResult(interp, Tcl_NewStringObj("inch", -1));
         break;
      case LINEAR_UNITS_MM:
         Tcl_SetObjResult(interp, Tcl_NewStringObj("mm", -1));
         break;
      case LINEAR_UNITS_CM:
         Tcl_SetObjResult(interp, Tcl_NewStringObj("cm", -1));
         break;
      case LINEAR_UNITS_AUTO:
         Tcl_SetObjResult(interp, Tcl_NewStringObj("auto", -1));
         break;
      default:
         Tcl_SetObjResult(interp, Tcl_NewStringObj("custom", -1));
         break;
      }
      return TCL_OK;
   }

   if (objc == 2)
   {
      objstr = Tcl_GetStringFromObj(objv[1], 0);
      if (!strcmp(objstr, "inch"))
      {
         linearUnitConversion = LINEAR_UNITS_INCH;
         return TCL_OK;
      }
      if (!strcmp(objstr, "mm"))
      {
         linearUnitConversion = LINEAR_UNITS_MM;
         return TCL_OK;
      }
      if (!strcmp(objstr, "cm"))
      {
         linearUnitConversion = LINEAR_UNITS_CM;
         return TCL_OK;
      }
      if (!strcmp(objstr, "auto"))
      {
         linearUnitConversion = LINEAR_UNITS_AUTO;
         return TCL_OK;
      }
      if (!strcmp(objstr, "custom"))
      {
         linearUnitConversion = LINEAR_UNITS_CUSTOM;
         return TCL_OK;
      }
   }

   Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_linear_unit_conversion: need 'inch', 'mm', 'cm', 'auto', 'custom', or no args", -1));
   return TCL_ERROR;
}

static int emc_angular_unit_conversion(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   char *objstr;

   DBG("emc_angular_unit_conversion()\n");
   if (objc == 1)
   {
      // no arg-- return unit setting
      switch (angularUnitConversion)
      {
      case ANGULAR_UNITS_DEG:
         Tcl_SetObjResult(interp, Tcl_NewStringObj("deg", -1));
         break;
      case ANGULAR_UNITS_RAD:
         Tcl_SetObjResult(interp, Tcl_NewStringObj("rad", -1));
         break;
      case ANGULAR_UNITS_GRAD:
         Tcl_SetObjResult(interp, Tcl_NewStringObj("grad", -1));
         break;
      case ANGULAR_UNITS_AUTO:
         Tcl_SetObjResult(interp, Tcl_NewStringObj("auto", -1));
         break;
      default:
         Tcl_SetObjResult(interp, Tcl_NewStringObj("custom", -1));
         break;
      }
      return TCL_OK;
   }

   if (objc == 2)
   {
      objstr = Tcl_GetStringFromObj(objv[1], 0);
      if (!strcmp(objstr, "deg"))
      {
         angularUnitConversion = ANGULAR_UNITS_DEG;
         return TCL_OK;
      }
      if (!strcmp(objstr, "rad"))
      {
         angularUnitConversion = ANGULAR_UNITS_RAD;
         return TCL_OK;
      }
      if (!strcmp(objstr, "grad"))
      {
         angularUnitConversion = ANGULAR_UNITS_GRAD;
         return TCL_OK;
      }
      if (!strcmp(objstr, "auto"))
      {
         angularUnitConversion = ANGULAR_UNITS_AUTO;
         return TCL_OK;
      }
      if (!strcmp(objstr, "custom"))
      {
         angularUnitConversion = ANGULAR_UNITS_CUSTOM;
         return TCL_OK;
      }
   }

   Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_angular_unit_conversion: need 'deg', 'rad', 'grad', 'auto', 'custom', or no args", -1));
   return TCL_ERROR;
}

static int emc_task_heartbeat(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   Tcl_Obj *hbobj;

   DBG("emc_task_heartbeat()\n");
   if (objc != 1)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_task_heartbeat: need no args", -1));
      return TCL_ERROR;
   }

   hbobj = Tcl_NewIntObj(emcStatus->task.heartbeat);

   Tcl_SetObjResult(interp, hbobj);
   return TCL_OK;
}

static int emc_task_command(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   Tcl_Obj *commandobj;

   DBG("emc_task_command()\n");
   if (objc != 1)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_task_command: need no args", -1));
      return TCL_ERROR;
   }

   commandobj = Tcl_NewIntObj(emcStatus->task.command_type);

   Tcl_SetObjResult(interp, commandobj);
   return TCL_OK;
}

static int emc_task_command_number(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   Tcl_Obj *commandnumber;

   DBG("emc_task_command_number()\n");
   if (objc != 1)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_task_command_number: need no args", -1));
      return TCL_ERROR;
   }

   commandnumber = Tcl_NewIntObj(emcStatus->task.echo_serial_number);

   Tcl_SetObjResult(interp, commandnumber);
   return TCL_OK;
}

static int emc_task_command_status(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   Tcl_Obj *commandstatus;

   DBG("emc_task_command_status()\n");
   if (objc != 1)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_task_command_status: need no args", -1));
      return TCL_ERROR;
   }

   commandstatus = Tcl_NewIntObj(emcStatus->task.status);

   Tcl_SetObjResult(interp, commandstatus);
   return TCL_OK;
}

static int emc_io_command(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   Tcl_Obj *commandobj;

   DBG("emc_io_command()\n");
   if (objc != 1)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_io_command: need no args", -1));
      return TCL_ERROR;
   }

   commandobj = Tcl_NewIntObj(emcStatus->io.command_type);

   Tcl_SetObjResult(interp, commandobj);
   return TCL_OK;
}

static int emc_io_command_number(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   Tcl_Obj *commandnumber;

   DBG("emc_io_command_number()\n");
   if (objc != 1)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_io_command_number: need no args", -1));
      return TCL_ERROR;
   }

   commandnumber = Tcl_NewIntObj(emcStatus->io.echo_serial_number);

   Tcl_SetObjResult(interp, commandnumber);
   return TCL_OK;
}

static int emc_io_command_status(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   Tcl_Obj *commandstatus;

   DBG("emc_io_command_status()\n");
   if (objc != 1)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_io_command_status: need no args", -1));
      return TCL_ERROR;
   }

   commandstatus = Tcl_NewIntObj(emcStatus->io.status);

   Tcl_SetObjResult(interp, commandstatus);
   return TCL_OK;
}

static int emc_motion_heartbeat(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   Tcl_Obj *hbobj;

   DBG("emc_motion_heartbeat()\n");
   if (objc != 1)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_motion_heartbeat: need no args", -1));
      return TCL_ERROR;
   }

   hbobj = Tcl_NewIntObj(emcStatus->motion.heartbeat);

   Tcl_SetObjResult(interp, hbobj);
   return TCL_OK;
}

static int emc_motion_command_status(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   Tcl_Obj *commandstatus;

   DBG("emc_motion_command_status()\n");
   if (objc != 1)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_motion_command_status: need no args", -1));
      return TCL_ERROR;
   }

   commandstatus = Tcl_NewIntObj(emcStatus->motion.status);

   Tcl_SetObjResult(interp, commandstatus);
   return TCL_OK;
}

static int emc_axis_backlash(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   Tcl_Obj *valobj;
   int axis;
   double backlash;

   DBG("emc_axis_backlash()\n");
   // syntax is emc_axis_backlash <axis> {<backlash>}
   // if <backlash> is not specified, returns current value,
   // otherwise, sets backlash to specified value

   // check number of args supplied
   if ((objc < 2) || (objc > 3))
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_axis_backlash: need <axis> {<backlash>}", -1));
      return TCL_ERROR;
   }
   // get axis number
   if (Tcl_GetIntFromObj(0, objv[1], &axis) != TCL_OK || axis < 0 || axis >= EMC_AXIS_MAX)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_axis_backlash: need axis as integer, 0..EMC_AXIS_MAX-1", -1));
      return TCL_ERROR;
   }
   // test for get or set
   if (objc == 2)
   {
      // want to get present value
      valobj = Tcl_NewDoubleObj(emcStatus->motion.axis[axis].backlash);
      Tcl_SetObjResult(interp, valobj);
      return TCL_OK;
   }
   else
   {
      // want to set new value
      if (Tcl_GetDoubleFromObj(0, objv[2], &backlash) != TCL_OK)
      {
         Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_axis_backlash: need backlash as real number", -1));
         return TCL_ERROR;
      }
      // write it out
      emc_ui_axis_backlash(cd, axis, backlash);
      return TCL_OK;
   }
}

static int emc_teleop_enable(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   int enable;

   if (objc != 1)
   {
      if (Tcl_GetIntFromObj(0, objv[1], &enable) != TCL_OK)
      {
         Tcl_SetObjResult(interp, Tcl_NewStringObj("emc_teleop_enable: <enable> must be an integer", -1));
         return TCL_ERROR;
      }
      DBG("emc_teleop_enable() WRITE val=%d\n", enable);
      emc_ui_teleop_enable(cd, enable);
   }

   enable = emcStatus->motion.traj.mode == EMC_TRAJ_MODE_TELEOP;

   DBG("emc_teleop_enable() QUERY val=%d\n", enable);
   Tcl_SetObjResult(interp, Tcl_NewIntObj(enable));
   return TCL_OK;
}

static int emc_kinematics_type(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   DBG("emc_kinematics_type()\n");

   Tcl_SetObjResult(interp, Tcl_NewIntObj(emcStatus->motion.traj.kinematics_type));
   return TCL_OK;
}

// "int", as in "int 3.9" which returns 3
static int localint(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   double val;
   char resstring[80];

   if (objc != 2)
   {
      // need exactly one arg
      Tcl_SetObjResult(interp, Tcl_NewStringObj("wrong # args: should be \"int value\"", -1));
      return TCL_ERROR;
   }

   if (Tcl_GetDoubleFromObj(0, objv[1], &val) != TCL_OK)
   {
      resstring[0] = 0;
      strcat(resstring, "expected number but got \"");
      strncat(resstring, Tcl_GetStringFromObj(objv[1], 0), sizeof(resstring) - strlen(resstring) - 2);
      strcat(resstring, "\"");
      Tcl_SetObjResult(interp, Tcl_NewStringObj(resstring, -1));
      return TCL_ERROR;
   }

   Tcl_SetObjResult(interp, Tcl_NewIntObj((int) val));
   return TCL_OK;
}

// "round", as in "round 3.9" which returns 4
static int localround(ClientData cd, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
   double val;
   char resstring[80];

   if (objc != 2)
   {
      // need exactly one arg
      Tcl_SetObjResult(interp, Tcl_NewStringObj("wrong # args: should be \"round value\"", -1));
      return TCL_ERROR;
   }

   if (Tcl_GetDoubleFromObj(0, objv[1], &val) != TCL_OK)
   {
      resstring[0] = 0;
      strcat(resstring, "expected number but got \"");
      strncat(resstring, Tcl_GetStringFromObj(objv[1], 0), sizeof(resstring) - strlen(resstring) - 2);
      strcat(resstring, "\"");
      Tcl_SetObjResult(interp, Tcl_NewStringObj(resstring, -1));
      return TCL_ERROR;
   }

   Tcl_SetObjResult(interp, Tcl_NewIntObj(val < 0.0 ? (int) (val - 0.5) : (int) (val + 0.5)));
   return TCL_OK;
}

static void thisQuit(ClientData clientData)
{
   DBG("thisQuit()\n");
   emc_ui_close(clientData);
   Tcl_Exit(0);
}

static int emc_init(ClientData cd, Tcl_Interp * interp, int argc, const char **argv)
{
   void *hd;
   char ini[LINELEN];

   linearUnitConversion = LINEAR_UNITS_AUTO;
   angularUnitConversion = ANGULAR_UNITS_AUTO;

   ini[0]=0;

   /* Process any command line args. */
   if (argc > 1)
   {
      if (strncmp(argv[1], "-psn", 4) == 0)
      {
         /* ignore any commands, running from OSX application bundle. */ 
      }
      else if (strncmp(argv[1], "-ini", 4) == 0)
      {
         strncpy(ini, argv[2], sizeof(ini));
         ini[sizeof(ini)-1]=0;  /* force zero termination */
      }
   }

   if ((hd = emc_ui_open(ini)) == NULL)
   {
      Tcl_SetObjResult(interp, Tcl_NewStringObj("error in command line arguments\n", -1));
      return TCL_ERROR;
   }

   // update tcl's idea of the inifile name
   Tcl_SetVar(interp, "EMC_INIFILE", EMC_INIFILE, TCL_GLOBAL_ONLY);

   Tcl_CreateObjCommand(interp, "emc_plat", emc_plat, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_ini", emc_ini, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_wait", emc_wait, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_update", emc_update, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_operator_message", emc_operator_message, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_estop", emc_estop, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_machine", emc_machine, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_mode", emc_mode, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_mist", emc_mist, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_flood", emc_flood, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_lube", emc_lube, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_lube_level", emc_lube_level, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_spindle", emc_spindle, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
//   Tcl_CreateObjCommand(interp, "emc_brake", emc_brake, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_tool", emc_tool, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_tool_offset", emc_tool_offset, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_load_tool_table", emc_load_tool_table, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_set_tool_offset", emc_set_tool_offset, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_abs_cmd_pos", emc_abs_cmd_pos, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_abs_act_pos", emc_abs_act_pos, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_rel_cmd_pos", emc_rel_cmd_pos, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_rel_act_pos", emc_rel_act_pos, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_joint_pos", emc_joint_pos, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_pos_offset", emc_pos_offset, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_joint_limit", emc_joint_limit, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_joint_fault", emc_joint_fault, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_override_limit", emc_override_limit, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_joint_homed", emc_joint_homed, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_mdi", emc_mdi, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_home", emc_home, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_unhome", emc_unhome, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_jog_stop", emc_jog_stop, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
//   Tcl_CreateObjCommand(interp, "emc_jog", emc_jog, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_jog_incr", emc_jog_incr, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_feed_override", emc_feed_override, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_task_plan_init", emc_task_plan_init, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_open", emc_open, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_run", emc_run, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_pause", emc_pause, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_resume", emc_resume, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_step", emc_step, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_abort", emc_abort, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_program", emc_program, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_program_line", emc_program_line, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_program_status", emc_program_status, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_program_codes", emc_program_codes, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_joint_type", emc_joint_type, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_joint_units", emc_joint_units, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_program_linear_units", emc_program_linear_units, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_program_angular_units", emc_program_angular_units, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_user_linear_units", emc_user_linear_units, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_user_angular_units", emc_user_angular_units, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_display_linear_units", emc_display_linear_units, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_display_angular_units", emc_display_angular_units, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_linear_unit_conversion", emc_linear_unit_conversion, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_angular_unit_conversion", emc_angular_unit_conversion, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_task_heartbeat", emc_task_heartbeat, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_task_command", emc_task_command, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_task_command_number", emc_task_command_number, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_task_command_status", emc_task_command_status, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_io_command", emc_io_command, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_io_command_number", emc_io_command_number, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_io_command_status", emc_io_command_status, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_motion_heartbeat", emc_motion_heartbeat, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_motion_command_status", emc_motion_command_status, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_axis_backlash", emc_axis_backlash, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_teleop_enable", emc_teleop_enable, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "emc_kinematics_type", emc_kinematics_type, (ClientData)hd, (Tcl_CmdDeleteProc *) NULL);

   // attach our quit function to exit
   Tcl_CreateExitHandler(thisQuit, (ClientData)hd);

   Tcl_SetObjResult(interp, Tcl_NewStringObj("", -1));
   return TCL_OK;
}

//extern "C" int DLLEXPORT Rtstepperemc_Init(Tcl_Interp * interp) DLLEXPORT is ok with tcl 8.5 but not 8.4
extern "C" DLL_EXPORT int Rtstepperemc_Init(Tcl_Interp * interp)
{
   if (Tcl_InitStubs(interp, TCL_VERSION, 0) == NULL)
   {
      return TCL_ERROR;
   }

   /* 
    * Call Tcl_CreateCommand for application-specific commands, if
    * they weren't already created by the init procedures called above.
    */

   Tcl_CreateCommand(interp, "emc_init", emc_init, (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

   // provide builtins that may have been left out
   Tcl_CreateObjCommand(interp, "int", localint, (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);
   Tcl_CreateObjCommand(interp, "round", localround, (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

   return TCL_OK;
}
