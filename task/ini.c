/********************************************************************
* ini.c - INI file support for EMC2
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author:
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2004 All rights reserved.
*
*  (c) 2012 Copyright Eckler Software
*
* Last change:
********************************************************************/

#include <unistd.h>
#include <stdio.h>      // NULL
#include <stdlib.h>     // atol(), _itoa()
#include <string.h>     // strcmp()
#include <ctype.h>      // isdigit()
#include <math.h>       // M_PI
#include <sys/types.h>
#include <sys/stat.h>
#include "emc.h"
#include "ini.h"        // these decls
#include "bug.h"

static double _map_linear_units(const char *units)
{
   if (strncasecmp(units, "mm", 2) == 0)
      return 1.0;
   if (strncasecmp(units, "metric", 6) == 0)
      return 1.0;
   if (strncasecmp(units, "in", 2) == 0)
      return 1 / 25.4;
   if (strncasecmp(units, "inch", 4)  == 0)
      return 1 / 25.4;
   if (strncasecmp(units, "imperial", 8) == 0)
      return 1 / 25.4;
   return 0;
}

static double _map_angular_units(const char *units)
{
   if (strncasecmp(units, "deg", 3) == 0)
      return 1.0;
   if (strncasecmp(units, "degree", 6) == 0)
      return 1.0;
   if (strncasecmp(units, "grad", 4) == 0)
      return 0.9;
   if (strncasecmp(units, "gon", 3) == 0)
      return 0.9;
   if (strncasecmp(units, "rad", 3) == 0)
      return M_PI / 180;
   if (strncasecmp(units, "radian", 6) == 0)
      return M_PI / 180;
   return 0;
}

static enum EmcAxisType _map_axis_type(const char *type)
{
   if (strncasecmp(type, "linear", 6) == 0)
      return EMC_AXIS_LINEAR;
   if (strncasecmp(type, "angular", 7) == 0)
      return EMC_AXIS_ANGULAR;
   return EMC_AXIS_LINEAR;
}

/*
  iniAxis(int axis)

  Loads ini file params for axis, axis = 0, ...

  TYPE <LINEAR ANGULAR>        type of axis
  UNITS <float>                units per mm or deg
  MAX_VELOCITY <float>         max vel for axis
  MAX_ACCELERATION <float>     max accel for axis
  BACKLASH <float>             backlash
  INPUT_SCALE <float> <float>  scale, offset
  OUTPUT_SCALE <float> <float> scale, offset
  MIN_LIMIT <float>            minimum soft position limit
  MAX_LIMIT <float>            maximum soft position limit
  FERROR <float>               maximum following error, scaled to max vel
  MIN_FERROR <float>           minimum following error
  HOME <float>                 home position (where to go after home)
  HOME_FINAL_VEL <float>       speed to move from HOME_OFFSET to HOME location (at the end of homing)
  HOME_OFFSET <float>          home switch/index pulse location
  HOME_SEARCH_VEL <float>      homing speed, search phase
  HOME_LATCH_VEL <float>       homing speed, latch phase
  HOME_USE_INDEX <bool>        use index pulse when homing?
  HOME_IGNORE_LIMITS <bool>    ignore limit switches when homing?
  COMP_FILE <filename>         file of axis compensation points

  calls:

  emcAxisSetAxis(int axis, unsigned char axisType);
  emcAxisSetUnits(int axis, double units);
  emcAxisSetBacklash(int axis, double backlash);
  emcAxisSetInterpolationRate(int axis, int rate);
  emcAxisSetInputScale(int axis, double scale, double offset);
  emcAxisSetOutputScale(int axis, double scale, double offset);
  emcAxisSetMinPositionLimit(int axis, double limit);
  emcAxisSetMaxPositionLimit(int axis, double limit);
  emcAxisSetFerror(int axis, double ferror);
  emcAxisSetMinFerror(int axis, double ferror);
  emcAxisSetHomingParams(int axis, double home, double offset,
  double search_vel, double latch_vel, int use_index, int ignore_limits );
  emcAxisActivate(int axis);
  emcAxisDeactivate(int axis);
  emcAxisSetMaxVelocity(int axis, double vel);
  emcAxisSetMaxAcceleration(int axis, double acc);
  emcAxisLoadComp(int axis, const char * file);
  emcAxisLoadComp(int axis, const char * file);
  */
int iniAxis(int axis)
{
   enum EmcAxisType axisType;
   double units, backlash, offset, limit;
   double home, search_vel, latch_vel;
   double home_final_vel;       // moving from OFFSET to HOME
   int use_index, ignore_limits, is_shared, sequence, volatile_home;
   double maxVelocity, maxAcceleration, ferror, scale;
   int axes, pin, retval;
   char inistring[LINELEN];
   char section[32];

   DBG("iniAxis() axis=%d\n", axis);

   axes = 0;
   if (iniGetKeyValue("TRAJ", "AXES", inistring, sizeof(inistring)) > 0)
      axes = strtod(inistring, NULL);

   if (axis < 0 || axis >= axes)
   {
      // requested axis exceeds machine axes
      return EMC_R_ERROR;
   }

   sprintf(section, "AXIS_%d", axis);
   axisType = EMC_AXIS_LINEAR;       // default
   if (iniGetKeyValue(section, "TYPE", inistring, sizeof(inistring)) > 0)
      axisType = _map_axis_type(inistring);

   if ((retval = emcAxisSetAxis(axis, axisType)) != EMC_R_OK)
   {
      BUG("bad return from emcAxisSetAxis\n");
      return retval;
   }

   // set units
   if (axisType == EMC_AXIS_LINEAR)
   {
      units = emcmotStatus.traj.linearUnits;
      if (iniGetKeyValue(section, "UNITS", inistring, sizeof(inistring)) > 0)
         units = _map_linear_units(inistring);
   }
   else
   {
      units = emcmotStatus.traj.angularUnits;
      if (iniGetKeyValue(section, "UNITS", inistring, sizeof(inistring)) > 0)
         units = _map_angular_units(inistring);
   }
   if ((retval = emcAxisSetUnits(axis, units)) != EMC_R_OK)
   {
      BUG("bad return from emcAxisSetUnits\n");
      return retval;
   }

   // set backlash
   backlash = 0;     // default
   if (iniGetKeyValue(section, "BACKLASH", inistring, sizeof(inistring)) > 0)
      backlash = strtod(inistring, NULL);
   if ((retval = emcAxisSetBacklash(axis, backlash)) != EMC_R_OK)
   {
      BUG("bad return from emcAxisSetBacklash\n");
      return retval;
   }

   // set min position limit
   limit = -1e99;    // default
   if (iniGetKeyValue(section, "MIN_LIMIT", inistring, sizeof(inistring)) > 0)
      limit = strtod(inistring, NULL);
   if ((retval = emcAxisSetMinPositionLimit(axis, limit)) != EMC_R_OK)
   {
      BUG("bad return from emcAxisSetMinPositionLimit\n");
      return retval;
   }

   // set max position limit
   limit = 1e99;     // default
   if (iniGetKeyValue(section, "MAX_LIMIT", inistring, sizeof(inistring)) > 0)
      limit = strtod(inistring, NULL);
   if ((retval = emcAxisSetMaxPositionLimit(axis, limit)) != EMC_R_OK)
   {
      BUG("bad return from emcAxisSetMaxPositionLimit\n");
      return retval;
   }

   // set following error limit (at max speed)
   ferror = 1;       // default
   if (iniGetKeyValue(section, "FERROR", inistring, sizeof(inistring)) > 0)
      ferror = strtod(inistring, NULL);
   if ((retval = emcAxisSetFerror(axis, ferror)) != EMC_R_OK)
   {
      BUG("bad return from emcAxisSetFerror\n");
      return retval;
   }

   // do MIN_FERROR, if it's there. If not, use value of maxFerror above
   if (iniGetKeyValue(section, "MIN_FERROR", inistring, sizeof(inistring)) > 0)
      ferror = strtod(inistring, NULL);
   if ((retval = emcAxisSetMinFerror(axis, ferror)) != EMC_R_OK)
   {
      BUG("bad return from emcAxisSetMinFerror\n");
      return retval;
   }

   // set homing paramsters
   home = 0; // default
   if (iniGetKeyValue(section, "HOME", inistring, sizeof(inistring)) > 0)
      home = strtod(inistring, NULL);
   offset = 0;       // default
   if (iniGetKeyValue(section, "HOME_OFFSET", inistring, sizeof(inistring)) > 0)
      offset = strtod(inistring, NULL);
   search_vel = 0;   // default
   if (iniGetKeyValue(section, "HOME_SEARCH_VEL", inistring, sizeof(inistring)) > 0)
      search_vel = strtod(inistring, NULL);
   latch_vel = 0;    // default
   if (iniGetKeyValue(section, "HOME_LATCH_VEL", inistring, sizeof(inistring)) > 0)
      latch_vel = strtod(inistring, NULL);
   home_final_vel = -1;      // default (rapid)
   if (iniGetKeyValue(section, "HOME_FINAL_VEL", inistring, sizeof(inistring)) > 0)
      home_final_vel = strtod(inistring, NULL);
   is_shared = 0;        // default
   if (iniGetKeyValue(section, "HOME_IS_SHARED", inistring, sizeof(inistring)) > 0)
      is_shared = strtod(inistring, NULL);
   use_index = 0;        // default
   if (iniGetKeyValue(section, "HOME_USE_INDEX", inistring, sizeof(inistring)) > 0)
      use_index = strtod(inistring, NULL);
   ignore_limits = 0;    // default
   if (iniGetKeyValue(section, "HOME_IGNORE_LIMITS", inistring, sizeof(inistring)) > 0)
      ignore_limits = strtod(inistring, NULL);
   sequence = -1;    // default
   if (iniGetKeyValue(section, "HOME_SEQUENCE", inistring, sizeof(inistring)) > 0)
      sequence = strtod(inistring, NULL);
   volatile_home = 0;        // default
   if (iniGetKeyValue(section, "VOLATILE_HOME", inistring, sizeof(inistring)) > 0)
      volatile_home = strtod(inistring, NULL);

   if ((retval = emcAxisSetHomingParams(axis, home, offset, home_final_vel, search_vel, latch_vel,
                                        (int)use_index, (int)ignore_limits, (int)is_shared, sequence, volatile_home)) != EMC_R_OK)
   {
      BUG("bad return from emcAxisSetHomingParams\n");
      return retval;
   }

   // set maximum velocity
   maxVelocity = DEFAULT_AXIS_MAX_VELOCITY;
   if (iniGetKeyValue(section, "MAX_VELOCITY", inistring, sizeof(inistring)) > 0)
      maxVelocity = strtod(inistring, NULL);
   if ((retval = emcAxisSetMaxVelocity(axis, maxVelocity)) != EMC_R_OK)
   {
      BUG("bad return from emcAxisSetMaxVelocity\n");
      return retval;
   }

   // set max acceleration
   maxAcceleration = DEFAULT_AXIS_MAX_ACCELERATION;
   if (iniGetKeyValue(section, "MAX_ACCELERATION", inistring, sizeof(inistring)) > 0)
      maxAcceleration = strtod(inistring, NULL);
   if ((retval != emcAxisSetMaxAcceleration(axis, maxAcceleration)) != EMC_R_OK)
   {
      BUG("bad return from emcAxisSetMaxAcceleration\n");
      return retval;
   }

   // set input scale
   scale = 0;
   if (iniGetKeyValue(section, "INPUT_SCALE", inistring, sizeof(inistring)) > 0)
      scale = strtod(inistring, NULL);
   if ((retval = emcAxisSetInputScale(axis, scale)) != EMC_R_OK)
   {
      BUG("bad return from emcAxisSetInputScale\n");
      return retval;
   }

   // set step pin
   pin = 0;
   if (iniGetKeyValue(section, "STEP_PIN", inistring, sizeof(inistring)) > 0)
      pin = strtod(inistring, NULL);
   if ((retval = emcAxisSetStepPin(axis, pin)) != EMC_R_OK)
   {
      BUG("bad return from emcAxisSetStepPin\n");
      return retval;
   }

   // set direction pin
   pin = 0;
   if (iniGetKeyValue(section, "DIRECTION_PIN", inistring, sizeof(inistring)) > 0)
      pin = strtod(inistring, NULL);
   if ((retval = emcAxisSetDirectionPin(axis, pin)) != EMC_R_OK)
   {
      BUG("bad return from emcAxisSetDirectionPin\n");
      return retval;
   }

   // set step pen polarity
   pin = 0;
   if (iniGetKeyValue(section, "STEP_ACTIVE_HIGH", inistring, sizeof(inistring)) > 0)
      pin = strtod(inistring, NULL);
   if ((retval = emcAxisSetStepPolarity(axis, pin)) != EMC_R_OK)
   {
      BUG("bad return from emcAxisSetStepPolarity\n");
      return retval;
   }

   // set direction pin polarity
   pin = 0;
   if (iniGetKeyValue(section, "DIRECTION_ACTIVE_HIGH", inistring, sizeof(inistring)) > 0)
      pin = strtod(inistring, NULL);
   if ((retval = emcAxisSetDirectionPolarity(axis, pin)) != EMC_R_OK)
   {
      BUG("bad return from emcAxisSetDirectionPolarity\n");
      return retval;
   }

   // lastly, activate axis. Do this last so that the motion controller
   // won't flag errors midway during configuration
   return emcAxisActivate(axis);
}       /* iniAxis() */

/*
  iniTraj()

  Loads ini file params for traj

  AXES <int>                    number of axes in system
  LINEAR_UNITS <float>          units per mm
  ANGULAR_UNITS <float>         units per degree
  CYCLE_TIME <float>            cycle time for traj calculations
  DEFAULT_VELOCITY <float>      default velocity
  MAX_VELOCITY <float>          max velocity
  MAX_ACCELERATION <float>      max acceleration
  DEFAULT_ACCELERATION <float>  default acceleration
  HOME <float> ...              world coords of home, in X Y Z R P W

  calls:

  emcTrajSetAxes(int axes);
  emcTrajSetUnits(double linearUnits, double angularUnits);
  emcTrajSetCycleTime(double cycleTime);
  emcTrajSetVelocity(double vel);
  emcTrajSetAcceleration(double acc);
  emcTrajSetMaxVelocity(double vel);
  emcTrajSetMaxAcceleration(double acc);
  emcTrajSetHome(EmcPose home);
  */
int iniTraj(void)
{
   EmcLinearUnits linearUnits;
   EmcAngularUnits angularUnits;
   double vel, acc, d;
   int i, retval, axes, axismask;
   EmcPose homePose = { {0.0, 0.0, 0.0}, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
   char inistring[LINELEN];
   char *coord, *next;

   DBG("iniTraj()\n");

   axismask = 0;
   coord = NULL;
   if (iniGetKeyValue("TRAJ", "COORDINATES", inistring, sizeof(inistring)) > 0)
      coord = inistring;
   if (coord)
   {
      if (strchr(coord, 'x') || strchr(coord, 'X'))
         axismask |= 1;
      if (strchr(coord, 'y') || strchr(coord, 'Y'))
         axismask |= 2;
      if (strchr(coord, 'z') || strchr(coord, 'Z'))
         axismask |= 4;
      if (strchr(coord, 'a') || strchr(coord, 'A'))
         axismask |= 8;
      if (strchr(coord, 'b') || strchr(coord, 'B'))
         axismask |= 16;
      if (strchr(coord, 'c') || strchr(coord, 'C'))
         axismask |= 32;
      if (strchr(coord, 'u') || strchr(coord, 'U'))
         axismask |= 64;
      if (strchr(coord, 'v') || strchr(coord, 'V'))
         axismask |= 128;
      if (strchr(coord, 'w') || strchr(coord, 'W'))
         axismask |= 256;
   }
   else
   {
      axismask = 1 | 2 | 4;  // default: XYZ machine
   }

   axes = 0;
   if (iniGetKeyValue("TRAJ", "AXES", inistring, sizeof(inistring)) > 0)
      axes = strtod(inistring, NULL);
   if ((retval = emcTrajSetAxes(axes, axismask)) != EMC_R_OK)
   {
      BUG("bad return value from emcTrajSetAxes\n");
      return retval;
   }

   linearUnits = 0;
   if (iniGetKeyValue("TRAJ", "LINEAR_UNITS", inistring, sizeof(inistring)) > 0)
      linearUnits = _map_linear_units(inistring);

   angularUnits = 0;
   if (iniGetKeyValue("TRAJ", "ANGULAR_UNITS", inistring, sizeof(inistring)) > 0)
      angularUnits = _map_angular_units(inistring);

   if ((retval = emcTrajSetUnits(linearUnits, angularUnits)) != EMC_R_OK)
   {
      BUG("bad return value from emcTrajSetUnits\n");
      return retval;
   }

   vel = 1e99;       // by default, use AXIS limit
   if (iniGetKeyValue("TRAJ", "MAX_VELOCITY", inistring, sizeof(inistring)) > 0)
      vel = strtod(inistring, NULL);

   // set the corresponding global
   TRAJ_MAX_VELOCITY = vel;

   // and set dynamic value
   if ((retval = emcTrajSetMaxVelocity(vel)) != EMC_R_OK)
   {
      BUG("bad return value from emcTrajSetMaxVelocity\n");
      return retval;
   }

   vel = 1.0;
   if (iniGetKeyValue("TRAJ", "DEFAULT_VELOCITY", inistring, sizeof(inistring)) > 0)
      vel = strtod(inistring, NULL);

   // set the corresponding global
   TRAJ_DEFAULT_VELOCITY = vel;

   // and set dynamic value
   if ((retval = emcTrajSetVelocity(0, vel)) != EMC_R_OK)
   { //default velocity on startup 0
      BUG("bad return value from emcTrajSetVelocity\n");
      return retval;
   }

   acc = 1e99;       // let the axis values apply
   if (iniGetKeyValue("TRAJ", "MAX_ACCELERATION", inistring, sizeof(inistring)) > 0)
      acc = strtod(inistring, NULL);

   if ((retval = emcTrajSetMaxAcceleration(acc)) != EMC_R_OK)
   {
      BUG("bad return value from emcTrajSetMaxAcceleration\n");
      return retval;
   }

   acc = 1e99;       // let the axis values apply
   if (iniGetKeyValue("TRAJ", "DEFAULT_ACCELERATION", inistring, sizeof(inistring)) > 0)
      acc = strtod(inistring, NULL);

   if ((retval = emcTrajSetAcceleration(acc)) != EMC_R_OK)
   {
      BUG("bad return value from emcTrajSetAcceleration\n");
      return retval;
   }

   if (iniGetKeyValue("TRAJ", "HOME", inistring, sizeof(inistring)) > 0)
   {
      // found it, now interpret it according to coordinateMark[]
      d = strtod(inistring, &next);
      for (i=0; i<6; i++)
      {
         if (i==0)
            homePose.tran.x = d;
         else if (i == 1)
            homePose.tran.y = d;
         else if (i == 2)
            homePose.tran.z = d;
         else if (i == 3)
            homePose.a = d;
         else if (i == 4)
            homePose.b = d;
         else
            homePose.c = d;
         d = strtod(next, &next);
      }
   }

   if ((retval = emcTrajSetHome(homePose)) != EMC_R_OK)
   {
      BUG("bad return value from emcTrajSetHome\n");
      return retval;
   }

   return retval;
}  /* iniTraj() */

/* Loads ini file parameters for tool controller, from [EMCIO] section. */
int iniTool(void)
{
   int retval = EMC_R_OK;
   char inistring[LINELEN];

   DBG("iniTool()\n");

   // load tool values
   if (iniGetKeyValue("EMCIO", "TOOL_TABLE", inistring, sizeof(inistring)) > 0)
   {
      if ((retval = emcToolSetToolTableFile(inistring) != EMC_R_OK))
      {
         emcOperatorMessage(0, EMC_I18N("emcToolSetToolTableFile() failed"));
      }
   }

   // read the tool change positions
   if (iniGetKeyValue("EMCIO", "TOOL_CHANGE_POSITION", inistring, sizeof(inistring)) > 0)
   {
      /* found an entry */
      if (9 == sscanf(inistring, "%lf %lf %lf %lf %lf %lf %lf %lf %lf",
                      &TOOL_CHANGE_POSITION.tran.x,
                      &TOOL_CHANGE_POSITION.tran.y,
                      &TOOL_CHANGE_POSITION.tran.z,
                      &TOOL_CHANGE_POSITION.a,
                      &TOOL_CHANGE_POSITION.b, &TOOL_CHANGE_POSITION.c, &TOOL_CHANGE_POSITION.u, &TOOL_CHANGE_POSITION.v, &TOOL_CHANGE_POSITION.w))
      {
         HAVE_TOOL_CHANGE_POSITION = 1;
      }
      else if (6 == sscanf(inistring, "%lf %lf %lf %lf %lf %lf",
                           &TOOL_CHANGE_POSITION.tran.x,
                           &TOOL_CHANGE_POSITION.tran.y,
                           &TOOL_CHANGE_POSITION.tran.z, &TOOL_CHANGE_POSITION.a, &TOOL_CHANGE_POSITION.b, &TOOL_CHANGE_POSITION.c))
      {
         TOOL_CHANGE_POSITION.u = 0.0;
         TOOL_CHANGE_POSITION.v = 0.0;
         TOOL_CHANGE_POSITION.w = 0.0;
         HAVE_TOOL_CHANGE_POSITION = 1;
      }
      else if (3 == sscanf(inistring, "%lf %lf %lf", &TOOL_CHANGE_POSITION.tran.x, &TOOL_CHANGE_POSITION.tran.y, &TOOL_CHANGE_POSITION.tran.z))
      {
         /* read them OK */
         TOOL_CHANGE_POSITION.a = 0.0;
         TOOL_CHANGE_POSITION.b = 0.0;
         TOOL_CHANGE_POSITION.c = 0.0;
         TOOL_CHANGE_POSITION.u = 0.0;
         TOOL_CHANGE_POSITION.v = 0.0;
         TOOL_CHANGE_POSITION.w = 0.0;
         HAVE_TOOL_CHANGE_POSITION = 1;
      }
      else
      {
         /* bad format */
         emcOperatorMessage(0, EMC_I18N("bad format for TOOL_CHANGE_POSITION"));
         HAVE_TOOL_CHANGE_POSITION = 0;
         retval = EMC_R_ERROR;
      }
   }
   else
   {
      /* didn't find an entry */
      HAVE_TOOL_CHANGE_POSITION = 0;
   }

   if (iniGetKeyValue("EMCIO", "TOOL_HOLDER_CLEAR", inistring, sizeof(inistring)) > 0)
   {
      /* found an entry */
      if (3 == sscanf(inistring, "%lf %lf %lf", &TOOL_HOLDER_CLEAR.tran.x, &TOOL_HOLDER_CLEAR.tran.y, &TOOL_HOLDER_CLEAR.tran.z))
      {
         /* read them OK */
         TOOL_HOLDER_CLEAR.a = 0.0;     // not supporting ABC for now
         TOOL_HOLDER_CLEAR.b = 0.0;
         TOOL_HOLDER_CLEAR.c = 0.0;
         HAVE_TOOL_HOLDER_CLEAR = 1;
      }
      else
      {
         /* bad format */
         emcOperatorMessage(0, EMC_I18N("bad format for TOOL_HOLDER_CLEAR"));
         HAVE_TOOL_HOLDER_CLEAR = 0;
         retval = EMC_R_ERROR;
      }
   }
   else
   {
      /* didn't find an entry */
      HAVE_TOOL_HOLDER_CLEAR = 0;
   }

   return retval;
} /* iniTool() */

int iniTask(void)
{
   struct rtstepper_app_session *ps = &session.dongle;
   char inistring[LINELEN], buf[LINELEN];
   int i, r;

   DBG("iniTask()\n");

   /* First make sure we have a valid ini file. */
   if ((r = iniGetKeyValue("EMC", "MACHINE", inistring, sizeof(inistring))) == EMC_R_INVALID_INI_FILE)
      return r;

   /* Get rtstepper dongle specific configuration parameters. */
   ps->snum[0] = 0;
   if (iniGetKeyValue("TASK", "SERIAL_NUMBER", inistring, sizeof(inistring)) > 0)
   {
      strncpy(ps->snum, inistring, sizeof(ps->snum));
      ps->snum[sizeof(ps->snum) - 1] = 0;       /* force zero termination */
   }
   ps->input0_abort_enabled = 0;
   if (iniGetKeyValue("TASK", "INPUT0_ABORT", inistring, sizeof(inistring)) > 0)
      ps->input0_abort_enabled = strtod(inistring, NULL);
   ps->input1_abort_enabled = 0;
   if (iniGetKeyValue("TASK", "INPUT1_ABORT", inistring, sizeof(inistring)) > 0)
      ps->input1_abort_enabled = strtod(inistring, NULL);
   ps->input2_abort_enabled = 0;
   if (iniGetKeyValue("TASK", "INPUT2_ABORT", inistring, sizeof(inistring)) > 0)
      ps->input2_abort_enabled = strtod(inistring, NULL);

   /* Get any general purpose outputs that might be enabled. */   
   for (i=0; i<RTSTEPPER_GPO_MAX; i++)
   {
      ps->gpo_pin[i] = 0;
      sprintf(buf, "OUTPUT%d_PIN", i);
      if (iniGetKeyValue("TASK", buf, inistring, sizeof(inistring)) > 0)
         ps->gpo_pin[i] = strtod(inistring, NULL);
   }

   /* Get initial state of general purpose outputs. */
   for (i=0; i<RTSTEPPER_GPO_MAX; i++)
   {
      ps->gpo[i] = 0;
      sprintf(buf, "OUTPUT%d", i);
      if (iniGetKeyValue("TASK", buf, inistring, sizeof(inistring)) > 0)
         ps->gpo[i] = strtod(inistring, NULL);
   }

   if (iniGetKeyValue("TASK", "INTERP_MAX_LEN", inistring, sizeof(inistring)) > 0)
      EMC_TASK_INTERP_MAX_LEN = strtod(inistring, NULL);

   if (iniGetKeyValue("EMC", "RS274NGC_STARTUP_CODE", inistring, sizeof(inistring)) > 0)
   {
      // copy to global
      strncpy(RS274NGC_STARTUP_CODE, inistring, sizeof(RS274NGC_STARTUP_CODE));
      RS274NGC_STARTUP_CODE[sizeof(RS274NGC_STARTUP_CODE) - 1] = 0;  /* force zero termination */
   }
   else
   {
      if (iniGetKeyValue("RS274NGC", "RS274NGC_STARTUP_CODE", inistring, sizeof(inistring)) > 0)
      {
         // copy to global
         strncpy(RS274NGC_STARTUP_CODE, inistring, sizeof(RS274NGC_STARTUP_CODE));
         RS274NGC_STARTUP_CODE[sizeof(RS274NGC_STARTUP_CODE) - 1] = 0;  /* force zero termination */
      }
      else
      {
         // not found, use default
      }
   }

   if (iniGetKeyValue("DISPLAY", "PROGRAM_PREFIX", inistring, sizeof(inistring)) > 0)
   {
      /* Check for tilde expansion. */
      if (inistring[0] == '~' && inistring[1] == '/')
         snprintf(EMC_PROGRAM_PREFIX, sizeof(EMC_PROGRAM_PREFIX), "%s%s", USER_HOME_DIR, inistring+1);
      else
         snprintf(EMC_PROGRAM_PREFIX, sizeof(EMC_PROGRAM_PREFIX), "%s", inistring);
   }

   return EMC_R_OK;
}     /* iniTask() */

static char *_trim_tail(char *buf)
{
   int len;
   /* eat trailing white space and remove ']' */
   for (len = strlen(buf)-1; (buf[len] < ' ' || buf[len] == ']') && len >= 0; len--); 
   buf[len+1] = 0;
   return buf;
}

/* Get value for specified section and key from the ini file. */
int iniGetKeyValue(const char *section, const char *key, char *value, int value_size)
{
   char rcbuf[255];
   char new_section[64];
   FILE *inFile;
   enum EMC_RESULT stat = EMC_R_INVALID_INI_FILE;
   int i, j, len;

   value[0] = j = 0;

   if((inFile = fopen(EMC_INIFILE, "r")) == NULL) 
   {
      BUG("unable to open %s\n", EMC_INIFILE);
      goto bugout;
   } 

   len = strlen(key);
   new_section[0] = 0;
   stat = EMC_R_INVALID_INI_KEY;

   /* Read the config file */
   while ((fgets(rcbuf, sizeof(rcbuf), inFile) != NULL))
   {
      if (rcbuf[0] == '[')
      {
         /* Found new section. Remove [] brackets */
         strncpy(new_section, _trim_tail(rcbuf+1), sizeof(new_section));
         new_section[sizeof(new_section) -1] = 0;
      }
      else if ((strcasecmp(new_section, section) == 0) && (strncasecmp(rcbuf, key, len) == 0))
      {
         for (i=len; rcbuf[i] == ' ' && rcbuf[i]; i++);  /* eat white space before = */

         if (rcbuf[i] != '=')
            continue;  /* keep looking... */

         for (i++; rcbuf[i] == ' ' && rcbuf[i]; i++);  /* eat white space after = */
  
         for (j=0; rcbuf[i] >= ' ' && j < value_size; i++, j++) /* copy value(s) */
            value[j] = rcbuf[i];

         if (j < value_size)
            value[j] = 0;      /* zero terminate */
         else
            value[value_size -1] = 0;  /* force zero termination */

         stat = EMC_R_OK;
         break;  /* done */
      }
   }
        
bugout:        
   if (inFile != NULL)
      fclose(inFile);

   if (stat != EMC_R_OK)
   {
      BUG("unable to find %s %s in %s\n", section, key, EMC_INIFILE);
      return stat;
   }
   
   return j;   /* return length does not include zero termination */
}   /* iniGetKeyValue() */


