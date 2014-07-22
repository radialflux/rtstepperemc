/********************************************************************
* Description: emccanon.cc
*   Canonical definitions for 3-axis NC application
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author:
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2004 All rights reserved.
********************************************************************/
/*

  Notes:

  Units
  -----
  Values are stored internally as mm and degree units, e.g, program
  offsets, end point, tool length offset.  These are "internal
  units". "External units" are the units used by the EMC motion planner.
  All lengths and units output by the interpreter are converted to
  internal units here, using FROM_PROG_LEN,ANG, and then
  TO_EXT_LEN(),ANG are called to convert these to external units.

  Tool Length Offsets
  -------------------
  The interpreter does not subtract off tool length offsets. It calls
  USE_TOOL_LENGTH_OFFSETS(length), which we record here and apply to
  all appropriate values subsequently.
  */

#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <string.h>     // strncpy()
#include <ctype.h>      // isspace()
#include "emc.h"        // EMC NML
#include "canon.h"      // these decls
#include "interpl.h"    // interp_list
//#include "emcglb.h"           // TRAJ_MAX_VELOCITY

static int debug_velacc = 0;
static double css_maximum, css_numerator;

static const double tiny = 1e-7;
static double xy_rotation = 0.;

#ifndef MIN
#define MIN(a,b) ((a)<(b)?(a):(b))
#endif

#ifndef MIN3
#define MIN3(a,b,c) (MIN(MIN((a),(b)),(c)))
#endif

#ifndef MAX
#define MAX(a,b) ((a)>(b)?(a):(b))
#endif

#ifndef MAX3
#define MAX3(a,b,c) (MAX(MAX((a),(b)),(c)))
#endif

#ifndef MAX4
#define MAX4(a,b,c,d) (MAX(MAX((a),(b)),MAX((c),(d))))
#endif

#ifndef MAX9
#define MAX9(a,b,c,d,e,f,g,h,i) (MAX3((MAX3(a,b,c)),(MAX3(d,e,f)),(MAX3(g,h,i))))
#endif

/* macros for converting internal (mm/deg) units to external units */
#define TO_EXT_LEN(mm) ((mm) * GET_EXTERNAL_LENGTH_UNITS())
#define TO_EXT_ANG(deg) ((deg) * GET_EXTERNAL_ANGLE_UNITS())

/* macros for converting external units to internal (mm/deg) units */
#define FROM_EXT_LEN(ext) ((ext) / GET_EXTERNAL_LENGTH_UNITS())
#define FROM_EXT_ANG(ext) ((ext) / GET_EXTERNAL_ANGLE_UNITS())

/* macros for converting internal (mm/deg) units to program units */
#define TO_PROG_LEN(mm) ((mm) / (lengthUnits == CANON_UNITS_INCHES ? 25.4 : lengthUnits == CANON_UNITS_CM ? 10.0 : 1.0))
#define TO_PROG_ANG(deg) (deg)

/* macros for converting program units to internal (mm/deg) units */
#define FROM_PROG_LEN(prog) ((prog) * (lengthUnits == CANON_UNITS_INCHES ? 25.4 : lengthUnits == CANON_UNITS_CM ? 10.0 : 1.0))
#define FROM_PROG_ANG(prog) (prog)

/* Certain axes are periodic.  Hardcode this for now */
#define IS_PERIODIC(axisnum) \
    ((axisnum) == 3 || (axisnum) == 4 || (axisnum) == 5)

// this doesn't quite work yet: disable
#undef IS_PERIODIC
#define IS_PERIODIC(axisnum) (0)

#define AXIS_PERIOD(axisnum) (IS_PERIODIC(axisnum) ? 360 : 0)

static PM_QUATERNION quat(1, 0, 0, 0);

static void flush_segments(void);

/*
  These decls were from the old 3-axis canon.hh, and refer functions
  defined here that are used for convenience but no longer have decls
  in the 6-axis canon.hh. So, we declare them here now.
*/
extern void CANON_ERROR(const char *fmt, ...);

/*
  Origin offsets, length units, and active plane are all maintained
  here in this file. Controller runs in absolute mode, and does not
  have plane select concept.

  programOrigin is stored in mm always, and converted when set or read.
  When it's applied to positions, convert positions to mm units first
  and then add programOrigin.

  Units are then converted from mm to external units, as reported by
  the GET_EXTERNAL_LENGTH_UNITS() function.
  */
static CANON_POSITION programOrigin(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
static CANON_UNITS lengthUnits = CANON_UNITS_MM;
static CANON_PLANE activePlane = CANON_PLANE_XY;

static int feed_mode = 0;
static int synched = 0;

/* Tool length offset is saved here */
static EmcPose currentToolOffset;

static double offset_x(double x)
{
   return x + programOrigin.x + currentToolOffset.tran.x;
}

static double offset_y(double y)
{
   return y + programOrigin.y + currentToolOffset.tran.y;
}

static double offset_z(double z)
{
   return z + programOrigin.z + currentToolOffset.tran.z;
}

static double offset_a(double a)
{
   return a + programOrigin.a + currentToolOffset.a;
}

static double offset_b(double b)
{
   return b + programOrigin.b + currentToolOffset.b;
}

static double offset_c(double c)
{
   return c + programOrigin.c + currentToolOffset.c;
}

static double offset_u(double u)
{
   return u + programOrigin.u + currentToolOffset.u;
}

static double offset_v(double v)
{
   return v + programOrigin.v + currentToolOffset.v;
}

static double offset_w(double w)
{
   return w + programOrigin.w + currentToolOffset.w;
}

static double unoffset_x(double x)
{
   return x - programOrigin.x - currentToolOffset.tran.x;
}

static double unoffset_y(double y)
{
   return y - programOrigin.y - currentToolOffset.tran.y;
}

static double unoffset_z(double z)
{
   return z - programOrigin.z - currentToolOffset.tran.z;
}

static double unoffset_a(double a)
{
   return a - programOrigin.a - currentToolOffset.a;
}

static double unoffset_b(double b)
{
   return b - programOrigin.b - currentToolOffset.b;
}

static double unoffset_c(double c)
{
   return c - programOrigin.c - currentToolOffset.c;
}

static double unoffset_u(double u)
{
   return u - programOrigin.u - currentToolOffset.u;
}

static double unoffset_v(double v)
{
   return v - programOrigin.v - currentToolOffset.v;
}

static double unoffset_w(double w)
{
   return w - programOrigin.w - currentToolOffset.w;
}

#ifndef D2R
#define D2R(r) ((r)*M_PI/180.0)
#endif

static void rotate(double &x, double &y, double theta)
{
   double xx, yy;
   double t = D2R(theta);
   xx = x, yy = y;
   x = xx * cos(t) - yy * sin(t);
   y = xx * sin(t) + yy * cos(t);
}

static void rotate_and_offset_pos(double &x, double &y, double &z, double &a, double &b, double &c, double &u, double &v, double &w)
{
   rotate(x, y, xy_rotation);
   x = offset_x(x);
   y = offset_y(y);
   z = offset_z(z);
   a = offset_a(a);
   b = offset_b(b);
   c = offset_c(c);
   u = offset_u(u);
   v = offset_v(v);
   w = offset_w(w);
}

static CANON_POSITION unoffset_and_unrotate_pos(const CANON_POSITION pos)
{
   CANON_POSITION res;
   res.x = unoffset_x(pos.x);
   res.y = unoffset_y(pos.y);
   rotate(res.x, res.y, -xy_rotation);
   res.z = unoffset_z(pos.z);
   res.a = unoffset_a(pos.a);
   res.b = unoffset_b(pos.b);
   res.c = unoffset_c(pos.c);
   res.u = unoffset_u(pos.u);
   res.v = unoffset_v(pos.v);
   res.w = unoffset_w(pos.w);
   return res;
}

static CANON_POSITION unoffset_and_unrotate_pos(const EmcPose pos)
{
   CANON_POSITION res;
   res.x = unoffset_x(pos.tran.x);
   res.y = unoffset_y(pos.tran.y);
   rotate(res.x, res.y, -xy_rotation);
   res.z = unoffset_z(pos.tran.z);
   res.a = unoffset_a(pos.a);
   res.b = unoffset_b(pos.b);
   res.c = unoffset_c(pos.c);
   res.u = unoffset_u(pos.u);
   res.v = unoffset_v(pos.v);
   res.w = unoffset_w(pos.w);
   return res;
}

// for c in "xyzabcuvw": print "    %s = offset_%s(%s)" % (c,c,c)

static void from_prog(double &x, double &y, double &z, double &a, double &b, double &c, double &u, double &v, double &w)
{
   x = FROM_PROG_LEN(x);
   y = FROM_PROG_LEN(y);
   z = FROM_PROG_LEN(z);
   a = FROM_PROG_ANG(a);
   b = FROM_PROG_ANG(b);
   c = FROM_PROG_ANG(c);
   u = FROM_PROG_LEN(u);
   v = FROM_PROG_LEN(v);
   w = FROM_PROG_LEN(w);
}

#if 0   // not yet used; uncomment if you want it
static void to_ext(double &x, double &y, double &z, double &a, double &b, double &c, double &u, double &v, double &w)
{
   x = TO_EXT_LEN(x);
   y = TO_EXT_LEN(y);
   z = TO_EXT_LEN(z);
   a = TO_EXT_ANG(a);
   b = TO_EXT_ANG(b);
   c = TO_EXT_ANG(c);
   u = TO_EXT_LEN(u);
   v = TO_EXT_LEN(v);
   w = TO_EXT_LEN(w);
}
#endif

static EmcPose to_ext_pose(double x, double y, double z, double a, double b, double c, double u, double v, double w)
{
   EmcPose result;
   result.tran.x = TO_EXT_LEN(x);
   result.tran.y = TO_EXT_LEN(y);
   result.tran.z = TO_EXT_LEN(z);
   result.a = TO_EXT_ANG(a);
   result.b = TO_EXT_ANG(b);
   result.c = TO_EXT_ANG(c);
   result.u = TO_EXT_LEN(u);
   result.v = TO_EXT_LEN(v);
   result.w = TO_EXT_LEN(w);
   return result;
}

static void to_prog(CANON_POSITION & e)
{
   e.x = TO_PROG_LEN(e.x);
   e.y = TO_PROG_LEN(e.y);
   e.z = TO_PROG_LEN(e.z);
   e.a = TO_PROG_ANG(e.a);
   e.b = TO_PROG_ANG(e.b);
   e.c = TO_PROG_ANG(e.c);
   e.u = TO_PROG_LEN(e.u);
   e.v = TO_PROG_LEN(e.v);
   e.w = TO_PROG_LEN(e.w);
}

static int axis_valid(int n)
{
   return emcStatus->motion.traj.axis_mask & (1 << n);
}

/*
  canonEndPoint is the last programmed end point, stored in case it's
  needed for subsequent calculations. It's in absolute frame, mm units.

  note that when segments are queued for the naive cam detector that the
  canonEndPoint may not be the last programmed endpoint.  get_last_pos()
  retrieves the xyz position after the last of the queued segments.  these
  are also in absolute frame, mm units.
  */
static CANON_POSITION canonEndPoint;
static void canonUpdateEndPoint(double x, double y, double z, double a, double b, double c, double u, double v, double w)
{
   canonEndPoint.x = x;
   canonEndPoint.y = y;
   canonEndPoint.z = z;

   canonEndPoint.a = a;
   canonEndPoint.b = b;
   canonEndPoint.c = c;

   canonEndPoint.u = u;
   canonEndPoint.v = v;
   canonEndPoint.w = w;
}

/* External call to update the canon end point.
   Called by emctask during skipping of lines (run-from-line) */
void CANON_UPDATE_END_POINT(double x, double y, double z, double a, double b, double c, double u, double v, double w)
{
   canonUpdateEndPoint(FROM_PROG_LEN(x), FROM_PROG_LEN(y), FROM_PROG_LEN(z),
                       FROM_PROG_ANG(a), FROM_PROG_ANG(b), FROM_PROG_ANG(c), FROM_PROG_LEN(u), FROM_PROG_LEN(v), FROM_PROG_LEN(w));
}


/* motion control mode is used to signify blended v. stop-at-end moves.
   Set to 0 (invalid) at start, so first call will send command out */
static CANON_MOTION_MODE canonMotionMode = 0;

/* motion path-following tolerance is used to set the max path-following
   deviation during CANON_CONTINUOUS.
   If this param is 0, then it will behave as emc always did, allowing
   almost any deviation trying to keep speed up. */
static double canonMotionTolerance = 0.0;

static double canonNaivecamTolerance = 0.0;

/* Spindle speed is saved here */
static double spindleSpeed = 0.0;

/* Prepped tool is saved here */
static int preppedTool = 0;

/* optional program stop */
static bool optional_program_stop = ON; //set enabled by default (previous EMC behaviour)

/* optional block delete */
static bool block_delete = ON;  //set enabled by default (previous EMC behaviour)

/*
  Feed rate is saved here; values are in mm/sec or deg/sec.
  It will be initially set in INIT_CANON() below.
*/
static double currentLinearFeedRate = 0.0;
static double currentAngularFeedRate = 0.0;

/* Used to indicate whether the current move is linear, angular, or 
   a combination of both. */
   //AJ says: linear means axes XYZ move (lines or even circles)
   //         angular means axes ABC move
static int cartesian_move = 0;
static int angular_move = 0;

static double toExtVel(double vel)
{
   if (cartesian_move && !angular_move)
   {
      return TO_EXT_LEN(vel);
   }
   else if (!cartesian_move && angular_move)
   {
      return TO_EXT_ANG(vel);
   }
   else if (cartesian_move && angular_move)
   {
      return TO_EXT_LEN(vel);
   }
   else
   {    //seems this case was forgotten, neither linear, neither angular move (we are only sending vel)
      return TO_EXT_LEN(vel);
   }
}

static double toExtAcc(double acc)
{
   return toExtVel(acc);
}

static void send_origin_msg(void)
{
   flush_segments();

   /* append it to interp list so it gets updated at the right time, not at
      read-ahead time */
   emc_traj_set_origin_msg_t set_origin_msg = { {EMC_TRAJ_SET_ORIGIN_TYPE} };

   set_origin_msg.origin.tran.x = TO_EXT_LEN(programOrigin.x);
   set_origin_msg.origin.tran.y = TO_EXT_LEN(programOrigin.y);
   set_origin_msg.origin.tran.z = TO_EXT_LEN(programOrigin.z);

   set_origin_msg.origin.a = TO_EXT_ANG(programOrigin.a);
   set_origin_msg.origin.b = TO_EXT_ANG(programOrigin.b);
   set_origin_msg.origin.c = TO_EXT_ANG(programOrigin.c);

   set_origin_msg.origin.u = TO_EXT_LEN(programOrigin.u);
   set_origin_msg.origin.v = TO_EXT_LEN(programOrigin.v);
   set_origin_msg.origin.w = TO_EXT_LEN(programOrigin.w);

   if (css_maximum)
   {
      emc_spindle_speed_msg_t emc_spindle_speed_msg = { {EMC_SPINDLE_SPEED_TYPE}
      };
      emc_spindle_speed_msg.speed = css_maximum;
      emc_spindle_speed_msg.factor = css_numerator;
      emc_spindle_speed_msg.xoffset = TO_EXT_LEN(programOrigin.x + currentToolOffset.tran.x);
      interp_list.append((emc_command_msg_t *) & emc_spindle_speed_msg);
   }
   interp_list.append((emc_command_msg_t *) & set_origin_msg);
}

/* Representation */

void SET_XY_ROTATION(double t)
{
   emc_traj_set_rotation_msg_t sr = { {EMC_TRAJ_SET_ROTATION_TYPE} };
   sr.rotation = t;
   interp_list.append((emc_command_msg_t *) & sr);

   xy_rotation = t;
}

void SET_ORIGIN_OFFSETS(double x, double y, double z, double a, double b, double c, double u, double v, double w)
{
   /* convert to mm units */
   x = FROM_PROG_LEN(x);
   y = FROM_PROG_LEN(y);
   z = FROM_PROG_LEN(z);
   a = FROM_PROG_ANG(a);
   b = FROM_PROG_ANG(b);
   c = FROM_PROG_ANG(c);
   u = FROM_PROG_LEN(u);
   v = FROM_PROG_LEN(v);
   w = FROM_PROG_LEN(w);

   programOrigin.x = x;
   programOrigin.y = y;
   programOrigin.z = z;

   programOrigin.a = a;
   programOrigin.b = b;
   programOrigin.c = c;

   programOrigin.u = u;
   programOrigin.v = v;
   programOrigin.w = w;

   send_origin_msg();
}


void USE_LENGTH_UNITS(CANON_UNITS in_unit)
{
   lengthUnits = in_unit;

   emcStatus->task.programUnits = in_unit;
}

/* Free Space Motion */
void SET_TRAVERSE_RATE(double rate)
{
   // nothing need be done here
}

void SET_FEED_MODE(int mode)
{
   flush_segments();
   feed_mode = mode;
   if (feed_mode == 0)
      STOP_SPEED_FEED_SYNCH();
}

void SET_FEED_RATE(double rate)
{

   if (feed_mode)
   {
      START_SPEED_FEED_SYNCH(rate, 1);
      currentLinearFeedRate = rate;
   }
   else
   {
      /* convert from /min to /sec */
      rate /= 60.0;


      /* convert to traj units (mm & deg) if needed */
      double newLinearFeedRate = FROM_PROG_LEN(rate), newAngularFeedRate = FROM_PROG_ANG(rate);

      if (newLinearFeedRate != currentLinearFeedRate || newAngularFeedRate != currentAngularFeedRate)
         flush_segments();

      currentLinearFeedRate = newLinearFeedRate;
      currentAngularFeedRate = newAngularFeedRate;
   }
}

void SET_FEED_REFERENCE(CANON_FEED_REFERENCE reference)
{
   // nothing need be done here
}

double getStraightAcceleration(double x, double y, double z, double a, double b, double c, double u, double v, double w)
{
   double dx, dy, dz, du, dv, dw, da, db, dc;
   double tx, ty, tz, tu, tv, tw, ta, tb, tc, tmax;
   double acc, dtot;

   acc = 0.0;   // if a move to nowhere

   // Compute absolute travel distance for each axis:
   dx = fabs(x - canonEndPoint.x);
   dy = fabs(y - canonEndPoint.y);
   dz = fabs(z - canonEndPoint.z);
   da = fabs(a - canonEndPoint.a);
   db = fabs(b - canonEndPoint.b);
   dc = fabs(c - canonEndPoint.c);
   du = fabs(u - canonEndPoint.u);
   dv = fabs(v - canonEndPoint.v);
   dw = fabs(w - canonEndPoint.w);

   if (!axis_valid(0) || dx < tiny)
      dx = 0.0;
   if (!axis_valid(1) || dy < tiny)
      dy = 0.0;
   if (!axis_valid(2) || dz < tiny)
      dz = 0.0;
   if (!axis_valid(3) || da < tiny)
      da = 0.0;
   if (!axis_valid(4) || db < tiny)
      db = 0.0;
   if (!axis_valid(5) || dc < tiny)
      dc = 0.0;
   if (!axis_valid(6) || du < tiny)
      du = 0.0;
   if (!axis_valid(7) || dv < tiny)
      dv = 0.0;
   if (!axis_valid(8) || dw < tiny)
      dw = 0.0;

   if (debug_velacc)
      printf("getStraightAcceleration dx %g dy %g dz %g da %g db %g dc %g du %g dv %g dw %g ", dx, dy, dz, da, db, dc, du, dv, dw);

   // Figure out what kind of move we're making.  This is used to determine
   // the units of vel/acc.
   if (dx <= 0.0 && dy <= 0.0 && dz <= 0.0 && du <= 0.0 && dv <= 0.0 && dw <= 0.0)
   {
      cartesian_move = 0;
   }
   else
   {
      cartesian_move = 1;
   }
   if (da <= 0.0 && db <= 0.0 && dc <= 0.0)
   {
      angular_move = 0;
   }
   else
   {
      angular_move = 1;
   }

   // Pure linear move:
   if (cartesian_move && !angular_move)
   {
      tx = dx ? (dx / FROM_EXT_LEN(AXIS_MAX_ACCELERATION[0])) : 0.0;
      ty = dy ? (dy / FROM_EXT_LEN(AXIS_MAX_ACCELERATION[1])) : 0.0;
      tz = dz ? (dz / FROM_EXT_LEN(AXIS_MAX_ACCELERATION[2])) : 0.0;
      tu = du ? (du / FROM_EXT_LEN(AXIS_MAX_ACCELERATION[6])) : 0.0;
      tv = dv ? (dv / FROM_EXT_LEN(AXIS_MAX_ACCELERATION[7])) : 0.0;
      tw = dw ? (dw / FROM_EXT_LEN(AXIS_MAX_ACCELERATION[8])) : 0.0;
      tmax = MAX3(tx, ty, tz);
      tmax = MAX4(tu, tv, tw, tmax);

      if (dx || dy || dz)
         dtot = sqrt(dx * dx + dy * dy + dz * dz);
      else
         dtot = sqrt(du * du + dv * dv + dw * dw);

      if (tmax > 0.0)
      {
         acc = dtot / tmax;
      }
   }
   // Pure angular move:
   else if (!cartesian_move && angular_move)
   {
      ta = da ? (da / FROM_EXT_ANG(AXIS_MAX_ACCELERATION[3])) : 0.0;
      tb = db ? (db / FROM_EXT_ANG(AXIS_MAX_ACCELERATION[4])) : 0.0;
      tc = dc ? (dc / FROM_EXT_ANG(AXIS_MAX_ACCELERATION[5])) : 0.0;
      tmax = MAX3(ta, tb, tc);

      dtot = sqrt(da * da + db * db + dc * dc);
      if (tmax > 0.0)
      {
         acc = dtot / tmax;
      }
   }
   // Combination angular and linear move:
   else if (cartesian_move && angular_move)
   {
      tx = dx ? (dx / FROM_EXT_LEN(AXIS_MAX_ACCELERATION[0])) : 0.0;
      ty = dy ? (dy / FROM_EXT_LEN(AXIS_MAX_ACCELERATION[1])) : 0.0;
      tz = dz ? (dz / FROM_EXT_LEN(AXIS_MAX_ACCELERATION[2])) : 0.0;
      ta = da ? (da / FROM_EXT_ANG(AXIS_MAX_ACCELERATION[3])) : 0.0;
      tb = db ? (db / FROM_EXT_ANG(AXIS_MAX_ACCELERATION[4])) : 0.0;
      tc = dc ? (dc / FROM_EXT_ANG(AXIS_MAX_ACCELERATION[5])) : 0.0;
      tu = du ? (du / FROM_EXT_LEN(AXIS_MAX_ACCELERATION[6])) : 0.0;
      tv = dv ? (dv / FROM_EXT_LEN(AXIS_MAX_ACCELERATION[7])) : 0.0;
      tw = dw ? (dw / FROM_EXT_LEN(AXIS_MAX_ACCELERATION[8])) : 0.0;
      tmax = MAX9(tx, ty, tz, ta, tb, tc, tu, tv, tw);

/*  According to NIST IR6556 Section 2.1.2.5 Paragraph A
    a combnation move is handled like a linear move, except
    that the angular axes are allowed sufficient time to
    complete their motion coordinated with the motion of
    the linear axes.
*/
      if (dx || dy || dz)
         dtot = sqrt(dx * dx + dy * dy + dz * dz);
      else
         dtot = sqrt(du * du + dv * dv + dw * dw);

      if (tmax > 0.0)
      {
         acc = dtot / tmax;
      }
   }
   if (debug_velacc)
      printf("cartesian %d ang %d acc %g\n", cartesian_move, angular_move, acc);
   return acc;
}

double getStraightVelocity(double x, double y, double z, double a, double b, double c, double u, double v, double w)
{
   double dx, dy, dz, da, db, dc, du, dv, dw;
   double tx, ty, tz, ta, tb, tc, tu, tv, tw, tmax;
   double vel, dtot;

/* If we get a move to nowhere (!cartesian_move && !angular_move)
   we might as well go there at the currentLinearFeedRate...
*/
   vel = currentLinearFeedRate;

   // Compute absolute travel distance for each axis:
   dx = fabs(x - canonEndPoint.x);
   dy = fabs(y - canonEndPoint.y);
   dz = fabs(z - canonEndPoint.z);
   da = fabs(a - canonEndPoint.a);
   db = fabs(b - canonEndPoint.b);
   dc = fabs(c - canonEndPoint.c);
   du = fabs(u - canonEndPoint.u);
   dv = fabs(v - canonEndPoint.v);
   dw = fabs(w - canonEndPoint.w);

   if (!axis_valid(0) || dx < tiny)
      dx = 0.0;
   if (!axis_valid(1) || dy < tiny)
      dy = 0.0;
   if (!axis_valid(2) || dz < tiny)
      dz = 0.0;
   if (!axis_valid(3) || da < tiny)
      da = 0.0;
   if (!axis_valid(4) || db < tiny)
      db = 0.0;
   if (!axis_valid(5) || dc < tiny)
      dc = 0.0;
   if (!axis_valid(6) || du < tiny)
      du = 0.0;
   if (!axis_valid(7) || dv < tiny)
      dv = 0.0;
   if (!axis_valid(8) || dw < tiny)
      dw = 0.0;

   if (debug_velacc)
      printf("getStraightVelocity dx %g dy %g dz %g da %g db %g dc %g du %g dv %g dw %g ", dx, dy, dz, da, db, dc, du, dv, dw);

   // Figure out what kind of move we're making:
   if (dx <= 0.0 && dy <= 0.0 && dz <= 0.0 && du <= 0.0 && dv <= 0.0 && dw <= 0.0)
   {
      cartesian_move = 0;
   }
   else
   {
      cartesian_move = 1;
   }
   if (da <= 0.0 && db <= 0.0 && dc <= 0.0)
   {
      angular_move = 0;
   }
   else
   {
      angular_move = 1;
   }

   // Pure linear move:
   if (cartesian_move && !angular_move)
   {
      tx = dx ? fabs(dx / FROM_EXT_LEN(AXIS_MAX_VELOCITY[0])) : 0.0;
      ty = dy ? fabs(dy / FROM_EXT_LEN(AXIS_MAX_VELOCITY[1])) : 0.0;
      tz = dz ? fabs(dz / FROM_EXT_LEN(AXIS_MAX_VELOCITY[2])) : 0.0;
      tu = du ? fabs(du / FROM_EXT_LEN(AXIS_MAX_VELOCITY[6])) : 0.0;
      tv = dv ? fabs(dv / FROM_EXT_LEN(AXIS_MAX_VELOCITY[7])) : 0.0;
      tw = dw ? fabs(dw / FROM_EXT_LEN(AXIS_MAX_VELOCITY[8])) : 0.0;
      tmax = MAX3(tx, ty, tz);
      tmax = MAX4(tu, tv, tw, tmax);

      if (dx || dy || dz)
         dtot = sqrt(dx * dx + dy * dy + dz * dz);
      else
         dtot = sqrt(du * du + dv * dv + dw * dw);

      if (tmax <= 0.0)
      {
         vel = currentLinearFeedRate;
      }
      else
      {
         vel = dtot / tmax;
      }
   }
   // Pure angular move:
   else if (!cartesian_move && angular_move)
   {
      ta = da ? fabs(da / FROM_EXT_ANG(AXIS_MAX_VELOCITY[3])) : 0.0;
      tb = db ? fabs(db / FROM_EXT_ANG(AXIS_MAX_VELOCITY[4])) : 0.0;
      tc = dc ? fabs(dc / FROM_EXT_ANG(AXIS_MAX_VELOCITY[5])) : 0.0;
      tmax = MAX3(ta, tb, tc);

      dtot = sqrt(da * da + db * db + dc * dc);
      if (tmax <= 0.0)
      {
         vel = currentAngularFeedRate;
      }
      else
      {
         vel = dtot / tmax;
      }
   }
   // Combination angular and linear move:
   else if (cartesian_move && angular_move)
   {
      tx = dx ? fabs(dx / FROM_EXT_LEN(AXIS_MAX_VELOCITY[0])) : 0.0;
      ty = dy ? fabs(dy / FROM_EXT_LEN(AXIS_MAX_VELOCITY[1])) : 0.0;
      tz = dz ? fabs(dz / FROM_EXT_LEN(AXIS_MAX_VELOCITY[2])) : 0.0;
      ta = da ? fabs(da / FROM_EXT_ANG(AXIS_MAX_VELOCITY[3])) : 0.0;
      tb = db ? fabs(db / FROM_EXT_ANG(AXIS_MAX_VELOCITY[4])) : 0.0;
      tc = dc ? fabs(dc / FROM_EXT_ANG(AXIS_MAX_VELOCITY[5])) : 0.0;
      tu = du ? fabs(du / FROM_EXT_LEN(AXIS_MAX_VELOCITY[6])) : 0.0;
      tv = dv ? fabs(dv / FROM_EXT_LEN(AXIS_MAX_VELOCITY[7])) : 0.0;
      tw = dw ? fabs(dw / FROM_EXT_LEN(AXIS_MAX_VELOCITY[8])) : 0.0;
      tmax = MAX9(tx, ty, tz, ta, tb, tc, tu, tv, tw);

/*  According to NIST IR6556 Section 2.1.2.5 Paragraph A
    a combnation move is handled like a linear move, except
    that the angular axes are allowed sufficient time to
    complete their motion coordinated with the motion of
    the linear axes.
*/
      if (dx || dy || dz)
         dtot = sqrt(dx * dx + dy * dy + dz * dz);
      else
         dtot = sqrt(du * du + dv * dv + dw * dw);

      if (tmax <= 0.0)
      {
         vel = currentLinearFeedRate;
      }
      else
      {
         vel = dtot / tmax;
      }
   }
   if (debug_velacc)
      printf("cartesian %d ang %d vel %g\n", cartesian_move, angular_move, vel);
   return vel;
}

#include <vector>
struct pt
{
   double x, y, z, a, b, c, u, v, w;
   int line_no;
};

static std::vector < struct pt >&chained_points(void)
{
   static std::vector < struct pt >points;
   return points;
}

static void flush_segments(void)
{
   if (chained_points().empty())
      return;

   struct pt &pos = chained_points().back();

   double x = pos.x, y = pos.y, z = pos.z;
   double a = pos.a, b = pos.b, c = pos.c;
   double u = pos.u, v = pos.v, w = pos.w;

   int line_no = pos.line_no;

#ifdef SHOW_JOINED_SEGMENTS
   for (unsigned int i = 0; i != chained_points().size(); i++)
   {
      printf(".");
   }
   printf("\n");
#endif

   double ini_maxvel = getStraightVelocity(x, y, z, a, b, c, u, v, w), vel = ini_maxvel;

   if (cartesian_move && !angular_move)
   {
      if (vel > currentLinearFeedRate)
      {
         vel = currentLinearFeedRate;
      }
   }
   else if (!cartesian_move && angular_move)
   {
      if (vel > currentAngularFeedRate)
      {
         vel = currentAngularFeedRate;
      }
   }
   else if (cartesian_move && angular_move)
   {
      if (vel > currentLinearFeedRate)
      {
         vel = currentLinearFeedRate;
      }
   }

   emc_traj_linear_move_msg_t linearMoveMsg = { {EMC_TRAJ_LINEAR_MOVE_TYPE}
   };
   linearMoveMsg.feed_mode = feed_mode;

   // now x, y, z, and b are in absolute mm or degree units
   linearMoveMsg.end.tran.x = TO_EXT_LEN(x);
   linearMoveMsg.end.tran.y = TO_EXT_LEN(y);
   linearMoveMsg.end.tran.z = TO_EXT_LEN(z);

   linearMoveMsg.end.u = TO_EXT_LEN(u);
   linearMoveMsg.end.v = TO_EXT_LEN(v);
   linearMoveMsg.end.w = TO_EXT_LEN(w);

   // fill in the orientation
   linearMoveMsg.end.a = TO_EXT_ANG(a);
   linearMoveMsg.end.b = TO_EXT_ANG(b);
   linearMoveMsg.end.c = TO_EXT_ANG(c);

   linearMoveMsg.vel = toExtVel(vel);
   linearMoveMsg.ini_maxvel = toExtVel(ini_maxvel);
   double acc = getStraightAcceleration(x, y, z, a, b, c, u, v, w);
   linearMoveMsg.acc = toExtAcc(acc);

   linearMoveMsg.type = EMC_MOTION_TYPE_FEED;
   if ((vel && acc) || synched)
   {
      interp_list.set_line_number(line_no);
      interp_list.append((emc_command_msg_t *) & linearMoveMsg);
   }
   canonUpdateEndPoint(x, y, z, a, b, c, u, v, w);

   chained_points().clear();
}

static void get_last_pos(double &lx, double &ly, double &lz)
{
   if (chained_points().empty())
   {
      lx = canonEndPoint.x;
      ly = canonEndPoint.y;
      lz = canonEndPoint.z;
   }
   else
   {
      struct pt &pos = chained_points().back();
      lx = pos.x;
      ly = pos.y;
      lz = pos.z;
   }
}

static bool linkable(double x, double y, double z, double a, double b, double c, double u, double v, double w)
{
   struct pt &pos = chained_points().back();
   if (canonMotionMode != CANON_CONTINUOUS || canonNaivecamTolerance == 0)
      return false;

   if (chained_points().size() > 100)
      return false;

   if (a != pos.a)
      return false;
   if (b != pos.b)
      return false;
   if (c != pos.c)
      return false;
   if (u != pos.u)
      return false;
   if (v != pos.v)
      return false;
   if (w != pos.w)
      return false;

   if (x == canonEndPoint.x && y == canonEndPoint.y && z == canonEndPoint.z)
      return false;

   for (std::vector < struct pt >::iterator it = chained_points().begin(); it != chained_points().end(); it++)
   {
      PM_CARTESIAN M(x - canonEndPoint.x, y - canonEndPoint.y, z - canonEndPoint.z),
         B(canonEndPoint.x, canonEndPoint.y, canonEndPoint.z), P(it->x, it->y, it->z);
      double t0 = dot(M, P - B) / dot(M, M);
      if (t0 < 0)
         t0 = 0;
      if (t0 > 1)
         t0 = 1;

      double D = mag(P - (B + t0 * M));
      if (D > canonNaivecamTolerance)
         return false;
   }
   return true;
}

static void see_segment(int line_number, double x, double y, double z, double a, double b, double c, double u, double v, double w)
{
   bool changed_abc = (a != canonEndPoint.a) || (b != canonEndPoint.b) || (c != canonEndPoint.c);

   bool changed_uvw = (u != canonEndPoint.u) || (v != canonEndPoint.v) || (w != canonEndPoint.w);

   if (!chained_points().empty() && !linkable(x, y, z, a, b, c, u, v, w))
   {
      flush_segments();
   }
   pt pos = { x, y, z, a, b, c, u, v, w, line_number };
   chained_points().push_back(pos);
   if (changed_abc || changed_uvw)
   {
      flush_segments();
   }
}

void FINISH()
{
   flush_segments();
}

void STRAIGHT_TRAVERSE(int line_number, double x, double y, double z, double a, double b, double c, double u, double v, double w)
{
   double vel, acc;

   flush_segments();

   emc_traj_linear_move_msg_t linearMoveMsg = { {EMC_TRAJ_LINEAR_MOVE_TYPE} };

   linearMoveMsg.feed_mode = 0;
   linearMoveMsg.type = EMC_MOTION_TYPE_TRAVERSE;

   from_prog(x, y, z, a, b, c, u, v, w);
   rotate_and_offset_pos(x, y, z, a, b, c, u, v, w);

   vel = getStraightVelocity(x, y, z, a, b, c, u, v, w);
   acc = getStraightAcceleration(x, y, z, a, b, c, u, v, w);

   linearMoveMsg.end = to_ext_pose(x, y, z, a, b, c, u, v, w);
   linearMoveMsg.vel = linearMoveMsg.ini_maxvel = toExtVel(vel);
   linearMoveMsg.acc = toExtAcc(acc);

   int old_feed_mode = feed_mode;
   if (feed_mode)
      STOP_SPEED_FEED_SYNCH();

   if (vel && acc)
   {
      interp_list.set_line_number(line_number);
      interp_list.append((emc_command_msg_t *) & linearMoveMsg);
   }

   if (old_feed_mode)
      START_SPEED_FEED_SYNCH(currentLinearFeedRate, 1);

   canonUpdateEndPoint(x, y, z, a, b, c, u, v, w);
}

void STRAIGHT_FEED(int line_number, double x, double y, double z, double a, double b, double c, double u, double v, double w)
{
   from_prog(x, y, z, a, b, c, u, v, w);
   rotate_and_offset_pos(x, y, z, a, b, c, u, v, w);
   see_segment(line_number, x, y, z, a, b, c, u, v, w);
}


void RIGID_TAP(int line_number, double x, double y, double z)
{
   double ini_maxvel, vel, acc;
   emc_traj_rigid_tap_msg_t rigidTapMsg = { {EMC_TRAJ_RIGID_TAP_TYPE} };
   double unused = 0;

   from_prog(x, y, z, unused, unused, unused, unused, unused, unused);
   rotate_and_offset_pos(x, y, z, unused, unused, unused, unused, unused, unused);

   vel = getStraightVelocity(x, y, z, canonEndPoint.a, canonEndPoint.b, canonEndPoint.c, canonEndPoint.u, canonEndPoint.v, canonEndPoint.w);
   ini_maxvel = vel;

   acc = getStraightAcceleration(x, y, z, canonEndPoint.a, canonEndPoint.b, canonEndPoint.c, canonEndPoint.u, canonEndPoint.v, canonEndPoint.w);

   rigidTapMsg.pos = to_ext_pose(x, y, z, canonEndPoint.a, canonEndPoint.b, canonEndPoint.c, canonEndPoint.u, canonEndPoint.v, canonEndPoint.w);

   rigidTapMsg.vel = toExtVel(vel);
   rigidTapMsg.ini_maxvel = toExtVel(ini_maxvel);
   rigidTapMsg.acc = toExtAcc(acc);

   flush_segments();

   if (vel && acc)
   {
      interp_list.set_line_number(line_number);
      interp_list.append((emc_command_msg_t *) & rigidTapMsg);
   }

   // don't move the endpoint because after this move, we are back where we started
}


/*
  STRAIGHT_PROBE is exactly the same as STRAIGHT_FEED, except that it
  uses a probe message instead of a linear move message.
*/

void STRAIGHT_PROBE(int line_number, double x, double y, double z, double a, double b, double c, double u, double v, double w, unsigned char probe_type)
{
   double ini_maxvel, vel, acc;
   emc_traj_probe_msg_t probeMsg = { {EMC_TRAJ_PROBE_TYPE} };

   from_prog(x, y, z, a, b, c, u, v, w);
   rotate_and_offset_pos(x, y, z, a, b, c, u, v, w);

   flush_segments();

   ini_maxvel = vel = getStraightVelocity(x, y, z, a, b, c, u, v, w);

   if (cartesian_move && !angular_move)
   {
      if (vel > currentLinearFeedRate)
      {
         vel = currentLinearFeedRate;
      }
   }
   else if (!cartesian_move && angular_move)
   {
      if (vel > currentAngularFeedRate)
      {
         vel = currentAngularFeedRate;
      }
   }
   else if (cartesian_move && angular_move)
   {
      if (vel > currentLinearFeedRate)
      {
         vel = currentLinearFeedRate;
      }
   }

   acc = getStraightAcceleration(x, y, z, a, b, c, u, v, w);

   probeMsg.vel = toExtVel(vel);
   probeMsg.ini_maxvel = toExtVel(ini_maxvel);
   probeMsg.acc = toExtAcc(acc);

   probeMsg.type = EMC_MOTION_TYPE_PROBING;
   probeMsg.probe_type = probe_type;

   probeMsg.pos = to_ext_pose(x, y, z, a, b, c, u, v, w);

   if (vel && acc)
   {
      interp_list.set_line_number(line_number);
      interp_list.append((emc_command_msg_t *) & probeMsg);
   }
   canonUpdateEndPoint(x, y, z, a, b, c, u, v, w);
}

/* Machining Attributes */

void SET_MOTION_CONTROL_MODE(CANON_MOTION_MODE mode, double tolerance)
{
   emc_traj_set_term_cond_msg_t setTermCondMsg = { {EMC_TRAJ_SET_TERM_COND_TYPE} };

   flush_segments();

   canonMotionMode = mode;
   canonMotionTolerance = FROM_PROG_LEN(tolerance);

   switch (mode)
   {
   case CANON_CONTINUOUS:
      setTermCondMsg.cond = EMC_TRAJ_TERM_COND_BLEND;
      setTermCondMsg.tolerance = TO_EXT_LEN(canonMotionTolerance);
      break;

   default:
      setTermCondMsg.cond = EMC_TRAJ_TERM_COND_STOP;
      break;
   }

   interp_list.append((emc_command_msg_t *) & setTermCondMsg);
}

void SET_NAIVECAM_TOLERANCE(double tolerance)
{
   canonNaivecamTolerance = FROM_PROG_LEN(tolerance);
}

void SELECT_PLANE(CANON_PLANE in_plane)
{
   activePlane = in_plane;
}

void SET_CUTTER_RADIUS_COMPENSATION(double radius)
{
   // nothing need be done here
}

void START_CUTTER_RADIUS_COMPENSATION(int side)
{
   // nothing need be done here
}

void STOP_CUTTER_RADIUS_COMPENSATION()
{
   // nothing need be done here
}

void START_SPEED_FEED_SYNCH(double feed_per_revolution, bool velocity_mode)
{
   flush_segments();
   emc_traj_set_spindlesync_msg_t spindlesyncMsg = { {EMC_TRAJ_SET_SPINDLESYNC_TYPE} };
   spindlesyncMsg.feed_per_revolution = TO_EXT_LEN(FROM_PROG_LEN(feed_per_revolution));
   spindlesyncMsg.velocity_mode = velocity_mode;
   interp_list.append((emc_command_msg_t *) & spindlesyncMsg);
   synched = 1;
}

void STOP_SPEED_FEED_SYNCH()
{
   flush_segments();
   emc_traj_set_spindlesync_msg_t spindlesyncMsg = { {EMC_TRAJ_SET_SPINDLESYNC_TYPE} };
   spindlesyncMsg.feed_per_revolution = 0.0;
   spindlesyncMsg.velocity_mode = false;
   interp_list.append((emc_command_msg_t *) & spindlesyncMsg);
   synched = 0;
}

/* Machining Functions */

static double chord_deviation(double sx, double sy, double ex, double ey, double cx, double cy, int rotation, double &mx, double &my)
{
   double th1 = atan2(sy - cy, sx - cx), th2 = atan2(ey - cy, ex - cx), r = hypot(sy - cy, sx - cx), dth = th2 - th1;

   if (rotation < 0)
   {
      if (dth >= -1e-5)
         th2 -= 2 * M_PI;
      // in the edge case where atan2 gives you -pi and pi, a second iteration is needed
      // to get these in the right order
      dth = th2 - th1;
      if (dth >= -1e-5)
         th2 -= 2 * M_PI;
   }
   else
   {
      if (dth <= 1e-5)
         th2 += 2 * M_PI;
      dth = th2 - th1;
      if (dth <= 1e-5)
         th2 += 2 * M_PI;
   }

   double included = fabs(th2 - th1);
   double mid = (th2 + th1) / 2;
   mx = cx + r * cos(mid);
   my = cy + r * sin(mid);
   double dev = r * (1 - cos(included / 2));
   return dev;
}

/* Spline and NURBS additional functions; */

static double max(double a, double b)
{
   if (a < b)
      return b;
   return a;
}

static void unit(double *x, double *y)
{
   double h = hypot(*x, *y);
   if (h != 0)
   {
      *x /= h;
      *y /= h;
   }
}

static void arc(int lineno, double x0, double y0, double x1, double y1, double dx, double dy)
{
   double small_ = 0.000001;    /* "small" is a reserved word in windows.h */
   double x = x1 - x0, y = y1 - y0;
   double den = 2 * (y * dx - x * dy);
   CANON_POSITION p = unoffset_and_unrotate_pos(canonEndPoint);
   to_prog(p);
   if (fabs(den) > small_)
   {
      double r = -(x * x + y * y) / den;
      double i = dy * r, j = -dx * r;
      double cx = x1 + i, cy = y1 + j;
      ARC_FEED(lineno, x1, y1, cx, cy, r < 0 ? 1 : -1, p.z, p.a, p.b, p.c, p.u, p.v, p.w);
   }
   else
   {
      STRAIGHT_FEED(lineno, x1, y1, p.z, p.a, p.b, p.c, p.u, p.v, p.w);
   }
}

static int biarc(int lineno, double p0x, double p0y, double tsx, double tsy, double p4x, double p4y, double tex, double tey, double r = 1.0)
{
   unit(&tsx, &tsy);
   unit(&tex, &tey);

   double vx = p0x - p4x, vy = p0y - p4y;
   double c = vx * vx + vy * vy;
   double b = 2 * (vx * (r * tsx + tex) + vy * (r * tsy + tey));
   double a = 2 * r * (tsx * tex + tsy * tey - 1);

   double discr = b * b - 4 * a * c;
   if (discr < 0)
      return 0;

   double disq = sqrt(discr);
   double beta1 = (-b - disq) / 2 / a;
   double beta2 = (-b + disq) / 2 / a;

   if (beta1 > 0 && beta2 > 0)
      return 0;
   double beta = max(beta1, beta2);
   double alpha = beta * r;
   double ab = alpha + beta;
   double p1x = p0x + alpha * tsx, p1y = p0y + alpha * tsy,
      p3x = p4x - beta * tex, p3y = p4y - beta * tey, p2x = (p1x * beta + p3x * alpha) / ab, p2y = (p1y * beta + p3y * alpha) / ab;
   double tmx = p3x - p2x, tmy = p3y - p2y;
   unit(&tmx, &tmy);

   arc(lineno, p0x, p0y, p2x, p2y, tsx, tsy);
   arc(lineno, p2x, p2y, p4x, p4y, tmx, tmy);
   return 1;
}


/* Canon calls */

void NURBS_FEED(int lineno, std::vector < CONTROL_POINT > nurbs_control_points, unsigned int k)
{
   flush_segments();

   unsigned int n = nurbs_control_points.size() - 1;
   double umax = n - k + 2;
   unsigned int div = nurbs_control_points.size() * 4;
   std::vector < unsigned int >knot_vector = knot_vector_creator(n, k);
   PLANE_POINT P0, P0T, P1, P1T;

   P0 = nurbs_point(0, k, nurbs_control_points, knot_vector);
   P0T = nurbs_tangent(0, k, nurbs_control_points, knot_vector);

   for (unsigned int i = 1; i <= div; i++)
   {
      double u = umax * i / div;
      P1 = nurbs_point(u, k, nurbs_control_points, knot_vector);
      P1T = nurbs_tangent(u, k, nurbs_control_points, knot_vector);
      biarc(lineno, P0.X, P0.Y, P0T.X, P0T.Y, P1.X, P1.Y, P1T.X, P1T.Y);
      P0 = P1;
      P0T = P1T;
   }
   knot_vector.clear();
}


void ARC_FEED(int line_number,
              double first_end, double second_end,
              double first_axis, double second_axis, int rotation, double axis_end_point, double a, double b, double c, double u, double v, double w)
{
   EmcPose end;
//    PM_CARTESIAN center, normal;
   PmCartesian center, normal;
   emc_traj_circular_move_msg_t circularMoveMsg = { {EMC_TRAJ_CIRCULAR_MOVE_TYPE} };
   emc_traj_linear_move_msg_t linearMoveMsg = { {EMC_TRAJ_LINEAR_MOVE_TYPE} };
   double v1, v2, a1, a2, vel, ini_maxvel, circ_maxvel, axial_maxvel = 0.0, circ_acc, acc = 0.0;
   double radius, angle, theta1, theta2, helical_length, axis_len;
   double tcircle, taxial, tmax, thelix, ta, tb, tc, da, db, dc;
   double tu, tv, tw, du, dv, dw;
   double mx, my;

   double lx, ly, lz;
   double unused = 0;

   get_last_pos(lx, ly, lz);

   // XXX rotation?
   if ((activePlane == CANON_PLANE_XY)
       && canonMotionMode == CANON_CONTINUOUS
       && chord_deviation(lx, ly,
                          offset_x(FROM_PROG_LEN(first_end)), offset_y(FROM_PROG_LEN(second_end)),
                          offset_x(FROM_PROG_LEN(first_axis)), offset_y(FROM_PROG_LEN(second_axis)), rotation, mx, my) < canonNaivecamTolerance)
   {
      double x = FROM_PROG_LEN(first_end), y = FROM_PROG_LEN(second_end), z = FROM_PROG_LEN(axis_end_point);
      rotate_and_offset_pos(x, y, z, a, b, c, u, v, w);
      see_segment(line_number, mx, my,
                  (lz + z) / 2,
                  (canonEndPoint.a + a) / 2,
                  (canonEndPoint.b + b) / 2, (canonEndPoint.c + c) / 2, (canonEndPoint.u + u) / 2, (canonEndPoint.v + v) / 2, (canonEndPoint.w + w) / 2);
      see_segment(line_number, x, y, z, a, b, c, u, v, w);
      return;
   }
   //ini_maxvel = max vel defined by various ini constraints
   //circ_maxvel = max vel defined by ini constraints in the circle plane (XY, YZ or XZ)
   //axial_maxvel = max vel defined by ini constraints in the axial direction (Z, X or Y)

   linearMoveMsg.feed_mode = feed_mode;
   circularMoveMsg.feed_mode = feed_mode;
   flush_segments();

   a = FROM_PROG_ANG(a);
   b = FROM_PROG_ANG(b);
   c = FROM_PROG_ANG(c);
   u = FROM_PROG_LEN(u);
   v = FROM_PROG_LEN(v);
   w = FROM_PROG_LEN(w);

   rotate_and_offset_pos(unused, unused, unused, a, b, c, u, v, w);

   da = fabs(canonEndPoint.a - a);
   db = fabs(canonEndPoint.b - b);
   dc = fabs(canonEndPoint.c - c);

   du = fabs(canonEndPoint.u - u);
   dv = fabs(canonEndPoint.v - v);
   dw = fabs(canonEndPoint.w - w);

   /* Since there's no default case here,
      we need to initialise vel to something safe! */
   vel = ini_maxvel = currentLinearFeedRate;

   // convert to absolute mm units
   first_axis = FROM_PROG_LEN(first_axis);
   second_axis = FROM_PROG_LEN(second_axis);
   first_end = FROM_PROG_LEN(first_end);
   second_end = FROM_PROG_LEN(second_end);
   axis_end_point = FROM_PROG_LEN(axis_end_point);

   /* associate x with x, etc., offset by program origin, and set normals */
   switch (activePlane)
   {
   default:    // to eliminate "uninitalized" warnings
   case CANON_PLANE_XY:

      // offset and align args properly
      end.tran.x = first_end;
      end.tran.y = second_end;
      end.tran.z = axis_end_point;
      rotate_and_offset_pos(end.tran.x, end.tran.y, end.tran.z, unused, unused, unused, unused, unused, unused);
      center.x = first_axis;
      center.y = second_axis;
      center.z = end.tran.z;
      rotate_and_offset_pos(center.x, center.y, center.z, unused, unused, unused, unused, unused, unused);
      normal.x = 0.0;
      normal.y = 0.0;
      normal.z = 1.0;

      theta1 = atan2(canonEndPoint.y - center.y, canonEndPoint.x - center.x);
      theta2 = atan2(end.tran.y - center.y, end.tran.x - center.x);
      radius = hypot(canonEndPoint.x - center.x, canonEndPoint.y - center.y);
      axis_len = fabs(end.tran.z - canonEndPoint.z);

      v1 = FROM_EXT_LEN(AXIS_MAX_VELOCITY[0]);
      v2 = FROM_EXT_LEN(AXIS_MAX_VELOCITY[1]);
      a1 = FROM_EXT_LEN(AXIS_MAX_ACCELERATION[0]);
      a2 = FROM_EXT_LEN(AXIS_MAX_ACCELERATION[1]);
      circ_maxvel = ini_maxvel = MIN(v1, v2);
      circ_acc = acc = MIN(a1, a2);
      if (axis_valid(2) && axis_len > 0.001)
      {
         axial_maxvel = v1 = FROM_EXT_LEN(AXIS_MAX_VELOCITY[2]);
         ini_maxvel = MIN(ini_maxvel, v1);
         acc = MIN(acc, a1);
      }
      break;

   case CANON_PLANE_YZ:

      // offset and align args properly
      end.tran.y = first_end;
      end.tran.z = second_end;
      end.tran.x = axis_end_point;
      rotate_and_offset_pos(end.tran.x, end.tran.y, end.tran.z, unused, unused, unused, unused, unused, unused);

      center.y = first_axis;
      center.z = second_axis;
      center.x = end.tran.x;
      rotate_and_offset_pos(center.x, center.y, center.z, unused, unused, unused, unused, unused, unused);
      normal.y = 0.0;
      normal.z = 0.0;
      normal.x = 1.0;
      rotate(normal.x, normal.y, xy_rotation);

      theta1 = atan2(canonEndPoint.z - center.z, canonEndPoint.y - center.y);
      theta2 = atan2(end.tran.z - center.z, end.tran.y - center.y);
      radius = hypot(canonEndPoint.y - center.y, canonEndPoint.z - center.z);
      axis_len = fabs(end.tran.x - canonEndPoint.x);

      v1 = FROM_EXT_LEN(AXIS_MAX_VELOCITY[1]);
      v2 = FROM_EXT_LEN(AXIS_MAX_VELOCITY[2]);
      a1 = FROM_EXT_LEN(AXIS_MAX_ACCELERATION[1]);
      a2 = FROM_EXT_LEN(AXIS_MAX_ACCELERATION[2]);
      circ_maxvel = ini_maxvel = MIN(v1, v2);
      circ_acc = acc = MIN(a1, a2);
      if (axis_valid(0) && axis_len > 0.001)
      {
         axial_maxvel = v1 = FROM_EXT_LEN(AXIS_MAX_VELOCITY[0]);
         ini_maxvel = MIN(ini_maxvel, v1);
         acc = MIN(acc, a1);
      }

      break;

   case CANON_PLANE_XZ:

      // offset and align args properly
      end.tran.z = first_end;
      end.tran.x = second_end;
      end.tran.y = axis_end_point;
      rotate_and_offset_pos(end.tran.x, end.tran.y, end.tran.z, unused, unused, unused, unused, unused, unused);

      center.z = first_axis;
      center.x = second_axis;
      center.y = end.tran.y;
      rotate_and_offset_pos(center.x, center.y, center.z, unused, unused, unused, unused, unused, unused);
      normal.z = 0.0;
      normal.x = 0.0;
      normal.y = 1.0;
      rotate(normal.x, normal.y, xy_rotation);

      theta1 = atan2(canonEndPoint.x - center.x, canonEndPoint.z - center.z);
      theta2 = atan2(end.tran.x - center.x, end.tran.z - center.z);
      radius = hypot(canonEndPoint.x - center.x, canonEndPoint.z - center.z);
      axis_len = fabs(end.tran.y - canonEndPoint.y);

      v1 = FROM_EXT_LEN(AXIS_MAX_VELOCITY[0]);
      v2 = FROM_EXT_LEN(AXIS_MAX_VELOCITY[2]);
      a1 = FROM_EXT_LEN(AXIS_MAX_ACCELERATION[0]);
      a2 = FROM_EXT_LEN(AXIS_MAX_ACCELERATION[2]);
      circ_maxvel = ini_maxvel = MIN(v1, v2);
      circ_acc = acc = MIN(a1, a2);
      if (axis_valid(1) && axis_len > 0.001)
      {
         axial_maxvel = v1 = FROM_EXT_LEN(AXIS_MAX_VELOCITY[1]);
         ini_maxvel = MIN(ini_maxvel, v1);
         acc = MIN(acc, a1);
      }
      break;
   }

   if (rotation < 0)
   {
      if (theta2 >= theta1)
         theta2 -= M_PI * 2.0;
   }
   else
   {
      if (theta2 <= theta1)
         theta2 += M_PI * 2.0;
   }
   angle = theta2 - theta1;
   helical_length = hypot(angle * radius, axis_len);

// COMPUTE VELOCITIES
   ta = (axis_valid(3) && da) ? fabs(da / FROM_EXT_ANG(AXIS_MAX_VELOCITY[3])) : 0.0;
   tb = (axis_valid(4) && db) ? fabs(db / FROM_EXT_ANG(AXIS_MAX_VELOCITY[4])) : 0.0;
   tc = (axis_valid(5) && dc) ? fabs(dc / FROM_EXT_ANG(AXIS_MAX_VELOCITY[5])) : 0.0;

   tu = (axis_valid(6) && du) ? (du / FROM_EXT_LEN(AXIS_MAX_VELOCITY[6])) : 0.0;
   tv = (axis_valid(7) && dv) ? (dv / FROM_EXT_LEN(AXIS_MAX_VELOCITY[7])) : 0.0;
   tw = (axis_valid(8) && dw) ? (dw / FROM_EXT_LEN(AXIS_MAX_VELOCITY[8])) : 0.0;

   //we have accel, check what the max_vel is that doesn't violate the centripetal accel=accel
   v1 = sqrt(circ_acc * radius);
   circ_maxvel = MIN(v1, circ_maxvel);

   // find out how long the arc takes at ini_maxvel
   tcircle = fabs(angle * radius / circ_maxvel);

   if (axial_maxvel)
   {
      taxial = fabs(axis_len / axial_maxvel);
      tmax = MAX(taxial, tcircle);
   }
   else
      tmax = tcircle;

   tmax = MAX4(tmax, ta, tb, tc);
   tmax = MAX4(tmax, tu, tv, tw);

   if (tmax <= 0.0)
   {
      vel = currentLinearFeedRate;
   }
   else
   {
      ini_maxvel = helical_length / tmax;       //compute the new maxvel based on all previous constraints
      vel = MIN(vel, ini_maxvel);       //the programmed vel is either feedrate or machine_maxvel if lower
   }

   // for arcs we always user linear move since there is no
   // arc possible with only ABC motion

   cartesian_move = 1;

// COMPUTE ACCELS

   // the next calcs are not really times.  the units are time^2, but
   // the division at the end gives the right units for accel.  if you
   // try to think of these in terms of any real-world value (time to
   // do what?), you're probably doomed.  think of them as a parametric
   // expression of the acceleration in the various directions.

   thelix = (helical_length / acc);
   ta = (axis_valid(3) && da) ? (da / FROM_EXT_ANG(AXIS_MAX_ACCELERATION[3])) : 0.0;
   tb = (axis_valid(4) && db) ? (db / FROM_EXT_ANG(AXIS_MAX_ACCELERATION[4])) : 0.0;
   tc = (axis_valid(5) && dc) ? (dc / FROM_EXT_ANG(AXIS_MAX_ACCELERATION[5])) : 0.0;

   tu = (axis_valid(6) && du) ? (du / FROM_EXT_LEN(AXIS_MAX_ACCELERATION[6])) : 0.0;
   tv = (axis_valid(7) && dv) ? (dv / FROM_EXT_LEN(AXIS_MAX_ACCELERATION[7])) : 0.0;
   tw = (axis_valid(8) && dw) ? (dw / FROM_EXT_LEN(AXIS_MAX_ACCELERATION[8])) : 0.0;

   tmax = MAX4(thelix, ta, tb, tc);
   tmax = MAX4(tmax, tu, tv, tw);

   if (tmax > 0.0)
   {
      acc = helical_length / tmax;
   }

   /* 
      mapping of rotation to turns:

      rotation turns 
      -------- ----- 
      0 none (linear move) 
      1 0 
      2 1 
      -1 -1 
      -2 -2 */

   if (rotation == 0)
   {
      // linear move

      linearMoveMsg.end.tran.x = TO_EXT_LEN(end.tran.x);
      linearMoveMsg.end.tran.y = TO_EXT_LEN(end.tran.y);
      linearMoveMsg.end.tran.z = TO_EXT_LEN(end.tran.z);

      // fill in the orientation
      linearMoveMsg.end.a = TO_EXT_ANG(a);
      linearMoveMsg.end.b = TO_EXT_ANG(b);
      linearMoveMsg.end.c = TO_EXT_ANG(c);

      linearMoveMsg.end.u = TO_EXT_LEN(u);
      linearMoveMsg.end.v = TO_EXT_LEN(v);
      linearMoveMsg.end.w = TO_EXT_LEN(w);

      linearMoveMsg.type = EMC_MOTION_TYPE_ARC;
      linearMoveMsg.vel = toExtVel(vel);
      linearMoveMsg.ini_maxvel = toExtVel(ini_maxvel);
      linearMoveMsg.acc = toExtAcc(acc);
      if (vel && acc)
      {
         interp_list.set_line_number(line_number);
         interp_list.append((emc_command_msg_t *) & linearMoveMsg);
      }
   }
   else
   {
      circularMoveMsg.end.tran.x = TO_EXT_LEN(end.tran.x);
      circularMoveMsg.end.tran.y = TO_EXT_LEN(end.tran.y);
      circularMoveMsg.end.tran.z = TO_EXT_LEN(end.tran.z);

      circularMoveMsg.center.x = TO_EXT_LEN(center.x);
      circularMoveMsg.center.y = TO_EXT_LEN(center.y);
      circularMoveMsg.center.z = TO_EXT_LEN(center.z);

      circularMoveMsg.normal = normal;

      if (rotation > 0)
         circularMoveMsg.turn = rotation - 1;
      else
         // reverse turn
         circularMoveMsg.turn = rotation;

      // fill in the orientation
      circularMoveMsg.end.a = TO_EXT_ANG(a);
      circularMoveMsg.end.b = TO_EXT_ANG(b);
      circularMoveMsg.end.c = TO_EXT_ANG(c);

      circularMoveMsg.end.u = TO_EXT_LEN(u);
      circularMoveMsg.end.v = TO_EXT_LEN(v);
      circularMoveMsg.end.w = TO_EXT_LEN(w);

      circularMoveMsg.type = EMC_MOTION_TYPE_ARC;

      // These are suboptimal but safe values.  The actual maximums
      // are hard to calculate but may be somewhat larger than
      // these.  Imagine an arc with very large radius going from
      // 0,0,0 to 1,1,1 on a machine with maxvel=1 and maxaccel=1 on
      // all axes.  The actual maximums will be near sqrt(3) but
      // we'll be using 1 instead.
      circularMoveMsg.vel = toExtVel(vel);
      circularMoveMsg.ini_maxvel = toExtVel(ini_maxvel);
      circularMoveMsg.acc = toExtAcc(acc);
      if (vel && acc)
      {
         interp_list.set_line_number(line_number);
         interp_list.append((emc_command_msg_t *) & circularMoveMsg);
      }
   }
   // update the end point
   canonUpdateEndPoint(end.tran.x, end.tran.y, end.tran.z, a, b, c, u, v, w);
}


void DWELL(double seconds)
{
   emc_traj_delay_msg_t delayMsg = { {EMC_TRAJ_DELAY_TYPE} };

   flush_segments();

   delayMsg.delay = seconds;

   interp_list.append((emc_command_msg_t *) & delayMsg);
}

/* Spindle Functions */
void SPINDLE_RETRACT_TRAVERSE()
{
   /*! \todo FIXME-- unimplemented */
}

void SET_SPINDLE_MODE(double css_max)
{
   css_maximum = css_max;
}

void START_SPINDLE_CLOCKWISE()
{
   emc_spindle_on_msg_t emc_spindle_on_msg = { {EMC_SPINDLE_ON_TYPE} };

   flush_segments();

   if (css_maximum)
   {
      if (lengthUnits == CANON_UNITS_INCHES)
         css_numerator = 12 / (2 * M_PI) * spindleSpeed * TO_EXT_LEN(25.4);
      else
         css_numerator = 1000 / (2 * M_PI) * spindleSpeed * TO_EXT_LEN(1);
      emc_spindle_on_msg.speed = css_maximum;
      emc_spindle_on_msg.factor = css_numerator;
      emc_spindle_on_msg.xoffset = TO_EXT_LEN(programOrigin.x + currentToolOffset.tran.x);
   }
   else
   {
      emc_spindle_on_msg.speed = spindleSpeed;
      css_numerator = 0;
   }
   interp_list.append((emc_command_msg_t *) & emc_spindle_on_msg);
}

void START_SPINDLE_COUNTERCLOCKWISE()
{
   emc_spindle_on_msg_t emc_spindle_on_msg = { {EMC_SPINDLE_ON_TYPE} };

   flush_segments();

   if (css_maximum)
   {
      if (lengthUnits == CANON_UNITS_INCHES)
         css_numerator = -12 / (2 * M_PI) * spindleSpeed;
      else
         css_numerator = -1000 / (2 * M_PI) * spindleSpeed;
      emc_spindle_on_msg.speed = css_maximum;
      emc_spindle_on_msg.factor = css_numerator;
      emc_spindle_on_msg.xoffset = TO_EXT_LEN(programOrigin.x + currentToolOffset.tran.x);
   }
   else
   {
      emc_spindle_on_msg.speed = -spindleSpeed;
      css_numerator = 0;
   }


   interp_list.append((emc_command_msg_t *) & emc_spindle_on_msg);
}

void SET_SPINDLE_SPEED(double r)
{
   // speed is in RPMs everywhere
   spindleSpeed = r;

   emc_spindle_speed_msg_t emc_spindle_speed_msg = { {EMC_SPINDLE_SPEED_TYPE} };

   flush_segments();

   if (css_maximum)
   {
      if (lengthUnits == CANON_UNITS_INCHES)
         css_numerator = 12 / (2 * M_PI) * spindleSpeed;
      else
         css_numerator = 1000 / (2 * M_PI) * spindleSpeed;
      emc_spindle_speed_msg.speed = css_maximum;
      emc_spindle_speed_msg.factor = css_numerator;
      emc_spindle_speed_msg.xoffset = TO_EXT_LEN(programOrigin.x + currentToolOffset.tran.x);
   }
   else
   {
      emc_spindle_speed_msg.speed = spindleSpeed;
      css_numerator = 0;
   }
   interp_list.append((emc_command_msg_t *) & emc_spindle_speed_msg);

}

void STOP_SPINDLE_TURNING()
{
   emc_msg_t emc_spindle_off_msg = { EMC_SPINDLE_OFF_TYPE };

   flush_segments();

   interp_list.append((emc_command_msg_t *) & emc_spindle_off_msg);
}

void SPINDLE_RETRACT()
{
   /*! \todo FIXME-- unimplemented */
}

void ORIENT_SPINDLE(double orientation, CANON_DIRECTION direction)
{
   /*! \todo FIXME-- unimplemented */
}

void USE_SPINDLE_FORCE(void)
{
   /*! \todo FIXME-- unimplemented */
}

void LOCK_SPINDLE_Z(void)
{
   /*! \todo FIXME-- unimplemented */
}

void USE_NO_SPINDLE_FORCE(void)
{
   /*! \todo FIXME-- unimplemented */
}

/* Tool Functions */

/* this is called with distances in external (machine) units */
void SET_TOOL_TABLE_ENTRY(int pocket, int toolno, EmcPose offset, double diameter, double frontangle, double backangle, int orientation)
{
   emc_tool_set_offset_msg_t o = { {EMC_TOOL_SET_OFFSET_TYPE} };
   flush_segments();
   o.pocket = pocket;
   o.toolno = toolno;
   o.offset = offset;
   o.diameter = diameter;
   o.frontangle = frontangle;
   o.backangle = backangle;
   o.orientation = orientation;
   interp_list.append((emc_command_msg_t *) & o);
}

/*
  EMC has no tool length offset. To implement it, we save it here,
  and apply it when necessary
  */
void USE_TOOL_LENGTH_OFFSET(EmcPose offset)
{
   emc_traj_set_offset_msg_t set_offset_msg = { {EMC_TRAJ_SET_OFFSET_TYPE} };

   flush_segments();

   /* convert to mm units for internal canonical use */
   currentToolOffset.tran.x = FROM_PROG_LEN(offset.tran.x);
   currentToolOffset.tran.y = FROM_PROG_LEN(offset.tran.y);
   currentToolOffset.tran.z = FROM_PROG_LEN(offset.tran.z);
   currentToolOffset.a = FROM_PROG_ANG(offset.a);
   currentToolOffset.b = FROM_PROG_ANG(offset.b);
   currentToolOffset.c = FROM_PROG_ANG(offset.c);
   currentToolOffset.u = FROM_PROG_LEN(offset.u);
   currentToolOffset.v = FROM_PROG_LEN(offset.v);
   currentToolOffset.w = FROM_PROG_LEN(offset.w);

   /* append it to interp list so it gets updated at the right time, not at
      read-ahead time */
   set_offset_msg.offset.tran.x = TO_EXT_LEN(currentToolOffset.tran.x);
   set_offset_msg.offset.tran.y = TO_EXT_LEN(currentToolOffset.tran.y);
   set_offset_msg.offset.tran.z = TO_EXT_LEN(currentToolOffset.tran.z);
   set_offset_msg.offset.a = TO_EXT_ANG(currentToolOffset.a);
   set_offset_msg.offset.b = TO_EXT_ANG(currentToolOffset.b);
   set_offset_msg.offset.c = TO_EXT_ANG(currentToolOffset.c);
   set_offset_msg.offset.u = TO_EXT_LEN(currentToolOffset.u);
   set_offset_msg.offset.v = TO_EXT_LEN(currentToolOffset.v);
   set_offset_msg.offset.w = TO_EXT_LEN(currentToolOffset.w);

   if (css_maximum)
   {
      emc_spindle_speed_msg_t emc_spindle_speed_msg = { {EMC_SPINDLE_SPEED_TYPE}
      };
      emc_spindle_speed_msg.speed = css_maximum;
      emc_spindle_speed_msg.factor = css_numerator;
      emc_spindle_speed_msg.xoffset = TO_EXT_LEN(programOrigin.x + currentToolOffset.tran.x);
      interp_list.append((emc_command_msg_t *) & emc_spindle_speed_msg);
   }
   interp_list.append((emc_command_msg_t *) & set_offset_msg);
}

/* CHANGE_TOOL results from M6, for example */
void CHANGE_TOOL(int slot)
{
   emc_traj_linear_move_msg_t linearMoveMsg = { {EMC_TRAJ_LINEAR_MOVE_TYPE} };
   linearMoveMsg.feed_mode = feed_mode;
   emc_msg_t load_tool_msg = { EMC_TOOL_LOAD_TYPE };

   flush_segments();

   /* optional move to tool change position.  This
    * is a mess because we really want a configurable chain
    * of events to happen when a tool change is called for.
    * Since they'll probably involve motion, we can't just
    * do it in HAL.  This is basic support for making one
    * move to a particular coordinate before the tool change
    * is called.  */

   if (HAVE_TOOL_CHANGE_POSITION)
   {
      double vel, acc, x, y, z, a, b, c, u, v, w;

      x = FROM_EXT_LEN(TOOL_CHANGE_POSITION.tran.x);
      y = FROM_EXT_LEN(TOOL_CHANGE_POSITION.tran.y);
      z = FROM_EXT_LEN(TOOL_CHANGE_POSITION.tran.z);
      a = FROM_EXT_ANG(TOOL_CHANGE_POSITION.a);
      b = FROM_EXT_ANG(TOOL_CHANGE_POSITION.b);
      c = FROM_EXT_ANG(TOOL_CHANGE_POSITION.c);
      u = FROM_EXT_LEN(TOOL_CHANGE_POSITION.u);
      v = FROM_EXT_LEN(TOOL_CHANGE_POSITION.v);
      w = FROM_EXT_LEN(TOOL_CHANGE_POSITION.w);


      vel = getStraightVelocity(x, y, z, a, b, c, u, v, w);
      acc = getStraightAcceleration(x, y, z, a, b, c, u, v, w);

      linearMoveMsg.end = to_ext_pose(x, y, z, a, b, c, u, v, w);

      linearMoveMsg.vel = linearMoveMsg.ini_maxvel = toExtVel(vel);
      linearMoveMsg.acc = toExtAcc(acc);
      linearMoveMsg.type = EMC_MOTION_TYPE_TOOLCHANGE;
      linearMoveMsg.feed_mode = 0;

      int old_feed_mode = feed_mode;
      if (feed_mode)
         STOP_SPEED_FEED_SYNCH();

      if (vel && acc)
         interp_list.append((emc_command_msg_t *) & linearMoveMsg);

      if (old_feed_mode)
         START_SPEED_FEED_SYNCH(currentLinearFeedRate, 1);

      canonUpdateEndPoint(x, y, z, a, b, c, u, v, w);
   }

   /* regardless of optional moves above, we'll always send a load tool
      message */
   interp_list.append((emc_command_msg_t *) & load_tool_msg);
}

/* SELECT_POCKET results from T1, for example */
void SELECT_POCKET(int slot)
{
   emc_tool_prepare_msg_t prep_for_tool_msg = { {EMC_TOOL_PREPARE_TYPE} };

   prep_for_tool_msg.tool = slot;

   interp_list.append((emc_command_msg_t *) & prep_for_tool_msg);
}

/* CHANGE_TOOL_NUMBER results from M61, for example */
void CHANGE_TOOL_NUMBER(int number)
{
   emc_tool_set_number_msg_t emc_tool_set_number_msg = { {EMC_TOOL_SET_NUMBER_TYPE} };

   emc_tool_set_number_msg.tool = number;

   interp_list.append((emc_command_msg_t *) & emc_tool_set_number_msg);
}


/* Misc Functions */

void CLAMP_AXIS(CANON_AXIS axis)
{
   /*! \todo FIXME-- unimplemented */
}

/*
  setString and addString initializes or adds src to dst, never exceeding
  dst's maxlen chars.
*/

static char *setString(char *dst, const char *src, int maxlen)
{
   dst[0] = 0;
   strncat(dst, src, maxlen - 1);
   dst[maxlen - 1] = 0;
   return dst;
}

static char *addString(char *dst, const char *src, int maxlen)
{
   int dstlen = strlen(dst);
   int srclen = strlen(src);
   int actlen;

   if (srclen >= maxlen - dstlen)
   {
      actlen = maxlen - dstlen - 1;
      dst[maxlen - 1] = 0;
   }
   else
   {
      actlen = srclen;
   }

   strncat(dst, src, actlen);

   return dst;
}

/*
  The probe file is opened with a hot-comment (PROBEOPEN <filename>),
  and the results of each probed point are written to that file.
  The file is closed with a (PROBECLOSE) comment.
*/

static FILE *probefile = NULL;

void COMMENT(const char *comment)
{
   // nothing need be done here, but you can play tricks with hot comments

   char msg[LINELEN];
   char probefilename[LINELEN];
   const char *ptr;

   // set RPY orientation for subsequent moves
   if (!strncmp(comment, "RPY", strlen("RPY")))
   {
      PM_RPY rpy;
      // it's RPY <R> <P> <Y>
      if (3 != sscanf(comment, "%*s %lf %lf %lf", &rpy.r, &rpy.p, &rpy.y))
      {
         // print current orientation
         printf("rpy = %f %f %f, quat = %f %f %f %f\n", rpy.r, rpy.p, rpy.y, quat.s, quat.x, quat.y, quat.z);
      }
      else
      {
         // set and print orientation
         quat = rpy;
         printf("rpy = %f %f %f, quat = %f %f %f %f\n", rpy.r, rpy.p, rpy.y, quat.s, quat.x, quat.y, quat.z);
      }
      return;
   }
   // open probe output file
   if (!strncmp(comment, "PROBEOPEN", strlen("PROBEOPEN")))
   {
      // position ptr to first char after PROBEOPEN
      ptr = &comment[strlen("PROBEOPEN")];
      // and step over white space to name, or NULL
      while (isspace(*ptr))
      {
         ptr++;
      }
      setString(probefilename, ptr, LINELEN);
      if (NULL == (probefile = fopen(probefilename, "wt")))
      {
         // pop up a warning message
         setString(msg, "can't open probe file ", LINELEN);
         addString(msg, probefilename, LINELEN);
         MESSAGE(msg);
         probefile = NULL;
      }
      return;
   }
   // close probe output file
   if (!strncmp(comment, "PROBECLOSE", strlen("PROBECLOSE")))
   {
      if (probefile != NULL)
      {
         fclose(probefile);
         probefile = NULL;
      }
      return;
   }

   return;
}

// refers to feed rate
void DISABLE_FEED_OVERRIDE()
{
   emc_traj_set_fo_enable_msg_t set_fo_enable_msg = { {EMC_TRAJ_SET_FO_ENABLE_TYPE} };
   flush_segments();

   set_fo_enable_msg.mode = 0;
   interp_list.append((emc_command_msg_t *) & set_fo_enable_msg);
}

void ENABLE_FEED_OVERRIDE()
{
   emc_traj_set_fo_enable_msg_t set_fo_enable_msg = { {EMC_TRAJ_SET_FO_ENABLE_TYPE} };
   flush_segments();

   set_fo_enable_msg.mode = 1;
   interp_list.append((emc_command_msg_t *) & set_fo_enable_msg);
}

//refers to adaptive feed override (HAL input, usefull for EDM for example)
void DISABLE_ADAPTIVE_FEED()
{
   emc_motion_adaptive_msg_t emcmotAdaptiveMsg = { {EMC_MOTION_ADAPTIVE_TYPE} };
   flush_segments();

   emcmotAdaptiveMsg.status = 0;
   interp_list.append((emc_command_msg_t *) & emcmotAdaptiveMsg);
}

void ENABLE_ADAPTIVE_FEED()
{
   emc_motion_adaptive_msg_t emcmotAdaptiveMsg = { {EMC_MOTION_ADAPTIVE_TYPE} };
   flush_segments();

   emcmotAdaptiveMsg.status = 1;
   interp_list.append((emc_command_msg_t *) & emcmotAdaptiveMsg);
}

//refers to spindle speed
void DISABLE_SPEED_OVERRIDE()
{
   emc_traj_set_so_enable_msg_t set_so_enable_msg = { {EMC_TRAJ_SET_SO_ENABLE_TYPE} };
   flush_segments();

   set_so_enable_msg.mode = 0;
   interp_list.append((emc_command_msg_t *) & set_so_enable_msg);
}


void ENABLE_SPEED_OVERRIDE()
{
   emc_traj_set_so_enable_msg_t set_so_enable_msg = { {EMC_TRAJ_SET_SO_ENABLE_TYPE} };
   flush_segments();

   set_so_enable_msg.mode = 1;
   interp_list.append((emc_command_msg_t *) & set_so_enable_msg);
}

void ENABLE_FEED_HOLD()
{
   emc_traj_set_fh_enable_msg_t set_feed_hold_msg = { {EMC_TRAJ_SET_FH_ENABLE_TYPE} };
   flush_segments();

   set_feed_hold_msg.mode = 1;
   interp_list.append((emc_command_msg_t *) & set_feed_hold_msg);
}

void DISABLE_FEED_HOLD()
{
   emc_traj_set_fh_enable_msg_t set_feed_hold_msg = { {EMC_TRAJ_SET_FH_ENABLE_TYPE} };
   flush_segments();

   set_feed_hold_msg.mode = 0;
   interp_list.append((emc_command_msg_t *) & set_feed_hold_msg);
}

void FLOOD_OFF()
{
   emc_msg_t flood_off_msg = { EMC_COOLANT_FLOOD_OFF_TYPE };

   flush_segments();

   interp_list.append((emc_command_msg_t *) & flood_off_msg);
}

void FLOOD_ON()
{
   emc_msg_t flood_on_msg = { EMC_COOLANT_FLOOD_ON_TYPE };

   flush_segments();

   interp_list.append((emc_command_msg_t *) & flood_on_msg);
}

void MESSAGE(char *s)
{
   emc_operator_message_msg_t operator_message_msg = { {EMC_OPERATOR_MESSAGE_TYPE} };

   flush_segments();

   operator_message_msg.id = 0;
   strncpy(operator_message_msg.text, s, LINELEN);
   operator_message_msg.text[LINELEN - 1] = 0;

   interp_list.append((emc_command_msg_t *) & operator_message_msg);
}

static FILE *logfile = NULL;

void LOG(char *s)
{
   flush_segments();
   if (logfile)
   {
      fprintf(logfile, "%s\n", s);
      fflush(logfile);
   }
   fprintf(stderr, "LOG(%s)\n", s);

}

void LOGOPEN(char *name)
{
   if (logfile)
      fclose(logfile);
   logfile = fopen(name, "wt");
   fprintf(stderr, "LOGOPEN(%s) -> %p\n", name, logfile);
}

void LOGCLOSE()
{
   if (logfile)
      fclose(logfile);
   logfile = NULL;
   fprintf(stderr, "LOGCLOSE()\n");
}

void MIST_OFF()
{
   emc_msg_t mist_off_msg = { EMC_COOLANT_MIST_OFF_TYPE };

   flush_segments();

   interp_list.append((emc_command_msg_t *) & mist_off_msg);
}

void MIST_ON()
{
   emc_msg_t mist_on_msg = { EMC_COOLANT_MIST_ON_TYPE };

   flush_segments();

   interp_list.append((emc_command_msg_t *) & mist_on_msg);
}

void PALLET_SHUTTLE()
{
   /*! \todo FIXME-- unimplemented */
}

void TURN_PROBE_OFF()
{
   // don't do anything-- this is called when the probing is done
}

void TURN_PROBE_ON()
{
   emc_msg_t clearMsg = { EMC_TRAJ_CLEAR_PROBE_TRIPPED_FLAG_TYPE };

   interp_list.append((emc_command_msg_t *) & clearMsg);
}

void UNCLAMP_AXIS(CANON_AXIS axis)
{
   /*! \todo FIXME-- unimplemented */
}

/* Program Functions */

void PROGRAM_STOP()
{
   /* implement this as a pause. A resume will cause motion to proceed. */
   emc_msg_t pauseMsg = { EMC_TASK_PLAN_PAUSE_TYPE };

   flush_segments();

   interp_list.append((emc_command_msg_t *) & pauseMsg);
}

void SET_BLOCK_DELETE(bool state)
{
   block_delete = state;        //state == ON, means we don't interpret lines starting with "/"
}

bool GET_BLOCK_DELETE()
{
   return block_delete; //state == ON, means we  don't interpret lines starting with "/"
}


void SET_OPTIONAL_PROGRAM_STOP(bool state)
{
   optional_program_stop = state;       //state == ON, means we stop
}

bool GET_OPTIONAL_PROGRAM_STOP()
{
   return optional_program_stop;        //state == ON, means we stop
}

void OPTIONAL_PROGRAM_STOP()
{
   emc_msg_t stopMsg = { EMC_TASK_PLAN_OPTIONAL_STOP_TYPE };

   flush_segments();

   interp_list.append((emc_command_msg_t *) & stopMsg);
}

void PROGRAM_END()
{
   flush_segments();

   emc_msg_t endMsg = { EMC_TASK_PLAN_END_TYPE };

   interp_list.append((emc_command_msg_t *) & endMsg);
}

double GET_EXTERNAL_TOOL_LENGTH_XOFFSET()
{
   return TO_PROG_LEN(currentToolOffset.tran.x);
}

double GET_EXTERNAL_TOOL_LENGTH_YOFFSET()
{
   return TO_PROG_LEN(currentToolOffset.tran.y);
}

double GET_EXTERNAL_TOOL_LENGTH_ZOFFSET()
{
   return TO_PROG_LEN(currentToolOffset.tran.z);
}

double GET_EXTERNAL_TOOL_LENGTH_AOFFSET()
{
   return TO_PROG_ANG(currentToolOffset.a);
}

double GET_EXTERNAL_TOOL_LENGTH_BOFFSET()
{
   return TO_PROG_ANG(currentToolOffset.b);
}

double GET_EXTERNAL_TOOL_LENGTH_COFFSET()
{
   return TO_PROG_ANG(currentToolOffset.c);
}

double GET_EXTERNAL_TOOL_LENGTH_UOFFSET()
{
   return TO_PROG_LEN(currentToolOffset.u);
}

double GET_EXTERNAL_TOOL_LENGTH_VOFFSET()
{
   return TO_PROG_LEN(currentToolOffset.v);
}

double GET_EXTERNAL_TOOL_LENGTH_WOFFSET()
{
   return TO_PROG_LEN(currentToolOffset.w);
}

/*
  INIT_CANON()
  Initialize canonical local variables to defaults
  */
void INIT_CANON()
{
   double units;

   chained_points().clear();

   // initialize locals to original values
   programOrigin.x = 0.0;
   programOrigin.y = 0.0;
   programOrigin.z = 0.0;
   programOrigin.a = 0.0;
   programOrigin.b = 0.0;
   programOrigin.c = 0.0;
   programOrigin.u = 0.0;
   programOrigin.v = 0.0;
   programOrigin.w = 0.0;
   xy_rotation = 0.;
   activePlane = CANON_PLANE_XY;
   canonUpdateEndPoint(0, 0, 0, 0, 0, 0, 0, 0, 0);
   SET_MOTION_CONTROL_MODE(CANON_CONTINUOUS, 0);
   spindleSpeed = 0.0;
   preppedTool = 0;
   cartesian_move = 0;
   angular_move = 0;
   currentLinearFeedRate = 0.0;
   currentAngularFeedRate = 0.0;
   ZERO_EMC_POSE(currentToolOffset);
   /* 
      to set the units, note that GET_EXTERNAL_LENGTH_UNITS() returns
      traj->linearUnits, which is already set from the .ini file in
      iniTraj(). This is a floating point number, in user units per mm. We
      can compare this against known values and set the symbolic values
      accordingly. If it doesn't match, we have an error. */
   units = GET_EXTERNAL_LENGTH_UNITS();
   if (fabs(units - 1.0 / 25.4) < 1.0e-3)
   {
      lengthUnits = CANON_UNITS_INCHES;
   }
   else if (fabs(units - 1.0) < 1.0e-3)
   {
      lengthUnits = CANON_UNITS_MM;
   }
   else
   {
      CANON_ERROR("non-standard length units, setting interpreter to mm");
      lengthUnits = CANON_UNITS_MM;
   }
}

/* Sends error message */
void CANON_ERROR(const char *fmt, ...)
{
   va_list ap;
   emc_operator_message_msg_t operator_message_msg = { {EMC_OPERATOR_MESSAGE_TYPE} };

   flush_segments();

   operator_message_msg.id = 0;
   if (fmt != NULL)
   {
      va_start(ap, fmt);
      vsprintf(operator_message_msg.text, fmt, ap);
      va_end(ap);
   }
   else
   {
      operator_message_msg.text[0] = 0;
   }

   interp_list.append((emc_command_msg_t *) & operator_message_msg);
}

/*
  GET_EXTERNAL_TOOL_TABLE(int pocket)

  Returns the tool table structure associated with pocket. Note that
  pocket can run from 0 (by definition, the spindle), to pocket CANON_POCKETS_MAX - 1.

  Tool table is always in machine units.

  */
CANON_TOOL_TABLE GET_EXTERNAL_TOOL_TABLE(int pocket)
{
   CANON_TOOL_TABLE retval;

   if (pocket < 0 || pocket >= CANON_POCKETS_MAX)
   {
      retval.toolno = -1;
      ZERO_EMC_POSE(retval.offset);
      retval.frontangle = 0.0;
      retval.backangle = 0.0;
      retval.diameter = 0.0;
      retval.orientation = 0;
   }
   else
   {
      retval = emcStatus->io.tool.toolTable[pocket];
   }

   return retval;
}

CANON_POSITION GET_EXTERNAL_POSITION()
{
   CANON_POSITION position;
   EmcPose pos;

   chained_points().clear();

   pos = emcStatus->motion.traj.position;

   // first update internal record of last position
   canonUpdateEndPoint(FROM_EXT_LEN(pos.tran.x), FROM_EXT_LEN(pos.tran.y), FROM_EXT_LEN(pos.tran.z),
                       FROM_EXT_ANG(pos.a), FROM_EXT_ANG(pos.b), FROM_EXT_ANG(pos.c), FROM_EXT_LEN(pos.u), FROM_EXT_LEN(pos.v), FROM_EXT_LEN(pos.w));

   // now calculate position in program units, for interpreter
   position = unoffset_and_unrotate_pos(canonEndPoint);
   to_prog(position);

   return position;
}

CANON_POSITION GET_EXTERNAL_PROBE_POSITION()
{
   CANON_POSITION position;
   EmcPose pos;
   static CANON_POSITION last_probed_position;

   flush_segments();

   pos = emcStatus->motion.traj.probedPosition;

   // first update internal record of last position
   pos.tran.x = FROM_EXT_LEN(pos.tran.x);
   pos.tran.y = FROM_EXT_LEN(pos.tran.y);
   pos.tran.z = FROM_EXT_LEN(pos.tran.z);

   pos.a = FROM_EXT_ANG(pos.a);
   pos.b = FROM_EXT_ANG(pos.b);
   pos.c = FROM_EXT_ANG(pos.c);

   pos.u = FROM_EXT_LEN(pos.u);
   pos.v = FROM_EXT_LEN(pos.v);
   pos.w = FROM_EXT_LEN(pos.w);

   // now calculate position in program units, for interpreter
   position = unoffset_and_unrotate_pos(pos);
   to_prog(position);

   if (probefile != NULL)
   {
      if (last_probed_position != position)
      {
         fprintf(probefile, "%f %f %f %f %f %f %f %f %f\n",
                 position.x, position.y, position.z, position.a, position.b, position.c, position.u, position.v, position.w);
         last_probed_position = position;
      }
   }

   return position;
}

int GET_EXTERNAL_PROBE_TRIPPED_VALUE()
{
   return emcStatus->motion.traj.probe_tripped;
}

double GET_EXTERNAL_PROBE_VALUE()
{
   // only for analog non-contact probe, so force a 0
   return 0.0;
}

// feed rate wanted is in program units per minute
double GET_EXTERNAL_FEED_RATE()
{
   double feed;

   // convert from internal to program units
   // it is wrong to use emcStatus->motion.traj.velocity here, as that is the traj speed regardless of G0 / G1
   feed = TO_PROG_LEN(currentLinearFeedRate);
   // now convert from per-sec to per-minute
   feed *= 60.0;

   return feed;
}

// traverse rate wanted is in program units per minute
double GET_EXTERNAL_TRAVERSE_RATE()
{
   double traverse;

   // convert from external to program units
   traverse = TO_PROG_LEN(FROM_EXT_LEN(emcStatus->motion.traj.maxVelocity));

   // now convert from per-sec to per-minute
   traverse *= 60.0;

   return traverse;
}

double GET_EXTERNAL_LENGTH_UNITS(void)
{
   double u;

   u = emcStatus->motion.traj.linearUnits;

   if (u == 0)
   {
      CANON_ERROR("external length units are zero");
      return 1.0;
   }
   else
   {
      return u;
   }
}

double GET_EXTERNAL_ANGLE_UNITS(void)
{
   double u;

   u = emcStatus->motion.traj.angularUnits;

   if (u == 0)
   {
      CANON_ERROR("external angle units are zero");
      return 1.0;
   }
   else
   {
      return u;
   }
}

int GET_EXTERNAL_MIST()
{
   return emcStatus->io.coolant.mist;
}

int GET_EXTERNAL_FLOOD()
{
   return emcStatus->io.coolant.flood;
}

double GET_EXTERNAL_SPEED()
{
   // speed is in RPMs everywhere
   return emcStatus->motion.spindle.speed;
}

CANON_DIRECTION GET_EXTERNAL_SPINDLE()
{
   if (emcStatus->motion.spindle.speed == 0)
   {
      return CANON_STOPPED;
   }

   if (emcStatus->motion.spindle.speed >= 0.0)
   {
      return CANON_CLOCKWISE;
   }

   return CANON_COUNTERCLOCKWISE;
}

int GET_EXTERNAL_POCKETS_MAX()
{
   return CANON_POCKETS_MAX;
}

#if 0
char _parameter_file_name[LINELEN];     /* Not static.Driver
                                           writes */

void GET_EXTERNAL_PARAMETER_FILE_NAME(char *file_name,  /* string: to copy
                                                           file name into */
                                      int max_size)
{       /* maximum number of characters to copy */
   // Paranoid checks
   if (0 == file_name)
      return;

   if (max_size < 0)
      return;

   if (strlen(_parameter_file_name) < ((size_t) max_size))
      strcpy(file_name, _parameter_file_name);
   else
      file_name[0] = 0;
}
#endif

double GET_EXTERNAL_POSITION_X(void)
{
   CANON_POSITION position;
   position = GET_EXTERNAL_POSITION();
   return position.x;
}

double GET_EXTERNAL_POSITION_Y(void)
{
   CANON_POSITION position;
   position = GET_EXTERNAL_POSITION();
   return position.y;
}

double GET_EXTERNAL_POSITION_Z(void)
{
   CANON_POSITION position;
   position = GET_EXTERNAL_POSITION();
   return position.z;
}

double GET_EXTERNAL_POSITION_A(void)
{
   CANON_POSITION position;
   position = GET_EXTERNAL_POSITION();
   return position.a;
}

double GET_EXTERNAL_POSITION_B(void)
{
   CANON_POSITION position;
   position = GET_EXTERNAL_POSITION();
   return position.b;
}

double GET_EXTERNAL_POSITION_C(void)
{
   CANON_POSITION position;
   position = GET_EXTERNAL_POSITION();
   return position.c;
}

double GET_EXTERNAL_POSITION_U(void)
{
   CANON_POSITION position;
   position = GET_EXTERNAL_POSITION();
   return position.u;
}

double GET_EXTERNAL_POSITION_V(void)
{
   CANON_POSITION position;
   position = GET_EXTERNAL_POSITION();
   return position.v;
}

double GET_EXTERNAL_POSITION_W(void)
{
   CANON_POSITION position;
   position = GET_EXTERNAL_POSITION();
   return position.w;
}

double GET_EXTERNAL_PROBE_POSITION_X(void)
{
   CANON_POSITION position;
   position = GET_EXTERNAL_PROBE_POSITION();
   return position.x;
}

double GET_EXTERNAL_PROBE_POSITION_Y(void)
{
   CANON_POSITION position;
   position = GET_EXTERNAL_PROBE_POSITION();
   return position.y;
}

double GET_EXTERNAL_PROBE_POSITION_Z(void)
{
   CANON_POSITION position;
   position = GET_EXTERNAL_PROBE_POSITION();
   return position.z;
}

double GET_EXTERNAL_PROBE_POSITION_A(void)
{
   CANON_POSITION position;
   position = GET_EXTERNAL_PROBE_POSITION();
   return position.a;
}

double GET_EXTERNAL_PROBE_POSITION_B(void)
{
   CANON_POSITION position;
   position = GET_EXTERNAL_PROBE_POSITION();
   return position.b;
}

double GET_EXTERNAL_PROBE_POSITION_C(void)
{
   CANON_POSITION position;
   position = GET_EXTERNAL_PROBE_POSITION();
   return position.c;
}

double GET_EXTERNAL_PROBE_POSITION_U(void)
{
   CANON_POSITION position;
   position = GET_EXTERNAL_PROBE_POSITION();
   return position.u;
}

double GET_EXTERNAL_PROBE_POSITION_V(void)
{
   CANON_POSITION position;
   position = GET_EXTERNAL_PROBE_POSITION();
   return position.v;
}

double GET_EXTERNAL_PROBE_POSITION_W(void)
{
   CANON_POSITION position;
   position = GET_EXTERNAL_PROBE_POSITION();
   return position.w;
}

CANON_MOTION_MODE GET_EXTERNAL_MOTION_CONTROL_MODE()
{
   return canonMotionMode;
}

double GET_EXTERNAL_MOTION_CONTROL_TOLERANCE()
{
   return TO_PROG_LEN(canonMotionTolerance);
}


CANON_UNITS GET_EXTERNAL_LENGTH_UNIT_TYPE()
{
   return lengthUnits;
}

int GET_EXTERNAL_QUEUE_EMPTY(void)
{
   flush_segments();

   return emcStatus->motion.traj.queue == 0 ? 1 : 0;
}

int GET_EXTERNAL_TOOL_SLOT()
{
   return emcStatus->io.tool.toolInSpindle;
}

int GET_EXTERNAL_SELECTED_TOOL_SLOT()
{
   return emcStatus->io.tool.pocketPrepped;
}

int GET_EXTERNAL_FEED_OVERRIDE_ENABLE()
{
   return emcStatus->motion.traj.feed_override_enabled;
}

int GET_EXTERNAL_SPINDLE_OVERRIDE_ENABLE()
{
   return emcStatus->motion.traj.spindle_override_enabled;
}

int GET_EXTERNAL_ADAPTIVE_FEED_ENABLE()
{
   return emcStatus->motion.traj.adaptive_feed_enabled;
}

int GET_EXTERNAL_FEED_HOLD_ENABLE()
{
   return emcStatus->motion.traj.feed_hold_enabled;
}

int GET_EXTERNAL_AXIS_MASK()
{
   return emcStatus->motion.traj.axis_mask;
}

CANON_PLANE GET_EXTERNAL_PLANE()
{
   return activePlane;
}

/* returns current value of the digital input selected by index.*/
int GET_EXTERNAL_DIGITAL_INPUT(int index, int def)
{
   if ((index < 0) || (index >= EMC_MAX_DIO))
      return -1;

   if (emcStatus->task.input_timeout == 1)
      return -1;

#ifdef INPUT_DEBUG
   printf("GET_EXTERNAL_DIGITAL_INPUT called\n di[%d]=%d \n timeout=%d \n", index, emcStatus->motion.synch_di[index], emcStatus->task.input_timeout);
#endif
   return (emcStatus->motion.synch_di[index] != 0) ? 1 : 0;
}

double GET_EXTERNAL_ANALOG_INPUT(int index, double def)
{
/* returns current value of the analog input selected by index.*/
#ifdef INPUT_DEBUG
   printf("GET_EXTERNAL_ANALOG_INPUT called\n ai[%d]=%g \n timeout=%d \n", index, emcStatus->motion.analog_input[index], emcStatus->task.input_timeout);
#endif
   if ((index < 0) || (index >= EMC_MAX_AIO))
      return -1;

   if (emcStatus->task.input_timeout == 1)
      return -1;

   return emcStatus->motion.analog_input[index];
}

void EXEC_USER_DEFINED_FUNCTION(int index, double p_number, double q_number)
{
   emc_system_cmd_msg_t emc_system_cmd_msg = { {EMC_SYSTEM_CMD_TYPE} };

   flush_segments();

   emc_system_cmd_msg.index = index;
   emc_system_cmd_msg.p_number = p_number;
   emc_system_cmd_msg.q_number = q_number;

   interp_list.append((emc_command_msg_t *)&emc_system_cmd_msg);
}

/*! \function SET_MOTION_OUTPUT_BIT

  sets a DIO pin
  this message goes to task, then to motion which sets the DIO 
  when the first motion starts.
  The pin gets set with value 1 at the begin of motion, and stays 1 at the end of motion
  (this behaviour can be changed if needed)
  
  warning: setting more then one for a motion segment will clear out the previous ones 
  (the TP doesn't implement a queue of these), 
  use SET_AUX_OUTPUT_BIT instead, that allows to set the value right away
*/
void SET_MOTION_OUTPUT_BIT(int index)
{
#if 0
   emc_motion_set_dout_msg_t dout_msg = { {EMC_MOTION_SET_DOUT_TYPE} };

   flush_segments();

   dout_msg.index = index;
   dout_msg.start = 1;  // startvalue = 1
   dout_msg.end = 1;    // endvalue = 1, means it doesn't get reset after current motion
   dout_msg.now = 0;    // not immediate, but synched with motion (goes to the TP)

   interp_list.append((emc_command_msg_t *) & dout_msg);
#endif
   return;
}

/*! \function CLEAR_MOTION_OUTPUT_BIT

  clears a DIO pin
  this message goes to task, then to motion which clears the DIO 
  when the first motion starts.
  The pin gets set with value 0 at the begin of motion, and stays 0 at the end of motion
  (this behaviour can be changed if needed)
  
  warning: setting more then one for a motion segment will clear out the previous ones 
  (the TP doesn't implement a queue of these), 
  use CLEAR_AUX_OUTPUT_BIT instead, that allows to set the value right away
*/
void CLEAR_MOTION_OUTPUT_BIT(int index)
{
#if 0
   emc_motion_set_dout_msg_t dout_msg = { {EMC_MOTION_SET_DOUT_TYPE} };

   flush_segments();

   dout_msg.index = index;
   dout_msg.start = 0;  // startvalue = 1
   dout_msg.end = 0;    // endvalue = 0, means it stays 0 after current motion
   dout_msg.now = 0;    // not immediate, but synched with motion (goes to the TP)

   interp_list.append((emc_command_msg_t *) & dout_msg);
#endif
   return;
}

/*! \function SET_AUX_OUTPUT_BIT

  sets a DIO pin
  this message goes to task, then to motion which sets the DIO 
  right away.
  The pin gets set with value 1 at the begin of motion, and stays 1 at the end of motion
  (this behaviour can be changed if needed)
  you can use any number of these, as the effect is imediate  
*/
void SET_AUX_OUTPUT_BIT(int index)
{
#if 0
   emc_motion_set_dout_msg_t dout_msg = { {EMC_MOTION_SET_DOUT_TYPE} };

   flush_segments();

   dout_msg.index = index;
   dout_msg.start = 1;  // startvalue = 1
   dout_msg.end = 1;    // endvalue = 1, means it doesn't get reset after current motion
   dout_msg.now = 1;    // immediate, we don't care about synching for AUX

   interp_list.append((emc_command_msg_t *) & dout_msg);
#endif
   return;
}

/*! \function CLEAR_AUX_OUTPUT_BIT

  clears a DIO pin
  this message goes to task, then to motion which clears the DIO 
  right away.
  The pin gets set with value 0 at the begin of motion, and stays 0 at the end of motion
  (this behaviour can be changed if needed)
  you can use any number of these, as the effect is imediate  
*/
void CLEAR_AUX_OUTPUT_BIT(int index)
{
#if 0
   emc_motion_set_dout_msg_t dout_msg = { {EMC_MOTION_SET_DOUT_TYPE} };

   flush_segments();

   dout_msg.index = index;
   dout_msg.start = 0;  // startvalue = 1
   dout_msg.end = 0;    // endvalue = 0, means it stays 0 after current motion
   dout_msg.now = 1;    // immediate, we don't care about synching for AUX

   interp_list.append((emc_command_msg_t *) & dout_msg);
#endif
   return;
}

/*! \function SET_MOTION_OUTPUT_VALUE

  sets a AIO value, not used by the RS274 Interp,
  not fully implemented in the motion controller either
*/
void SET_MOTION_OUTPUT_VALUE(int index, double value)
{
   emc_motion_set_aout_msg_t aout_msg = { {EMC_MOTION_SET_AOUT_TYPE} };

   flush_segments();

   aout_msg.index = index;      // which output
   aout_msg.start = value;      // start value
   aout_msg.end = value;        // end value
   aout_msg.now = 0;    // immediate=1, or synched when motion start=0

   interp_list.append((emc_command_msg_t *) & aout_msg);

   return;
}

/*! \function SET_AUX_OUTPUT_VALUE

  sets a AIO value, not used by the RS274 Interp,
  not fully implemented in the motion controller either
*/
void SET_AUX_OUTPUT_VALUE(int index, double value)
{
   emc_motion_set_aout_msg_t aout_msg = { {EMC_MOTION_SET_AOUT_TYPE} };

   flush_segments();

   aout_msg.index = index;      // which output
   aout_msg.start = value;      // start value
   aout_msg.end = value;        // end value
   aout_msg.now = 1;    // immediate=1, or synched when motion start=0

   interp_list.append((emc_command_msg_t *) & aout_msg);

   return;
}

/*! \function WAIT
   program execution and interpreting is stopped until the input selected by 
   index changed to the needed state (specified by wait_type).
   Return value: either wait_type if timeout didn't occur, or -1 otherwise. */

int WAIT(int index,             /* index of the motion exported input */
         int input_type,        /*DIGITAL_INPUT or ANALOG_INPUT */
         int wait_type,         /* 0 - rise, 1 - fall, 2 - be high, 3 - be low */
         double timeout)        /* time to wait [in seconds], if the input didn't change the value -1 is returned */
{
   if (input_type == DIGITAL_INPUT)
   {
      if ((index < 0) || (index >= EMC_MAX_DIO))
         return -1;
   }
   else if (input_type == ANALOG_INPUT)
   {
      if ((index < 0) || (index >= EMC_MAX_AIO))
         return -1;
   }

   emc_aux_input_wait_msg_t wait_msg = { {EMC_AUX_INPUT_WAIT_TYPE}
   };

   flush_segments();

   wait_msg.index = index;
   wait_msg.input_type = input_type;
   wait_msg.wait_type = wait_type;
   wait_msg.timeout = timeout;

   interp_list.append((emc_command_msg_t *) & wait_msg);
   return 0;
}
