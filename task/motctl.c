/********************************************************************
* motctl.c - Motion controller for EMC2. All state logic and trajectory
*            calcs are called from here.
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author:
* License: GPL Version 2
* Created on:
* System: Linux
*
* Copyright (c) 2004 All rights reserved.
********************************************************************/

#include <math.h>
#include "emc.h"
#include "bug.h"

static int first_pass = 1;      /* used to set initial conditions */

/* these variables have the servo cycle time and 1/cycle time */
static double servo_period;
static double servo_freq;

//static long base_period_nsec = 50000;       /* fastest thread period */
//static long servo_period_nsec = 1000000;        /* servo thread period */
//static long traj_period_nsec = 1000000;       /* trajectory planner period */

/* Length of delay between homing motions - this is intended to
   ensure that all motion has ceased and switch bouncing has
   ended.  We might want to make this user adjustable, but for
   now it's a constant.  It is in seconds */
#define HOME_DELAY 0.100

/* variable used internally by do_homing */
static int immediate_state;

/* 'process_inputs()' is responsible for reading hardware input
   signals (from the HAL) and doing basic processing on them.  In
   the case of position feedback, that means removing backlash or
   screw error comp and calculating the following error.  For 
   switches, it means debouncing them and setting flags in the
   emcmotStatus structure.
*/
static void process_inputs(void);

/* 'do forward kins()' takes the position feedback in joint coords
   and applies the forward kinematics to it to generate feedback
   in Cartesean coordinates.  It has code to handle machines that
   don't have forward kins, and other special cases, such as when
   the joints have not been homed.
*/
static void do_forward_kins(void);

/* probe inputs need to be handled after forward kins are run, since
   cartesian feedback position is latched when the probe fires, and it
   should be based on the feedback read in on this servo cycle.
*/
//static void process_probe_inputs(void);

/* 'check_for_faults()' is responsible for detecting fault conditions
   such as limit switches, amp faults, following error, etc.  It only
   checks active axes.  It is also responsible for generating an error
   message.  (Later, once I understand the cmd/status/error interface
   better, it will probably generate error codes that can be passed
   up the architecture toward the GUI - printing error messages
   directly seems a little messy)
*/
static void check_for_faults(void);

/* 'set_operating_mode()' handles transitions between the operating
   modes, which are free, coordinated, and teleop.  This stuff needs
   to be better documented.  It is basically a state machine, with
   a current state, a desired state, and rules determining when the
   state can change.  It should be rewritten as such, but for now
   it consists of code copied exactly from emc1.
*/
static void set_operating_mode(void);

/* 'handle_jogwheels()' reads jogwheels, decides if they should be
   enabled, and if so, changes the free mode planner's target position
   when the jogwheel(s) turn.
*/
//static void handle_jogwheels(void);

/* 'do_homing_sequence()' looks at emcmotStatus->homingSequenceState 
   to decide what, if anything, needs to be done related to multi-joint
   homing.

   no prototype here, implemented in homing.c, proto in mot_priv.h
 */

/* 'do_homing()' looks at the home_state field of each joint struct
    to decide what, if anything, needs to be done related to homing
    the joint.  Homing is implemented as a state machine, the exact
    sequence of states depends on the machine configuration.  It
    can be as simple as immediately setting the current position to
    zero, or a it can be a multi-step process (find switch, set
    approximate zero, back off switch, find index, set final zero,
    rapid to home position), or anywhere in between.

   no prototype here, implemented in homing.c, proto in mot_priv.h
*/

/* 'get_pos_cmds()' generates the position setpoints.  This includes
   calling the trajectory planner and interpolating its outputs.
*/
static void get_pos_cmds(long period);

/* 'compute_screw_comp()' is responsible for calculating backlash and
   lead screw error compensation.  (Leadscrew error compensation is
   a more sophisticated version that includes backlash comp.)  It uses
   the velocity in emcmotStatus->joint_vel_cmd to determine which way
   each joint is moving, and the position in emcmotStatus->joint_pos_cmd
   to determine where the joint is at.  That information is used to
   create the compensation value that is added to the joint_pos_cmd
   to create motor_pos_cmd, and is subtracted from motor_pos_fb to
   get joint_pos_fb.  (This function does not add or subtract the
   compensation value, it only computes it.)  The basic compensation
   value is in backlash_corr, however has makes step changes when
   the direction reverses.  backlash_filt is a ramped version, and
   that is the one that is later added/subtracted from the position.
*/
static void compute_screw_comp(void);

/* 'output_pos_cmds()' handles the final stages of the
   control function.  It applies screw comp and writes the
   final motor position to the output device.
*/
static void output_pos_cmds(void);

/* 'update_status()' copies assorted status information to shared
   memory (the emcmotStatus structure) so that it is available to
   higher level code.
*/
static void update_status(void);
static void do_homing(void);

/*
  emcmotController() runs the trajectory and interpolation calculations
  each control cycle

  This function gets called at regular intervals - therefore it does NOT
  have a loop within it!

  Inactive axes are still calculated, but the PIDs are inhibited and
  the amp enable/disable are inhibited
  */
void emcmotController(long period)
{
   /* calculate servo period as a double - period is in integer nsec */
   servo_period = period * 0.000000001;

   /* calculate servo frequency for calcs like vel = Dpos / period */
   /* it's faster to do vel = Dpos * freq */
   servo_freq = 1.0 / servo_period;
   /* increment head count to indicate work in progress */
   emcmotStatus.head++;

   /* here begins the core of the controller */
   process_inputs();
   do_forward_kins();
//    process_probe_inputs();
   check_for_faults();
   set_operating_mode();
//    handle_jogwheels();
//    do_homing_sequence();
   do_homing();
   get_pos_cmds(period);
   compute_screw_comp();
   output_pos_cmds();
   update_status();

   /* here ends the core of the controller */
   emcmotStatus.heartbeat++;
   /* set tail to head, to indicate work complete */
   emcmotStatus.tail = emcmotStatus.head;
   /* clear init flag */
   first_pass = 0;

}       /* emcmotController() */

/* call this when setting the trajectory cycle time */
int emcmotSetTrajCycleTime(double secs)
{
   DBG("MOTION: setting Traj cycle time to %ld nsecs\n", (long) (secs * 1e9));

   /* make sure it's not zero */
   if (secs <= 0.0)
   {
      return EMC_R_ERROR;
   }

   emcmot_config_change();

   /* compute the interpolation rate as nearest integer to traj/servo */
   if (emcmotConfig.servoCycleTime)
      emcmotConfig.interpolationRate = (int) (secs / emcmotConfig.servoCycleTime + 0.5);
   else
      emcmotConfig.interpolationRate = 1;

   /* set traj planner */
   tpSetCycleTime(&emcmotDebug.queue, secs);

   /* set the free planners, cubic interpolation rate and segment time */
// Moved to emcMotionInit(). DES 
//   for (t = 0; t < emcmotStatus.traj.axes; t++)
//   {
//      cubicSetInterpolationRate(&(joints[t].cubic), emcmotConfig.interpolationRate);
//   }

   /* copy into status out */
   emcmotConfig.trajCycleTime = secs;

   return EMC_R_OK;
}       /* emcmotSetTrajCycleTime() */

/* call this when setting the servo cycle time */
int emcmotSetServoCycleTime(double secs)
{
   DBG("MOTION: setting Servo cycle time to %ld nsecs\n", (long) (secs * 1e9));

   /* make sure it's not zero */
   if (secs <= 0.0)
   {
      return EMC_R_ERROR;
   }

   emcmot_config_change();

   /* compute the interpolation rate as nearest integer to traj/servo */
   emcmotConfig.interpolationRate = (int) (emcmotConfig.trajCycleTime / secs + 0.5);

   /* set the cubic interpolation rate and PID cycle time */
// Moved to emcMotionInit(). DES 
//   for (t = 0; t < emcmotStatus.traj.axes; t++)
//   {
//      cubicSetInterpolationRate(&(joints[t].cubic), emcmotConfig.interpolationRate);
//      cubicSetSegmentTime(&(joints[t].cubic), secs);
//   }

   /* copy into status out */
   emcmotConfig.servoCycleTime = secs;

   return EMC_R_OK;
}       /* emcmotSetServoCycleTime() */

#if 0
static void process_probe_inputs(void)
{
   static int old_probeVal = 0;
   unsigned char probe_type = emcmotStatus.probe_type;

   // don't error
   char probe_suppress = probe_type & 1;

   // trigger when the probe clears, instead of the usual case of triggering when it trips
   char probe_whenclears = ! !(probe_type & 2);

   /* read probe input */
//    emcmotStatus.probeVal = *(emcmot_hal_data->probe_input);
   if (emcmotStatus.probing)
   {
      /* check if the probe has been tripped */
      if (emcmotStatus.probeVal ^ probe_whenclears)
      {
         /* remember the current position */
         emcmotStatus.probedPos = emcmotStatus.carte_pos_fb;
         /* stop! */
         tpAbort(&emcmotDebug.queue);
         emcmotStatus.probing = 0;
         emcmotStatus.probeTripped = 1;
         /* check if the probe hasn't tripped, but the move finished */
      }
      else if (GET_MOTION_INPOS_FLAG() && tpQueueDepth(&emcmotDebug.queue) == 0)
      {
         /* we are already stopped, but we need to remember the current 
            position here, because it will still be queried */
         emcmotStatus.probedPos = emcmotStatus.carte_pos_fb;
         emcmotStatus.probing = 0;
         if (probe_suppress)
         {
            emcmotStatus.probeTripped = 0;
         }
         else if (probe_whenclears)
         {
            reportError("G38.4 move finished without breaking contact.");
            SET_MOTION_ERROR_FLAG(1);
         }
         else
         {
            reportError("G38.2 move finished without making contact.");
            SET_MOTION_ERROR_FLAG(1);
         }
      }
   }
   else if (!old_probeVal && emcmotStatus.probeVal)
   {
      // not probing, but we have a rising edge on the probe.
      // this could be expensive if we don't stop.
      int i;
      int aborted = 0;

      if (!GET_MOTION_INPOS_FLAG() && tpQueueDepth(&emcmotDebug.queue) && tpGetExecId(&emcmotDebug.queue) <= 0)
      {
         // running an MDI command
         reportError("Probe tripped during non-probe MDI command.");
         tpAbort(&emcmotDebug.queue);
      }

      for (i = 0; i < emcmotStatus.traj.axes; i++)
      {
         emcmot_joint_t *joint = &joints[i];

         if (!GET_JOINT_ACTIVE_FLAG(joint))
         {
            /* if joint is not active, skip it */
            continue;
         }

         // abort any homing
         if (GET_JOINT_HOMING_FLAG(joint))
         {
            joint->home_state = HOME_ABORT;
            aborted = 1;
         }

         // abort any jogs
         if (joint->free_tp_enable == 1)
         {
            joint->free_tp_enable = 0;
            // since homing uses free_tp, this protection of aborted
            // is needed so the user gets the correct error.
            if (!aborted)
               aborted = 2;
         }
      }

      if (aborted == 1)
      {
         reportError("Probe tripped during homing motion.");
      }

      if (aborted == 2)
      {
         reportError("Probe tripped during a jog.");
      }
   }
   old_probeVal = emcmotStatus.probeVal;
}
#endif

static void process_inputs(void)
{
   int joint_num;
   double abs_ferror, scale;
//    joint_hal_t *joint_data;
   emcmot_joint_t *joint;
   unsigned char enables;

   /* read spindle angle (for threading, etc) */
//    emcmotStatus.spindleRevs = *emcmot_hal_data->spindle_revs;
//    emcmotStatus.spindleSpeedIn = *emcmot_hal_data->spindle_speed_in;
//    emcmotStatus.spindle_is_atspeed = *emcmot_hal_data->spindle_is_atspeed;

   /* Dongle change. */
   emcmotStatus.spindle_is_atspeed = 1;

   /* compute net feed and spindle scale factors */
   if (emcmotStatus.motion_state == EMCMOT_MOTION_COORD)
   {
      /* use the enables that were queued with the current move */
      enables = emcmotStatus.enables_queued;
   }
   else
   {
      /* use the enables that are in effect right now */
      enables = emcmotStatus.enables_new;
   }

   /* feed scaling first:  feed_scale, adaptive_feed, and feed_hold */
   scale = 1.0;
   if (enables & FS_ENABLED)
   {
      scale *= emcmotStatus.feed_scale;
   }
#if 0
   if (enables & AF_ENABLED)
   {
      /* read and clamp (0.0 to 1.0) adaptive feed HAL pin */
      tmp = *emcmot_hal_data->adaptive_feed;
      if (tmp > 1.0)
      {
         tmp = 1.0;
      }
      else if (tmp < 0.0)
      {
         tmp = 0.0;
      }
      scale *= tmp;
   }
   if (enables & FH_ENABLED)
   {
      /* read feed hold HAL pin */
      if (*emcmot_hal_data->feed_hold)
      {
         scale = 0;
      }
   }
#endif

   /* save the resulting combined scale factor */
   emcmotStatus.net_feed_scale = scale;

   /* now do spindle scaling: only one item to consider */
   scale = 1.0;
   if (enables & SS_ENABLED)
   {
      scale *= emcmotStatus.spindle_scale;
   }
   /* save the resulting combined scale factor */
   emcmotStatus.net_spindle_scale = scale;

   /* read and process per-joint inputs */
   for (joint_num = 0; joint_num < emcmotStatus.traj.axes; joint_num++)
   {
      /* point to joint HAL data */
//      joint_data = &(emcmot_hal_data->joint[joint_num]);
      /* point to joint data */
      joint = &joints[joint_num];
      if (!GET_JOINT_ACTIVE_FLAG(joint))
      {
         /* if joint is not active, skip it */
         continue;
      }
      /* copy data from HAL to joint structure */
//      joint->index_enable = *(joint_data->index_enable);
//        joint->motor_pos_fb = *(joint_data->motor_pos_fb);

      /* dongle change... no feedback, open loop */
      joint->motor_pos_fb = joint->motor_pos_cmd;

      /* calculate pos_fb */
      if ((joint->home_state == HOME_INDEX_SEARCH_WAIT) && (joint->index_enable == 0))
      {
         /* special case - we're homing the joint, and it just
            hit the index.  The encoder count might have made a
            step change.  The homing code will correct for it
            later, so we ignore motor_pos_fb and set pos_fb
            to match the commanded value instead. */
         joint->pos_fb = joint->pos_cmd;
      }
      else
      {
         /* normal case: subtract backlash comp and motor offset */
         joint->pos_fb = joint->motor_pos_fb - (joint->backlash_filt + joint->motor_offset);
      }

      /* calculate following error */
      joint->ferror = joint->pos_cmd - joint->pos_fb;
      abs_ferror = fabs(joint->ferror);

      /* update maximum ferror if needed */
      if (abs_ferror > joint->ferror_high_mark)
      {
         joint->ferror_high_mark = abs_ferror;
      }

      /* calculate following error limit */
      if (joint->vel_limit > 0.0)
      {
         joint->ferror_limit = joint->max_ferror * fabs(joint->vel_cmd) / joint->vel_limit;
      }
      else
      {
         joint->ferror_limit = 0;
      }
      if (joint->ferror_limit < joint->min_ferror)
      {
         joint->ferror_limit = joint->min_ferror;
      }
      /* update following error flag */
      if (abs_ferror > joint->ferror_limit)
      {
         SET_JOINT_FERROR_FLAG(joint, 1);
      }
      else
      {
         SET_JOINT_FERROR_FLAG(joint, 0);
      }

      /* read limit switches */
//        if (*(joint_data->pos_lim_sw)) {
//            SET_JOINT_PHL_FLAG(joint, 1);
//        } else {
      SET_JOINT_PHL_FLAG(joint, 0);
//        }
//        if (*(joint_data->neg_lim_sw)) {
//            SET_JOINT_NHL_FLAG(joint, 1);
//        } else {
      SET_JOINT_NHL_FLAG(joint, 0);
//        }
      joint->on_pos_limit = GET_JOINT_PHL_FLAG(joint);
      joint->on_neg_limit = GET_JOINT_NHL_FLAG(joint);

      /* read amp fault input */
//        if (*(joint_data->amp_fault)) {
//            SET_JOINT_FAULT_FLAG(joint, 1);
//        } else {
      SET_JOINT_FAULT_FLAG(joint, 0);
//        }

      /* read home switch input */
//        if (*(joint_data->home_sw)) {
//            SET_JOINT_HOME_SWITCH_FLAG(joint, 1);
//        } else {
      SET_JOINT_HOME_SWITCH_FLAG(joint, 0);
//        }
      /* end of read and process joint inputs loop */
   }
}       /* process_inputs() */

static void do_forward_kins(void)
{
/* there are four possibilities for kinType:

   IDENTITY: Both forward and inverse kins are available, and they
   can used without an initial guess, even if one or more joints
   are not homed.  In this case, we apply the forward kins to the
   joint->pos_fb to produce carte_pos_fb, and if all axes are homed
   we set carte_pos_fb_ok to 1 to indicate that the feedback data
   is good.

   BOTH: Both forward and inverse kins are available, but the forward
   kins need an initial guess, and/or the kins require all joints to
   be homed before they work properly.  Here we must tread carefully.
   IF all the joints have been homed, we apply the forward kins to
   the joint->pos_fb to produce carte_pos_fb, and set carte_pos_fb_ok
   to indicate that the feedback is good.  We use the previous value
   of carte_pos_fb as the initial guess.  If all joints have not been
   homed, we don't call the kinematics, instead we set carte_pos_fb to
   the cartesean coordinates of home, as stored in the global worldHome,
   and we set carte_fb_ok to 0 to indicate that the feedback is invalid.
\todo  FIXME - maybe setting to home isn't the right thing to do.  We need
   it to be set to home eventually, (right before the first attemt to
   run the kins), but that doesn't mean we should say we're at home
   when we're not.

   INVERSE_ONLY: Only inverse kinematics are available, forward
   kinematics cannot be used.  So we have to fake it, the question is
   simply "what way of faking it is best".  In free mode, or if all
   axes have not been homed, the feedback position is unknown.  If
   we are in teleop or coord mode, or if we are in free mode and all
   axes are homed, and haven't been moved since they were homed, then
   we set carte_pos_fb to carte_pos_cmd, and set carte_pos_fb_ok to 1.
   If we are in free mode, and any joint is not homed, or any joint has
   moved since it was homed, we leave cart_pos_fb alone, and set
   carte_pos_fb_ok to 0.

   FORWARD_ONLY: Only forward kinematics are available, inverse kins
   cannot be used.  This exists for completeness only, since EMC won't
   work without inverse kinematics.

*/

/*! \todo FIXME FIXME FIXME - need to put a rate divider in here, run it
   at the traj rate */

   double joint_pos[EMCMOT_MAX_JOINTS] = { 0, };
   int joint_num, result;
   emcmot_joint_t *joint;

   /* copy joint position feedback to local array */
   for (joint_num = 0; joint_num < emcmotStatus.traj.axes; joint_num++)
   {
      /* point to joint struct */
      joint = &joints[joint_num];
      /* copy feedback */
      joint_pos[joint_num] = joint->pos_fb;
   }
   switch (emcmotStatus.traj.kinematics_type)
   {

   case KINEMATICS_IDENTITY:
      kinematicsForward(joint_pos, &emcmotStatus.carte_pos_fb, &fflags, &iflags);
      if (emcmotCheckAllHomed())
      {
         emcmotStatus.carte_pos_fb_ok = 1;
      }
      else
      {
         emcmotStatus.carte_pos_fb_ok = 0;
      }
      break;

   case KINEMATICS_BOTH:
      if (emcmotCheckAllHomed())
      {
         /* is previous value suitable for use as initial guess? */
         if (!emcmotStatus.carte_pos_fb_ok)
         {
            /* no, use home position as initial guess */
            emcmotStatus.carte_pos_fb = emcmotStatus.world_home;
         }
         /* calculate Cartesean position feedback from joint pos fb */
         result = kinematicsForward(joint_pos, &emcmotStatus.carte_pos_fb, &fflags, &iflags);
         /* check to make sure kinematics converged */
         if (result < 0)
         {
            /* error during kinematics calculations */
            emcmotStatus.carte_pos_fb_ok = 0;
         }
         else
         {
            /* it worked! */
            emcmotStatus.carte_pos_fb_ok = 1;
         }
      }
      else
      {
         emcmotStatus.carte_pos_fb_ok = 0;
      }
      break;

   case KINEMATICS_INVERSE_ONLY:

      if ((GET_MOTION_COORD_FLAG()) || (GET_MOTION_TELEOP_FLAG()))
      {
         /* use Cartesean position command as feedback value */
         emcmotStatus.carte_pos_fb = emcmotStatus.carte_pos_cmd;
         emcmotStatus.carte_pos_fb_ok = 1;
      }
      else
      {
         emcmotStatus.carte_pos_fb_ok = 0;
      }
      break;

   default:
      emcmotStatus.carte_pos_fb_ok = 0;
      break;
   }
}       /* do_forward_kins() */

static void check_for_faults(void)
{
   int joint_num;
   emcmot_joint_t *joint;
   int neg_limit_override, pos_limit_override;

   /* check for various global fault conditions */
   /* only check enable input if running */
#if 0
   if (GET_MOTION_ENABLE_FLAG() != 0)
   {
      if (*(emcmot_hal_data->enable) == 0)
      {
         DBG("motion stopped by enable input\n");
         emcmotDebug.enabling = 0;
      }
   }
#endif

   /* check for various joint fault conditions */
   for (joint_num = 0; joint_num < emcmotStatus.traj.axes; joint_num++)
   {
      /* point to joint data */
      joint = &joints[joint_num];
      /* only check active, enabled axes */
      if (GET_JOINT_ACTIVE_FLAG(joint) && GET_JOINT_ENABLE_FLAG(joint))
      {
         /* are any limits for this joint overridden? */
         neg_limit_override = emcmotStatus.overrideLimitMask & (1 << (joint_num * 2));
         pos_limit_override = emcmotStatus.overrideLimitMask & (2 << (joint_num * 2));
         /* check for hard limits */
         if ((GET_JOINT_PHL_FLAG(joint) && !pos_limit_override) || (GET_JOINT_NHL_FLAG(joint) && !neg_limit_override))
         {
            /* joint is on limit switch, should we trip? */
            if (GET_JOINT_HOMING_FLAG(joint))
            {
               /* no, ignore limits */
            }
            else
            {
               /* trip on limits */
               if (!GET_JOINT_ERROR_FLAG(joint))
               {
                  /* report the error just this once */
                  BUG("joint %d on limit switch error\n", joint_num);
               }
               SET_JOINT_ERROR_FLAG(joint, 1);
               emcmotDebug.enabling = 0;
            }
         }
         /* check for amp fault */
         if (GET_JOINT_FAULT_FLAG(joint))
         {
            /* joint is faulted, trip */
            if (!GET_JOINT_ERROR_FLAG(joint))
            {
               /* report the error just this once */
               BUG("joint %d amplifier fault\n", joint_num);
            }
            SET_JOINT_ERROR_FLAG(joint, 1);
            emcmotDebug.enabling = 0;
         }
         /* check for excessive following error */
         if (GET_JOINT_FERROR_FLAG(joint))
         {
            if (!GET_JOINT_ERROR_FLAG(joint))
            {
               /* report the error just this once */
               BUG("joint %d following error\n", joint_num);
            }
            SET_JOINT_ERROR_FLAG(joint, 1);
            emcmotDebug.enabling = 0;
         }
      } /* if ( GET_JOINT_ACTIVE_FLAG(joint) && GET_JOINT_ENABLE_FLAG(joint) ) */
   }    /* for (joint_num = 0; joint_num < emcmotStatus.traj.axes; joint_num++) */
}       /*  check_for_faults() */

static void set_operating_mode(void)
{
   int joint_num;
   emcmot_joint_t *joint;
//    joint_hal_t *joint_data;

   /* check for disabling */
   if (!emcmotDebug.enabling && GET_MOTION_ENABLE_FLAG())
   {

      /* clear out the motion emcmotDebug.queue and interpolators */
      tpClear(&emcmotDebug.queue);
      for (joint_num = 0; joint_num < emcmotStatus.traj.axes; joint_num++)
      {
         /* point to joint data */
//            joint_data = &(emcmot_hal_data->joint[joint_num]);
         joint = &joints[joint_num];
         /* disable free mode planner */
         joint->free_tp_enable = 0;
         /* drain coord mode interpolators */
         cubicDrain(&(joint->cubic));
         if (GET_JOINT_ACTIVE_FLAG(joint))
         {
            SET_JOINT_ENABLE_FLAG(joint, 0);
            SET_JOINT_HOMING_FLAG(joint, 0);
            joint->home_state = HOME_IDLE;
         }
         /* don't clear the joint error flag, since that may signify why
            we just went into disabled state */
      }

      /* reset the trajectory interpolation counter, so that the kinematics
         functions called during the disabled state run at the nominal
         trajectory rate rather than the servo rate. It's loaded with
         emcmotConfig.interpolationRate when it goes to zero. */
/*! \todo FIXME - interpolation is still under construction */
/*! \todo Another #if 0 */
#if 0
      interpolationCounter = 0;
#endif
      SET_MOTION_ENABLE_FLAG(0);
      /* don't clear the motion error flag, since that may signify why we
         just went into disabled state */
   }

   /* check for emcmotDebug.enabling */
   if (emcmotDebug.enabling && !GET_MOTION_ENABLE_FLAG())
   {
      tpSetPos(&emcmotDebug.queue, emcmotStatus.carte_pos_cmd);
      for (joint_num = 0; joint_num < emcmotStatus.traj.axes; joint_num++)
      {
         /* point to joint data */
//            joint_data = &(emcmot_hal_data->joint[joint_num]);
         joint = &joints[joint_num];

         joint->free_pos_cmd = joint->pos_cmd;
         if (GET_JOINT_ACTIVE_FLAG(joint))
         {
            SET_JOINT_ENABLE_FLAG(joint, 1);
            SET_JOINT_HOMING_FLAG(joint, 0);
            joint->home_state = HOME_IDLE;
         }
         /* clear any outstanding joint errors when going into enabled state */
         SET_JOINT_ERROR_FLAG(joint, 0);
      }

      SET_MOTION_ENABLE_FLAG(1);
      /* clear any outstanding motion errors when going into enabled state */
      SET_MOTION_ERROR_FLAG(0);
   }

   /* check for entering teleop mode */
   if (emcmotDebug.teleoperating && !GET_MOTION_TELEOP_FLAG())
   {
      if (GET_MOTION_INPOS_FLAG())
      {

         /* update coordinated emcmotDebug.queue position */
         tpSetPos(&emcmotDebug.queue, emcmotStatus.carte_pos_cmd);
         /* drain the cubics so they'll synch up */
         for (joint_num = 0; joint_num < emcmotStatus.traj.axes; joint_num++)
         {
            /* point to joint data */
            joint = &joints[joint_num];
            cubicDrain(&(joint->cubic));
         }
         /* Initialize things to do when starting teleop mode. */
         ZERO_EMC_POSE(emcmotDebug.teleop_data.currentVel);
         ZERO_EMC_POSE(emcmotDebug.teleop_data.desiredVel);
         ZERO_EMC_POSE(emcmotDebug.teleop_data.currentAccell);
         ZERO_EMC_POSE(emcmotDebug.teleop_data.desiredAccell);
         SET_MOTION_TELEOP_FLAG(1);
         SET_MOTION_ERROR_FLAG(0);
      }
      else
      {
         /* not in position-- don't honor mode change */
         emcmotDebug.teleoperating = 0;
      }
   }
   else
   {
      if (GET_MOTION_INPOS_FLAG())
      {
         if (!emcmotDebug.teleoperating && GET_MOTION_TELEOP_FLAG())
         {
            SET_MOTION_TELEOP_FLAG(0);
            if (!emcmotDebug.coordinating)
            {
               for (joint_num = 0; joint_num < emcmotStatus.traj.axes; joint_num++)
               {
                  /* point to joint data */
                  joint = &joints[joint_num];
                  /* update free planner positions */
                  joint->free_pos_cmd = joint->pos_cmd;
               }
            }
         }
      }

      /* check for entering coordinated mode */
      if (emcmotDebug.coordinating && !GET_MOTION_COORD_FLAG())
      {
         if (GET_MOTION_INPOS_FLAG())
         {
            /* preset traj planner to current position */
            tpSetPos(&emcmotDebug.queue, emcmotStatus.carte_pos_cmd);
            /* drain the cubics so they'll synch up */
            for (joint_num = 0; joint_num < emcmotStatus.traj.axes; joint_num++)
            {
               /* point to joint data */
               joint = &joints[joint_num];
               cubicDrain(&(joint->cubic));
            }
            /* clear the override limits flags */
            emcmotDebug.overriding = 0;
            emcmotStatus.overrideLimitMask = 0;
            SET_MOTION_COORD_FLAG(1);
            SET_MOTION_TELEOP_FLAG(0);
            SET_MOTION_ERROR_FLAG(0);
         }
         else
         {
            /* not in position-- don't honor mode change */
            emcmotDebug.coordinating = 0;
         }
      }

      /* check entering free space mode */
      if (!emcmotDebug.coordinating && GET_MOTION_COORD_FLAG())
      {
         if (GET_MOTION_INPOS_FLAG())
         {
            for (joint_num = 0; joint_num < emcmotStatus.traj.axes; joint_num++)
            {
               /* point to joint data */
               joint = &joints[joint_num];
               /* set joint planner pos cmd to current location */
               joint->free_pos_cmd = joint->pos_cmd;
               /* but it can stay disabled until a move is required */
               joint->free_tp_enable = 0;
            }
            SET_MOTION_COORD_FLAG(0);
            SET_MOTION_TELEOP_FLAG(0);
            SET_MOTION_ERROR_FLAG(0);
         }
         else
         {
            /* not in position-- don't honor mode change */
            emcmotDebug.coordinating = 1;
         }
      }
   }
   /*! \todo FIXME - this code is temporary - eventually this function will be
      cleaned up and simplified, and 'motion_state' will become the master
      for this info, instead of having to gather it from several flags */
   if (!GET_MOTION_ENABLE_FLAG())
   {
      emcmotStatus.motion_state = EMCMOT_MOTION_DISABLED;
   }
   else if (GET_MOTION_TELEOP_FLAG())
   {
      emcmotStatus.motion_state = EMCMOT_MOTION_TELEOP;
   }
   else if (GET_MOTION_COORD_FLAG())
   {
      emcmotStatus.motion_state = EMCMOT_MOTION_COORD;
   }
   else
   {
      emcmotStatus.motion_state = EMCMOT_MOTION_FREE;
   }
}       /* set_operating_mode() */

#if 0
static void handle_jogwheels(void)
{
   int joint_num;
   emcmot_joint_t *joint;
   joint_hal_t *joint_data;
   int new_jog_counts, delta;
   double distance, pos, stop_dist;

   for (joint_num = 0; joint_num < emcmotStatus.traj.axes; joint_num++)
   {
      /* point to joint data */
      joint_data = &(emcmot_hal_data->joint[joint_num]);
      joint = &joints[joint_num];
      if (!GET_JOINT_ACTIVE_FLAG(joint))
      {
         /* if joint is not active, skip it */
         continue;
      }
      /* get counts from jogwheel */
      new_jog_counts = *(joint_data->jog_counts);
      delta = new_jog_counts - joint->old_jog_counts;
      /* save value for next time */
      joint->old_jog_counts = new_jog_counts;
      /* initialization complete */
      if (first_pass)
      {
         continue;
      }
      /* did the wheel move? */
      if (delta == 0)
      {
         /* no, nothing to do */
         continue;
      }
      /* must be in free mode and enabled */
      if (GET_MOTION_COORD_FLAG())
      {
         continue;
      }
      if (!GET_MOTION_ENABLE_FLAG())
      {
         continue;
      }
      /* the jogwheel input for this joint must be enabled */
      if (*(joint_data->jog_enable) == 0)
      {
         continue;
      }
      /* must not be homing */
      if (emcmotStatus.homing_active)
      {
         continue;
      }
      /* must not be doing a keyboard jog */
      if (joint->kb_jog_active)
      {
         continue;
      }
      if (emcmotStatus.net_feed_scale < 0.0001)
      {
         /* don't jog if feedhold is on or if feed override is zero */
         break;
      }
      /* calculate distance to jog */
      distance = delta * *(joint_data->jog_scale);
      /* check for joint already on hard limit */
      if (distance > 0.0 && GET_JOINT_PHL_FLAG(joint))
      {
         continue;
      }
      if (distance < 0.0 && GET_JOINT_NHL_FLAG(joint))
      {
         continue;
      }
      /* calc target position for jog */
      pos = joint->free_pos_cmd + distance;
      /* don't jog past limits */
      emcmot_refresh_jog_limits(joint);
      if (pos > joint->max_jog_limit)
      {
         continue;
      }
      if (pos < joint->min_jog_limit)
      {
         continue;
      }
      /* The default is to move exactly as far as the wheel commands,
         even if that move takes much longer than the wheel movement
         that commanded it.  Some folks prefer that the move stop as
         soon as the wheel does, even if that means not moving the
         commanded distance.  Velocity mode is for those folks.  If
         the command is faster than the machine can track, excess
         command is simply dropped. */
      if (*(joint_data->jog_vel_mode))
      {
         double v = joint->vel_limit * emcmotStatus.net_feed_scale;
         /* compute stopping distance at max speed */
         stop_dist = v * v / (2 * joint->acc_limit);
         /* if commanded position leads the actual position by more
            than stopping distance, discard excess command */
         if (pos > joint->pos_cmd + stop_dist)
         {
            pos = joint->pos_cmd + stop_dist;
         }
         else if (pos < joint->pos_cmd - stop_dist)
         {
            pos = joint->pos_cmd - stop_dist;
         }
      }
      /* set target position and use full velocity */
      joint->free_pos_cmd = pos;
      joint->free_vel_lim = joint->vel_limit;
      /* lock out other jog sources */
      joint->wheel_jog_active = 1;
      /* and let it go */
      joint->free_tp_enable = 1;
      SET_JOINT_ERROR_FLAG(joint, 0);
      /* clear joint homed flag(s) if we don't have forward kins.
         Otherwise, a transition into coordinated mode will incorrectly
         assume the homed position. Do all if they've all been moved
         since homing, otherwise just do this one */
      clearHomes(joint_num);
   }
}
#endif

static void get_pos_cmds(long period)
{
   struct emc_session *ps = &session;
   int joint_num, result;
   emcmot_joint_t *joint;
   double sm_pos[EMCMOT_MAX_AXIS];
   double positions[EMCMOT_MAX_JOINTS];
/*! \todo Another #if 0 */
#if 0
   static int interpolationCounter = 0;
#endif
   double old_pos_cmd;
   double max_dv, tiny_dp, pos_err, vel_req, vel_lim;

   /* used in teleop mode to compute the max accell requested */
   double accell_mag;
   int onlimit;

   /* copy joint position feedback to local array */
   for (joint_num = 0; joint_num < emcmotStatus.traj.axes; joint_num++)
   {
      /* point to joint struct */
      joint = &joints[joint_num];
      /* copy coarse command */
      positions[joint_num] = joint->coarse_pos;
   }

#if 0
   /* if less than a full complement of joints, zero out the rest */
   while (joint_num < EMCMOT_MAX_JOINTS)
   {
      positions[joint_num++] = 0.0;
   }
#endif

   /* RUN MOTION CALCULATIONS: */

   /* run traj planner code depending on the state */
   switch (emcmotStatus.motion_state)
   {
   case EMCMOT_MOTION_FREE:
      /* in free mode, each joint is planned independently */
      /* Each joint has a very simple "trajectory planner".  If the planner
         is enabled (free_tp_enable), then it moves toward free_pos_cmd at
         free_vel_lim, obeying the joint's accel and velocity limits, and
         stopping when it gets there.  If it is not enabled, it stops as
         quickly as possible, again obeying the accel limit.  When
         disabled, free_pos_cmd is set to the current position. */
      /* initial value for flag, if needed it will be cleared below */
      SET_MOTION_INPOS_FLAG(1);
      for (joint_num = 0; joint_num < emcmotStatus.traj.axes; joint_num++)
      {
         /* point to joint struct */
         joint = &joints[joint_num];

         if (joint->acc_limit > emcmotStatus.acc)
            joint->acc_limit = emcmotStatus.acc;

         if (GET_JOINT_ACTIVE_FLAG(joint))
         {
            joint->free_tp_active = 0;
            /* compute max change in velocity per servo period */
            max_dv = joint->acc_limit * servo_period;
            /* compute a tiny position range, to be treated as zero */
            tiny_dp = max_dv * servo_period * 0.001;
            /* calculate desired velocity */
            if (joint->free_tp_enable)
            {
               /* Got a valid jog request, planner enabled, request a velocity that tends to drive
                  pos_err to zero, but allows for stopping without position overshoot. */
               pos_err = joint->free_pos_cmd - joint->pos_cmd;
               /* positive and negative errors require some sign flipping to
                  avoid sqrt(negative) */
               if (pos_err > tiny_dp)
               {
                  vel_req = -max_dv + sqrt(2.0 * joint->acc_limit * pos_err + max_dv * max_dv);
                  /* mark joint active */
                  joint->free_tp_active = 1;
               }
               else if (pos_err < -tiny_dp)
               {
                  vel_req = max_dv - sqrt(-2.0 * joint->acc_limit * pos_err + max_dv * max_dv);
                  /* mark joint active */
                  joint->free_tp_active = 1;
               }
               else
               {
                  /* within 'tiny_dp' of desired pos, no need to move */
                  vel_req = 0.0;
               }
            }
            else
            {
               /* planner disabled, request zero velocity */
               vel_req = 0.0;
               /* and set command to present position to avoid movement when
                  next enabled */
               joint->free_pos_cmd = joint->pos_cmd;
            }
            /* if we move at all, clear AT_HOME flag */
            if (joint->free_tp_active)
            {
               SET_JOINT_AT_HOME_FLAG(joint, 0);
            }
            if (joint->home_state == HOME_IDLE)
            {
               /* velocity limit = planner limit * global scale factor */
               /* the global factor is used for feedrate override */
               vel_lim = joint->free_vel_lim * emcmotStatus.net_feed_scale;
            }
            else
            {
               /* except if homing, when we ignore FO */
               vel_lim = joint->free_vel_lim;
            }
            /* must not be greater than the joint physical limit */
            if (vel_lim > joint->vel_limit)
            {
               vel_lim = joint->vel_limit;
            }
            /* limit velocity request */
            if (vel_req > vel_lim)
            {
               vel_req = vel_lim;
            }
            else if (vel_req < -vel_lim)
            {
               vel_req = -vel_lim;
            }
            /* ramp velocity toward request at joint accel limit */
            if (vel_req > joint->vel_cmd + max_dv)
            {
               joint->vel_cmd += max_dv;
            }
            else if (vel_req < joint->vel_cmd - max_dv)
            {
               joint->vel_cmd -= max_dv;
            }
            else
            {
               joint->vel_cmd = vel_req;
            }
            /* check for still moving */
            if (joint->vel_cmd != 0.0)
            {
               /* yes, mark joint active */
               joint->free_tp_active = 1;
            }
            /* integrate velocity to get new position */
            joint->pos_cmd += joint->vel_cmd * servo_period;
            /* copy to coarse_pos */
            joint->coarse_pos = joint->pos_cmd;
            /* update joint status flag and overall status flag */
            if (joint->free_tp_active)
            {
               /* active TP means we're moving, so not in position */
               SET_JOINT_INPOS_FLAG(joint, 0);
               SET_MOTION_INPOS_FLAG(0);
               /* is any limit disabled for this move? */
               if (emcmotStatus.overrideLimitMask)
               {
                  emcmotDebug.overriding = 1;
               }
            }
            else
            {
               SET_JOINT_INPOS_FLAG(joint, 1);
               /* joint has stopped, so any outstanding jogs are done */
               joint->kb_jog_active = 0;
               joint->wheel_jog_active = 0;
            }
         }      //if (GET_JOINT_ACTIVE_FLAG(join))
      } //for loop for joints
      /* if overriding is true and we're in position, the jog
         is complete, and the limits should be re-enabled */
      if ((emcmotDebug.overriding) && (GET_MOTION_INPOS_FLAG()))
      {
         emcmotStatus.overrideLimitMask = 0;
         emcmotDebug.overriding = 0;
      }

      switch (emcmotStatus.traj.kinematics_type)
      {

      case KINEMATICS_IDENTITY:
         kinematicsForward(positions, &emcmotStatus.carte_pos_cmd, &fflags, &iflags);
         if (emcmotCheckAllHomed())
         {
            emcmotStatus.carte_pos_cmd_ok = 1;
         }
         else
         {
            emcmotStatus.carte_pos_cmd_ok = 0;
         }
         break;

      case KINEMATICS_BOTH:
         if (emcmotCheckAllHomed())
         {
            /* is previous value suitable for use as initial guess? */
            if (!emcmotStatus.carte_pos_cmd_ok)
            {
               /* no, use home position as initial guess */
               emcmotStatus.carte_pos_cmd = emcmotStatus.world_home;
            }
            /* calculate Cartesean position command from joint coarse pos cmd */
            result = kinematicsForward(positions, &emcmotStatus.carte_pos_cmd, &fflags, &iflags);
            /* check to make sure kinematics converged */
            if (result < 0)
            {
               /* error during kinematics calculations */
               emcmotStatus.carte_pos_cmd_ok = 0;
            }
            else
            {
               /* it worked! */
               emcmotStatus.carte_pos_cmd_ok = 1;
            }
         }
         else
         {
            emcmotStatus.carte_pos_cmd_ok = 0;
         }
         break;

      case KINEMATICS_INVERSE_ONLY:
         emcmotStatus.carte_pos_cmd_ok = 0;
         break;

      default:
         emcmotStatus.carte_pos_cmd_ok = 0;
         break;
      }
      /* end of FREE mode */
      break;
   case EMCMOT_MOTION_COORD:
      /* check joint 0 to see if the interpolators are empty */
      while (cubicNeedNextPoint(&(joints[0].cubic)))
      {
         /* they're empty, pull next point(s) off Cartesian planner */
         /* run coordinated trajectory planning cycle */
         tpRunCycle(&emcmotDebug.queue, period);
         /* gt new commanded traj pos */
         emcmotStatus.carte_pos_cmd = tpGetPos(&emcmotDebug.queue);
         /* OUTPUT KINEMATICS - convert to joints in local array */
         kinematicsInverse(&emcmotStatus.carte_pos_cmd, positions, &iflags, &fflags);
         /* copy to joint structures and spline them up */
         for (joint_num = 0; joint_num < emcmotStatus.traj.axes; joint_num++)
         {
            /* point to joint struct */
            joint = &joints[joint_num];
            joint->coarse_pos = positions[joint_num];
            /* spline joints up-- note that we may be adding points
               that fail soft limits, but we'll abort at the end of
               this cycle so it doesn't really matter */
            cubicAddPoint(&(joint->cubic), joint->coarse_pos);
         }
         /* END OF OUTPUT KINS */
      }
      /* there is data in the interpolators */
      /* run interpolation */
      for (joint_num = 0; joint_num < emcmotStatus.traj.axes; joint_num++)
      {
         /* point to joint struct */
         joint = &joints[joint_num];
         /* save old command */
         old_pos_cmd = joint->pos_cmd;
         /* interpolate to get new one */
         joint->pos_cmd = cubicInterpolate(&(joint->cubic), 0, 0, 0, 0);
         joint->vel_cmd = (joint->pos_cmd - old_pos_cmd) * servo_freq;
      }
      /* report motion status */
      SET_MOTION_INPOS_FLAG(0);
      if (tpIsDone(&emcmotDebug.queue))
      {
         SET_MOTION_INPOS_FLAG(1);
      }
      break;
   case EMCMOT_MOTION_TELEOP:

      /* first the desired Accell's are computed based on
         desired Velocity, current velocity and period */
      emcmotDebug.teleop_data.desiredAccell.tran.x = (emcmotDebug.teleop_data.desiredVel.tran.x - emcmotDebug.teleop_data.currentVel.tran.x) / servo_period;
      emcmotDebug.teleop_data.desiredAccell.tran.y = (emcmotDebug.teleop_data.desiredVel.tran.y - emcmotDebug.teleop_data.currentVel.tran.y) / servo_period;
      emcmotDebug.teleop_data.desiredAccell.tran.z = (emcmotDebug.teleop_data.desiredVel.tran.z - emcmotDebug.teleop_data.currentVel.tran.z) / servo_period;

      /* a Carthesian Accell is computed */
      pmCartMag(emcmotDebug.teleop_data.desiredAccell.tran, &accell_mag);

      /* then the accells for the rotary axes */
      emcmotDebug.teleop_data.desiredAccell.a = (emcmotDebug.teleop_data.desiredVel.a - emcmotDebug.teleop_data.currentVel.a) / servo_period;
      emcmotDebug.teleop_data.desiredAccell.b = (emcmotDebug.teleop_data.desiredVel.b - emcmotDebug.teleop_data.currentVel.b) / servo_period;
      emcmotDebug.teleop_data.desiredAccell.c = (emcmotDebug.teleop_data.desiredVel.c - emcmotDebug.teleop_data.currentVel.c) / servo_period;
      if (emcmotDebug.teleop_data.desiredAccell.a > accell_mag)
      {
         accell_mag = emcmotDebug.teleop_data.desiredAccell.a;
      }
      if (emcmotDebug.teleop_data.desiredAccell.b > accell_mag)
      {
         accell_mag = emcmotDebug.teleop_data.desiredAccell.b;
      }
      if (emcmotDebug.teleop_data.desiredAccell.c > accell_mag)
      {
         accell_mag = emcmotDebug.teleop_data.desiredAccell.c;
      }

      /* accell_mag should now hold the max accell */

      if (accell_mag > emcmotStatus.acc)
      {
         /* if accell_mag is too great, all need resizing */
         pmCartScalMult(emcmotDebug.teleop_data.desiredAccell.tran, emcmotStatus.acc / accell_mag, &emcmotDebug.teleop_data.currentAccell.tran);
         emcmotDebug.teleop_data.currentAccell.a = emcmotDebug.teleop_data.desiredAccell.a * emcmotStatus.acc / accell_mag;
         emcmotDebug.teleop_data.currentAccell.b = emcmotDebug.teleop_data.desiredAccell.b * emcmotStatus.acc / accell_mag;
         emcmotDebug.teleop_data.currentAccell.c = emcmotDebug.teleop_data.desiredAccell.c * emcmotStatus.acc / accell_mag;
         emcmotDebug.teleop_data.currentVel.tran.x += emcmotDebug.teleop_data.currentAccell.tran.x * servo_period;
         emcmotDebug.teleop_data.currentVel.tran.y += emcmotDebug.teleop_data.currentAccell.tran.y * servo_period;
         emcmotDebug.teleop_data.currentVel.tran.z += emcmotDebug.teleop_data.currentAccell.tran.z * servo_period;
         emcmotDebug.teleop_data.currentVel.a += emcmotDebug.teleop_data.currentAccell.a * servo_period;
         emcmotDebug.teleop_data.currentVel.b += emcmotDebug.teleop_data.currentAccell.b * servo_period;
         emcmotDebug.teleop_data.currentVel.c += emcmotDebug.teleop_data.currentAccell.c * servo_period;
      }
      else
      {
         /* if accell_mag is not greater, the computed accell's stay as is */
         emcmotDebug.teleop_data.currentAccell = emcmotDebug.teleop_data.desiredAccell;
         emcmotDebug.teleop_data.currentVel = emcmotDebug.teleop_data.desiredVel;
      }


      /* based on curent position, current vel and period, 
         the next position is computed */
      emcmotStatus.carte_pos_cmd.tran.x += emcmotDebug.teleop_data.currentVel.tran.x * servo_period;
      emcmotStatus.carte_pos_cmd.tran.y += emcmotDebug.teleop_data.currentVel.tran.y * servo_period;
      emcmotStatus.carte_pos_cmd.tran.z += emcmotDebug.teleop_data.currentVel.tran.z * servo_period;
      emcmotStatus.carte_pos_cmd.a += emcmotDebug.teleop_data.currentVel.a * servo_period;
      emcmotStatus.carte_pos_cmd.b += emcmotDebug.teleop_data.currentVel.b * servo_period;
      emcmotStatus.carte_pos_cmd.c += emcmotDebug.teleop_data.currentVel.c * servo_period;

      /* the next position then gets run through the inverse kins,
         to compute the next positions of the joints */

      /* OUTPUT KINEMATICS - convert to joints in local array */
      kinematicsInverse(&emcmotStatus.carte_pos_cmd, positions, &iflags, &fflags);
      /* copy to joint structures and spline them up */
      for (joint_num = 0; joint_num < emcmotStatus.traj.axes; joint_num++)
      {
         /* point to joint struct */
         joint = &joints[joint_num];
         joint->coarse_pos = positions[joint_num];
         /* spline joints up-- note that we may be adding points
            that fail soft limits, but we'll abort at the end of
            this cycle so it doesn't really matter */
         cubicAddPoint(&(joint->cubic), joint->coarse_pos);
      }
      /* END OF OUTPUT KINS */

      /* there is data in the interpolators */
      /* run interpolation */
      for (joint_num = 0; joint_num < emcmotStatus.traj.axes; joint_num++)
      {
         /* point to joint struct */
         joint = &joints[joint_num];
         /* save old command */
         old_pos_cmd = joint->pos_cmd;
         /* interpolate to get new one */
         joint->pos_cmd = cubicInterpolate(&(joint->cubic), 0, 0, 0, 0);
         joint->vel_cmd = (joint->pos_cmd - old_pos_cmd) * servo_freq;
      }

      /* end of teleop mode */
      break;

   case EMCMOT_MOTION_DISABLED:
      /* set position commands to match feedbacks, this avoids
         disturbances and/or following errors when enabling */
      emcmotStatus.carte_pos_cmd = emcmotStatus.carte_pos_fb;
      for (joint_num = 0; joint_num < emcmotStatus.traj.axes; joint_num++)
      {
         /* point to joint struct */
         joint = &joints[joint_num];
         /* save old command */
         joint->pos_cmd = joint->pos_fb;
         /* set joint velocity to zero */
         joint->vel_cmd = 0.0;
      }

      break;
   default:
      break;
   }
   /* check command against soft limits */
   /* This is a backup check, it should be impossible to command
      a move outside the soft limits.  However there is at least
      one case that isn't caught upstream: if an arc has both
      endpoints inside the limits, but the curve extends outside,
      the upstream checks will pass it.
    */
   onlimit = 0;
   for (joint_num = 0; joint_num < emcmotStatus.traj.axes; joint_num++)
   {
      /* point to joint data */
      joint = &joints[joint_num];
      /* skip inactive or unhomed axes */
      if (GET_JOINT_ACTIVE_FLAG(joint) && GET_JOINT_HOMED_FLAG(joint))
      {
         /* check for soft limits */
         if (joint->pos_cmd > joint->max_pos_limit)
         {
            onlimit = 1;
         }
         if (joint->pos_cmd < joint->min_pos_limit)
         {
            onlimit = 1;
         }
      }
   }
   if (onlimit)
   {
      if (!emcmotStatus.on_soft_limit)
      {
         /* just hit the limit */
         BUG("Exceeded soft limit\n");
         emcOperatorMessage(0, EMC_I18N("Exceeded soft limit"));
         SET_MOTION_ERROR_FLAG(1);
         emcmotStatus.on_soft_limit = 1;
      }
   }
   else
   {
      emcmotStatus.on_soft_limit = 0;
   }

   /* Check for any dout here. */
   if (emcmotStatus.dout.active)
   {
      emcmotStatus.dout.active = 0;
      if (rtstepper_set_gpo(&ps->dongle, emcmotStatus.dout.output_num, emcmotStatus.dout.value) == RTSTEPPER_R_OK)
      {
         if (!GET_MOTION_INPOS_FLAG())
         {
            /* A move command is active so dout will be synchronized. */ 
            DBG("Setting dout output=%d value=%d while moving.\n", emcmotStatus.dout.output_num, emcmotStatus.dout.value);
         }
         else
         {
            if (!emcmotStatus.dout.sync)
            {
               /* No move command expected, so fake a move command for one control cycle. */
               for (joint_num = 0; joint_num < emcmotStatus.traj.axes; joint_num++)
               {
                  joint = &joints[joint_num];
                  sm_pos[joint_num] = joint->motor_pos_cmd;  /* use the current commanded position */
               }         
               rtstepper_encode(&ps->dongle, tpGetExecId(&emcmotDebug.queue), sm_pos, emcmotStatus.traj.axes);
            }
         }
      }
   }
}       /* get_pos_cmds() */

/* NOTES:  These notes are just my understanding of how things work.

There are seven sets of position information.

1) emcmotStatus.carte_pos_cmd
2) emcmotStatus.joints[n].coarse_pos
3) emcmotStatus.joints[n].pos_cmd
4) emcmotStatus.joints[n].motor_pos_cmd
5) emcmotStatus.joints[n].motor_pos_fb
6) emcmotStatus.joints[n].pos_fb
7) emcmotStatus.carte_pos_fb

Their exact contents and meaning are as follows:

1) This is the desired position, in Cartesean coordinates.  It is
   updated at the traj rate, not the servo rate.
   In coord mode, it is determined by the traj planner
   In teleop mode, it is determined by the traj planner?
   In free mode, it is not used, since free mode motion takes
     place in joint space, not cartesean space.  It may be
     displayed by the GUI however, so it is updated by
     applying forward kins to (2), unless forward kins are
     not available, in which case it is copied from (7).

2) This is the desired position, in joint coordinates, but
   before interpolation.  It is updated at the traj rate, not
   the servo rate..
   In coord mode, it is generated by applying inverse kins to (1)
   In teleop mode, it is generated by applying inverse kins to (1)
   In free mode, it is not used, since the free mode planner generates
     a new (3) position every servo period without interpolation.
     However, it is used indirectly by GUIs, so it is copied from (3).

3) This is the desired position, in joint coords, after interpolation.
   A new set of these coords is generated every servo period.
   In coord mode, it is generated from (2) by the interpolator.
   In teleop mode, it is generated from (2) by the interpolator.
   In free mode, it is generated by the simple free mode traj planner.

4) This is the desired position, in motor coords.  Motor coords are
   generated by adding backlash compensation, lead screw error
   compensation, and offset (for homing) to (3).
   It is generated the same way regardless of the mode, and is the
   output to the PID loop or other position loop.

5) This is the actual position, in motor coords.  It is the input from
   encoders or other feedback device (or from virtual encoders on open
   loop machines).  It is "generated" by reading the feedback device.

6) This is the actual position, in joint coordinates.  It is generated
   by subtracting offset, lead screw error compensation, and backlash
   compensation from (5).  It is generated the same way regardless of
   the operating mode.

7) This is the actual position, in Cartesean coordinates.  It is updated
   at the traj rate, not the servo rate.
   OLD VERSION:
   In the old version, there are four sets of code to generate actualPos.
   One for each operating mode, and one for when motion is disabled.
   The code for coord and teleop modes is identical.  The code for free
   mode is somewhat different, in particular to deal with the case where
   one or more axes are not homed.  The disabled code is quite similar,
   but not identical, to the coord mode code.  In general, the code
   calculates actualPos by applying the forward kins to (6).  However,
   where forward kins are not available, actualPos is either copied
   from (1) (assumes no following error), or simply left alone.
   These special cases are handled differently for each operating mode.
   NEW VERSION:
   I would like to both simplify and relocate this.  As far as I can
   tell, actualPos should _always_ be the best estimate of the actual
   machine position in Cartesean coordinates.  So it should always be
   calculated the same way.
   In addition to always using the same code to calculate actualPos,
   I want to move that code.  It is really a feedback calculation, and
   as such it belongs with the rest of the feedback calculations early
   in control.c, not as part of the output generation code as it is now.
   Ideally, actualPos would always be calculated by applying forward
   kinematics to (6).  However, forward kinematics may not be available,
   or they may be unusable because one or more axes aren't homed.  In
   that case, the options are: A) fake it by copying (1), or B) admit
   that we don't really know the Cartesean coordinates, and simply
   don't update actualPos.  Whatever approach is used, I can see no
   reason not to do it the same way regardless of the operating mode.
   I would propose the following:  If there are forward kins, use them,
   unless they don't work because of unhomed axes or other problems,
   in which case do (B).  If no forward kins, do (A), since otherwise
   actualPos would _never_ get updated.

*/

static void compute_screw_comp(void)
{
   int joint_num;
   emcmot_joint_t *joint;
   emcmot_comp_t *comp;
   double dpos;
   double a_max, v_max, v, s_to_go, ds_stop, ds_vel, ds_acc, dv_acc;

   /* compute the correction */
   for (joint_num = 0; joint_num < emcmotStatus.traj.axes; joint_num++)
   {
      /* point to joint struct */
      joint = &joints[joint_num];
      if (!GET_JOINT_ACTIVE_FLAG(joint))
      {
         /* if joint is not active, skip it */
         continue;
      }
      /* point to compensation data */
      comp = &(joint->comp);
      if (comp->entries > 0)
      {
         /* there is data in the comp table, use it */
         /* first make sure we're in the right spot in the table */
         while (joint->pos_cmd < comp->entry->nominal)
         {
            comp->entry--;
         }
         while (joint->pos_cmd >= (comp->entry + 1)->nominal)
         {
            comp->entry++;
         }
         /* now interpolate */
         dpos = joint->pos_cmd - comp->entry->nominal;
         if (joint->vel_cmd > 0.0)
         {
            /* moving "up". apply forward screw comp */
            joint->backlash_corr = comp->entry->fwd_trim + comp->entry->fwd_slope * dpos;
         }
         else if (joint->vel_cmd < 0.0)
         {
            /* moving "down". apply reverse screw comp */
            joint->backlash_corr = comp->entry->rev_trim + comp->entry->rev_slope * dpos;
         }
         else
         {
            /* not moving, use whatever was there before */
         }
      }
      else
      {
         /* no compensation data, just use +/- 1/2 of backlash */
            /** FIXME: this can actually be removed - if the user space code
                sends a single compensation entry with any nominal value,
                and with fwd_trim = +0.5 times the backlash value, and 
                rev_trim = -0.5 times backlash, the above screw comp code
                will give exactly the same result as this code. */
         /* determine which way the compensation should be applied */
         if (joint->vel_cmd > 0.0)
         {
            /* moving "up". apply positive backlash comp */
            joint->backlash_corr = 0.5 * joint->backlash;
         }
         else if (joint->vel_cmd < 0.0)
         {
            /* moving "down". apply negative backlash comp */
            joint->backlash_corr = -0.5 * joint->backlash;
         }
         else
         {
            /* not moving, use whatever was there before */
         }
      }
      /* at this point, the correction has been computed, but
         the value may make abrupt jumps on direction reversal */
      /*
       * 07/09/2005 - S-curve implementation by Bas Laarhoven
       *
       * Implementation:
       *   Generate a ramped velocity profile for backlash or screw error comp.
       *   The velocity is ramped up to the maximum speed setting (if possible),
       *   using the maximum acceleration setting.
       *   At the end, the speed is ramped dowm using the same acceleration.
       *   The algorithm keeps looking ahead. Depending on the distance to go,
       *   the speed is increased, kept constant or decreased.
       *   
       * Limitations:
       *   Since the compensation adds up to the normal movement, total
       *   accelleration and total velocity may exceed maximum settings!
       *   Currently this is limited to 150% by implementation.
       *   To fix this, the calculations in get_pos_cmd should include
       *   information from the backlash corection. This makes things
       *   rather complicated and it might be better to implement the
       *   backlash compensation at another place to prevent this kind
       *   of interaction.
       *   More testing under different circumstances will show if this
       *   needs a more complicate solution.
       *   For now this implementation seems to generate smoother
       *   movements and less following errors than the original code.
       */

      /* Limit maximum accelleration and velocity 'overshoot'
       * to 150% of the maximum settings.
       * The TP and backlash shouldn't use more than 100%
       * (together) but this requires some interaction that
       * isn't implemented yet.
       */
      v_max = 0.5 * joint->vel_limit * emcmotStatus.net_feed_scale;
      a_max = 0.5 * joint->acc_limit;
      v = joint->backlash_vel;
      if (joint->backlash_corr >= joint->backlash_filt)
      {
         s_to_go = joint->backlash_corr - joint->backlash_filt; /* abs val */
         if (s_to_go > 0)
         {
            // off target, need to move
            ds_vel = v * servo_period;  /* abs val */
            dv_acc = a_max * servo_period;      /* abs val */
            ds_stop = 0.5 * (v + dv_acc) * (v + dv_acc) / a_max;        /* abs val */
            if (s_to_go <= ds_stop + ds_vel)
            {
               // ramp down
               if (v > dv_acc)
               {
                  // decellerate one period
                  ds_acc = 0.5 * dv_acc * servo_period; /* abs val */
                  joint->backlash_vel -= dv_acc;
                  joint->backlash_filt += ds_vel - ds_acc;
               }
               else
               {
                  // last step to target
                  joint->backlash_vel = 0.0;
                  joint->backlash_filt = joint->backlash_corr;
               }
            }
            else
            {
               if (v + dv_acc > v_max)
               {
                  dv_acc = v_max - v;   /* abs val */
               }
               ds_acc = 0.5 * dv_acc * servo_period;    /* abs val */
               ds_stop = 0.5 * (v + dv_acc) * (v + dv_acc) / a_max;     /* abs val */
               if (s_to_go > ds_stop + ds_vel + ds_acc)
               {
                  // ramp up
                  joint->backlash_vel += dv_acc;
                  joint->backlash_filt += ds_vel + ds_acc;
               }
               else
               {
                  // constant velocity
                  joint->backlash_filt += ds_vel;
               }
            }
         }
         else if (s_to_go < 0)
         {
            // safely handle overshoot (should not occur)
            joint->backlash_vel = 0.0;
            joint->backlash_filt = joint->backlash_corr;
         }
      }
      else
      { /* joint->backlash_corr < 0.0 */
         s_to_go = joint->backlash_filt - joint->backlash_corr; /* abs val */
         if (s_to_go > 0)
         {
            // off target, need to move
            ds_vel = -v * servo_period; /* abs val */
            dv_acc = a_max * servo_period;      /* abs val */
            ds_stop = 0.5 * (v - dv_acc) * (v - dv_acc) / a_max;        /* abs val */
            if (s_to_go <= ds_stop + ds_vel)
            {
               // ramp down
               if (-v > dv_acc)
               {
                  // decellerate one period
                  ds_acc = 0.5 * dv_acc * servo_period; /* abs val */
                  joint->backlash_vel += dv_acc;        /* decrease */
                  joint->backlash_filt -= ds_vel - ds_acc;
               }
               else
               {
                  // last step to target
                  joint->backlash_vel = 0.0;
                  joint->backlash_filt = joint->backlash_corr;
               }
            }
            else
            {
               if (-v + dv_acc > v_max)
               {
                  dv_acc = v_max + v;   /* abs val */
               }
               ds_acc = 0.5 * dv_acc * servo_period;    /* abs val */
               ds_stop = 0.5 * (v - dv_acc) * (v - dv_acc) / a_max;     /* abs val */
               if (s_to_go > ds_stop + ds_vel + ds_acc)
               {
                  // ramp up
                  joint->backlash_vel -= dv_acc;        /* increase */
                  joint->backlash_filt -= ds_vel + ds_acc;
               }
               else
               {
                  // constant velocity
                  joint->backlash_filt -= ds_vel;
               }
            }
         }
         else if (s_to_go < 0)
         {
            // safely handle overshoot (should not occur)
            joint->backlash_vel = 0.0;
            joint->backlash_filt = joint->backlash_corr;
         }
      }
      /* backlash (and motor offset) will be applied to output later */
      /* end of joint loop */
   }
}       /* compute_screw_comp() */

static void output_pos_cmds(void)
{
   struct emc_session *ps = &session;
   emcmot_joint_t *joint;
   double sm_pos[EMCMOT_MAX_AXIS];
   int joint_num;

   /* Output joint info. */
   for (joint_num = 0; joint_num < emcmotStatus.traj.axes; joint_num++)
   {
      /* point to joint struct */
      joint = &joints[joint_num];

      /* apply backlash and motor offset to output */
      joint->motor_pos_cmd = joint->pos_cmd + joint->backlash_filt + joint->motor_offset;

      /* Save commanded position for stepper motors. */
      sm_pos[joint_num] = joint->motor_pos_cmd;

#if 0
//        if (tpQueueDepth(&emcmotDebug.queue)) {
      if (!GET_MOTION_INPOS_FLAG())
      {
         /* Display waypoints. */
         if (joint_num == 0)
         {
            DBG("--id=%d axis=%d pos_cmd=%0.8f vel_cmd=%e motor_pos_cmd=%0.8f motor_pos_fb=%0.8f\n",
                tpGetExecId(&emcmotDebug.queue), joint_num, joint->pos_cmd, joint->vel_cmd, joint->motor_pos_cmd, joint->motor_pos_fb);
         }
         else
         {
            DBG("  id=%d axis=%d pos_cmd=%0.8f vel_cmd=%e motor_pos_cmd=%0.8f motor_pos_fb=%0.8f\n",
                tpGetExecId(&emcmotDebug.queue), joint_num, joint->pos_cmd, joint->vel_cmd, joint->motor_pos_cmd, joint->motor_pos_fb);
         }
      }
#endif
   }    /* for (joint_num = 0; joint_num < emcmotStatus.traj.axes; joint_num++) */

   if (!GET_MOTION_INPOS_FLAG())
   {
      rtstepper_encode(&ps->dongle, tpGetExecId(&emcmotDebug.queue), sm_pos, emcmotStatus.traj.axes);
   }
#if 0
   else
   {
      rtstepper_start_xfr(&ps->dongle, tpGetExecId(&emcmotDebug.queue), emcmotStatus.traj.axes);
   }
#endif

}       /* output_pos_cmds() */

static void update_status(void)
{
   int joint_num;
   emcmot_joint_t *joint;
   emcmot_joint_status_t *joint_status;

   /* copy status info from private joint structure to status
      struct in shared memory */
   for (joint_num = 0; joint_num < emcmotStatus.traj.axes; joint_num++)
   {
      /* point to joint data */
      joint = &joints[joint_num];
      /* point to joint status */
      joint_status = &(emcmotStatus.joint_status[joint_num]);
      /* copy stuff */

      joint_status->flag = joint->flag;
      joint_status->pos_cmd = joint->pos_cmd;
      joint_status->pos_fb = joint->pos_fb;
      joint_status->vel_cmd = joint->vel_cmd;
      joint_status->ferror = joint->ferror;
      joint_status->ferror_high_mark = joint->ferror_high_mark;
      joint_status->backlash = joint->backlash;
      joint_status->max_pos_limit = joint->max_pos_limit;
      joint_status->min_pos_limit = joint->min_pos_limit;
      joint_status->min_ferror = joint->min_ferror;
      joint_status->max_ferror = joint->max_ferror;
      joint_status->home_offset = joint->home_offset;
   }
#if 0
   for (dio = 0; dio < num_dio; dio++)
   {
      emcmotStatus.synch_di[dio] = *(emcmot_hal_data->synch_di[dio]);
      emcmotStatus.synch_do[dio] = *(emcmot_hal_data->synch_do[dio]);
   }

   for (aio = 0; aio < num_aio; aio++)
   {
      emcmotStatus.analog_input[aio] = *(emcmot_hal_data->analog_input[aio]);
      emcmotStatus.analog_output[aio] = *(emcmot_hal_data->analog_output[aio]);
   }
#endif

   /* Get trajectory planner queue status */
   emcmotStatus.depth = tpQueueDepth(&emcmotDebug.queue);   /* number of total queued motions */
   emcmotStatus.activeDepth = tpActiveDepth(&emcmotDebug.queue); /* number of motions blending */
   emcmotStatus.id = tpGetExecId(&emcmotDebug.queue);  /* gcode line number */
   emcmotStatus.motionType = tpGetMotionType(&emcmotDebug.queue);
   emcmotStatus.queueFull = tcqFull(&emcmotDebug.queue.queue);

   /* check to see if we should pause in order to implement single emcmotDebug.stepping */
   if (emcmotDebug.stepping && emcmotDebug.idForStep != emcmotStatus.id)
   {
      tpPause(&emcmotDebug.queue);
      emcmotDebug.stepping = 0;
      emcmotStatus.paused = 1;
   }
}       /* update_status() */

static void do_homing(void)
{
   int joint_num;
   emcmot_joint_t *joint;
   double offset;
   int home_sw_active, homing_flag;

   homing_flag = 0;
   if (emcmotStatus.motion_state != EMCMOT_MOTION_FREE)
   {
      /* can't home unless in free mode */
      return;
   }
   /* loop thru joints, treat each one individually */
   for (joint_num = 0; joint_num < emcmotStatus.traj.axes; joint_num++)
   {
      /* point to joint struct */
      joint = &joints[joint_num];
      if (!GET_JOINT_ACTIVE_FLAG(joint))
      {
         /* if joint is not active, skip it */
         continue;
      }
      home_sw_active = GET_JOINT_HOME_SWITCH_FLAG(joint);
      if (joint->home_state != HOME_IDLE)
      {
         homing_flag = 1;       /* at least one joint is homing */
      }

      /* when an joint is homing, 'check_for_faults()' ignores its limit
         switches, so that this code can do the right thing with them. Once
         the homing process is finished, the 'check_for_faults()' resumes
         checking */

      /* homing state machine */

      /* Some portions of the homing sequence can run thru two or more
         states during a single servo period.  This is done using
         'immediate_state'.  If a state transition sets it true (non-zero),
         this 'do-while' will loop executing switch(home_state) immediately
         to run the new state code.  Otherwise, the loop will fall thru, and
         switch(home_state) runs only once per servo period. Do _not_ set
         'immediate_state' true unless you also change 'home_state', unless
         you want an infinite loop! */
      do
      {
         immediate_state = 0;
         switch (joint->home_state)
         {
         case HOME_IDLE:
            /* nothing to do */
            break;

         case HOME_START:
            /* This state is responsible for getting the homing process
               started.  It doesn't actually do anything, it simply
               determines what state is next */
            if (joint->home_flags & HOME_IS_SHARED && home_sw_active)
            {
               BUG("Cannot home while shared home switch is closed\n");
               joint->home_state = HOME_IDLE;
               break;
            }
            /* set flags that communicate with the rest of EMC */
            SET_JOINT_HOMING_FLAG(joint, 1);
            SET_JOINT_HOMED_FLAG(joint, 0);
            SET_JOINT_AT_HOME_FLAG(joint, 0);
            /* stop any existing motion */
            joint->free_tp_enable = 0;
            /* reset delay counter */
            joint->home_pause_timer = 0;
            /* figure out exactly what homing sequence is needed */
            if (joint->home_search_vel == 0.0)
            {
               if (joint->home_latch_vel == 0.0)
               {
                  /* both vels == 0 means home at current position */
                  joint->home_state = HOME_SET_SWITCH_POSITION;
                  immediate_state = 1;
               }
               else if (joint->home_flags & HOME_USE_INDEX)
               {
                  /* home using index pulse only */
                  joint->home_state = HOME_INDEX_ONLY_START;
                  immediate_state = 1;
               }
               else
               {
                  BUG("invalid homing config: non-zero LATCH_VEL needs either SEARCH_VEL or USE_INDEX\n");
                  joint->home_state = HOME_IDLE;
               }
            }
            else
            {
               if (joint->home_latch_vel != 0.0)
               {
                  /* need to find home switch */
                  joint->home_state = HOME_INITIAL_SEARCH_START;
                  immediate_state = 1;
               }
               else
               {
                  BUG("invalid homing config: non-zero SEARCH_VEL needs LATCH_VEL\n");
                  joint->home_state = HOME_IDLE;
               }
            }
            break;
#if 0
         case HOME_INITIAL_BACKOFF_START:
            /* This state is called if the homing sequence starts at a
               location where the home switch is already tripped. It
               starts a move away from the switch. */
            /* is the joint still moving? */
            if (joint->free_tp_active)
            {
               /* yes, reset delay, wait until joint stops */
               joint->home_pause_timer = 0;
               break;
            }
            /* has delay timed out? */
            if (joint->home_pause_timer < (HOME_DELAY * servo_freq))
            {
               /* no, update timer and wait some more */
               joint->home_pause_timer++;
               break;
            }
            joint->home_pause_timer = 0;
            /* set up a move at '-search_vel' to back off of switch */
            home_start_move(joint, -joint->home_search_vel);
            /* next state */
            joint->home_state = HOME_INITIAL_BACKOFF_WAIT;
            break;

         case HOME_INITIAL_BACKOFF_WAIT:
            /* This state is called while the machine is moving off of
               the home switch.  It terminates when the switch is cleared
               successfully.  If the move ends or hits a limit before it
               clears the switch, the home is aborted. */
            /* are we off home switch yet? */
            if (!home_sw_active)
            {
               /* yes, stop motion */
               joint->free_tp_enable = 0;
               /* begin initial search */
               joint->home_state = HOME_INITIAL_SEARCH_START;
               immediate_state = 1;
               break;
            }
            home_do_moving_checks(joint);
            break;

         case HOME_INITIAL_SEARCH_START:
            /* This state is responsible for starting a move toward the
               home switch.  This move is at 'search_vel', which can be
               fairly fast, because once the switch is found another
               slower move will be used to set the exact home position. */
            /* is the joint already moving? */
            if (joint->free_tp_active)
            {
               /* yes, reset delay, wait until joint stops */
               joint->home_pause_timer = 0;
               break;
            }
            /* has delay timed out? */
            if (joint->home_pause_timer < (HOME_DELAY * servo_freq))
            {
               /* no, update timer and wait some more */
               joint->home_pause_timer++;
               break;
            }
            joint->home_pause_timer = 0;
            /* make sure we aren't already on home switch */
            if (home_sw_active)
            {
               /* already on switch, need to back off it first */
               joint->home_state = HOME_INITIAL_BACKOFF_START;
               immediate_state = 1;
               break;
            }
            /* set up a move at 'search_vel' to find switch */
            home_start_move(joint, joint->home_search_vel);
            /* next state */
            joint->home_state = HOME_INITIAL_SEARCH_WAIT;
            break;

         case HOME_INITIAL_SEARCH_WAIT:
            /* This state is called while the machine is looking for the
               home switch.  It terminates when the switch is found.  If
               the move ends or hits a limit before it finds the switch,
               the home is aborted. */
            /* have we hit home switch yet? */
            if (home_sw_active)
            {
               /* yes, stop motion */
               joint->free_tp_enable = 0;
               /* go to next step */
               joint->home_state = HOME_SET_COARSE_POSITION;
               immediate_state = 1;
               break;
            }
            home_do_moving_checks(joint);
            break;

         case HOME_SET_COARSE_POSITION:
            /* This state is called after the first time the switch is
               found.  At this point, we are approximately home. Although
               we will do another slower pass to get the exact home
               location, we reset the joint coordinates now so that screw
               error comp will be appropriate for this portion of the
               screw (previously we didn't know where we were at all). */
            /* set the current position to 'home_offset' */
            offset = joint->home_offset - joint->pos_fb;
            /* this moves the internal position but does not affect the
               motor position */
            joint->pos_cmd += offset;
            joint->pos_fb += offset;
            joint->free_pos_cmd += offset;
            joint->motor_offset -= offset;
            /* The next state depends on the signs of 'search_vel' and
               'latch_vel'.  If they are the same, that means we must
               back up, then do the final homing moving the same
               direction as the initial search, on a rising edge of the
               switch.  If they are opposite, it means that the final
               homing will take place on a falling edge as the machine
               moves off of the switch. */
            tmp = joint->home_search_vel * joint->home_latch_vel;
            if (tmp > 0.0)
            {
               /* search and latch vel are same direction */
               joint->home_state = HOME_FINAL_BACKOFF_START;
            }
            else
            {
               /* search and latch vel are opposite directions */
               joint->home_state = HOME_FALL_SEARCH_START;
            }
            immediate_state = 1;
            break;

         case HOME_FINAL_BACKOFF_START:
            /* This state is called once the approximate location of the
               switch has been found.  It is responsible for starting a
               move that will back off of the switch in preparation for a
               final slow move that captures the exact switch location. */
            /* is the joint already moving? */
            if (joint->free_tp_active)
            {
               /* yes, reset delay, wait until joint stops */
               joint->home_pause_timer = 0;
               break;
            }
            /* has delay timed out? */
            if (joint->home_pause_timer < (HOME_DELAY * servo_freq))
            {
               /* no, update timer and wait some more */
               joint->home_pause_timer++;
               break;
            }
            joint->home_pause_timer = 0;
            /* we should still be on the switch */
            if (!home_sw_active)
            {
               reportError("Home switch inactive before start of backoff move");
               joint->home_state = HOME_IDLE;
               break;
            }
            /* set up a move at '-search_vel' to back off of switch */
            home_start_move(joint, -joint->home_search_vel);
            /* next state */
            joint->home_state = HOME_FINAL_BACKOFF_WAIT;
            break;

         case HOME_FINAL_BACKOFF_WAIT:
            /* This state is called while the machine is moving off of
               the home switch after finding its approximate location.
               It terminates when the switch is cleared successfully.  If
               the move ends or hits a limit before it clears the switch,
               the home is aborted. */
            /* are we off home switch yet? */
            if (!home_sw_active)
            {
               /* yes, stop motion */
               joint->free_tp_enable = 0;
               /* begin final search */
               joint->home_state = HOME_RISE_SEARCH_START;
               immediate_state = 1;
               break;
            }
            home_do_moving_checks(joint);
            break;

         case HOME_RISE_SEARCH_START:
            /* This state is called to start the final search for the
               point where the home switch trips.  It moves at
               'latch_vel' and looks for a rising edge on the switch */
            /* is the joint already moving? */
            if (joint->free_tp_active)
            {
               /* yes, reset delay, wait until joint stops */
               joint->home_pause_timer = 0;
               break;
            }
            /* has delay timed out? */
            if (joint->home_pause_timer < (HOME_DELAY * servo_freq))
            {
               /* no, update timer and wait some more */
               joint->home_pause_timer++;
               break;
            }
            joint->home_pause_timer = 0;
            /* we should still be off of the switch */
            if (home_sw_active)
            {
               reportError("Home switch active before start of latch move");
               joint->home_state = HOME_IDLE;
               break;
            }
            /* set up a move at 'latch_vel' to locate the switch */
            home_start_move(joint, joint->home_latch_vel);
            /* next state */
            joint->home_state = HOME_RISE_SEARCH_WAIT;
            break;

         case HOME_RISE_SEARCH_WAIT:
            /* This state is called while the machine is moving towards
               the home switch on its final, low speed pass.  It
               terminates when the switch is detected. If the move ends
               or hits a limit before it hits the switch, the home is
               aborted. */
            /* have we hit the home switch yet? */
            if (home_sw_active)
            {
               /* yes, where do we go next? */
               if (joint->home_flags & HOME_USE_INDEX)
               {
                  /* look for index pulse */
                  joint->home_state = HOME_INDEX_SEARCH_START;
                  immediate_state = 1;
                  break;
               }
               else
               {
                  /* no index pulse, stop motion */
                  joint->free_tp_enable = 0;
                  /* go to next step */
                  joint->home_state = HOME_SET_SWITCH_POSITION;
                  immediate_state = 1;
                  break;
               }
            }
            home_do_moving_checks(joint);
            break;

         case HOME_FALL_SEARCH_START:
            /* This state is called to start the final search for the
               point where the home switch releases.  It moves at
               'latch_vel' and looks for a falling edge on the switch */
            /* is the joint already moving? */
            if (joint->free_tp_active)
            {
               /* yes, reset delay, wait until joint stops */
               joint->home_pause_timer = 0;
               break;
            }
            /* has delay timed out? */
            if (joint->home_pause_timer < (HOME_DELAY * servo_freq))
            {
               /* no, update timer and wait some more */
               joint->home_pause_timer++;
               break;
            }
            joint->home_pause_timer = 0;
            /* we should still be on the switch */
            if (!home_sw_active)
            {
               reportError("Home switch inactive before start of latch move");
               joint->home_state = HOME_IDLE;
               break;
            }
            /* set up a move at 'latch_vel' to locate the switch */
            home_start_move(joint, joint->home_latch_vel);
            /* next state */
            joint->home_state = HOME_FALL_SEARCH_WAIT;
            break;

         case HOME_FALL_SEARCH_WAIT:
            /* This state is called while the machine is moving away from
               the home switch on its final, low speed pass.  It
               terminates when the switch is cleared. If the move ends or
               hits a limit before it clears the switch, the home is
               aborted. */
            /* have we cleared the home switch yet? */
            if (!home_sw_active)
            {
               /* yes, where do we go next? */
               if (joint->home_flags & HOME_USE_INDEX)
               {
                  /* look for index pulse */
                  joint->home_state = HOME_INDEX_SEARCH_START;
                  immediate_state = 1;
                  break;
               }
               else
               {
                  /* no index pulse, stop motion */
                  joint->free_tp_enable = 0;
                  /* go to next step */
                  joint->home_state = HOME_SET_SWITCH_POSITION;
                  immediate_state = 1;
                  break;
               }
            }
            home_do_moving_checks(joint);
            break;
#endif

         case HOME_SET_SWITCH_POSITION:
            /* This state is called when the machine has determined the
               switch position as accurately as possible.  It sets the
               current joint position to 'home_offset', which is the
               location of the home switch in joint coordinates. */
            /* set the current position to 'home_offset' */
            offset = joint->home_offset - joint->pos_fb;
            /* this moves the internal position but does not affect the
               motor position */
            joint->pos_cmd += offset;
            joint->pos_fb += offset;
            joint->free_pos_cmd += offset;
            joint->motor_offset -= offset;
            /* next state */
            joint->home_state = HOME_FINAL_MOVE_START;
            immediate_state = 1;
            break;
#if 0
         case HOME_INDEX_ONLY_START:
            /* This state is used if the machine has been pre-positioned
               near the home position, and simply needs to find the
               next index pulse.  It starts a move at latch_vel, and
               sets index-enable, which tells the encoder driver to
               reset its counter to zero and clear the enable when the
               next index pulse arrives. */
            /* is the joint already moving? */
            if (joint->free_tp_active)
            {
               /* yes, reset delay, wait until joint stops */
               joint->home_pause_timer = 0;
               break;
            }
            /* has delay timed out? */
            if (joint->home_pause_timer < (HOME_DELAY * servo_freq))
            {
               /* no, update timer and wait some more */
               joint->home_pause_timer++;
               break;
            }
            joint->home_pause_timer = 0;
            /* Although we don't know the exact home position yet, we
               we reset the joint coordinates now so that screw error
               comp will be appropriate for this portion of the screw
               (previously we didn't know where we were at all). */
            /* set the current position to 'home_offset' */
            offset = joint->home_offset - joint->pos_fb;
            /* this moves the internal position but does not affect the
               motor position */
            joint->pos_cmd += offset;
            joint->pos_fb += offset;
            joint->free_pos_cmd += offset;
            joint->motor_offset -= offset;
            /* set the index enable */
            joint->index_enable = 1;
            /* set up a move at 'latch_vel' to find the index pulse */
            home_start_move(joint, joint->home_latch_vel);
            /* next state */
            joint->home_state = HOME_INDEX_SEARCH_WAIT;
            break;

         case HOME_INDEX_SEARCH_START:
            /* This state is called after the machine has made a low
               speed pass to determine the limit switch location. It
               sets index-enable, which tells the encoder driver to
               reset its counter to zero and clear the enable when the
               next index pulse arrives. */
            /* set the index enable */
            joint->index_enable = 1;
            /* and move right into the waiting state */
            joint->home_state = HOME_INDEX_SEARCH_WAIT;
            immediate_state = 1;
            home_do_moving_checks(joint);
            break;

         case HOME_INDEX_SEARCH_WAIT:
            /* This state is called after the machine has found the
               home switch and "armed" the encoder counter to reset on
               the next index pulse. It continues at low speed until
               an index pulse is detected, at which point it sets the
               final home position.  If the move ends or hits a limit
               before an index pulse occurs, the home is aborted. */
            /* has an index pulse arrived yet? encoder driver clears
               enable when it does */
            if (joint->index_enable == 0)
            {
               /* yes, stop motion */
               joint->free_tp_enable = 0;
               /* go to next step */
               joint->home_state = HOME_SET_INDEX_POSITION;
               immediate_state = 1;
               break;
            }
            home_do_moving_checks(joint);
            break;

         case HOME_SET_INDEX_POSITION:
            /* This state is called when the encoder has been reset at
               the index pulse position.  It sets the current joint 
               position to 'home_offset', which is the location of the
               index pulse in joint coordinates. */
            /* set the current position to 'home_offset' */
            joint->motor_offset = -joint->home_offset;
            joint->pos_fb = joint->motor_pos_fb - (joint->backlash_filt + joint->motor_offset);
            joint->pos_cmd = joint->pos_fb;
            joint->free_pos_cmd = joint->pos_fb;
            /* next state */
            joint->home_state = HOME_FINAL_MOVE_START;
            immediate_state = 1;
            break;
#endif

         case HOME_FINAL_MOVE_START:
            /* This state is called once the joint coordinate system is
               set properly.  It moves to the actual 'home' position,
               which is not neccessarily the position of the home switch
               or index pulse. */
            /* is the joint already moving? */
            if (joint->free_tp_active)
            {
               /* yes, reset delay, wait until joint stops */
               joint->home_pause_timer = 0;
               immediate_state = 1;
               break;
            }
            /* has delay timed out? */
            if (joint->home_pause_timer < (HOME_DELAY * servo_freq))
            {
               /* no, update timer and wait some more */
               joint->home_pause_timer++;
               immediate_state = 1;
               break;
            }
            joint->home_pause_timer = 0;
            /* plan a move to home position */
            joint->free_pos_cmd = joint->home;
            /* do the move at max speed */
            /* if home_vel is set (>0) then we use that, otherwise we rapid there */
            if (joint->home_final_vel > 0)
            {
               joint->free_vel_lim = fabs(joint->home_final_vel);
               /* clamp on max vel for this joint */
               if (joint->free_vel_lim > joint->vel_limit)
                  joint->free_vel_lim = joint->vel_limit;
            }
            else
            {
               joint->free_vel_lim = joint->vel_limit;
            }
            /* start the move */
            joint->free_tp_enable = 1;
            joint->home_state = HOME_FINAL_MOVE_WAIT;
            immediate_state = 1;
            break;

         case HOME_FINAL_MOVE_WAIT:
            /* This state is called while the machine makes its final
               move to the home position.  It terminates when the machine 
               arrives at the final location. If the move hits a limit
               before it arrives, the home is aborted. */
            /* have we arrived (and stopped) at home? */
            if (!joint->free_tp_active)
            {
               /* yes, stop motion */
               joint->free_tp_enable = 0;
               /* we're finally done */
               joint->home_state = HOME_FINISHED;
               immediate_state = 1;
               break;
            }
            if (joint->on_pos_limit || joint->on_neg_limit)
            {
               /* on limit, check to see if we should trip */
               if (!(joint->home_flags & HOME_IGNORE_LIMITS))
               {
                  /* not ignoring limits, time to quit */
                  BUG("hit limit in home state %d\n", joint->home_state);
                  joint->home_state = HOME_ABORT;
                  immediate_state = 1;
                  break;
               }
            }
            break;

         case HOME_FINISHED:
            SET_JOINT_HOMING_FLAG(joint, 0);
            SET_JOINT_HOMED_FLAG(joint, 1);
            SET_JOINT_AT_HOME_FLAG(joint, 1);
            joint->home_state = HOME_IDLE;
            immediate_state = 1;
//                rtstepper_home(&dongle, joint_num);
            break;

         case HOME_ABORT:
            SET_JOINT_HOMING_FLAG(joint, 0);
            SET_JOINT_HOMED_FLAG(joint, 0);
            SET_JOINT_AT_HOME_FLAG(joint, 0);
            joint->free_tp_enable = 0;
            joint->home_state = HOME_IDLE;
            joint->index_enable = 0;
            immediate_state = 1;
            break;

         default:
            /* should never get here */
            BUG("unknown state '%d' during homing\n", joint->home_state);
            joint->home_state = HOME_ABORT;
            immediate_state = 1;
            break;
         }      /* end of switch(joint->home_state) */
      }
      while (immediate_state);
   }    /* end of loop through all joints */

   if (homing_flag)
   {
      /* at least one joint is homing, set global flag */
      emcmotStatus.homing_active = 1;
   }
   else
   {
      /* is a homing sequence in progress? */
      if (emcmotStatus.homingSequenceState == HOME_SEQUENCE_IDLE)
      {
         /* no, single joint only, we're done */
         emcmotStatus.homing_active = 0;
      }
   }

}       /* do_homing() */
