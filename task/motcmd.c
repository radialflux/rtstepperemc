/********************************************************************
* motcmd.c - motion support commands for EMC2
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
*
********************************************************************/

#include <stdio.h>
#include <float.h>
#include <math.h>
#include "emc.h"
#include "bug.h"

static int rehomeAll;

/* loops through the active joints and checks if any are not homed */
int emcmotCheckAllHomed(void)
{
   int joint_num;
   emcmot_joint_t *joint;

   /* bail out if the allHomed flag is already set */
   if (emcmotDebug.allHomed)
      return 1;

   for (joint_num = 0; joint_num < emcmotStatus.traj.axes; joint_num++)
   {
      /* point to joint data */
      joint = &joints[joint_num];
      if (!GET_JOINT_ACTIVE_FLAG(joint))
      {
         /* if joint is not active, don't even look at its limits */
         continue;
      }
      if (!GET_JOINT_HOMED_FLAG(joint))
      {
         /* if any of the joints is not homed return false */
         return 0;
      }
   }
   /* set the global flag that all joints are homed */
   emcmotDebug.allHomed = 1;

   /* return true if all active joints are homed */
   return 1;
}

void emcmot_config_change(void)
{
   if (emcmotConfig.head == emcmotConfig.tail)
   {
      emcmotConfig.config_num++;
//  dongle change    emcmotStatus.config_num = emcmotConfig.config_num;
      emcmotConfig.head++;
   }
}

/* limits_ok() returns 1 if none of the hard limits are set,
   0 if any are set. Called on a linear and circular move. */
static int limits_ok(void)
{
   int joint_num;
   emcmot_joint_t *joint;

   for (joint_num = 0; joint_num < emcmotStatus.traj.axes; joint_num++)
   {
      /* point to joint data */
      joint = &joints[joint_num];
      if (!GET_JOINT_ACTIVE_FLAG(joint))
      {
         /* if joint is not active, don't even look at its limits */
         continue;
      }

      if (GET_JOINT_PHL_FLAG(joint) || GET_JOINT_NHL_FLAG(joint))
      {
         return 0;
      }
   }

   return 1;
}

/* check the value of the joint and velocity against current position,
   returning 1 (okay) if the request is to jog off the limit, 0 (bad)
   if the request is to jog further past a limit. */
static int jog_ok(int joint_num, double vel)
{
   emcmot_joint_t *joint;
   int neg_limit_override, pos_limit_override;

   /* point to joint data */
   joint = &joints[joint_num];
   /* are any limits for this joint overridden? */
   neg_limit_override = emcmotStatus.overrideLimitMask & (1 << (joint_num * 2));
   pos_limit_override = emcmotStatus.overrideLimitMask & (2 << (joint_num * 2));
   if (neg_limit_override && pos_limit_override)
   {
      /* both limits have been overridden at the same time.  This
         happens only when they both share an input, but means it
         is impossible to know which direction is safe to move.  So
         we skip the following tests... */
      return 1;
   }
   if (joint_num < 0 || joint_num >= emcmotStatus.traj.axes)
   {
      BUG("Can't jog invalid joint number %d.\n", joint_num);
      emcOperatorMessage(0, EMC_I18N("Can't jog invalid joint number %d."), joint_num);
      return 0;
   }
   if (vel > 0.0 && GET_JOINT_PHL_FLAG(joint))
   {
      BUG("Can't jog joint %d further past max hard limit.\n", joint_num);
      emcOperatorMessage(0, EMC_I18N("Can't jog joint %d further past max hard limit."), joint_num);
      return 0;
   }
   if (vel < 0.0 && GET_JOINT_NHL_FLAG(joint))
   {
      BUG("Can't jog joint %d further past min hard limit.\n", joint_num);
      emcOperatorMessage(0, EMC_I18N("Can't jog joint %d further past min hard limit."), joint_num);
      return 0;
   }
   emcmot_refresh_jog_limits(joint);
   if (vel > 0.0 && (joint->pos_cmd > joint->max_jog_limit))
   {
      BUG("Can't jog joint %d further past max soft limit.\n", joint_num);
      emcOperatorMessage(0, EMC_I18N("Can't jog joint %d further past max soft limit."), joint_num);
      return 0;
   }
   if (vel < 0.0 && (joint->pos_cmd < joint->min_jog_limit))
   {
      BUG("Can't jog joint %d further past min soft limit.\n", joint_num);
      emcOperatorMessage(0, EMC_I18N("Can't jog joint %d further past min soft limit."), joint_num);
      return 0;
   }
   /* okay to jog */
   return 1;
}

/* Jogs limits change, based on whether the machine is homed or
   or not.  If not homed, the limits are relative to the current
   position by +/- the full range of travel.  Once homed, they
   are absolute.
*/
void emcmot_refresh_jog_limits(emcmot_joint_t * joint)
{
   double range;

   if (GET_JOINT_HOMED_FLAG(joint))
   {
      /* if homed, set jog limits using soft limits */
      joint->max_jog_limit = joint->max_pos_limit;
      joint->min_jog_limit = joint->min_pos_limit;
   }
   else
   {
      /* not homed, set limits based on current position */
      range = joint->max_pos_limit - joint->min_pos_limit;
      joint->max_jog_limit = joint->pos_fb + range;
      joint->min_jog_limit = joint->pos_fb - range;
   }
}

/* inRange() returns non-zero if the position lies within the joint
   limits, or 0 if not.  It also reports an error for each joint limit
   violation.  It's possible to get more than one violation per move. */
static int inRange(EmcPose pos, int id, char *move_type)
{
   double joint_pos[EMCMOT_MAX_JOINTS];
   int joint_num;
   emcmot_joint_t *joint;
   int in_range = 1;

   /* fill in all joints with 0 */
   for (joint_num = 0; joint_num < emcmotStatus.traj.axes; joint_num++)
   {
      joint_pos[joint_num] = 0.0;
   }

   /* now fill in with real values, for joints that are used */
   kinematicsInverse(&pos, joint_pos, &iflags, &fflags);

   for (joint_num = 0; joint_num < emcmotStatus.traj.axes; joint_num++)
   {
      /* point to joint data */
      joint = &joints[joint_num];

      if (!GET_JOINT_ACTIVE_FLAG(joint))
      {
         /* if joint is not active, don't even look at its limits */
         continue;
      }
      if (joint_pos[joint_num] > joint->max_pos_limit)
      {
         in_range = 0;
         if (id > 0)
         {
            BUG("%s move on line %d would exceed joint %d's positive limit\n", move_type, id, joint_num);
            emcOperatorMessage(0, EMC_I18N("%s move on line %d would exceed joint %d's positive limit"),  move_type, id, joint_num);
         }
         else
         {
            BUG("%s move in MDI would exceed joint %d's positive limit\n", move_type, joint_num);
            emcOperatorMessage(0, EMC_I18N("%s move in MDI would exceed joint %d's positive limit"), move_type, joint_num);
         }
      }

      if (joint_pos[joint_num] < joint->min_pos_limit)
      {
         in_range = 0;
         if (id > 0)
         {
            BUG("%s move on line %d would exceed joint %d's negative limit\n", move_type, id, joint_num);
            emcOperatorMessage(0, EMC_I18N("%s move on line %d would exceed joint %d's negative limit"), move_type, id, joint_num);
         }
         else
         {
            BUG("%s move in MDI would exceed joint %d's negative limit\n", move_type, joint_num);
            emcOperatorMessage(0, EMC_I18N("%s move in MDI would exceed joint %d's negative limit"), move_type, joint_num);
         }
      }
   }
   return in_range;
}

/* clearHomes() will clear the homed flags for joints that have moved
   since homing, outside coordinated control, for machines with no
   forward kinematics. This is used in conjunction with the rehomeAll
   flag, which is set for any coordinated move that in general will
   result in all joints moving. The flag is consulted whenever a joint
   is jogged in joint mode, so that either its flag can be cleared if
   no other joints have moved, or all have to be cleared. */
static void clearHomes(int joint_num)
{
   int n;
   emcmot_joint_t *joint;

   if (emcmotStatus.traj.kinematics_type == KINEMATICS_INVERSE_ONLY)
   {
      if (rehomeAll)
      {
         for (n = 0; n < emcmotStatus.traj.axes; n++)
         {
            /* point at joint data */
            joint = &(joints[n]);
            /* clear flag */
            SET_JOINT_HOMED_FLAG(joint, 0);
         }
      }
      else
      {
         /* point at joint data */
         joint = &joints[joint_num];
         /* clear flag */
         SET_JOINT_HOMED_FLAG(joint, 0);
      }
   }
   emcmotDebug.allHomed = 0;
}

/*! \function emcmotDioWrite()

  sets or clears a HAL DIO pin, 
  pins get exported at runtime
  
  index is valid from 0 to num_dio <= EMCMOT_MAX_DIO, defined in emcmotcfg.h
  
*/
void emcmotDioWrite(int index, char value)
{
#if 0
   if ((index >= num_dio) || (index < 0))
   {
      BUG("ERROR: index out of range, %d not in [0..%d] (increase num_dio/EMCMOT_MAX_DIO=%d)\n", index, num_dio, EMCMOT_MAX_DIO);
   }
   else
   {
      if (value != 0)
      {
         *(emcmot_hal_data->synch_do[index]) = 1;
      }
      else
      {
         *(emcmot_hal_data->synch_do[index]) = 0;
      }
   }
#endif
}

/*! \function emcmotAioWrite()

  sets or clears a HAL AIO pin, 
  pins get exported at runtime
  
  \todo Implement function, it doesn't do anything right now
  RS274NGC doesn't support it now, only defined/used in emccanon.cc
  
*/
void emcmotAioWrite(int index, double value)
{
#if 0
   if ((index >= num_aio) || (index < 0))
   {
      BUG("ERROR: index out of range, %d not in [0..%d] (increase num_aio/EMCMOT_MAX_AIO=%d)\n", index, num_aio, EMCMOT_MAX_AIO);
   }
   else
   {
      *(emcmot_hal_data->analog_output[index]) = value;
   }
#endif
}

static int is_feed_type(int motion_type)
{
   switch (motion_type)
   {
   case EMC_MOTION_TYPE_ARC:
   case EMC_MOTION_TYPE_FEED:
   case EMC_MOTION_TYPE_PROBING:
      return 1;
   default:
      BUG("Internal error: unhandled motion type %d\n", motion_type);
   case EMC_MOTION_TYPE_TOOLCHANGE:
   case EMC_MOTION_TYPE_TRAVERSE:
      return 0;
   }
}

/*********************************************************************************
 * emcmotCommandHandler() - process commands from the control_thread()
 */
void emcmotCommandHandler(emcmot_command_t * emcmotCommand)
{
   emcmot_joint_t *joint;
   emcmot_comp_entry_t *comp_entry;
   struct emc_session *ps = &session;
   double tmp1;
   int joint_num, n;
   char issue_atspeed = 0;

   /* check for split read */
   if (emcmotCommand->head != emcmotCommand->tail)
   {
      emcmotDebug.split++;
      return;   /* not really an error */
   }
   if (emcmotCommand->commandNum != emcmotStatus.commandNumEcho)
   {
      /* increment head count-- we'll be modifying emcmotStatus */
      emcmotStatus.head++;
      emcmotDebug.head++;

      /* got a new command-- echo command and number... */
      emcmotStatus.commandEcho = emcmotCommand->command;
      emcmotStatus.commandNumEcho = emcmotCommand->commandNum;

      /* clear status value by default */
      emcmotStatus.commandStatus = EMCMOT_COMMAND_OK;

      /* ...and process command */

      /* Many commands uses "command->axis" to indicate which joint they
         wish to operate on.  This code eliminates the need to copy
         command->axis to "joint_num", limit check it, and then set "joint"
         to point to the joint data.  All the individual commands need to do
         is verify that "joint" is non-zero. */
      joint_num = emcmotCommand->axis;
      if (joint_num >= 0 && joint_num < emcmotStatus.traj.axes)
      {
         /* valid joint, point to it's data */
         joint = &joints[joint_num];
      }
      else
      {
         /* bad joint number */
         joint = NULL;
      }

      switch (emcmotCommand->command)
      {
      case EMCMOT_ABORT:
         /* abort motion */
         /* can happen at any time */
         /* this command attempts to stop all machine motion. it looks at
            the current mode and acts accordingly, if in teleop mode, it
            sets the desired velocities to zero, if in coordinated mode,
            it calls the traj planner abort function (don't know what that
            does yet), and if in free mode, it disables the free mode traj
            planners which stops joint motion */

         /* check for coord or free space motion active */
         if (GET_MOTION_TELEOP_FLAG())
         {
            ZERO_EMC_POSE(emcmotDebug.teleop_data.desiredVel);
         }
         else if (GET_MOTION_COORD_FLAG())
         {
            tpAbort(&emcmotDebug.queue);
         }
         else
         {
            for (joint_num = 0; joint_num < emcmotStatus.traj.axes; joint_num++)
            {
               /* point to joint struct */
               joint = &joints[joint_num];
               /* tell joint planner to stop */
               joint->free_tp_enable = 0;
               /* stop homing if in progress */
               if (joint->home_state != HOME_IDLE)
               {
                  joint->home_state = HOME_ABORT;
               }
            }
         }
         SET_MOTION_ERROR_FLAG(0);
         /* clear joint errors (regardless of mode */
         for (joint_num = 0; joint_num < emcmotStatus.traj.axes; joint_num++)
         {
            /* point to joint struct */
            joint = &joints[joint_num];
            /* update status flags */
            SET_JOINT_ERROR_FLAG(joint, 0);
            SET_JOINT_FAULT_FLAG(joint, 0);
         }
         emcmotStatus.paused = 0;
         break;

      case EMCMOT_AXIS_ABORT:
         /* abort one joint */
         /* can happen at any time */
         /* this command stops a single joint.  It is only usefull
            in free mode, so in coord or teleop mode it does
            nothing. */
         if (GET_MOTION_TELEOP_FLAG())
         {
            /* do nothing in teleop mode */
         }
         else if (GET_MOTION_COORD_FLAG())
         {
            /* do nothing in coord mode */
         }
         else
         {
            /* validate joint */
            if (joint == NULL)
               break;
            /* tell joint planner to stop */
            joint->free_tp_enable = 0;
            /* stop homing if in progress */
            if (joint->home_state != HOME_IDLE)
            {
               joint->home_state = HOME_ABORT;
            }
            /* update status flags */
            SET_JOINT_ERROR_FLAG(joint, 0);
         }
         break;

      case EMCMOT_FREE:
         /* change the mode to free mode motion (joint mode) */
         /* can be done at any time */
         /* this code doesn't actually make the transition, it merely
            requests the transition by clearing a couple of flags */
         /* reset the emcmotDebug.coordinating flag to defer transition
            to controller cycle */
         emcmotDebug.coordinating = 0;
         emcmotDebug.teleoperating = 0;
         break;

      case EMCMOT_COORD:
         /* change the mode to coordinated axis motion */
         /* can be done at any time */
         /* this code doesn't actually make the transition, it merely
            tests a condition and then sets a flag requesting the
            transition */
         /* set the emcmotDebug.coordinating flag to defer transition to
            controller cycle */
         emcmotDebug.coordinating = 1;
         emcmotDebug.teleoperating = 0;
         if (emcmotStatus.traj.kinematics_type != KINEMATICS_IDENTITY)
         {
            if (!emcmotCheckAllHomed())
            {
               BUG("all joints must be homed before going into coordinated mode\n");
               emcOperatorMessage(0, EMC_I18N("all joints must be homed before going into coordinated mode"));
               emcmotDebug.coordinating = 0;
               break;
            }
         }
         break;

      case EMCMOT_TELEOP:
         /* change the mode to teleop motion */
         /* can be done at any time */
         /* this code doesn't actually make the transition, it merely
            tests a condition and then sets a flag requesting the
            transition */
         /* set the emcmotDebug.teleoperating flag to defer transition to
            controller cycle */
         emcmotDebug.teleoperating = 1;
         if (emcmotStatus.traj.kinematics_type != KINEMATICS_IDENTITY)
         {
            if (!emcmotCheckAllHomed())
            {
               BUG("all joints must be homed before going into teleop mode\n");
               emcmotDebug.teleoperating = 0;
               break;
            }
         }
         break;

      case EMCMOT_SET_NUM_AXES:
         emcmotStatus.traj.axes = emcmotCommand->axis;
         emcmotConfig.numJoints = emcmotStatus.traj.axes;
         break;

      case EMCMOT_SET_WORLD_HOME:
         emcmotStatus.world_home = emcmotCommand->pos;
         break;

      case EMCMOT_SET_HOMING_PARAMS:
         emcmot_config_change();
         if (joint == NULL)
            break;
         joint->home_offset = emcmotCommand->offset;
         joint->home = emcmotCommand->home;
         joint->home_final_vel = emcmotCommand->home_final_vel;
         joint->home_search_vel = emcmotCommand->search_vel;
         joint->home_latch_vel = emcmotCommand->latch_vel;
         joint->home_flags = emcmotCommand->flags;
         joint->home_sequence = emcmotCommand->home_sequence;
         joint->volatile_home = emcmotCommand->volatile_home;
         break;

      case EMCMOT_OVERRIDE_LIMITS:
         /* this command can be issued with axix < 0 to re-enable
            limits, but they are automatically re-enabled at the
            end of the next jog */
         if (joint_num < 0)
         {
            /* don't override limits */
            emcmotStatus.overrideLimitMask = 0;
         }
         else
         {
            emcmotStatus.overrideLimitMask = 0;
            for (joint_num = 0; joint_num < emcmotStatus.traj.axes; joint_num++)
            {
               /* point at joint data */
               joint = &joints[joint_num];
               /* only override limits that are currently tripped */
               if (GET_JOINT_NHL_FLAG(joint))
               {
                  emcmotStatus.overrideLimitMask |= (1 << (joint_num * 2));
               }
               if (GET_JOINT_PHL_FLAG(joint))
               {
                  emcmotStatus.overrideLimitMask |= (2 << (joint_num * 2));
               }
            }
         }
         emcmotDebug.overriding = 0;
         for (joint_num = 0; joint_num < emcmotStatus.traj.axes; joint_num++)
         {
            /* point at joint data */
            joint = &joints[joint_num];
            /* clear joint errors */
            SET_JOINT_ERROR_FLAG(joint, 0);
         }
         break;

      case EMCMOT_SET_MOTOR_OFFSET:
         if (joint == NULL)
            break;
         joint->motor_offset = emcmotCommand->motor_offset;
         break;

      case EMCMOT_SET_POSITION_LIMITS:
         /* sets soft limits for a joint */
         emcmot_config_change();
         /* set the position limits for the joint */
         /* can be done at any time */
         if (joint == NULL)
            break;
         joint->min_pos_limit = emcmotCommand->minLimit;
         joint->max_pos_limit = emcmotCommand->maxLimit;
         break;

      case EMCMOT_SET_BACKLASH:
         /* sets backlash for a joint */
         emcmot_config_change();
         /* set the backlash for the joint */
         /* can be done at any time */
         if (joint == NULL)
            break;
         joint->backlash = emcmotCommand->backlash;
         break;

         /*
            Max and min ferror work like this: limiting ferror is
            determined by slope of ferror line, = maxFerror/limitVel ->
            limiting ferror = maxFerror/limitVel * vel. If ferror <
            minFerror then OK else if ferror < limiting ferror then OK
            else ERROR */
      case EMCMOT_SET_MAX_FERROR:
         emcmot_config_change();
         if (joint == NULL || emcmotCommand->maxFerror < 0.0)
            break;
         joint->max_ferror = emcmotCommand->maxFerror;
         break;

      case EMCMOT_SET_MIN_FERROR:
         emcmot_config_change();
         if (joint == NULL || emcmotCommand->minFerror < 0.0)
            break;
         joint->min_ferror = emcmotCommand->minFerror;
         break;

      case EMCMOT_JOG_CONT:
         /* do a continuous jog, implemented as an incremental jog to the
            limit.  When the user lets go of the button an abort will
            stop the jog. */
         /* check joint range */
         if (joint == NULL)
            break;
         /* must be in free mode and enabled */
         if (GET_MOTION_COORD_FLAG())
         {
            BUG("Can't jog joint in coordinated mode.\n");
            SET_JOINT_ERROR_FLAG(joint, 1);
            break;
         }
         if (!GET_MOTION_ENABLE_FLAG())
         {
            BUG("Can't jog joint when not enabled.\n");
            SET_JOINT_ERROR_FLAG(joint, 1);
            break;
         }
         if (emcmotStatus.homing_active)
         {
            BUG("Can't jog any joints while homing.\n");
            SET_JOINT_ERROR_FLAG(joint, 1);
            break;
         }
         if (joint->wheel_jog_active)
         {
            /* can't do two kinds of jog at once */
            break;
         }
         if (emcmotStatus.net_feed_scale < 0.0001)
         {
            /* don't jog if feedhold is on or if feed override is zero */
            BUG("Can't jog when FEEDHOLD is on\n");
            emcOperatorMessage(0, EMC_I18N("Can't jog when FEEDHOLD is on"));
            break;
         }
         /* don't jog further onto limits */
         if (!jog_ok(joint_num, emcmotCommand->vel))
         {
            SET_JOINT_ERROR_FLAG(joint, 1);
            break;
         }
         /* set destination of jog */
         emcmot_refresh_jog_limits(joint);
         if (emcmotCommand->vel > 0.0)
         {
            joint->free_pos_cmd = joint->max_jog_limit;
         }
         else
         {
            joint->free_pos_cmd = joint->min_jog_limit;
         }
         /* set velocity of jog */
         joint->free_vel_lim = fabs(emcmotCommand->vel);
         /* lock out other jog sources */
         joint->kb_jog_active = 1;
         /* and let it go */
         joint->free_tp_enable = 1;
         /*! \todo FIXME - should we really be clearing errors here? */
         SET_JOINT_ERROR_FLAG(joint, 0);
         /* clear joints homed flag(s) if we don't have forward kins.
            Otherwise, a transition into coordinated mode will incorrectly
            assume the homed position. Do all if they've all been moved
            since homing, otherwise just do this one */
         clearHomes(joint_num);
         break;

      case EMCMOT_JOG_INCR:
         /* do an incremental jog */

         /* check joints range */
         if (joint == NULL)
            break;
         /* must be in free mode and enabled */
         if (GET_MOTION_COORD_FLAG())
         {
            BUG("Can't jog joint in coordinated mode.\n");
            emcOperatorMessage(0, EMC_I18N("Can't jog joint in coordinated mode."));
            SET_JOINT_ERROR_FLAG(joint, 1);
            break;
         }
         if (!GET_MOTION_ENABLE_FLAG())
         {
            BUG("Can't jog joint when not enabled.\n");
            emcOperatorMessage(0, EMC_I18N("Can't jog joint when not enabled."));
            SET_JOINT_ERROR_FLAG(joint, 1);
            break;
         }
         if (emcmotStatus.homing_active)
         {
            BUG("Can't jog any joint while homing.\n");
            emcOperatorMessage(0, EMC_I18N("Can't jog any joint while homing."));
            SET_JOINT_ERROR_FLAG(joint, 1);
            break;
         }
         if (joint->wheel_jog_active)
         {
            /* can't do two kinds of jog at once */
            break;
         }
         if (emcmotStatus.net_feed_scale < 0.0001)
         {
            /* don't jog if feedhold is on or if feed override is zero */
            BUG("Can't jog when FEEDHOLD is on\n");
            emcOperatorMessage(0, EMC_I18N("Can't jog when FEEDHOLD is on"));
            break;
         }
         /* don't jog further onto limits */
         if (!jog_ok(joint_num, emcmotCommand->vel))
         {
            SET_JOINT_ERROR_FLAG(joint, 1);
            break;
         }
         /* set target position for jog */
         if (emcmotCommand->vel > 0.0)
            tmp1 = joint->free_pos_cmd + emcmotCommand->offset;
         else
            tmp1 = joint->free_pos_cmd - emcmotCommand->offset;

         /* don't jog past limits */
         emcmot_refresh_jog_limits(joint);
         if (tmp1 > joint->max_jog_limit)
         {
            BUG("Can not jog post soft limit: %0.6f.\n", joint->max_jog_limit);
            emcOperatorMessage(0, EMC_I18N("Can not jog past soft limit: %0.6f."), joint->max_jog_limit);
            break;
         }
         if (tmp1 < joint->min_jog_limit)
         {
            BUG("Can not jog post soft limit: %0.6f.\n", joint->min_jog_limit);
            emcOperatorMessage(0, EMC_I18N("Can not jog past soft limit: %0.6f."), joint->min_jog_limit);
            break;
         }

         /* set target position */
         joint->free_pos_cmd = tmp1;
         /* set velocity of jog */
         joint->free_vel_lim = fabs(emcmotCommand->vel);
         /* lock out other jog sources */
         joint->kb_jog_active = 1;
         /* and let it go */
         joint->free_tp_enable = 1;
         SET_JOINT_ERROR_FLAG(joint, 0);
         /* clear joint homed flag(s) if we don't have forward kins.
            Otherwise, a transition into coordinated mode will incorrectly
            assume the homed position. Do all if they've all been moved
            since homing, otherwise just do this one */
         clearHomes(joint_num);
         break;

      case EMCMOT_JOG_ABS:
         /* do an absolute jog */

         /* check joint range */
         if (joint == NULL)
            break;
         /* must be in free mode and enabled */
         if (GET_MOTION_COORD_FLAG())
         {
            BUG("Can't jog joint in coordinated mode.\n");
            emcOperatorMessage(0, EMC_I18N("Can't jog joint in coordinated mode."));
            SET_JOINT_ERROR_FLAG(joint, 1);
            break;
         }
         if (!GET_MOTION_ENABLE_FLAG())
         {
            BUG("Can't jog joint when not enabled.\n");
            emcOperatorMessage(0, EMC_I18N("Can't jog joint when not enabled."));
            SET_JOINT_ERROR_FLAG(joint, 1);
            break;
         }
         if (emcmotStatus.homing_active)
         {
            BUG("Can't jog any joints while homing.\n");
            emcOperatorMessage(0, EMC_I18N("Can't jog any joints while homing."));
            SET_JOINT_ERROR_FLAG(joint, 1);
            break;
         }
         if (joint->wheel_jog_active)
         {
            /* can't do two kinds of jog at once */
            break;
         }
         if (emcmotStatus.net_feed_scale < 0.0001)
         {
            /* don't jog if feedhold is on or if feed override is zero */
            BUG("Can't jog when FEEDHOLD is on\n");
            emcOperatorMessage(0, EMC_I18N("Can't jog when FEEDHOLD is on"));
            break;
         }
         /* don't jog further onto limits */
         if (!jog_ok(joint_num, emcmotCommand->vel))
         {
            SET_JOINT_ERROR_FLAG(joint, 1);
            break;
         }
         /*! \todo FIXME-- use 'goal' instead */
         joint->free_pos_cmd = emcmotCommand->offset;
         /* don't jog past limits */
         emcmot_refresh_jog_limits(joint);
         if (joint->free_pos_cmd > joint->max_jog_limit)
         {
            BUG("Can not jog post soft limit: %0.6f.\n", joint->max_jog_limit);
            emcOperatorMessage(0, EMC_I18N("Can not jog past soft limit: %0.6f."), joint->max_jog_limit);
            joint->free_pos_cmd = joint->max_jog_limit;
         }
         if (joint->free_pos_cmd < joint->min_jog_limit)
         {
            BUG("Can not jog post soft limit: %0.6f.\n", joint->min_jog_limit);
            emcOperatorMessage(0, EMC_I18N("Can not jog past soft limit: %0.6f."), joint->min_jog_limit);
            joint->free_pos_cmd = joint->min_jog_limit;
         }

         /* set velocity of jog */
         joint->free_vel_lim = fabs(emcmotCommand->vel);
         /* lock out other jog sources */
         joint->kb_jog_active = 1;
         /* and let it go */
         joint->free_tp_enable = 1;
         SET_JOINT_ERROR_FLAG(joint, 0);
         /* clear joint homed flag(s) if we don't have forward kins.
            Otherwise, a transition into coordinated mode will incorrectly
            assume the homed position. Do all if they've all been moved
            since homing, otherwise just do this one */
         clearHomes(joint_num);
         break;

      case EMCMOT_SET_TERM_COND:
         /* sets termination condition for motion emcmotDebug.queue */
         tpSetTermCond(&emcmotDebug.queue, emcmotCommand->termCond, emcmotCommand->tolerance);
         break;

      case EMCMOT_SET_SPINDLESYNC:
         tpSetSpindleSync(&emcmotDebug.queue, emcmotCommand->spindlesync, emcmotCommand->flags);
         break;

      case EMCMOT_SET_LINE:
         /* emcmotDebug.queue up a linear move */
         /* requires coordinated mode, enable off, not on limits */
         if (!GET_MOTION_COORD_FLAG() || !GET_MOTION_ENABLE_FLAG())
         {
            BUG("need to be enabled, in coord mode for linear move\n");
            emcOperatorMessage(0, EMC_I18N("need to be enabled, in coord mode for linear move"));
            emcmotStatus.commandStatus = EMCMOT_COMMAND_INVALID_COMMAND;
            SET_MOTION_ERROR_FLAG(1);
            break;
         }
         else if (!inRange(emcmotCommand->pos, emcmotCommand->id, "Linear"))
         {
            emcmotStatus.commandStatus = EMCMOT_COMMAND_INVALID_PARAMS;
            tpAbort(&emcmotDebug.queue);
            SET_MOTION_ERROR_FLAG(1);
            break;
         }
         else if (!limits_ok())
         {
            BUG("can't do linear move with limits exceeded\n");
            emcOperatorMessage(0, EMC_I18N("can't do linear move with limits exceeded"));
            emcmotStatus.commandStatus = EMCMOT_COMMAND_INVALID_PARAMS;
            tpAbort(&emcmotDebug.queue);
            SET_MOTION_ERROR_FLAG(1);
            break;
         }
         if (emcmotStatus.atspeed_next_feed && is_feed_type(emcmotCommand->motion_type))
         {
            issue_atspeed = 1;
            emcmotStatus.atspeed_next_feed = 0;
         }
         if (!is_feed_type(emcmotCommand->motion_type) && emcmotStatus.spindle.css_factor)
         {
            emcmotStatus.atspeed_next_feed = 1;
         }
         /* append it to the emcmotDebug.queue */
         tpSetId(&emcmotDebug.queue, emcmotCommand->id);
         if (tpAddLine(&emcmotDebug.queue, emcmotCommand->pos, emcmotCommand->motion_type,
                             emcmotCommand->vel, emcmotCommand->ini_maxvel, emcmotCommand->acc, emcmotStatus.enables_new, issue_atspeed) == -1)
         {
            BUG("can't add linear move\n");
            emcOperatorMessage(0, EMC_I18N("can't add linear move"));
            emcmotStatus.commandStatus = EMCMOT_COMMAND_BAD_EXEC;
            tpAbort(&emcmotDebug.queue);
            SET_MOTION_ERROR_FLAG(1);
            break;
         }
         else
         {
            SET_MOTION_ERROR_FLAG(0);
            /* set flag that indicates all joints need rehoming, if any
               joint is moved in joint mode, for machines with no forward
               kins */
            rehomeAll = 1;
         }
         break;

      case EMCMOT_SET_CIRCLE:
         /* emcmotDebug.queue up a circular move */
         /* requires coordinated mode, enable on, not on limits */
         if (!GET_MOTION_COORD_FLAG() || !GET_MOTION_ENABLE_FLAG())
         {
            BUG("need to be enabled, in coord mode for circular move\n");
            emcOperatorMessage(0, EMC_I18N("need to be enabled, in coord mode for circular move"));
            emcmotStatus.commandStatus = EMCMOT_COMMAND_INVALID_COMMAND;
            SET_MOTION_ERROR_FLAG(1);
            break;
         }
         else if (!inRange(emcmotCommand->pos, emcmotCommand->id, "Circular"))
         {
            emcmotStatus.commandStatus = EMCMOT_COMMAND_INVALID_PARAMS;
            tpAbort(&emcmotDebug.queue);
            SET_MOTION_ERROR_FLAG(1);
            break;
         }
         else if (!limits_ok())
         {
            BUG("can't do circular move with limits exceeded\n");
            emcOperatorMessage(0, EMC_I18N("can't do circular move with limits exceeded"));
            emcmotStatus.commandStatus = EMCMOT_COMMAND_INVALID_PARAMS;
            tpAbort(&emcmotDebug.queue);
            SET_MOTION_ERROR_FLAG(1);
            break;
         }
         if (emcmotStatus.atspeed_next_feed)
         {
            issue_atspeed = 1;
            emcmotStatus.atspeed_next_feed = 0;
         }
         /* append it to the emcmotDebug.queue */
         tpSetId(&emcmotDebug.queue, emcmotCommand->id);
         if (tpAddCircle(&emcmotDebug.queue, emcmotCommand->pos, emcmotCommand->center, emcmotCommand->normal,
                         emcmotCommand->turn, emcmotCommand->motion_type, emcmotCommand->vel, emcmotCommand->ini_maxvel, 
                         emcmotCommand->acc, emcmotStatus.enables_new, issue_atspeed) == -1)
         {
            BUG("can't add circular move\n");
            emcOperatorMessage(0, EMC_I18N("can't add circular move"));
            emcmotStatus.commandStatus = EMCMOT_COMMAND_BAD_EXEC;
            tpAbort(&emcmotDebug.queue);
            SET_MOTION_ERROR_FLAG(1);
            break;
         }
         else
         {
            SET_MOTION_ERROR_FLAG(0);
            /* set flag that indicates all joints need rehoming, if any
               joint is moved in joint mode, for machines with no forward
               kins */
            rehomeAll = 1;
         }
         break;

      case EMCMOT_SET_VEL:
         /* set the velocity for subsequent moves */
         /* can do it at any time */
         emcmotStatus.vel = emcmotCommand->vel;
         tpSetVmax(&emcmotDebug.queue, emcmotStatus.vel, emcmotCommand->ini_maxvel);
         break;

      case EMCMOT_SET_VEL_LIMIT:
         emcmot_config_change();
         /* set the absolute max velocity for all subsequent moves */
         /* can do it at any time */
         emcmotConfig.limitVel = emcmotCommand->vel;
         tpSetVlimit(&emcmotDebug.queue, emcmotConfig.limitVel);
         break;

      case EMCMOT_SET_INPUT_SCALE:
         emcmot_config_change();
         ps->dongle.steps_per_unit[joint_num] = emcmotCommand->scale;
         break;
      case EMCMOT_SET_STEP_PIN:
         emcmot_config_change();
         ps->dongle.step_pin[joint_num] = emcmotCommand->pin;
         break;
      case EMCMOT_SET_DIRECTION_PIN:
         emcmot_config_change();
         ps->dongle.direction_pin[joint_num] = emcmotCommand->pin;
         break;
      case EMCMOT_SET_STEP_POLARITY:
         emcmot_config_change();
         ps->dongle.step_active_high[joint_num] = emcmotCommand->polarity;
         break;
      case EMCMOT_SET_DIRECTION_POLARITY:
         emcmot_config_change();
         ps->dongle.direction_active_high[joint_num] = emcmotCommand->polarity;
         break;
      case EMCMOT_ENABLE_DIN_ABORT:
         emcmot_config_change();
         if (emcmotCommand->input_num == 0)
            ps->dongle.input0_abort_enabled = 1;
         else if (emcmotCommand->input_num == 1)
            ps->dongle.input1_abort_enabled = 1;
         else if (emcmotCommand->input_num == 2)
            ps->dongle.input2_abort_enabled = 1;
         else
            BUG("invalid input number=%d\n", emcmotCommand->input_num);
         break;
      case EMCMOT_DISABLE_DIN_ABORT:
         emcmot_config_change();
         if (emcmotCommand->input_num == 0)
            ps->dongle.input0_abort_enabled = 0;
         else if (emcmotCommand->input_num == 1)
            ps->dongle.input1_abort_enabled = 0;
         else if (emcmotCommand->input_num == 2)
            ps->dongle.input2_abort_enabled = 0;
         else
            BUG("invalid input number=%d\n", emcmotCommand->input_num);
         break;

      case EMCMOT_SET_JOINT_VEL_LIMIT:
         emcmot_config_change();
         /* check joint range */
         if (joint == NULL)
            break;
         joint->vel_limit = emcmotCommand->vel;
         joint->big_vel = 10 * emcmotCommand->vel;
         break;

      case EMCMOT_SET_JOINT_ACC_LIMIT:
         emcmot_config_change();
         /* check joint range */
         if (joint == NULL)
            break;
         joint->acc_limit = emcmotCommand->acc;
         break;

      case EMCMOT_SET_ACC:
         /* set the max acceleration */
         /* can do it at any time */
         emcmotStatus.acc = emcmotCommand->acc;
         tpSetAmax(&emcmotDebug.queue, emcmotStatus.acc);
         break;

      case EMCMOT_PAUSE:
         /* pause the motion */
         /* can happen at any time */
         tpPause(&emcmotDebug.queue);
         emcmotStatus.paused = 1;
         break;

      case EMCMOT_RESUME:
         /* resume paused motion */
         /* can happen at any time */
         emcmotDebug.stepping = 0;
         tpResume(&emcmotDebug.queue);
         emcmotStatus.paused = 0;
         break;

      case EMCMOT_STEP:
         /* resume paused motion until id changes */
         /* can happen at any time */
         if (emcmotStatus.paused)
         {
            emcmotDebug.idForStep = emcmotStatus.id;
            emcmotDebug.stepping = 1;
            tpResume(&emcmotDebug.queue);
            emcmotStatus.paused = 1;
         }
         else
         {
            BUG("MOTION: can't STEP while already executing\n");
            emcOperatorMessage(0, EMC_I18N("MOTION: can't STEP while already executing"));
         }
         break;

      case EMCMOT_FEED_SCALE:
         /* override speed */
         /* can happen at any time */
         if (emcmotCommand->scale < 0.0)
            emcmotCommand->scale = 0.0; /* clamp it */
         emcmotStatus.feed_scale = emcmotCommand->scale;
         break;

      case EMCMOT_FS_ENABLE:
         /* enable/disable overriding speed */
         /* can happen at any time */
         if (emcmotCommand->mode != 0)
            emcmotStatus.enables_new |= FS_ENABLED;
         else
            emcmotStatus.enables_new &= ~FS_ENABLED;
         break;

      case EMCMOT_FH_ENABLE:
         /* enable/disable feed hold */
         /* can happen at any time */
         if (emcmotCommand->mode != 0)
            emcmotStatus.enables_new |= FH_ENABLED;
         else
            emcmotStatus.enables_new &= ~FH_ENABLED;
         break;

      case EMCMOT_SPINDLE_SCALE:
         /* override spindle speed */
         /* can happen at any time */
         if (emcmotCommand->scale < 0.0)
            emcmotCommand->scale = 0.0; /* clamp it */
         emcmotStatus.spindle_scale = emcmotCommand->scale;
         break;

      case EMCMOT_SS_ENABLE:
         /* enable/disable overriding spindle speed */
         /* can happen at any time */
         if (emcmotCommand->mode != 0)
            emcmotStatus.enables_new |= SS_ENABLED;
         else
            emcmotStatus.enables_new &= ~SS_ENABLED;
         break;

      case EMCMOT_AF_ENABLE:
         /* enable/disable adaptive feedrate override from HAL pin */
         /* can happen at any time */
         if (emcmotCommand->flags != 0)
            emcmotStatus.enables_new |= AF_ENABLED;
         else
            emcmotStatus.enables_new &= ~AF_ENABLED;
         break;

      case EMCMOT_DISABLE:
         /* go into disable */
         /* can happen at any time */
         /* reset the emcmotDebug.enabling flag to defer disable until
            controller cycle (it *will* be honored) */
         emcmotDebug.enabling = 0;
         if (emcmotStatus.traj.kinematics_type == KINEMATICS_INVERSE_ONLY)
         {
            emcmotDebug.teleoperating = 0;
            emcmotDebug.coordinating = 0;
         }
         break;

      case EMCMOT_ENABLE:
         /* come out of disable */
         /* can happen at any time */
         /* set the emcmotDebug.enabling flag to defer enable until
            controller cycle */
         emcmotDebug.enabling = 1;
         if (emcmotStatus.traj.kinematics_type == KINEMATICS_INVERSE_ONLY)
         {
            emcmotDebug.teleoperating = 0;
            emcmotDebug.coordinating = 0;
         }
         break;

      case EMCMOT_ACTIVATE_JOINT:
         /* make joint active, so that amps will be enabled when system is
            enabled or disabled */
         /* can be done at any time */
         if (joint == NULL)
            break;
         SET_JOINT_ACTIVE_FLAG(joint, 1);
         break;

      case EMCMOT_DEACTIVATE_JOINT:
         /* make joint inactive, so that amps won't be affected when system
            is enabled or disabled */
         /* can be done at any time */
         if (joint == NULL)
            break;
         SET_JOINT_ACTIVE_FLAG(joint, 0);
         break;
      case EMCMOT_ENABLE_AMPLIFIER:
         /* enable the amplifier directly, but don't enable calculations */
         /* can be done at any time */
         break;
      case EMCMOT_DISABLE_AMPLIFIER:
         /* disable the joint calculations and amplifier, but don't disable
            calculations */
         /* can be done at any time */
         break;

      case EMCMOT_HOME:
         /* home the specified joint */
         /* need to be in free mode, enable on */
         /* this just sets the initial state, then the state machine in
            control.c does the rest */

         if (emcmotStatus.motion_state != EMCMOT_MOTION_FREE)
         {
            /* can't home unless in free mode */
            BUG("must be in joint mode to home\n");
            return;
         }
         if (!GET_MOTION_ENABLE_FLAG())
            break;

         if (joint_num == -1)
         {
            if (emcmotStatus.homingSequenceState == HOME_SEQUENCE_IDLE)
               emcmotStatus.homingSequenceState = HOME_SEQUENCE_START;
            else
               BUG("homing sequence already in progress\n");
            break;
         }

         if (joint == NULL)
            break;

         if (joint->home_state != HOME_IDLE)
         {
            BUG("homing already in progress\n");
         }
         else if (emcmotStatus.homingSequenceState != HOME_SEQUENCE_IDLE)
         {
            BUG("homing sequence already in progress\n");
         }
         else
         {
            /* abort any movement (jog, etc) that is in progress */
            joint->free_tp_enable = 0;

            /* prime the homing state machine */
            joint->home_state = HOME_START;
         }
         break;

      case EMCMOT_ENABLE_WATCHDOG:
         break;

      case EMCMOT_UNHOME:
         /* unhome the specified joint, or all joints if -1 */
         if ((emcmotStatus.motion_state != EMCMOT_MOTION_FREE) && (emcmotStatus.motion_state != EMCMOT_MOTION_DISABLED))
         {
            BUG("must be in joint mode or disabled to unhome\n");
            emcOperatorMessage(0, EMC_I18N("must be in joint mode or disabled to unhome"));
            return;
         }

         if (joint_num < 0)
         {
            /* we want all or none, so these checks need to all be done first.
             * but, let's only report the first error.  There might be several,
             * for instance if a homing sequence is running. */
            for (n = 0; n < emcmotStatus.traj.axes; n++)
            {
               joint = &joints[n];
               if (GET_JOINT_ACTIVE_FLAG(joint))
               {
                  if (GET_JOINT_HOMING_FLAG(joint))
                  {
                     BUG("Cannot unhome while homing, joint %d\n", n);
                     emcOperatorMessage(0, EMC_I18N("Cannot unhome while homing, joint %d"), n);
                     return;
                  }
                  if (!GET_JOINT_INPOS_FLAG(joint))
                  {
                     BUG("Cannot unhome while moving, joint %d\n", n);
                     emcOperatorMessage(0, EMC_I18N("Cannot unhome while moving, joint %d"), n);
                     return;
                  }
               }
            }
            /* we made it through the checks, so unhome them all */
            for (n = 0; n < emcmotStatus.traj.axes; n++)
            {
               joint = &joints[n];
               if (GET_JOINT_ACTIVE_FLAG(joint))
               {
                  /* if -2, only unhome the volatile_home joints */
                  if (joint_num != -2 || joint->volatile_home)
                  {
                     SET_JOINT_HOMED_FLAG(joint, 0);
                  }
               }
            }
         }
         else if (joint_num < emcmotStatus.traj.axes)
         {
            /* request was for only one joint */
            if (GET_JOINT_ACTIVE_FLAG(joint))
            {
               if (GET_JOINT_HOMING_FLAG(joint))
               {
                  BUG("Cannot unhome while homing, joint %d\n", joint_num);
                  emcOperatorMessage(0, EMC_I18N("Cannot unhome while homing, joint %d"), joint_num);
                  return;
               }
               if (!GET_JOINT_INPOS_FLAG(joint))
               {
                  BUG("Cannot unhome while moving, joint %d\n", joint_num);
                  emcOperatorMessage(0, EMC_I18N("Cannot unhome while moving, joint %d"), joint_num);
                  return;
               }
               SET_JOINT_HOMED_FLAG(joint, 0);
            }
            else
            {
               BUG("Cannot unhome inactive joint %d\n", joint_num);
               emcOperatorMessage(0, EMC_I18N("Cannot unhome inactive joint %d"), joint_num);
            }
         }
         else
         {
            /* invalid joint number specified */
            BUG("Cannot unhome invalid joint %d (max %d)\n", joint_num, emcmotStatus.traj.axes - 1);
            emcOperatorMessage(0, EMC_I18N("Cannot unhome invalid joint %d (max %d)"), joint_num, emcmotStatus.traj.axes - 1);
            return;
         }

         break;

      case EMCMOT_DISABLE_WATCHDOG:
         break;

      case EMCMOT_CLEAR_PROBE_FLAGS:
         emcmotStatus.probing = 0;
         emcmotStatus.probeTripped = 0;
         break;

      case EMCMOT_PROBE:
         /* most of this is taken from EMCMOT_SET_LINE */
         /* emcmotDebug.queue up a linear move */
         /* requires coordinated mode, enable off, not on limits */
#if 0
         if (!GET_MOTION_COORD_FLAG() || !GET_MOTION_ENABLE_FLAG())
         {
            BUG("need to be enabled, in coord mode for probe move\n");
            emcmotStatus.commandStatus = EMCMOT_COMMAND_INVALID_COMMAND;
            SET_MOTION_ERROR_FLAG(1);
            break;
         }
         else if (!inRange(emcmotCommand->pos, emcmotCommand->id, "Probe"))
         {
            emcmotStatus.commandStatus = EMCMOT_COMMAND_INVALID_PARAMS;
            tpAbort(&emcmotDebug.queue);
            SET_MOTION_ERROR_FLAG(1);
            break;
         }
         else if (!limits_ok())
         {
            BUG("can't do probe move with limits exceeded\n");
            emcmotStatus.commandStatus = EMCMOT_COMMAND_INVALID_PARAMS;
            tpAbort(&emcmotDebug.queue);
            SET_MOTION_ERROR_FLAG(1);
            break;
         }
         else if (!(emcmotCommand->probe_type & 1))
         {
            // if suppress errors = off...

            int probeval = *(emcmot_hal_data->probe_input);
            int probe_whenclears = ! !(emcmotCommand->probe_type & 2);

            if (probeval != probe_whenclears)
            {
               // the probe is already in the state we're seeking.
               if (probe_whenclears)
                  BUG("Probe is already clear when starting G38.4 or G38.5 move\n");
               else
                  BUG("Probe is already tripped when starting G38.2 or G38.3 move\n");

               emcmotStatus.commandStatus = EMCMOT_COMMAND_BAD_EXEC;
               tpAbort(&emcmotDebug.queue);
               SET_MOTION_ERROR_FLAG(1);
               break;
            }
         }

         /* append it to the emcmotDebug.queue */
         tpSetId(&emcmotDebug.queue, emcmotCommand->id);
         if (-1 ==
             tpAddLine(&emcmotDebug.queue, emcmotCommand->pos, emcmotCommand->motion_type, emcmotCommand->vel, emcmotCommand->ini_maxvel, emcmotCommand->acc,
                       emcmotStatus.enables_new, 0))
         {
            BUG("can't add probe move\n");
            emcmotStatus.commandStatus = EMCMOT_COMMAND_BAD_EXEC;
            tpAbort(&emcmotDebug.queue);
            SET_MOTION_ERROR_FLAG(1);
            break;
         }
         else
         {
            emcmotStatus.probing = 1;
            emcmotStatus.probe_type = emcmotCommand->probe_type;
            SET_MOTION_ERROR_FLAG(0);
            /* set flag that indicates all joints need rehoming, if any
               joint is moved in joint mode, for machines with no forward
               kins */
            rehomeAll = 1;
         }
#endif
         break;


      case EMCMOT_RIGID_TAP:
         /* most of this is taken from EMCMOT_SET_LINE */
         /* emcmotDebug.queue up a linear move */
         /* requires coordinated mode, enable off, not on limits */
         if (!GET_MOTION_COORD_FLAG() || !GET_MOTION_ENABLE_FLAG())
         {
            BUG("need to be enabled, in coord mode for rigid tap move\n");
            emcmotStatus.commandStatus = EMCMOT_COMMAND_INVALID_COMMAND;
            SET_MOTION_ERROR_FLAG(1);
            break;
         }
         else if (!inRange(emcmotCommand->pos, emcmotCommand->id, "Rigid tap"))
         {
            emcmotStatus.commandStatus = EMCMOT_COMMAND_INVALID_PARAMS;
            tpAbort(&emcmotDebug.queue);
            SET_MOTION_ERROR_FLAG(1);
            break;
         }
         else if (!limits_ok())
         {
            BUG("can't do rigid tap move with limits exceeded\n");
            emcmotStatus.commandStatus = EMCMOT_COMMAND_INVALID_PARAMS;
            tpAbort(&emcmotDebug.queue);
            SET_MOTION_ERROR_FLAG(1);
            break;
         }

         /* append it to the emcmotDebug.queue */
         tpSetId(&emcmotDebug.queue, emcmotCommand->id);
         if (tpAddRigidTap(&emcmotDebug.queue, emcmotCommand->pos, emcmotCommand->vel, emcmotCommand->ini_maxvel, emcmotCommand->acc, emcmotStatus.enables_new) == -1)
         {
            emcmotStatus.atspeed_next_feed = 0; /* rigid tap always waits for spindle to be at-speed */
            BUG("can't add rigid tap move\n");
            emcmotStatus.commandStatus = EMCMOT_COMMAND_BAD_EXEC;
            tpAbort(&emcmotDebug.queue);
            SET_MOTION_ERROR_FLAG(1);
            break;
         }
         else
         {
            SET_MOTION_ERROR_FLAG(0);
         }
         break;

      case EMCMOT_SET_TELEOP_VECTOR:
         if (!GET_MOTION_TELEOP_FLAG() || !GET_MOTION_ENABLE_FLAG())
         {
            BUG("need to be enabled, in teleop mode for teleop move\n");
         }
         else
         {
            double velmag;
            emcmotDebug.teleop_data.desiredVel = emcmotCommand->pos;
            pmCartMag(emcmotDebug.teleop_data.desiredVel.tran, &velmag);
            if (emcmotDebug.teleop_data.desiredVel.a > velmag)
            {
               velmag = emcmotDebug.teleop_data.desiredVel.a;
            }
            if (emcmotDebug.teleop_data.desiredVel.b > velmag)
            {
               velmag = emcmotDebug.teleop_data.desiredVel.b;
            }
            if (emcmotDebug.teleop_data.desiredVel.c > velmag)
            {
               velmag = emcmotDebug.teleop_data.desiredVel.c;
            }
            if (velmag > emcmotConfig.limitVel)
            {
               pmCartScalMult(emcmotDebug.teleop_data.desiredVel.tran, emcmotConfig.limitVel / velmag, &emcmotDebug.teleop_data.desiredVel.tran);
               emcmotDebug.teleop_data.desiredVel.a *= emcmotConfig.limitVel / velmag;
               emcmotDebug.teleop_data.desiredVel.b *= emcmotConfig.limitVel / velmag;
               emcmotDebug.teleop_data.desiredVel.c *= emcmotConfig.limitVel / velmag;
            }
            /* flag that all joints need to be homed, if any joint is
               jogged individually later */
            rehomeAll = 1;
         }
         break;

      case EMCMOT_SET_DEBUG:
         emcmotConfig.debug = emcmotCommand->debug;
         emcmot_config_change();
         break;

#if 0
      case EMCMOT_SET_AOUT:
         if (emcmotCommand->now)
         {      //we set it right away
            emcmotAioWrite(emcmotCommand->out, emcmotCommand->minLimit);
         }
         else
         {      // we put it on the TP queue, warning: only room for one in there, any new ones will overwrite
            tpSetAout(&emcmotDebug.queue, emcmotCommand->out, emcmotCommand->start, emcmotCommand->end);
         }
         break;
#endif

      case EMCMOT_SET_DOUT:
         emcmotStatus.dout.output_num = emcmotCommand->output_num;
         emcmotStatus.dout.value = emcmotCommand->value;
         emcmotStatus.dout.sync = emcmotCommand->sync;
         emcmotStatus.dout.active = 1;
         break;

      case EMCMOT_SET_SPINDLE_VEL:
         emcmotStatus.spindle.speed = emcmotCommand->vel;
         emcmotStatus.atspeed_next_feed = 1;
         break;

      case EMCMOT_SPINDLE_ON:
         emcmotStatus.spindle.speed = emcmotCommand->vel;
         emcmotStatus.spindle.css_factor = emcmotCommand->ini_maxvel;
         emcmotStatus.spindle.xoffset = emcmotCommand->acc;
         if (emcmotCommand->vel >= 0)
         {
            emcmotStatus.spindle.direction = 1;
         }
         else
         {
            emcmotStatus.spindle.direction = -1;
         }
         emcmotStatus.spindle.brake = 0;        //disengage brake
         emcmotStatus.atspeed_next_feed = 1;
         break;

      case EMCMOT_SPINDLE_OFF:
         emcmotStatus.spindle.speed = 0;
         emcmotStatus.spindle.direction = 0; 
         emcmotStatus.spindle.brake = 1;        // engage brake
         break;

      case EMCMOT_SYSTEM_CMD:
         break;

      case EMCMOT_SPINDLE_INCREASE:
         if (emcmotStatus.spindle.speed > 0)
            emcmotStatus.spindle.speed += 100;
         else if (emcmotStatus.spindle.speed < 0)
            emcmotStatus.spindle.speed -= 100;
         break;

      case EMCMOT_SPINDLE_DECREASE:
         if (emcmotStatus.spindle.speed > 100)
            emcmotStatus.spindle.speed -= 100;
         else if (emcmotStatus.spindle.speed < -100)
            emcmotStatus.spindle.speed += 100;
         break;

      case EMCMOT_SPINDLE_BRAKE_ENGAGE:
         emcmotStatus.spindle.speed = 0;
         emcmotStatus.spindle.direction = 0;
         emcmotStatus.spindle.brake = 1;
         break;

      case EMCMOT_SPINDLE_BRAKE_RELEASE:
         emcmotStatus.spindle.brake = 0;
         break;

      case EMCMOT_SET_JOINT_COMP:
         if (joint == NULL)
            break;
         if (joint->comp.entries >= EMCMOT_COMP_SIZE)
         {
            BUG("joint %d: too many compensation entries\n", joint_num);
            break;
         }
         /* point to last entry */
         comp_entry = &(joint->comp.array[joint->comp.entries]);
         if (emcmotCommand->comp_nominal <= comp_entry[0].nominal)
         {
            BUG("joint %d: compensation values must increase\n", joint_num);
            break;
         }
         /* store data to new entry */
         comp_entry[1].nominal = emcmotCommand->comp_nominal;
         comp_entry[1].fwd_trim = emcmotCommand->comp_forward;
         comp_entry[1].rev_trim = emcmotCommand->comp_reverse;
         /* calculate slopes from previous entry to the new one */
         if (comp_entry[0].nominal != -DBL_MAX)
         {
            /* but only if the previous entry is "real" */
            tmp1 = comp_entry[1].nominal - comp_entry[0].nominal;
            comp_entry[0].fwd_slope = (comp_entry[1].fwd_trim - comp_entry[0].fwd_trim) / tmp1;
            comp_entry[0].rev_slope = (comp_entry[1].rev_trim - comp_entry[0].rev_trim) / tmp1;
         }
         else
         {
            /* previous entry is at minus infinity, slopes are zero */
            comp_entry[0].fwd_trim = comp_entry[1].fwd_trim;
            comp_entry[0].rev_trim = comp_entry[1].rev_trim;
         }
         joint->comp.entries++;
         break;

      case EMCMOT_SET_OFFSET:
         emcmotStatus.tool_offset = emcmotCommand->tool_offset;
         break;

      default:
         BUG("unrecognized command %d\n", emcmotCommand->command);
         emcOperatorMessage(0, EMC_I18N("unrecognized command %d"), emcmotCommand->command);
         emcmotStatus.commandStatus = EMCMOT_COMMAND_UNKNOWN_COMMAND;
         break;

      } /* end of: command switch */

      if (emcmotStatus.commandStatus != EMCMOT_COMMAND_OK)
      {
         BUG("ERROR: %d\n", emcmotStatus.commandStatus);
      }

      /* synch tail count */
      emcmotStatus.tail = emcmotStatus.head;
      emcmotConfig.tail = emcmotConfig.head;
      emcmotDebug.tail = emcmotDebug.head;
   }

   return;
} /* emcmotcommandHandler() */
