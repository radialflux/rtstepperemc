/********************************************************************
* emc_msg.h - Support commands for EMC2
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
#ifndef _EMC_MSG_H
#define _EMC_MSG_H

#include "list.h"

enum EMC_COMMAND_MSG_TYPE
{
   EMC_COMMAND_UNUSED = 0,
   EMC_OPERATOR_MESSAGE_TYPE = 1,
   EMC_COMMAND_DONE_TYPE,
   MAX_EMC_TO_GUI_COMMAND,

   /* Commands for control_thread() start here. */
   EMC_STAT_TYPE,           
   EMC_QUIT_TYPE,
   EMC_SYSTEM_CMD_TYPE,

   EMC_AXIS_SET_AXIS_TYPE,
   EMC_AXIS_SET_UNITS_TYPE,
   EMC_AXIS_SET_MIN_POSITION_LIMIT_TYPE,
   EMC_AXIS_SET_MAX_POSITION_LIMIT_TYPE,
   EMC_AXIS_SET_FERROR_TYPE,
   EMC_AXIS_SET_HOMING_PARAMS_TYPE,
   EMC_AXIS_SET_MIN_FERROR_TYPE,
   EMC_AXIS_SET_MAX_VELOCITY_TYPE,
   EMC_AXIS_INIT_TYPE,
   EMC_AXIS_HALT_TYPE,
   EMC_AXIS_ABORT_TYPE,
   EMC_AXIS_ENABLE_TYPE,
   EMC_AXIS_DISABLE_TYPE,
   EMC_AXIS_HOME_TYPE,
   EMC_AXIS_UNHOME_TYPE,
   EMC_AXIS_ACTIVATE_TYPE,
   EMC_AXIS_DEACTIVATE_TYPE,
   EMC_AXIS_OVERRIDE_LIMITS_TYPE,
   EMC_AXIS_LOAD_COMP_TYPE,
   EMC_AXIS_SET_BACKLASH_TYPE,
   EMC_AXIS_STAT_TYPE,

   EMC_TRAJ_SET_AXES_TYPE,
   EMC_TRAJ_SET_UNITS_TYPE,
   EMC_TRAJ_SET_CYCLE_TIME_TYPE,
   EMC_TRAJ_SET_MODE_TYPE,
   EMC_TRAJ_SET_VELOCITY_TYPE,
   EMC_TRAJ_SET_ACCELERATION_TYPE,
   EMC_TRAJ_SET_MAX_VELOCITY_TYPE,
   EMC_TRAJ_SET_MAX_ACCELERATION_TYPE,
   EMC_TRAJ_SET_SCALE_TYPE,
   EMC_TRAJ_SET_MOTION_ID_TYPE,
   EMC_TRAJ_INIT_TYPE,
   EMC_TRAJ_HALT_TYPE,
   EMC_TRAJ_ENABLE_TYPE,
   EMC_TRAJ_DISABLE_TYPE,
   EMC_TRAJ_ABORT_TYPE,
   EMC_TRAJ_PAUSE_TYPE,
   EMC_TRAJ_STEP_TYPE,
   EMC_TRAJ_RESUME_TYPE,
   EMC_TRAJ_DELAY_TYPE,
   EMC_TRAJ_LINEAR_MOVE_TYPE,
   EMC_TRAJ_CIRCULAR_MOVE_TYPE,
   EMC_TRAJ_SET_TERM_COND_TYPE,
   EMC_TRAJ_SET_OFFSET_TYPE,
   EMC_TRAJ_SET_ORIGIN_TYPE,
   EMC_TRAJ_SET_HOME_TYPE,
   EMC_TRAJ_SET_ROTATION_TYPE,
   EMC_TRAJ_CLEAR_PROBE_TRIPPED_FLAG_TYPE,
   EMC_TRAJ_PROBE_TYPE,
   EMC_TRAJ_SET_TELEOP_ENABLE_TYPE,
   EMC_TRAJ_SET_TELEOP_VECTOR_TYPE,
   EMC_TRAJ_SET_SPINDLESYNC_TYPE,
   EMC_TRAJ_SET_SPINDLE_SCALE_TYPE,
   EMC_TRAJ_SET_FO_ENABLE_TYPE,
   EMC_TRAJ_SET_SO_ENABLE_TYPE,
   EMC_TRAJ_SET_FH_ENABLE_TYPE,
   EMC_TRAJ_RIGID_TAP_TYPE,
   EMC_TRAJ_STAT_TYPE,

   EMC_MOTION_INIT_TYPE,
   EMC_MOTION_HALT_TYPE,
   EMC_MOTION_ABORT_TYPE,
   EMC_MOTION_SET_AOUT_TYPE,
   EMC_MOTION_SET_DOUT_TYPE,
   EMC_MOTION_ENABLE_DIN_ABORT_TYPE,
   EMC_MOTION_DISABLE_DIN_ABORT_TYPE,
   EMC_MOTION_ADAPTIVE_TYPE,
   EMC_MOTION_STAT_TYPE,

   EMC_TASK_INIT_TYPE,
   EMC_TASK_HALT_TYPE,
   EMC_TASK_ABORT_TYPE,
   EMC_TASK_SET_MODE_TYPE,
   EMC_TASK_SET_STATE_TYPE,
   EMC_TASK_PLAN_OPEN_TYPE,
   EMC_TASK_PLAN_RUN_TYPE,
   EMC_TASK_PLAN_READ_TYPE,
   EMC_TASK_PLAN_PAUSE_TYPE,
   EMC_TASK_PLAN_STEP_TYPE,
   EMC_TASK_PLAN_RESUME_TYPE,
   EMC_TASK_PLAN_END_TYPE,
   EMC_TASK_PLAN_CLOSE_TYPE,
   EMC_TASK_PLAN_INIT_TYPE,
   EMC_TASK_PLAN_SYNCH_TYPE,
   EMC_TASK_PLAN_SET_OPTIONAL_STOP_TYPE,
   EMC_TASK_PLAN_SET_BLOCK_DELETE_TYPE,
   EMC_TASK_PLAN_OPTIONAL_STOP_TYPE,
   EMC_TASK_STAT_TYPE,

   EMC_TOOL_INIT_TYPE,
   EMC_TOOL_HALT_TYPE,
   EMC_TOOL_ABORT_TYPE,
   EMC_TOOL_PREPARE_TYPE,
   EMC_TOOL_LOAD_TYPE,
   EMC_TOOL_UNLOAD_TYPE,
   EMC_TOOL_LOAD_TOOL_TABLE_TYPE,
   EMC_TOOL_SET_OFFSET_TYPE,
   EMC_TOOL_SET_NUMBER_TYPE,
   EMC_TOOL_STAT_TYPE,

   EMC_AUX_ESTOP_ON_TYPE,
   EMC_AUX_ESTOP_OFF_TYPE,
   EMC_AUX_ESTOP_RESET_TYPE,
   EMC_AUX_INPUT_WAIT_TYPE,
   EMC_AUX_STAT_TYPE,

   EMC_SPINDLE_ON_TYPE,
   EMC_SPINDLE_OFF_TYPE,
   EMC_SPINDLE_SPEED_TYPE,
   EMC_SPINDLE_STAT_TYPE,
   EMC_SPINDLE_INCREASE_TYPE,
   EMC_SPINDLE_DECREASE_TYPE,
   EMC_SPINDLE_CONSTANT_TYPE,
   EMC_SPINDLE_BRAKE_RELEASE_TYPE,
   EMC_SPINDLE_BRAKE_ENGAGE_TYPE,

   EMC_COOLANT_MIST_ON_TYPE,
   EMC_COOLANT_MIST_OFF_TYPE,
   EMC_COOLANT_FLOOD_ON_TYPE,
   EMC_COOLANT_FLOOD_OFF_TYPE,
   EMC_COOLANT_STAT_TYPE,

   EMC_LUBE_ON_TYPE,
   EMC_LUBE_OFF_TYPE,
   EMC_LUBE_STAT_TYPE,

   EMC_IO_INIT_TYPE,
   EMC_IO_HALT_TYPE,
   EMC_IO_ABORT_TYPE,
   EMC_IO_SET_CYCLE_TIME_TYPE,
   EMC_IO_STAT_TYPE,

   EMC_INIT_TYPE,
   EMC_HALT_TYPE,
   EMC_ABORT_TYPE,
   MAX_GUI_TO_EMC_IMMEDIATE_CMD,

   /* Following will block until RCS_DONE. */
   EMC_AXIS_JOG_TYPE,
   EMC_AXIS_INCR_JOG_TYPE,
   EMC_AXIS_ABS_JOG_TYPE,
   EMC_TASK_PLAN_EXECUTE_TYPE,  /* MDI */
};

/* message header */
typedef struct _emc_msg_t
{
   enum EMC_COMMAND_MSG_TYPE type;
   int serial_number;  /* obsolete, DES */
   unsigned int n;              /* sequence number */
   struct list_head list;
} emc_msg_t;

typedef struct _emc_system_cmd_msg_t
{
   emc_msg_t msg;
   int index;          /* user defined mcode m100-m199 */
   double p_number;
   double q_number;
} emc_system_cmd_msg_t;

typedef struct _emc_command_done_msg_t
{
   emc_msg_t msg;
   unsigned int ticket;  /* sequence number of completed command */
} emc_command_done_msg_t;

typedef struct _emc_operator_message_msg_t
{
   emc_msg_t msg;
   int id;
   char text[LINELEN];
} emc_operator_message_msg_t;

typedef struct _emc_traj_set_origin_msg_t
{
   emc_msg_t msg;
   EmcPose origin;
} emc_traj_set_origin_msg_t;

typedef struct _emc_spindle_speed_msg_t
{
   emc_msg_t msg;
   double speed;                // commanded speed in RPMs or maximum speed for CSS
   double factor;               // Zero for constant RPM.  numerator of speed for CSS
   double xoffset;              // X axis offset compared to center of rotation, for CSS
} emc_spindle_speed_msg_t;

typedef struct _emc_traj_set_rotation_msg_t
{
   emc_msg_t msg;
   double rotation;
} emc_traj_set_rotation_msg_t;

typedef struct _emc_traj_linear_move_msg_t
{
   emc_msg_t msg;
   int type;
   EmcPose end;                 // end point
   double vel, ini_maxvel, acc;
   int feed_mode;
} emc_traj_linear_move_msg_t;

typedef struct _emc_traj_rigid_tap_msg_t
{
   emc_msg_t msg;
   EmcPose pos;
   double vel, ini_maxvel, acc;
} emc_traj_rigid_tap_msg_t;

typedef struct _emc_traj_probe_msg_t
{
   emc_msg_t msg;
   EmcPose pos;
   int type;
   double vel, ini_maxvel, acc;
   unsigned char probe_type;
} emc_traj_probe_msg_t;

typedef struct _emc_traj_set_term_cond_msg_t
{
   emc_msg_t msg;
   int cond;
   double tolerance;            // used to set the precision/tolerance of path deviation 
   // during CONTINUOUS motion mode. 
} emc_traj_set_term_cond_msg_t;

typedef struct _emc_traj_set_spindlesync_msg_t
{
   emc_msg_t msg;
   double feed_per_revolution;
   int velocity_mode;
} emc_traj_set_spindlesync_msg_t;

typedef struct _emc_traj_circular_move_msg_t
{
   emc_msg_t msg;
   EmcPose end;
   PmCartesian center;
   PmCartesian normal;
   int turn;
   int type;
   double vel, ini_maxvel, acc;
   int feed_mode;
} emc_traj_circular_move_msg_t;

typedef struct _emc_traj_delay_msg_t
{
   emc_msg_t msg;
   double delay;                // seconds
} emc_traj_delay_msg_t;

typedef struct _emc_spindle_on_msg_t
{
   emc_msg_t msg;
   double speed;                // commanded speed in RPMs or maximum speed for CSS
   double factor;               // Zero for constant RPM.  numerator of speed for CSS
   double xoffset;              // X axis offset compared to center of rotation, for CSS
} emc_spindle_on_msg_t;

typedef struct _emc_tool_set_offset_msg_t
{
   emc_msg_t msg;
   int pocket;
   int toolno;
   EmcPose offset;
   double diameter;
   double frontangle;
   double backangle;
   int orientation;
} emc_tool_set_offset_msg_t;

typedef struct _emc_traj_set_offset_msg_t
{
   emc_msg_t msg;
   EmcPose offset;
} emc_traj_set_offset_msg_t;

typedef struct _emc_tool_prepare_msg_t
{
   emc_msg_t msg;
   int tool;
} emc_tool_prepare_msg_t;

typedef struct _emc_tool_set_number_msg_t
{
   emc_msg_t msg;
   int tool;                    //number to use for  currently loaded tool
} emc_tool_set_number_msg_t;

typedef struct _emc_traj_set_fo_enable_msg_t
{
   emc_msg_t msg;
   unsigned char mode;          //mode=0, override off (will work with 100% FO), mode != 0, override on, user can change FO
} emc_traj_set_fo_enable_msg_t;

typedef struct _emc_motion_adaptive_msg_t
{
   emc_msg_t msg;
   unsigned char status;        // status=0 stop; status=1 start.
} emc_motion_adaptive_msg_t;

typedef struct _emc_traj_set_so_enable_msg_t
{
   emc_msg_t msg;
   unsigned char mode;          //mode=0, override off (will work with 100% SO), mode != 0, override on, user can change SO
} emc_traj_set_so_enable_msg_t;

typedef struct _emc_traj_set_fh_enable_msg_t
{
   emc_msg_t msg;
   unsigned char mode;          //mode=0, override off (feedhold is disabled), mode != 0, override on, user can use feedhold
} emc_traj_set_fh_enable_msg_t;

typedef struct _emc_motion_set_dout_msg_t
{
   emc_msg_t msg;
   int output_num;   // output0-7
   int value;        // 0=false, 1=true
   int sync;        // 0=output immediately, 1=output with move command
} emc_motion_set_dout_msg_t;

typedef struct _emc_motion_din_msg_t
{
   emc_msg_t msg;
   int input_num;   // input0-2
} emc_motion_din_msg_t;

typedef struct _emc_motion_set_aout_msg_t
{
   emc_msg_t msg;
   unsigned char index;         // which to set
   double start;                // value at start
   double end;                  // value at end
   unsigned char now;           // wether command is imediate or synched with motion
} emc_motion_set_aout_msg_t;

typedef struct _emc_aux_input_wait_msg_t
{
   emc_msg_t msg;
   int index;                   // input channel to wait for
   int input_type;              // DIGITAL or ANALOG
   int wait_type;               // 0 - immediate, 1- rise, 2 - fall, 3 - be high, 4 - be low
   double timeout;              // timeout for waiting
} emc_aux_input_wait_msg_t;

typedef struct _emc_task_plan_run_msg_t
{
   emc_msg_t msg;
   int line;
} emc_task_plan_run_msg_t;

typedef struct _emc_axis_cmd_msg_t
{
   emc_msg_t msg;
   int axis;
} emc_axis_cmd_msg_t;

typedef struct _emc_axis_jog_msg_t
{
   emc_msg_t msg;
   int axis;
   double vel;
} emc_axis_jog_msg_t;

typedef struct _emc_axis_incr_jog_msg_t
{
   emc_msg_t msg;
   int axis;
   double vel;
   double incr;
} emc_axis_incr_jog_msg_t;

typedef struct _emc_axis_abs_jog_msg_t
{
   emc_msg_t msg;
   int axis;
   double vel;
   double pos;
} emc_axis_abs_jog_msg_t;

typedef struct _emc_axis_set_backlash_msg_t
{
   emc_msg_t msg;
   int axis;
   double backlash;
} emc_axis_set_backlash_msg_t;

typedef struct _emc_axis_set_ferror_msg_t
{
   emc_msg_t msg;
   int axis;
   double ferror;
} emc_axis_set_ferror_msg_t;

typedef struct _emc_axis_set_min_ferror_msg_t
{
   emc_msg_t msg;
   int axis;
   double ferror;
} emc_axis_set_min_ferror_msg_t;

typedef struct _emc_axis_set_position_limit_msg_t
{
   emc_msg_t msg;
   int axis;
   double limit;
} emc_axis_set_position_limit_msg_t;

typedef struct _emc_traj_set_scale_msg_t
{
   emc_msg_t msg;
   double scale;
} emc_traj_set_scale_msg_t;

typedef struct _emc_traj_set_max_velocity_msg_t
{
   emc_msg_t msg;
   double vel;
} emc_traj_set_max_velocity_msg_t;

typedef struct _emc_traj_set_teleop_enable_msg_t
{
   emc_msg_t msg;
   int enable;
} emc_traj_set_teleop_enable_msg_t;

typedef struct _emc_traj_set_teleop_vector_msg_t
{
   emc_msg_t msg;
   EmcPose vector;
} emc_traj_set_teleop_vector_msg_t;

typedef struct _emc_tool_load_tool_table_msg_t
{
   emc_msg_t msg;
   char file[LINELEN];
} emc_tool_load_tool_table_msg_t;

typedef struct _emc_task_set_mode_msg_t
{
   emc_msg_t msg;
   enum EMC_TASK_MODE mode;
} emc_task_set_mode_msg_t;

typedef struct _emc_task_set_state_msg_t
{
   emc_msg_t msg;
   enum EMC_TASK_STATE state;
} emc_task_set_state_msg_t;

typedef struct _emc_task_plan_open_msg_t
{
   emc_msg_t msg;
   char file[LINELEN];
} emc_task_plan_open_msg_t;

typedef struct _emc_task_plan_execute_msg_t
{
   emc_msg_t msg;
   char command[LINELEN];
} emc_task_plan_execute_msg_t;

typedef struct _emc_task_plan_set_optional_stop_msg_t
{
   emc_msg_t msg;
   int state;
} emc_task_plan_set_optional_stop_msg_t;

/* generic command message */
typedef struct _emc_command_msg_t
{
   union
   {
      emc_msg_t msg;
      emc_operator_message_msg_t m1;
      emc_traj_set_origin_msg_t m3;
      emc_traj_set_rotation_msg_t m5;
      emc_traj_linear_move_msg_t m6;
      emc_traj_rigid_tap_msg_t m7;
      emc_traj_probe_msg_t m8;
      emc_traj_set_term_cond_msg_t m9;
      emc_traj_set_spindlesync_msg_t m10;
      emc_traj_circular_move_msg_t m11;
      emc_traj_delay_msg_t m12;
      emc_traj_set_fo_enable_msg_t m21;
      emc_traj_set_offset_msg_t m17;
      emc_traj_set_so_enable_msg_t m23;
      emc_traj_set_fh_enable_msg_t m24;
      emc_traj_set_scale_msg_t m38;
      emc_traj_set_max_velocity_msg_t m39;
      emc_traj_set_teleop_enable_msg_t m40;
      emc_traj_set_teleop_vector_msg_t m41;
      emc_spindle_speed_msg_t m4;
      emc_spindle_on_msg_t m13;
      emc_spindle_speed_msg_t m14;
      emc_tool_set_offset_msg_t m16;
      emc_tool_prepare_msg_t m19;
      emc_tool_set_number_msg_t m20;
      emc_tool_load_tool_table_msg_t m42;
      emc_motion_adaptive_msg_t m22;
      emc_motion_set_dout_msg_t m26;
      emc_motion_din_msg_t m48;
      emc_motion_set_aout_msg_t m27;
      emc_aux_input_wait_msg_t m28;
      emc_axis_cmd_msg_t m30;
      emc_axis_jog_msg_t m31;
      emc_axis_incr_jog_msg_t m32;
      emc_axis_abs_jog_msg_t m33;
      emc_axis_set_backlash_msg_t m34;
      emc_axis_set_ferror_msg_t m35;
      emc_axis_set_min_ferror_msg_t m36;
      emc_axis_set_position_limit_msg_t m37;
      emc_task_plan_run_msg_t m29;
      emc_task_set_mode_msg_t m43;
      emc_task_set_state_msg_t m44;
      emc_task_plan_open_msg_t m45;
      emc_task_plan_execute_msg_t m46;
      emc_task_plan_set_optional_stop_msg_t m47;
      emc_command_done_msg_t m49;
      emc_system_cmd_msg_t m50;
   };
} emc_command_msg_t;

#endif // _COMMAND_H
