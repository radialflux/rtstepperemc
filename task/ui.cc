/*****************************************************************************\

  ui.cc - user interface support for EMC2

  (c) 2008-2013 Copyright Eckler Software

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
  unencumbered (ie: no Copyright or License).

  See project revision history the "configure.ac" file.

\*****************************************************************************/

#include <stdio.h>
#include <pthread.h>
#include <string.h>
#include <errno.h>
#include <sys/stat.h>
#include "emc.h"
#include "ini.h"
#include "interpl.h"
#include "bug.h"

#define EMC_COMMAND_DELAY   0.1 // how long to sleep between checks

struct emc_session session;

/* TODO: support multiple dongles by moving globals into emc_session. 7/10/2013 DES */ 
emc_status_t _emcStatus;
emc_status_t *emcStatus = &_emcStatus;
emcio_status_t emcioStatus;
emc_command_msg_t *emcTaskCommand;
emcmot_command_t emcmotCommand;
emcmot_status_t emcmotStatus;
emcmot_config_t emcmotConfig;
emcmot_debug_t emcmotDebug;
emcmot_joint_t joints[EMCMOT_MAX_JOINTS];

KINEMATICS_FORWARD_FLAGS fflags;
KINEMATICS_INVERSE_FLAGS iflags;

MSG_INTERP_LIST interp_list;    /* MSG Union, for interpreter */

char EMC_INIFILE[LINELEN] = DEFAULT_EMC_INIFILE;
char RS274NGC_STARTUP_CODE[LINELEN] = DEFAULT_RS274NGC_STARTUP_CODE;
char EMC_PROGRAM_PREFIX[LINELEN] = DEFAULT_EMC_PROGRAM_PREFIX;
double EMC_TASK_CYCLE_TIME = DEFAULT_EMC_TASK_CYCLE_TIME;
double EMC_IO_CYCLE_TIME = DEFAULT_EMC_IO_CYCLE_TIME;
int EMC_TASK_INTERP_MAX_LEN = DEFAULT_EMC_TASK_INTERP_MAX_LEN;
double TRAJ_DEFAULT_VELOCITY = DEFAULT_TRAJ_DEFAULT_VELOCITY;
double TRAJ_MAX_VELOCITY = DEFAULT_TRAJ_MAX_VELOCITY;
double AXIS_MAX_VELOCITY[EMC_AXIS_MAX];
double AXIS_MAX_ACCELERATION[EMC_AXIS_MAX];
const char *USER_HOME_DIR;

char TOOL_TABLE_FILE[LINELEN] = DEFAULT_TOOL_TABLE_FILE;
EmcPose TOOL_CHANGE_POSITION;
unsigned char HAVE_TOOL_CHANGE_POSITION;
EmcPose TOOL_HOLDER_CLEAR;
unsigned char HAVE_TOOL_HOLDER_CLEAR;

static int emcCommandSerialNumber;

const char _gui_tag[] = "gui";
const char _mcd_tag[] = "mcd";
const char _ctl_tag[] = "ctl";

static enum EMC_RESULT _wait_received(struct emc_session *ps, unsigned int seq_num, double timeout)
{
   double end = 0.0;

   // Wait for command to get received.
   while (timeout <= 0.0 || end < timeout)
   {
      if (emcStatus->echo_serial_number >= seq_num)
         return EMC_R_OK;

      esleep(EMC_COMMAND_DELAY);
      end += EMC_COMMAND_DELAY;
   }
   return EMC_R_TIMEOUT;
}       /* _wait_received() */

DLL_EXPORT enum EMC_RESULT emc_ui_update_status(void *hd)
{
   return EMC_R_OK;     /* no need to update emcStatus */
}

DLL_EXPORT int emc_ui_get_ini_key_value(void *hd, const char *section, const char *key, char *value, int value_size)
{
   return iniGetKeyValue(section, key, value, value_size);
}

DLL_EXPORT enum EMC_TASK_STATE emc_ui_get_task_state(void *hd)
{
   return emcStatus->task.state;
}

DLL_EXPORT enum EMC_TASK_MODE emc_ui_get_task_mode(void *hd)
{
   return emcStatus->task.mode;
}

DLL_EXPORT enum EMC_DIN_STATE emc_ui_get_din_state(void *hd, int input_num)
{
   struct emc_session *ps = (struct emc_session *)hd;
   enum RTSTEPPER_RESULT stat;

   if (input_num == 0)
      stat = rtstepper_input0_state(&ps->dongle);
   else if (input_num == 1)
      stat = rtstepper_input1_state(&ps->dongle);
   else if (input_num == 2)
      stat = rtstepper_input2_state(&ps->dongle);
   else
      BUG("invalid input number=%d\n", input_num);

   return (stat == RTSTEPPER_R_INPUT_TRUE) ? EMC_DIN_TRUE : EMC_DIN_FALSE;
}

DLL_EXPORT enum EMC_RESULT emc_ui_get_version(const char **ver)
{
   *ver = PACKAGE_VERSION;
   return EMC_R_OK;
}

DLL_EXPORT enum EMC_RESULT emc_ui_get_operator_message(void *hd, char *buf, int buf_size)
{
   struct emc_session *ps = (struct emc_session *)hd;
   emc_command_msg_t *m;
   unsigned int last = 0;
   int lock = 0, done = 0;
   const char tag[] = "gui";

   if (buf == NULL || buf_size <= 0)
   {
      BUG("invalid input\n");
      return EMC_R_ERROR;
   }

   buf[0] = 0;

   while (!done)
   {
      peek_message(ps, &m, &last, &lock, tag);

      if (m)
      {
         if (m->msg.type == EMC_OPERATOR_MESSAGE_TYPE)
         {
            remove_message(ps, m, &lock, tag);
            strncpy(buf, ((emc_operator_message_msg_t *) m)->text, buf_size);
            buf[buf_size - 1] = 0;
            free(m);
            done = 1;
         }
      }
      else
      {
         break; /* no more messages */
      }
   }

   return EMC_R_OK;
}       /* emc_ui_update_operator_message() */

DLL_EXPORT enum EMC_RESULT emc_ui_operator_message(void *hd, const char *buf)
{
   emc_command_msg_t message;
   emc_operator_message_msg_t *op_message;
   struct emc_session *ps = (emc_session *)hd;

   op_message = (emc_operator_message_msg_t *) &message;
   op_message->msg.type = EMC_OPERATOR_MESSAGE_TYPE;

   strncpy(op_message->text, buf, sizeof(op_message->text));
   op_message->text[sizeof(op_message->text) - 1] = 0;       /* force zero termination */

   send_message(ps, &message, _ctl_tag);
   return EMC_R_OK;
}       /* emc_ui_operator_message() */

/* Wait for last command to be received by control thread. */
DLL_EXPORT enum EMC_RESULT emc_ui_wait_command_received(void *hd, double timeout)
{
#if 0
   double end = 0.0;

   while (timeout <= 0.0 || end < timeout)
   {
      emc_ui_update_status(hd);

      if (emcStatus->echo_serial_number >= emcCommandSerialNumber)
      {
         return EMC_R_OK;
      }

      esleep(EMC_COMMAND_DELAY);
      end += EMC_COMMAND_DELAY;
   }
#endif
   return EMC_R_TIMEOUT;
}       /* emc_ui_command_wait_received() */

/* Wait for last command to finish executing. */
DLL_EXPORT enum EMC_RESULT emc_ui_wait_command_done(void *hd, double timeout)
{
   double end = 0.0;

   while (timeout <= 0.0 || end < timeout)
   {
      if (emcStatus->status == RCS_DONE)
         return EMC_R_OK;

      if (emcStatus->status == RCS_ERROR)
         return EMC_R_ERROR;

      esleep(EMC_COMMAND_DELAY);
      end += EMC_COMMAND_DELAY;
   }
   return EMC_R_TIMEOUT;
}       /* emc_ui_command_wait_done() */

/* Wait for jog command to finish executing. */
DLL_EXPORT enum EMC_RESULT emc_ui_wait_io_done(void *hd, double timeout)
{
   struct emc_session *ps = (struct emc_session *)hd;
   double end = 0.0;

   /* Wait till step buffer is generated. */
   while (timeout <= 0.0 || end < timeout)
   {
      if (emcStatus->motion.status == RCS_DONE)
         break;

      if (emcStatus->status == RCS_ERROR)
         return EMC_R_ERROR;

      esleep(EMC_COMMAND_DELAY);
      end += EMC_COMMAND_DELAY;
   }

   /* Wait till step buffer xfr (over-the-wire) is done. */
   pthread_mutex_lock(&ps->dongle.mutex);
   while (ps->dongle.xfr_active)
      pthread_cond_wait(&ps->dongle.write_done_cond, &ps->dongle.mutex);
   pthread_mutex_unlock(&ps->dongle.mutex);

   return EMC_R_OK;
}

DLL_EXPORT enum EMC_RESULT emc_ui_estop(void *hd)
{
   emc_command_msg_t mb;
   emc_task_set_state_msg_t *cmd;
   struct emc_session *ps = (struct emc_session *)hd;

   cmd = (emc_task_set_state_msg_t *) & mb;
   cmd->msg.type = EMC_TASK_SET_STATE_TYPE;
   cmd->state = EMC_TASK_STATE_ESTOP;
   cmd->msg.serial_number = ++emcCommandSerialNumber;
   _wait_received(ps, send_message(ps, &mb, _gui_tag), 0.0);
   return EMC_R_OK;
}       /* emc_ui_send_estop() */

DLL_EXPORT enum EMC_RESULT emc_ui_estop_reset(void *hd)
{
   emc_command_msg_t mb;
   emc_task_set_state_msg_t *cmd;
   struct emc_session *ps = (struct emc_session *)hd;

   cmd = (emc_task_set_state_msg_t *) & mb;
   cmd->msg.type = EMC_TASK_SET_STATE_TYPE;
   cmd->state = EMC_TASK_STATE_ESTOP_RESET;
   cmd->msg.serial_number = ++emcCommandSerialNumber;
   _wait_received(ps, send_message(ps, &mb, _gui_tag), 0.0);
   return EMC_R_OK;
}       /* emc_ui_send_estop_reset() */

DLL_EXPORT enum EMC_RESULT emc_ui_machine_on(void *hd)
{
   emc_command_msg_t mb;
   emc_task_set_state_msg_t *cmd;
   struct emc_session *ps = (struct emc_session *)hd;

   cmd = (emc_task_set_state_msg_t *) & mb;
   cmd->msg.type = EMC_TASK_SET_STATE_TYPE;
   cmd->state = EMC_TASK_STATE_ON;
   cmd->msg.serial_number = ++emcCommandSerialNumber;
   _wait_received(ps, send_message(ps, &mb, _gui_tag), 0.0);
   return EMC_R_OK;
}       /* emc_ui_send_machine_on() */

DLL_EXPORT enum EMC_RESULT emc_ui_machine_off(void *hd)
{
   emc_command_msg_t mb;
   emc_task_set_state_msg_t *cmd;
   struct emc_session *ps = (struct emc_session *)hd;

   cmd = (emc_task_set_state_msg_t *) & mb;
   cmd->msg.type = EMC_TASK_SET_STATE_TYPE;
   cmd->state = EMC_TASK_STATE_OFF;
   cmd->msg.serial_number = ++emcCommandSerialNumber;
   _wait_received(ps, send_message(ps, &mb, _gui_tag), 0.0);
   return EMC_R_OK;
}       /* emc_ui_send_machine_off() */

DLL_EXPORT enum EMC_RESULT emc_ui_manual_mode(void *hd)
{
   emc_command_msg_t mb;
   emc_task_set_mode_msg_t *cmd;
   struct emc_session *ps = (struct emc_session *)hd;

   cmd = (emc_task_set_mode_msg_t *) & mb;
   cmd->msg.type = EMC_TASK_SET_MODE_TYPE;
   cmd->mode = EMC_TASK_MODE_MANUAL;
   cmd->msg.serial_number = ++emcCommandSerialNumber;
   _wait_received(ps, send_message(ps, &mb, _gui_tag), 0.0);
   return EMC_R_OK;
}       /*  emc_ui_set_manual() */

DLL_EXPORT enum EMC_RESULT emc_ui_auto_mode(void *hd)
{
   emc_command_msg_t mb;
   emc_task_set_mode_msg_t *cmd;
   struct emc_session *ps = (struct emc_session *)hd;

   cmd = (emc_task_set_mode_msg_t *) & mb;
   cmd->msg.type = EMC_TASK_SET_MODE_TYPE;
   cmd->mode = EMC_TASK_MODE_AUTO;
   cmd->msg.serial_number = ++emcCommandSerialNumber;
   _wait_received(ps, send_message(ps, &mb, _gui_tag), 0.0);
   return EMC_R_OK;
}       /*  emc_ui_set_auto() */

DLL_EXPORT enum EMC_RESULT emc_ui_mdi_mode(void *hd)
{
   emc_command_msg_t mb;
   emc_task_set_mode_msg_t *cmd;
   struct emc_session *ps = (struct emc_session *)hd;

   cmd = (emc_task_set_mode_msg_t *) & mb;
   cmd->msg.type = EMC_TASK_SET_MODE_TYPE;
   cmd->mode = EMC_TASK_MODE_MDI;
   cmd->msg.serial_number = ++emcCommandSerialNumber;
   _wait_received(ps, send_message(ps, &mb, _gui_tag), 0.0);
   return EMC_R_OK;
}       /*  emc_ui_set_mdi() */

DLL_EXPORT enum EMC_RESULT emc_ui_mist_on(void *hd)
{
   emc_command_msg_t mb;
   struct emc_session *ps = (struct emc_session *)hd;

   mb.msg.type = EMC_COOLANT_MIST_ON_TYPE;
   mb.msg.serial_number = ++emcCommandSerialNumber;
   _wait_received(ps, send_message(ps, &mb, _gui_tag), 0.0);
   return EMC_R_OK;
}       /* emc_ui_send_mist_on() */

DLL_EXPORT enum EMC_RESULT emc_ui_mist_off(void *hd)
{
   emc_command_msg_t mb;
   struct emc_session *ps = (struct emc_session *)hd;

   mb.msg.type = EMC_COOLANT_MIST_OFF_TYPE;
   mb.msg.serial_number = ++emcCommandSerialNumber;
   _wait_received(ps, send_message(ps, &mb, _gui_tag), 0.0);
   return EMC_R_OK;
}       /* emc_ui_send_mist_off() */

DLL_EXPORT enum EMC_RESULT emc_ui_flood_on(void *hd)
{
   emc_command_msg_t mb;
   struct emc_session *ps = (struct emc_session *)hd;

   mb.msg.type = EMC_COOLANT_FLOOD_ON_TYPE;
   mb.msg.serial_number = ++emcCommandSerialNumber;
   _wait_received(ps, send_message(ps, &mb, _gui_tag), 0.0);
   return EMC_R_OK;
}       /* emc_ui_send_flood_on() */

DLL_EXPORT enum EMC_RESULT emc_ui_flood_off(void *hd)
{
   emc_command_msg_t mb;
   struct emc_session *ps = (struct emc_session *)hd;

   mb.msg.type = EMC_COOLANT_FLOOD_OFF_TYPE;
   mb.msg.serial_number = ++emcCommandSerialNumber;
   _wait_received(ps, send_message(ps, &mb, _gui_tag), 0.0);
   return EMC_R_OK;
}       /* emc_ui_send_flood_off() */

DLL_EXPORT enum EMC_RESULT emc_ui_lube_on(void *hd)
{
   emc_command_msg_t mb;
   struct emc_session *ps = (struct emc_session *)hd;

   mb.msg.type = EMC_LUBE_ON_TYPE;
   mb.msg.serial_number = ++emcCommandSerialNumber;
   _wait_received(ps, send_message(ps, &mb, _gui_tag), 0.0);
   return EMC_R_OK;
}       /* emc_ui_send_lube_on() */

DLL_EXPORT enum EMC_RESULT emc_ui_lube_off(void *hd)
{
   emc_command_msg_t mb;
   struct emc_session *ps = (struct emc_session *)hd;

   mb.msg.type = EMC_LUBE_OFF_TYPE;
   mb.msg.serial_number = ++emcCommandSerialNumber;
   _wait_received(ps, send_message(ps, &mb, _gui_tag), 0.0);
   return EMC_R_OK;
}       /* emc_ui_send_lube_off() */

DLL_EXPORT enum EMC_RESULT emc_ui_spindle_forward(void *hd)
{
   emc_command_msg_t mb;
   emc_spindle_on_msg_t *cmd;
   struct emc_session *ps = (struct emc_session *)hd;

   cmd = (emc_spindle_on_msg_t *) & mb;
   cmd->msg.type = EMC_SPINDLE_ON_TYPE;
   cmd->speed = 500;
   cmd->msg.serial_number = ++emcCommandSerialNumber;
   _wait_received(ps, send_message(ps, &mb, _gui_tag), 0.0);
   return EMC_R_OK;
}       /* emc_ui_send_spindle_forward() */

DLL_EXPORT enum EMC_RESULT emc_ui_spindle_reverse(void *hd)
{
   emc_command_msg_t mb;
   emc_spindle_on_msg_t *cmd;
   struct emc_session *ps = (struct emc_session *)hd;

   cmd = (emc_spindle_on_msg_t *) & mb;
   cmd->msg.type = EMC_SPINDLE_ON_TYPE;
   cmd->speed = -500;
   cmd->msg.serial_number = ++emcCommandSerialNumber;
   _wait_received(ps, send_message(ps, &mb, _gui_tag), 0.0);
   return EMC_R_OK;
}       /* emc_ui_send_spindle_reverse() */

DLL_EXPORT enum EMC_RESULT emc_ui_spindle_off(void *hd)
{
   emc_command_msg_t mb;
   struct emc_session *ps = (struct emc_session *)hd;

   mb.msg.type = EMC_SPINDLE_OFF_TYPE;
   mb.msg.serial_number = ++emcCommandSerialNumber;
   _wait_received(ps, send_message(ps, &mb, _gui_tag), 0.0);
   return EMC_R_OK;
}       /* emc_ui_send_spindle_off() */

DLL_EXPORT enum EMC_RESULT emc_ui_tool_table(void *hd, const char *file)
{
   emc_command_msg_t mb;
   emc_tool_load_tool_table_msg_t *cmd;
   struct emc_session *ps = (struct emc_session *)hd;

   cmd = (emc_tool_load_tool_table_msg_t *) & mb;
   cmd->msg.type = EMC_TOOL_LOAD_TOOL_TABLE_TYPE;
   strncpy(cmd->file, file, sizeof(cmd->file));
   cmd->msg.serial_number = ++emcCommandSerialNumber;
   _wait_received(ps, send_message(ps, &mb, _gui_tag), 0.0);
   return EMC_R_OK;
}       /* emc_ui_send_load_tool_table() */

DLL_EXPORT enum EMC_RESULT emc_ui_tool_offset(void *hd, int toolno, double zoffset, double diameter)
{
   emc_command_msg_t mb;
   emc_tool_set_offset_msg_t *cmd;
   struct emc_session *ps = (struct emc_session *)hd;

   cmd = (emc_tool_set_offset_msg_t *) & mb;
   cmd->msg.type = EMC_TOOL_SET_OFFSET_TYPE;
   cmd->toolno = toolno;
   cmd->offset.tran.z = zoffset;
   cmd->diameter = diameter;
   cmd->orientation = 0;        // mill style tool table
   cmd->msg.serial_number = ++emcCommandSerialNumber;
   _wait_received(ps, send_message(ps, &mb, _gui_tag), 0.0);
   return EMC_R_OK;
}       /* emc_ui_send_tool_set_offset() */

DLL_EXPORT enum EMC_RESULT emc_ui_override_limits(void *hd, int axis)
{
   emc_command_msg_t mb;
   emc_axis_cmd_msg_t *cmd;
   struct emc_session *ps = (struct emc_session *)hd;

   cmd = (emc_axis_cmd_msg_t *) & mb;
   cmd->msg.type = EMC_AXIS_OVERRIDE_LIMITS_TYPE;
   cmd->axis = axis;    /* negative means off */
   cmd->msg.serial_number = ++emcCommandSerialNumber;
   _wait_received(ps, send_message(ps, &mb, _gui_tag), 0.0);
   return EMC_R_OK;
}       /* emc_ui_send_override_limits() */

DLL_EXPORT enum EMC_RESULT emc_ui_mdi_cmd(void *hd, const char *mdi)
{
   emc_command_msg_t mb;
   emc_task_plan_execute_msg_t *cmd;
   struct emc_session *ps = (struct emc_session *)hd;

   cmd = (emc_task_plan_execute_msg_t *) & mb;
   cmd->msg.type = EMC_TASK_PLAN_EXECUTE_TYPE;
   strncpy(cmd->command, mdi, sizeof(cmd->command));
   cmd->msg.serial_number = ++emcCommandSerialNumber;
   _wait_received(ps, send_message(ps, &mb, _gui_tag), 0.0);
   return EMC_R_OK;
}       /* emc_ui_send_mdi_cmd() */

DLL_EXPORT enum EMC_RESULT emc_ui_home(void *hd, int axis)
{
   emc_command_msg_t mb;
   emc_axis_cmd_msg_t *cmd;
   struct emc_session *ps = (struct emc_session *)hd;

   cmd = (emc_axis_cmd_msg_t *) & mb;
   cmd->msg.type = EMC_AXIS_HOME_TYPE;
   cmd->axis = axis;
   cmd->msg.serial_number = ++emcCommandSerialNumber;
   _wait_received(ps, send_message(ps, &mb, _gui_tag), 0.0);
   return EMC_R_OK;
}       /* emc_ui_send_home() */

DLL_EXPORT enum EMC_RESULT emc_ui_unhome(void *hd, int axis)
{
   emc_command_msg_t mb;
   emc_axis_cmd_msg_t *cmd;
   struct emc_session *ps = (struct emc_session *)hd;

   cmd = (emc_axis_cmd_msg_t *) & mb;
   cmd->msg.type = EMC_AXIS_UNHOME_TYPE;
   cmd->axis = axis;
   cmd->msg.serial_number = ++emcCommandSerialNumber;
   _wait_received(ps, send_message(ps, &mb, _gui_tag), 0.0);
   return EMC_R_OK;
}       /* emc_ui_send_unhome() */

DLL_EXPORT enum EMC_RESULT emc_ui_jog_stop(void *hd, int axis)
{
   emc_command_msg_t mb;
   struct emc_session *ps = (struct emc_session *)hd;

   if (axis < 0 || axis >= EMC_AXIS_MAX)
   {
      BUG("invalid input axis=%d\n", axis);
      return EMC_R_ERROR;
   }

   // in case of TELEOP mode we really need to send an TELEOP_VECTOR message
   // not a simple AXIS_ABORT, as more than one axis would be moving
   // (hint TELEOP mode is for nontrivial kinematics)
   if (emcStatus->motion.traj.mode != EMC_TRAJ_MODE_TELEOP)
   {
      emc_axis_cmd_msg_t *cmd;
      cmd = (emc_axis_cmd_msg_t *) & mb;
      cmd->msg.type = EMC_AXIS_ABORT_TYPE;
      cmd->axis = axis;
      cmd->msg.serial_number = ++emcCommandSerialNumber;
   }
   else
   {
      emc_traj_set_teleop_vector_msg_t *cmd;
      cmd = (emc_traj_set_teleop_vector_msg_t *) & mb;
      cmd->msg.type = EMC_TRAJ_SET_TELEOP_VECTOR_TYPE;
      ZERO_EMC_POSE(cmd->vector);
      cmd->msg.serial_number = ++emcCommandSerialNumber;
   }
   _wait_received(ps, send_message(ps, &mb, _gui_tag), 0.0);
   return EMC_R_OK;
}       /* emc_ui_send_jog_stop() */

DLL_EXPORT enum EMC_RESULT emc_ui_jog_incr(void *hd, int axis, double speed, double incr)
{
   emc_command_msg_t mb;
   emc_axis_incr_jog_msg_t *cmd;
   struct emc_session *ps = (struct emc_session *)hd;

   if (axis < 0 || axis >= EMC_AXIS_MAX)
   {
      BUG("invalid input axis=%d\n", axis);
      return EMC_R_ERROR;
   }

   cmd = (emc_axis_incr_jog_msg_t *) & mb;
   cmd->msg.type = EMC_AXIS_INCR_JOG_TYPE;
   cmd->axis = axis;
   cmd->vel = speed / 60.0;   /* convert units/minute to units/second */
   cmd->incr = incr;
   cmd->msg.serial_number = ++emcCommandSerialNumber;
   _wait_received(ps, send_message(ps, &mb, _gui_tag), 0.0);
   return EMC_R_OK;
}       /*  emc_ui_send_jog_incr() */

DLL_EXPORT enum EMC_RESULT emc_ui_jog_abs(void *hd, int axis, double speed, double pos)
{
   emc_command_msg_t mb;
   emc_axis_abs_jog_msg_t *cmd;
   struct emc_session *ps = (struct emc_session *)hd;

   if (axis < 0 || axis >= EMC_AXIS_MAX)
   {
      BUG("invalid input axis=%d\n", axis);
      return EMC_R_ERROR;
   }

   cmd = (emc_axis_abs_jog_msg_t *) & mb;
   cmd->msg.type = EMC_AXIS_ABS_JOG_TYPE;
   cmd->axis = axis;
   cmd->vel = speed / 60.0;       /* convert units/minute to units/second */
   cmd->pos = pos;
   cmd->msg.serial_number = ++emcCommandSerialNumber;
   _wait_received(ps, send_message(ps, &mb, _gui_tag), 0.0);
   return EMC_R_OK;
}       /*  emc_ui_jog_abs() */

DLL_EXPORT enum EMC_RESULT emc_ui_feed_override(void *hd, double override)
{
   emc_command_msg_t mb;
   emc_traj_set_scale_msg_t *cmd;
   struct emc_session *ps = (struct emc_session *)hd;

   if (override < 0.0)
   {
      override = 0.0;
   }

   cmd = (emc_traj_set_scale_msg_t *) & mb;
   cmd->msg.type = EMC_TRAJ_SET_SCALE_TYPE;
   cmd->scale = override;
   cmd->msg.serial_number = ++emcCommandSerialNumber;
   _wait_received(ps, send_message(ps, &mb, _gui_tag), 0.0);
   return EMC_R_OK;
}       /* emc_ui_send_feed_override() */

DLL_EXPORT enum EMC_RESULT emc_ui_plan_init(void *hd)
{
   emc_command_msg_t mb;
   struct emc_session *ps = (struct emc_session *)hd;

   mb.msg.type = EMC_TASK_PLAN_INIT_TYPE;
   mb.msg.serial_number = ++emcCommandSerialNumber;
   _wait_received(ps, send_message(ps, &mb, _gui_tag), 0.0);
   return EMC_R_OK;
}       /* emc_ui_send_task_plan_init() */

DLL_EXPORT enum EMC_RESULT emc_ui_program_open(void *hd, const char *program)
{
   emc_command_msg_t mb;
   emc_task_plan_open_msg_t *cmd;
   struct emc_session *ps = (struct emc_session *)hd;

   cmd = (emc_task_plan_open_msg_t *) & mb;
   cmd->msg.type = EMC_TASK_PLAN_OPEN_TYPE;
   strncpy(cmd->file, program, sizeof(cmd->file));
   cmd->msg.serial_number = ++emcCommandSerialNumber;
   _wait_received(ps, send_message(ps, &mb, _gui_tag), 0.0);
   return EMC_R_OK;
}       /* emc_ui_send_program_open() */

DLL_EXPORT enum EMC_RESULT emc_ui_program_run(void *hd, int line)
{
   emc_command_msg_t mb;
   emc_task_plan_run_msg_t *cmd;
   struct emc_session *ps = (struct emc_session *)hd;

   cmd = (emc_task_plan_run_msg_t *) & mb;
   cmd->msg.type = EMC_TASK_PLAN_RUN_TYPE;
   cmd->line = line;
   cmd->msg.serial_number = ++emcCommandSerialNumber;
   _wait_received(ps, send_message(ps, &mb, _gui_tag), 0.0);
   return EMC_R_OK;
}       /* emc_ui_send_program_run() */

DLL_EXPORT enum EMC_RESULT emc_ui_program_pause(void *hd)
{
   emc_command_msg_t mb;
   struct emc_session *ps = (struct emc_session *)hd;

   mb.msg.type = EMC_TASK_PLAN_PAUSE_TYPE;
   mb.msg.serial_number = ++emcCommandSerialNumber;
   _wait_received(ps, send_message(ps, &mb, _gui_tag), 0.0);
   return EMC_R_OK;
}       /* emc_ui_send_program_pause() */

DLL_EXPORT enum EMC_RESULT emc_ui_program_resume(void *hd)
{
   emc_command_msg_t mb;
   struct emc_session *ps = (struct emc_session *)hd;

   mb.msg.type = EMC_TASK_PLAN_RESUME_TYPE;
   mb.msg.serial_number = ++emcCommandSerialNumber;
   _wait_received(ps, send_message(ps, &mb, _gui_tag), 0.0);
   return EMC_R_OK;
}       /* emc_ui_send_program_resume() */

DLL_EXPORT enum EMC_RESULT emc_ui_program_step(void *hd)
{
   emc_command_msg_t mb;
   struct emc_session *ps = (struct emc_session *)hd;

   mb.msg.type = EMC_TASK_PLAN_STEP_TYPE;
   mb.msg.serial_number = ++emcCommandSerialNumber;
   _wait_received(ps, send_message(ps, &mb, _gui_tag), 0.0);
   return EMC_R_OK;
}       /* emc_ui_send_program_step() */

DLL_EXPORT enum EMC_RESULT emc_ui_abort(void *hd)
{
   emc_command_msg_t mb;
   struct emc_session *ps = (struct emc_session *)hd;

   mb.msg.type = EMC_TASK_ABORT_TYPE;
   mb.msg.serial_number = ++emcCommandSerialNumber;
   _wait_received(ps, send_message(ps, &mb, _gui_tag), 0.0);
   return EMC_R_OK;
}       /* emc_ui_send_abort() */

DLL_EXPORT enum EMC_RESULT emc_ui_axis_backlash(void *hd, int axis, double backlash)
{
   emc_command_msg_t mb;
   emc_axis_set_backlash_msg_t *cmd;
   struct emc_session *ps = (struct emc_session *)hd;

   if (axis < 0 || axis >= EMC_AXIS_MAX)
   {
      BUG("invalid input axis=%d\n", axis);
      return EMC_R_ERROR;
   }

   cmd = (emc_axis_set_backlash_msg_t *) & mb;
   cmd->msg.type = EMC_AXIS_SET_BACKLASH_TYPE;
   cmd->axis = axis;
   cmd->backlash = backlash;
   cmd->msg.serial_number = ++emcCommandSerialNumber;
   _wait_received(ps, send_message(ps, &mb, _gui_tag), 0.0);
   return EMC_R_OK;
}       /*  emc_ui_send_axis_set_backlash() */

DLL_EXPORT enum EMC_RESULT emc_ui_teleop_enable(void *hd, int enable)
{
   emc_command_msg_t mb;
   emc_traj_set_teleop_enable_msg_t *cmd;
   struct emc_session *ps = (struct emc_session *)hd;

   cmd = (emc_traj_set_teleop_enable_msg_t *) & mb;
   cmd->msg.type = EMC_TRAJ_SET_TELEOP_ENABLE_TYPE;
   cmd->enable = enable;
   cmd->msg.serial_number = ++emcCommandSerialNumber;
   _wait_received(ps, send_message(ps, &mb, _gui_tag), 0.0);
   return EMC_R_OK;
}       /* emc_ui_send_teleop_enable() */

DLL_EXPORT enum EMC_RESULT emc_ui_dout(void *hd, int output_num, int value, int sync)
{
   emc_command_msg_t mb;
   emc_motion_set_dout_msg_t *cmd;
   struct emc_session *ps = (struct emc_session *)hd;

   cmd = (emc_motion_set_dout_msg_t *)&mb;
   cmd->msg.type = EMC_MOTION_SET_DOUT_TYPE;
   cmd->output_num = output_num;
   cmd->value = value;
   cmd->sync = sync;
   cmd->msg.serial_number = ++emcCommandSerialNumber;
   _wait_received(ps, send_message(ps, &mb, _gui_tag), 0.0);
   return EMC_R_OK;
}       /* emc_ui_set_dout() */

DLL_EXPORT enum EMC_RESULT emc_ui_enable_din_abort(void *hd, int input_num)
{
   emc_command_msg_t mb;
   emc_motion_din_msg_t *cmd;
   struct emc_session *ps = (struct emc_session *)hd;

   cmd = (emc_motion_din_msg_t *) & mb;
   cmd->msg.type = EMC_MOTION_ENABLE_DIN_ABORT_TYPE;
   cmd->input_num = input_num;
   cmd->msg.serial_number = ++emcCommandSerialNumber;
   _wait_received(ps, send_message(ps, &mb, _gui_tag), 0.0);
   return EMC_R_OK;
}       /*  emc_ui_enable_input_abort() */

DLL_EXPORT enum EMC_RESULT emc_ui_disable_din_abort(void *hd, int input_num)
{
   emc_command_msg_t mb;
   emc_motion_din_msg_t *cmd;
   struct emc_session *ps = (struct emc_session *)hd;

   cmd = (emc_motion_din_msg_t *) & mb;
   cmd->msg.type = EMC_MOTION_DISABLE_DIN_ABORT_TYPE;
   cmd->input_num = input_num;
   cmd->msg.serial_number = ++emcCommandSerialNumber;
   _wait_received(ps, send_message(ps, &mb, _gui_tag), 0.0);
   return EMC_R_OK;
}       /*  emc_ui_disable_input_abort() */

static void command_thread(struct emc_session *ps)
{
   emc_command_msg_t *m, *emcCommand=NULL;
   enum RTSTEPPER_RESULT ret;
   unsigned int last = 0;
   int lock = 0, done;

   pthread_detach(pthread_self());

   emcIoInit();
   emcMotionInit();
   emcIoUpdate(&emcStatus->io);
   emcMotionUpdate(&emcStatus->motion);

   /* Initialize the interpreter. */
   if (emcTaskPlanInit() != EMC_R_OK)
   {
      BUG("can't initialize interpreter\n");
      emcOperatorMessage(0, EMC_I18N("can't initialize interpreter"));
   }

   while (!ps->command_thread_abort)
   {
      /* With dongle connected, following query provides a 1ms cycle time. */
      ret = rtstepper_query_state(&ps->dongle);

      if (rtstepper_is_input0_triggered(&ps->dongle) == RTSTEPPER_R_INPUT_TRUE)
      {
         emcTaskSetState(EMC_TASK_STATE_ESTOP);
         BUG("INPUT0 estop...\n");
         emcOperatorMessage(0, "INPUT0 ESTOP...");
      }
      if (rtstepper_is_input1_triggered(&ps->dongle) == RTSTEPPER_R_INPUT_TRUE)
      {
         emcTaskSetState(EMC_TASK_STATE_ESTOP);
         BUG("INPUT1 estop...\n");
         emcOperatorMessage(0, "INPUT1 ESTOP...");
      }
      if (rtstepper_is_input2_triggered(&ps->dongle) == RTSTEPPER_R_INPUT_TRUE)
      {
         emcTaskSetState(EMC_TASK_STATE_ESTOP);
         BUG("INPUT2 estop...\n");
         emcOperatorMessage(0, "INPUT2 ESTOP...");
      }

      if (ret == RTSTEPPER_R_REQ_ERROR)
         esleep(0.01);  /* No dongle connected, let other processes have cpu time */

      done = 0;
      while (!done)
      {
         /* Check all messages looking for control_thread command. */
         peek_message(ps, &m, &last, &lock, _ctl_tag);

         if (m)
         {
            if (m->msg.type > MAX_EMC_TO_GUI_COMMAND)
            {
               /* Found GUI to EMC command. Do not remove jog command if until previous move is complete. */
               if ((m->msg.type <= MAX_GUI_TO_EMC_IMMEDIATE_CMD) || (m->msg.type > MAX_GUI_TO_EMC_IMMEDIATE_CMD && emcStatus->motion.traj.inpos))
               {
                  remove_message(ps, m, &lock, _ctl_tag);
                  emcCommand = m;
                  done = 1;
               }
            }
         }
         else
         {
            break; /* no more messages */
         }
      }

      emcTaskPlan(emcCommand);
      emcTaskExecute();

      // update subordinate status
      emcIoUpdate(&emcStatus->io);
      emcMotionUpdate(&emcStatus->motion);

      // check for subordinate errors, and halt task if so
      if (emcStatus->motion.status == RCS_ERROR || emcStatus->io.status == RCS_ERROR)
      {
         /* If already aborted don't do it again. */
         if (emcStatus->status != RCS_ERROR)
         {
            emcTaskAbort();
            emcIoAbort();
            emcSpindleOff();
         }

#if 0   // done in emcTaskAbort() DES
         // clear out the pending command
         emcTaskCommand = NULL;
         interp_list.clear();
         emcStatus->task.currentLine = 0;

         // clear out the interpreter state
         emcStatus->task.interpState = EMC_TASK_INTERP_IDLE;
         emcStatus->task.execState = EMC_TASK_EXEC_DONE;

         // now queue up command to resynch interpreter
         interp_list.append((emc_command_msg_t *) & taskPlanSynchCmd);
#endif
      }

      // update task-specific status
      emcTaskUpdate(&emcStatus->task);

      if (emcCommand)
      {
         // get task status
         emcStatus->task.command_type = emcCommand->msg.type;
         emcStatus->task.echo_serial_number = emcCommand->msg.n;
         // get top level status
         emcStatus->command_type = emcCommand->msg.type;
         emcStatus->echo_serial_number = emcCommand->msg.n;
      }

      if (emcStatus->task.planState == EMC_TASK_PLAN_ERROR || emcStatus->task.execState == EMC_TASK_EXEC_ERROR ||
          emcStatus->motion.status == RCS_ERROR || emcStatus->io.status == RCS_ERROR)
      {
         emcStatus->status = RCS_ERROR;
         emcStatus->task.status = RCS_ERROR;
      }
      else if (emcStatus->task.planState == EMC_TASK_PLAN_DONE && emcStatus->task.execState == EMC_TASK_EXEC_DONE &&
               emcStatus->motion.status == RCS_DONE && emcStatus->io.status == RCS_DONE && interp_list.len() == 0 &&
               emcTaskCommand == NULL && emcStatus->task.interpState == EMC_TASK_INTERP_IDLE)
      {
         emcStatus->status = RCS_DONE;
         emcStatus->task.status = RCS_DONE;
      }
      else
      {
         emcStatus->status = RCS_EXEC;
         emcStatus->task.status = RCS_EXEC;
      }

      if (emcCommand)
      {
         free(emcCommand);
         emcCommand = NULL;
      }

#if 0
      if (emcStatus->status != RCS_DONE)
      {
         DBG("**planState=%d execState=%s motion.status=%s interp.len=%d emcTaskCmd=%p interpState=%s motionFlag=%x\n",
            emcStatus->task.planState, lookup_task_exec_state(emcStatus->task.execState), lookup_rcs_status(emcStatus->motion.status),
            interp_list.len(), emcTaskCommand, lookup_task_interp_state(emcStatus->task.interpState), emcmotStatus.motionFlag);
      }
#endif
   }    /* while (!ps->control_thread_abort) */

   emcTaskPlanExit();
   emcMotionHalt();
   emcIoHalt();

   /* Reap any remaining messages. Free any outstanding lock. */
   DBG("reaping messages...\n");
   while (1)
   {
      peek_message(ps, &m, &last, &lock, _ctl_tag);
      if (m)
      {
         remove_message(ps, m, &lock, _ctl_tag);
         free(m);
         continue;
      }
      break;
   }
   DBG("done reaping\n");

   rtstepper_set_abort_wait(&ps->dongle);

   /* Wait for any outstanding control_cycle_thread to finish. */
   pthread_mutex_lock(&ps->mutex);
   while (ps->control_cycle_thread_active)
      pthread_cond_wait(&ps->control_cycle_thread_done_cond, &ps->mutex);
   pthread_mutex_unlock(&ps->mutex);

   DBG("exiting command_thread()\n");

   pthread_mutex_lock(&ps->mutex);
   ps->command_thread_active = 0;
   pthread_cond_signal(&ps->command_thread_done_cond);
   pthread_mutex_unlock(&ps->mutex);

   return;
}       /* conmand_thread() */

DLL_EXPORT void *emc_ui_open(const char *ini_file)
{
   struct emc_session *ret = NULL, *ps = &session; /* TODO: support multiple sessions. 7/10/2013 DES */
   FILE *logfd = NULL;
   char log_file[256], serial_num[64];

   if ((USER_HOME_DIR = getenv("HOME")) == NULL)
      USER_HOME_DIR = "";        /* no $HOME directory, default to top level */

   if (ini_file && ini_file[0])
   {
      /* Check for tilde expansion. */
      if (ini_file[0] == '~' && ini_file[1] == '/')
         snprintf(EMC_INIFILE, sizeof(EMC_INIFILE), "%s%s", USER_HOME_DIR, ini_file+1);
      else
         snprintf(EMC_INIFILE, sizeof(EMC_INIFILE), "%s", ini_file);
   }

   pthread_mutex_init(&ps->mutex, NULL);
   pthread_mutex_init(&ps->dongle.mutex, NULL);
   pthread_cond_init(&ps->command_thread_done_cond, NULL);
   pthread_cond_init(&ps->control_cycle_thread_done_cond, NULL);
   pthread_cond_init(&ps->mcode_thread_done_cond, NULL);
   pthread_cond_init(&ps->event_cond, NULL);
   pthread_cond_init(&ps->dongle.write_done_cond, NULL);
   INIT_LIST_HEAD(&ps->head.list);

   iniGetKeyValue("TASK", "SERIAL_NUMBER", serial_num, sizeof(serial_num));

   /* Make sure we can open the log file in the user's home directory. */
   snprintf(log_file, sizeof(log_file), "%s/.%s/%s%s.log", USER_HOME_DIR, PACKAGE_NAME, EMC2_TASKNAME, serial_num);
   if ((logfd = fopen(log_file, "r")) == NULL)
   {
      /* Open failed, create directory. */
      snprintf(log_file, sizeof(log_file), "%s/.%s", USER_HOME_DIR, PACKAGE_NAME);
#if (defined(__WIN32__) || defined(_WINDOWS))
      if (mkdir(log_file))
#else
      if (mkdir(log_file, 0700))
#endif
      {
         if (errno != EEXIST)
         {
            fprintf(stderr, "unable to create %s\n", log_file);
            emcOperatorMessage(0, EMC_I18N("unable to create %s"), log_file);
            goto bugout;        /* bail */
         }
      }
   }
   else
      fclose(logfd);

   snprintf(log_file, sizeof(log_file), "%s/.%s/%s%s", USER_HOME_DIR, PACKAGE_NAME, EMC2_TASKNAME, serial_num);
   rtstepper_open_log(log_file, RTSTEPPER_LOG_BACKUP);

   DBG("[%d] emc_ui_init() ini=%s\n", getpid(), ini_file);

   emcInitGlobals();
   emcCommandSerialNumber = 0;

   if (iniTask() != EMC_R_OK)
      goto bugout;

   /* Init some dongle defaults?? Was normally set by taskintf.cc (task to motion interface). */
   emcStatus->io.aux.estop = 1;
   emcStatus->motion.traj.enabled = 0;
   emcStatus->motion.traj.mode = EMC_TRAJ_MODE_FREE;    /* for manual mode */
   emcStatus->io.status = RCS_DONE;

   emcStatus->task.interpState = EMC_TASK_INTERP_IDLE;
   emcStatus->task.execState = EMC_TASK_EXEC_DONE;
   emcStatus->task.state = EMC_TASK_STATE_ESTOP;
   emcStatus->task.mode = EMC_TASK_MODE_MANUAL;

   if (rtstepper_init(&ps->dongle, emc_io_error_cb) != RTSTEPPER_R_OK)
      emcOperatorMessage(0, EMC_I18N("unable to connnect to rt-stepper dongle"));

   ps->command_thread_active = 1;
   ps->command_thread_abort = 0;
   if (pthread_create(&ps->command_thread_tid, NULL, (void *(*)(void *)) command_thread, (void *) ps) != 0)
   {
      BUG("unable to creat command_thread\n");
      ps->command_thread_active = 0;
      goto bugout;      /* bail */
   }

   ret = ps;

 bugout:
   return ret;  /* return an opaque handle for this rtstepper dongle */
}       /* emc_ui_open() */

DLL_EXPORT enum EMC_RESULT emc_ui_close(void *hd)
{
   struct emc_session *ps = (struct emc_session *)hd;
   emc_command_msg_t mb;

   DBG("[%d] emc_ui_exit()\n", getpid());

   if (ps->mcode_thread_active)
   {
      /* Wait for mcode_thread(s) to finish. */
      pthread_mutex_lock(&ps->mutex);
      while (ps->mcode_thread_active)
         pthread_cond_wait(&ps->mcode_thread_done_cond, &ps->mutex);
      pthread_mutex_unlock(&ps->mutex);
   }

   if (ps->command_thread_active)
   {
      /* Gracefully kill command_thread. */
      mb.msg.type = EMC_QUIT_TYPE;
      mb.msg.serial_number = ++emcCommandSerialNumber;
      send_message(ps, &mb, _gui_tag);

      pthread_mutex_lock(&ps->mutex);
      while (ps->command_thread_active)
         pthread_cond_wait(&ps->command_thread_done_cond, &ps->mutex);
      pthread_mutex_unlock(&ps->mutex);
   }

   rtstepper_exit(&ps->dongle);
   pthread_mutex_destroy(&ps->mutex);
   pthread_mutex_destroy(&ps->dongle.mutex);
   pthread_cond_destroy(&ps->command_thread_done_cond);
   pthread_cond_destroy(&ps->control_cycle_thread_done_cond);
   pthread_cond_destroy(&ps->mcode_thread_done_cond);
   pthread_cond_destroy(&ps->event_cond);
   pthread_cond_destroy(&ps->dongle.write_done_cond);

   return EMC_R_OK;
}       /* emc_ui_close() */

static void emc_dll_init(void)
{
} /* emc_dll_init() */

static void emc_dll_exit(void)
{
   struct emc_session *ps = &session;
   DBG("[%d] emc_dll_exit()\n", getpid());

   if (ps->command_thread_active)
      emc_ui_close(ps);

   rtstepper_close_log();
} /* emc_dll_exit() */

#if (defined(__WIN32__) || defined(_WINDOWS))
BOOL WINAPI DllMain(HANDLE module, DWORD reason, LPVOID reserved)
{
   switch (reason)
   {
   case DLL_PROCESS_ATTACH:
      emc_dll_init();
      break;
   case DLL_PROCESS_DETACH:
      emc_dll_exit();
      break;
   default:
      break;
   }
   return TRUE;
}
#else
static void __attribute__ ((constructor)) _emc_init(void)
{
   emc_dll_init();
}

static void __attribute__ ((destructor)) _emc_exit(void)
{
   emc_dll_exit();
}
#endif
