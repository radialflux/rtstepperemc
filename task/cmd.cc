/*****************************************************************************\

  cmd.cc - staging support commands for EMC2

  Derived from a work by Fred Proctor & Will Shackleford

  Author:
  License: GPL Version 2
  System: Linux
    
  Copyright (c) 2004 All rights reserved.
  (c) 2008-2012 Copyright Eckler Software

  Principles of operation:

  1.  The control_thread calls emcTaskPlan() and emcTaskExecute() cyclically.

  2.  emcTaskPlan() reads the new command, and decides what to do with
  it based on the mode (manual, auto, mdi) or state (estop, on) of the
  machine. Many of the commands just go out immediately to the
  subsystems (motion and IO). In auto mode, the interpreter is called
  and as a result the interp_list is appended with MSG commands.

  3.  emcTaskExecute() executes a big switch on execState. If it's done,
  it gets the next item off the interp_list, and sets execState to the
  preconditions for that. These preconditions include waiting for motion,
  waiting for IO, etc. Once they are satisfied, it issues the command, and
  sets execState to the postconditions. Once those are satisfied, it gets
  the next item off the interp_list, and so on.

  4.  preconditions and postconditions are only looked at in conjunction
  with commands on the interp_list. Immediate commands won't have any
  pre- or postconditions associated with them looked at.

  5. Single-stepping is handled in checkPreconditions() as the first
  condition. If we're in single-stepping mode, as indicated by the
  variable 'stepping', we set the state to waiting-for-step. This
  polls on the variable 'steppingWait' which is reset to zero when a
  step command is received, and set to one when the command is
  issued.

  History:
 
\*****************************************************************************/

#include <stdio.h>
#include <stdarg.h>
#include <errno.h>
#include <time.h>
#include <string.h>
#include <float.h>      /* DBL_MAX */
#include <math.h>
#include <sys/time.h>
#include "emc.h"
#include "interpl.h"
#include "interp_return.h"
#include "rs274ngc_interp.h"    // the interpreter
#include "canon.h"
#include "ini.h"
#include "bug.h"
#include "tinypy.h"

static int emcTaskPlanExecute(const char *command);
static int emcTaskPlanExecuteEx(const char *command, int line_number);
static int emcTaskExecScript(int index, double p1, double p2, unsigned int ticket);

static emc_task_plan_run_msg_t taskPlanRunCmd = { {EMC_TASK_PLAN_RUN_TYPE} };

static emc_msg_t taskPlanSynchCmd = { EMC_TASK_PLAN_SYNCH_TYPE };

static Interp interp;

static int new_config;
static int no_force_homing;     /* force home before running MDI or program */
static int pseudoMdiLineNumber = INT_MIN;
static int taskplanopen;
static int stepping;
static int steppingWait;
static int steppedLine;
static int interpResumeState = EMC_TASK_INTERP_IDLE;
static int programStartLine;
static int task_plan_wait;
static double saveMinLimit[EMCMOT_MAX_JOINTS];
static double saveMaxLimit[EMCMOT_MAX_JOINTS];
static enum EMC_TASK_MODE mdiOrAuto = EMC_TASK_MODE_AUTO;
static double taskExecDelayTimeout;

void esleep(double seconds_to_sleep)
{
   if (seconds_to_sleep <= 0.0)
      return;

#if (defined(__WIN32__) || defined(_WINDOWS))
   SleepEx(((unsigned long) (seconds_to_sleep * 1000)), FALSE);
#else
   struct timespec rqtp;
   rqtp.tv_sec = seconds_to_sleep;
   rqtp.tv_nsec = (seconds_to_sleep - rqtp.tv_sec) * 1E9;
   if (nanosleep(&rqtp, NULL) < 0)
   {
      if (errno != EINTR)
      {
         BUG("nanosleep({tv_sec=%d,tv_nsec=%ld}) error (errno=%d) %s\n", (int) rqtp.tv_sec, rqtp.tv_nsec, errno, strerror(errno));
      }
   }
#endif
   return;
}       /* esleep() */

/* Return number of seconds since epoch. */
double etime(void)
{
  double retval = 0.0;
  struct timeval tp;

  gettimeofday(&tp, NULL);
  retval = ((double) tp.tv_sec) + (((double) tp.tv_usec) / 1000000.0);
  return retval;
}

static int sendCommand(emcio_command_t * emcioCommand)
{
   static int emcIoCommandSerialNumber = 0;

   emcioCommand->serial_number = ++emcIoCommandSerialNumber;
   emciocommandHandler(emcioCommand);
   if (emcioStatus.status != RCS_DONE)
   {
      BUG("Failed to send command to IO level (%d)\n", emcioCommand->type);
      return EMC_R_ERROR;
   }

   return EMC_R_OK;
}       /* sendCommand() */

static int all_homed(void)
{
   for (int i = 0; i < 9; i++)
   {
      unsigned int mask = 1 << i;
      if ((emcStatus->motion.traj.axis_mask & mask) && !emcStatus->motion.axis[i].homed)
         return 0;
   }
   return 1;
}       /* all_homed() */

/* If stepping is active, wait for next step command. */
static int is_stepping_ok()
{
   if (stepping)
   {
      if (!steppingWait)
      {
         steppingWait = 1;
         steppedLine = emcStatus->task.currentLine;
      }
      else
      {
         if (emcStatus->task.currentLine != steppedLine)
         {
            return 0;   /* stepping is not complete */
         }
      }
   }
   return 1;    /* stepping is ok */
}       /* is_stepping_ok() */

static void print_interp_error(int retval)
{
   char buf[LINELEN];
   int i;

   emcStatus->task.interpreter_errcode = retval;

   buf[0] = 0;
   interp.error_text(retval, buf, sizeof(buf));
   if (buf[0] != 0)
   {
      BUG("interp_error: %s\n", buf);
   }
   emcOperatorMessage(0, buf);
   DBG("Interpreter stack:\n");
   for (i = 0; i < 5; i++)
   {
      buf[0] = 0;
      interp.stack_name(i, buf, sizeof(buf));
      if (buf[0] == 0)
         break;
      DBG("  ---%s\n", buf);
   }
}       /* print_interp_error() */

/*
  Pop each message off the interpreter list and checks it against limits,
  resource availability, etc. in the status.
 */
static int checkInterpList(MSG_INTERP_LIST * il, emc_status_t * stat)
{
   emc_command_msg_t *cmd;

   // let's create some shortcuts to casts at compile time
#define operator_msg ((emc_operator_message_msg_t *)cmd)
#define linear_move ((emc_traj_linear_move_msg_t *)cmd)
#define circular_move ((emc_traj_circular_move_msg_t *)cmd)

   while (il->len() > 0)
   {
      cmd = il->get();

      switch (cmd->msg.type)
      {
      case EMC_OPERATOR_MESSAGE_TYPE:
         emcOperatorMessage(operator_msg->id, operator_msg->text);
         break;
      case EMC_TRAJ_LINEAR_MOVE_TYPE:
         if (linear_move->end.tran.x > stat->motion.axis[0].maxPositionLimit)
         {
            emcOperatorMessage(0, EMC_I18N("%s exceeds +X limit"), stat->task.command);
            return EMC_R_ERROR;
         }
         if (linear_move->end.tran.y > stat->motion.axis[1].maxPositionLimit)
         {
            emcOperatorMessage(0, EMC_I18N("%s exceeds +Y limit"), stat->task.command);
            return EMC_R_ERROR;
         }
         if (linear_move->end.tran.z > stat->motion.axis[2].maxPositionLimit)
         {
            emcOperatorMessage(0, EMC_I18N("%s exceeds +Z limit"), stat->task.command);
            return EMC_R_ERROR;
         }
         if (linear_move->end.tran.x < stat->motion.axis[0].minPositionLimit)
         {
            emcOperatorMessage(0, EMC_I18N("%s exceeds -X limit"), stat->task.command);
            return EMC_R_ERROR;
         }
         if (linear_move->end.tran.y < stat->motion.axis[1].minPositionLimit)
         {
            emcOperatorMessage(0, EMC_I18N("%s exceeds -Y limit"), stat->task.command);
            return EMC_R_ERROR;
         }
         if (linear_move->end.tran.z < stat->motion.axis[2].minPositionLimit)
         {
            emcOperatorMessage(0, EMC_I18N("%s exceeds -Z limit"), stat->task.command);
            return EMC_R_ERROR;
         }
         break;
      case EMC_TRAJ_CIRCULAR_MOVE_TYPE:
         if (circular_move->end.tran.x > stat->motion.axis[0].maxPositionLimit)
         {
            emcOperatorMessage(0, EMC_I18N("%s exceeds +X limit"), stat->task.command);
            return EMC_R_ERROR;
         }
         if (circular_move->end.tran.y > stat->motion.axis[1].maxPositionLimit)
         {
            emcOperatorMessage(0, EMC_I18N("%s exceeds +Y limit"), stat->task.command);
            return EMC_R_ERROR;
         }
         if (circular_move->end.tran.z > stat->motion.axis[2].maxPositionLimit)
         {
            emcOperatorMessage(0, EMC_I18N("%s exceeds +Z limit"), stat->task.command);
            return EMC_R_ERROR;
         }
         if (circular_move->end.tran.x < stat->motion.axis[0].minPositionLimit)
         {
            emcOperatorMessage(0, EMC_I18N("%s exceeds -X limit"), stat->task.command);
            return EMC_R_ERROR;
         }
         if (circular_move->end.tran.y < stat->motion.axis[1].minPositionLimit)
         {
            emcOperatorMessage(0, EMC_I18N("%s exceeds -Y limit"), stat->task.command);
            return EMC_R_ERROR;
         }
         if (circular_move->end.tran.z < stat->motion.axis[2].minPositionLimit)
         {
            emcOperatorMessage(0, EMC_I18N("%s exceeds -Z limit"), stat->task.command);
            return EMC_R_ERROR;
         }
         break;
      default:
         break;
      }
   }    /*  while (il->len() > 0)  */

   return EMC_R_OK;

   // get rid of the compile-time cast shortcuts
#undef circular_move_msg
#undef linear_move_msg
#undef operator_error_msg
}       /* checkInterpList() */

static void readahead_reading(void)
{
   int readRetval, execRetval;

   if (interp_list.len() <= EMC_TASK_INTERP_MAX_LEN)
   {
      int count = 0;
      while (1)
      {
         if (task_plan_wait)
         {
            // delay reading of next line until all is done
            if (interp_list.len() == 0 && emcTaskCommand == NULL && emcStatus->task.execState == EMC_TASK_EXEC_DONE)
            {
               task_plan_wait = 0;
            }
            break;  /* done for now */
         }
         else
         {
            if ((readRetval = emcTaskPlanRead()) != INTERP_OK)
            {
               /* Signal to the rest of the system that that the interp is now in a paused state. */
               emcStatus->task.interpState = EMC_TASK_INTERP_WAITING;
               if (readRetval > INTERP_MIN_ERROR)
               {
                  interp_list.clear();
                  break;   /* IO error, abort interpret */
               }
            }
            else
            {
               // got a good line record the line number and command
               emcStatus->task.readLine = interp.line();
               interp_list.set_line_number(emcStatus->task.readLine);
               emcTaskPlanCommand((char *) &emcStatus->task.command);
               // and execute it
               execRetval = emcTaskPlanExecute(0);
               if (execRetval > INTERP_MIN_ERROR)
               {
                  emcStatus->task.interpState = EMC_TASK_INTERP_WAITING;
                  interp_list.clear();
               }
               else if (execRetval == INTERP_EXIT)
               {
                  emcStatus->task.interpState = EMC_TASK_INTERP_WAITING;
               }
               else if (execRetval == INTERP_EXECUTE_FINISH)
               {
                  // INTERP_EXECUTE_FINISH signifies that no more reading should be done until
                  // everything outstanding is completed
                  task_plan_wait = 1;
                  // and resynch interp WM
                  interp_list.append((emc_command_msg_t *) & taskPlanSynchCmd);
               }
               else if (execRetval == INTERP_ENDFILE)
               {
                  // end of file
                  emcStatus->task.interpState = EMC_TASK_INTERP_WAITING;
                  emcStatus->task.motionLine = 0;
                  emcStatus->task.readLine = 0;
               }
               else
               {
                  // executed a good line
               }

               // throw the results away if we're supposed to read through it
               if (programStartLine < 0 || emcStatus->task.readLine < programStartLine)
               {
                  // we're stepping over lines (verify), so check them for limits, etc. and clear then out
                  if (0 != checkInterpList(&interp_list, emcStatus))
                  {
                     // problem with actions, so do same as we did for a bad read from emcTaskPlanRead() above
                     emcStatus->task.interpState = EMC_TASK_INTERP_WAITING;
                  }
                  // and clear it regardless
                  interp_list.clear();
               }

               if (emcStatus->task.readLine < programStartLine)
               {
                  //update the position with our current position, as the other positions are only skipped through
                  CANON_UPDATE_END_POINT(emcStatus->motion.traj.actualPosition.tran.x,
                                         emcStatus->motion.traj.actualPosition.tran.y,
                                         emcStatus->motion.traj.actualPosition.tran.z,
                                         emcStatus->motion.traj.actualPosition.a,
                                         emcStatus->motion.traj.actualPosition.b,
                                         emcStatus->motion.traj.actualPosition.c,
                                         emcStatus->motion.traj.actualPosition.u,
                                         emcStatus->motion.traj.actualPosition.v, emcStatus->motion.traj.actualPosition.w);
                  if (emcStatus->task.readLine + 1 == programStartLine)
                  {
                     interp.synch();
                     // reset programStartLine so we don't fall into our stepping routines
                     // if we happen to execute lines before the current point later (due to subroutines).
                     programStartLine = 0;
                  }
               }

               if (!(count++ < EMC_TASK_INTERP_MAX_LEN && emcStatus->task.interpState == EMC_TASK_INTERP_READING
                     && interp_list.len() <= EMC_TASK_INTERP_MAX_LEN * 2 / 3))
               {
                  break;        // done interpret;
               }
            }   // else if ((readRetval = emcTaskPlanRead()) != INTERP_OK)
         }      // else emcTaskPlanIsWait()
      } // while (1)
   }    // if (interp_list.len() <= EMC_TASK_INTERP_MAX_LEN)
}       /* readahead_reading() */

static void readahead_waiting(void)
{
   if (interp_list.len() == 0 && emcTaskCommand == NULL && emcStatus->motion.traj.queue == 0 && emcStatus->io.status == RCS_DONE)
   {    // finished
      if (taskplanopen)
      {
         emcTaskPlanClose();
         DBG("emcTaskPlanClose() called.\n");
         // then resynch interpreter
         interp_list.append((emc_command_msg_t *) & taskPlanSynchCmd);
      }
      else
      {
         emcStatus->task.interpState = EMC_TASK_INTERP_IDLE;
      }
      emcStatus->task.readLine = 0;
      interp_list.set_line_number(0);
   }
   else
   {
      // still executing
   }
}       /* readahead_waiting() */

/*
  Determine mode depends on traj mode, and mdiOrAuto flag.

  traj mode   mdiOrAuto     mode
  ---------   ---------     ----
  FREE        XXX           MANUAL
  COORD       MDI           MDI
  COORD       AUTO          AUTO
  */
static enum EMC_TASK_MODE determineMode()
{
   // if traj is in free mode, then we're in manual mode
   if (emcStatus->motion.traj.mode == EMC_TRAJ_MODE_FREE || emcStatus->motion.traj.mode == EMC_TRAJ_MODE_TELEOP)
   {
      return EMC_TASK_MODE_MANUAL;
   }
   // else traj is in coord mode-- we can be in either mdi or auto
   return mdiOrAuto;
}       /* determineMode() */

/*
  Determine state depends on traj enabled, io estop, and desired task state

  traj enabled   io estop      state
  ------------   --------      -----
  DISABLED       ESTOP         ESTOP
  ENABLED        ESTOP         ESTOP
  DISABLED       OUT OF ESTOP  ESTOP_RESET
  ENABLED        OUT OF ESTOP  ON
  */
static enum EMC_TASK_STATE determineState()
{
   if (emcStatus->io.aux.estop)
   {
      return EMC_TASK_STATE_ESTOP;
   }
   if (!emcStatus->motion.traj.enabled)
   {
      return EMC_TASK_STATE_ESTOP_RESET;
   }
   return EMC_TASK_STATE_ON;
}       /* determineState() */

static void emcmotSetCycleTime(unsigned long nsec)
{
   emcmotSetTrajCycleTime(nsec * 1e-9);
   emcmotSetServoCycleTime(nsec * 1e-9);
}

int emc_is_tp_done(struct emc_session *ps)
{
   int stat;

   if (ps->control_cycle_thread_active)
   {
      stat = 0; /* false, trajectory planner is not done */
   }
   else
   {
      stat = 1; /* true, trajectory planner is done */
   }
   return stat;
}       /* is_tp_done() */

static void _run_control_cycle(struct emc_session *ps)
{
   do
   {
      /* Run operating mode changes, trajectory and interpolation control cycles. */
      emcmotController(RTSTEPPER_PERIOD);
   }
   while (!(emcmotStatus.motionFlag & EMCMOT_MOTION_INPOS_BIT && emcmotStatus.depth == 0 && emcmotStatus.homing_active == 0));

   /* Wait for any previous write to finish. */
   pthread_mutex_lock(&ps->dongle.mutex);
   while (ps->dongle.xfr_active)
      pthread_cond_wait(&ps->dongle.write_done_cond, &ps->dongle.mutex);
   pthread_mutex_unlock(&ps->dongle.mutex);

   /* Start a new write. */
   rtstepper_start_xfr(&ps->dongle, tpGetExecId(&emcmotDebug.queue), emcmotStatus.traj.axes);
} /* _run_control_cycle() */

static void control_cycle_thread(struct emc_session *ps)
{
   pthread_detach(pthread_self());

   _run_control_cycle(ps);

   pthread_mutex_lock(&ps->mutex);
   ps->control_cycle_thread_active = 0;
   pthread_cond_signal(&ps->control_cycle_thread_done_cond);
   pthread_mutex_unlock(&ps->mutex);

   return;
} /* control_cycle_thread() */

/* Asynchronous IO write command. Called by control_thread(). Used by motion commands. */
static int _emc_aio_write_command(emcmot_command_t *c)
{
   struct emc_session *ps = &session;
   static int commandNum = 0;
   static unsigned char headCount = 0;
   int stat = EMC_R_ERROR;

//   DBG("emcmotWriteCommand()\n");

   c->head = ++headCount;
   c->tail = c->head;
   c->commandNum = ++commandNum;

   emcmotCommandHandler(c);

   if (emcmotStatus.commandStatus != EMCMOT_COMMAND_OK)
   {
      BUG("invalid emcmotCommandHandler command\n");
      goto bugout;      /* bail */
   }

   /* Wait for any previous control_cycle_thread to finish. */
   pthread_mutex_lock(&ps->mutex);
   while (ps->control_cycle_thread_active)
      pthread_cond_wait(&ps->control_cycle_thread_done_cond, &ps->mutex);
   pthread_mutex_unlock(&ps->mutex);

   ps->control_cycle_thread_active = 1;
   if (pthread_create(&ps->control_cycle_thread_tid, NULL, (void *(*)(void *))control_cycle_thread, (void *)ps) != 0)
   {
      BUG("unable to creat control_cycle_thread\n");
      goto bugout;      /* bail */
   }

   stat = EMC_R_OK;

bugout:
   return stat;
}       /* _emc_aio_write_command() */

/* Synchronous IO write command. Called by control_thread(). Used by immediate commands.  */
static int _emc_sio_write_command(emcmot_command_t *c)
{
   struct emc_session *ps = &session;
   static int commandNum = 0;
   static unsigned char headCount = 0;
   int stat = EMC_R_ERROR;

//   DBG("emcmotWriteCommand()\n");

   c->head = ++headCount;
   c->tail = c->head;
   c->commandNum = ++commandNum;

   emcmotCommandHandler(c);

   if (emcmotStatus.commandStatus != EMCMOT_COMMAND_OK)
   {
      BUG("invalid emcmotCommandHandler command\n");
      goto bugout;      /* bail */
   }

   /* Wait for any previous control_cycle_thread to finish. */
   pthread_mutex_lock(&ps->mutex);
   while (ps->control_cycle_thread_active)
      pthread_cond_wait(&ps->control_cycle_thread_done_cond, &ps->mutex);
   pthread_mutex_unlock(&ps->mutex);

   _run_control_cycle(ps);

   stat = EMC_R_OK;

bugout:
   return stat;
}       /* _emc_sio_write_command() */

int emcOperatorMessage(int id, const char *fmt, ...)
{
   emc_command_msg_t message;
   emc_operator_message_msg_t *op_message;
   struct emc_session *ps = &session;
   va_list args;

   op_message = (emc_operator_message_msg_t *) &message;
   op_message->msg.type = EMC_OPERATOR_MESSAGE_TYPE;

   va_start(args, fmt);
   vsnprintf(op_message->text, sizeof(op_message->text), fmt, args);
   op_message->text[sizeof(op_message->text) - 1] = 0;       /* force zero termination */

   send_message(ps, &message, _ctl_tag);
   va_end(args);

   return EMC_R_OK;
}       /* emcOperatorMessage() */

/* Called from rt-stepper usb bulk write thread. */
int emc_io_error_cb(int result)
{
   emc_command_msg_t mb;
   emc_task_set_state_msg_t *cmd;
   struct emc_session *ps = &session;

   BUG("USB data write error=%d estop...\n", result);
   emcOperatorMessage(0, EMC_I18N("USB data write error=%d ESTOP..."), result);

   if (emcStatus->task.state != EMC_TASK_STATE_ESTOP)
   {
      cmd = (emc_task_set_state_msg_t *) & mb;
      cmd->msg.type = EMC_TASK_SET_STATE_TYPE;
      cmd->state = EMC_TASK_STATE_ESTOP;
      send_message(ps, &mb, "rts");
   }

   return 0;
}       /* emc_io_error_cb() */

void emcInitGlobals()
{
   int i;
   for (i = 0; i < EMC_AXIS_MAX; i++)
      AXIS_MAX_VELOCITY[i] = DEFAULT_AXIS_MAX_VELOCITY;
}       /* emcInitGlobals() */

int emcToolSetToolTableFile(const char *filename)
{
   snprintf(TOOL_TABLE_FILE, sizeof(TOOL_TABLE_FILE), "%s/.%s/%s", USER_HOME_DIR, PACKAGE_NAME, filename);
   BUG("using TOOL_TABLE_FILE:%s:\n", TOOL_TABLE_FILE);
   return EMC_R_OK;
}       /* emcToolSetToolTableFile() */

int emcAuxEstopOn()
{
   emcio_command_t emcioCommand;
   emcioCommand.type = EMCIO_AUX_ESTOP_ON_COMMAND;
   return sendCommand(&emcioCommand);
}

int emcAuxEstopOff()
{
   emcio_command_t emcioCommand;
   emcioCommand.type = EMCIO_AUX_ESTOP_OFF_COMMAND;
   return sendCommand(&emcioCommand);
}

int emcIoInit()
{
   emcio_command_t emcioCommand;
   int retval;

   DBG("emcIoInit()\n");

   /* set status values to 'normal' */
   emcioStatus.aux.estop = 1;   //estop=1 means to emc that ESTOP condition is met
   emcioStatus.tool.pocketPrepped = -1;
   emcioStatus.tool.toolInSpindle = 0;
   emcioStatus.coolant.mist = 0;
   emcioStatus.coolant.flood = 0;
   emcioStatus.lube.on = 0;
   emcioStatus.lube.level = 1;

   if ((retval = iniTool()) != EMC_R_OK)
   {
      return retval;
   }

   emcioCommand.type = EMCIO_TOOL_INIT_COMMAND;

   // send init command to emcio
   if (sendCommand(&emcioCommand))
   {
      BUG("Can't forceCommand(ioInitMsg)\n");
      return EMC_R_ERROR;
   }

   return EMC_R_OK;
}       /* emcIoInit() */

int emcIoUpdate(emcio_status_t * stat)
{
   int i;

   // copy status
   stat->tool.pocketPrepped = emcioStatus.tool.pocketPrepped;
   stat->tool.pocketPrepped = emcioStatus.tool.toolInSpindle;
   for (i = 0; i < CANON_POCKETS_MAX; i++)
      stat->tool.toolTable[i] = emcioStatus.tool.toolTable[i];
   stat->coolant.mist = emcioStatus.coolant.mist;
   stat->coolant.flood = emcioStatus.coolant.flood;
   stat->lube.on = emcioStatus.lube.on;
   stat->lube.level = emcioStatus.lube.level;

   stat->status = RCS_DONE;

   return 0;
}       /* emcIoUpdate() */

int emcIoHalt()
{
   emcio_command_t emcioCommand;
   DBG("emcIoHalt()\n");
   emcioCommand.type = EMCIO_TOOL_HALT_COMMAND;
   return sendCommand(&emcioCommand);
}       /* emcIoHalt() */

int emcIoAbort()
{
   emcio_command_t emcioCommand;
   emcioCommand.type = EMCIO_TOOL_ABORT_COMMAND;
   return sendCommand(&emcioCommand);
}       /* emcIoAbort() */

int emcAxisUpdate(emcaxis_status_t * stat, int numAxes)
{
   struct emc_session *ps = &session;
   emcmot_joint_t *joint;
   int axis;

   // check for valid range
   if (numAxes <= 0 || numAxes > EMCMOT_MAX_JOINTS)
   {
      BUG("invalid input numAxis=%d\n", numAxes);
      return EMC_R_ERROR;
   }

   for (axis = 0; axis < numAxes; axis++)
   {
      /* point to joint data */

      joint = &(joints[axis]);

      stat[axis].axisType = joint->type;        /* TYPE set by ini file */

      if (new_config)
      {
         stat[axis].backlash = joint->backlash;
         stat[axis].minPositionLimit = joint->min_pos_limit;
         stat[axis].maxPositionLimit = joint->max_pos_limit;
         stat[axis].minFerror = joint->min_ferror;
         stat[axis].maxFerror = joint->max_ferror;
      }

      stat[axis].output = joint->pos_cmd;
      stat[axis].input = joint->pos_fb;
      stat[axis].velocity = joint->vel_cmd;
      stat[axis].ferrorCurrent = joint->ferror;
      stat[axis].ferrorHighMark = joint->ferror_high_mark;

      stat[axis].homing = (joint->flag & EMCMOT_JOINT_HOMING_BIT ? 1 : 0);
      stat[axis].homed = (joint->flag & EMCMOT_JOINT_HOMED_BIT ? 1 : 0);
      stat[axis].fault = (joint->flag & EMCMOT_JOINT_FAULT_BIT ? 1 : 0);
      stat[axis].enabled = (joint->flag & EMCMOT_JOINT_ENABLE_BIT ? 1 : 0);
      stat[axis].inpos = (joint->flag & EMCMOT_JOINT_INPOS_BIT ? 1 : 0);

/* FIXME - soft limits are now applied to the command, and should never
   happen */
      stat[axis].minSoftLimit = 0;
      stat[axis].maxSoftLimit = 0;
      stat[axis].minHardLimit = (joint->flag & EMCMOT_JOINT_MIN_HARD_LIMIT_BIT ? 1 : 0);
      stat[axis].maxHardLimit = (joint->flag & EMCMOT_JOINT_MAX_HARD_LIMIT_BIT ? 1 : 0);
      stat[axis].overrideLimits = ! !(emcmotStatus.overrideLimitMask);  // one

      if (joint->flag & EMCMOT_JOINT_ERROR_BIT)
      {
         if (stat[axis].status != RCS_ERROR)
         {
            BUG("Error on axis %d, command number %d\n", axis, emcmotStatus.commandNumEcho); 
            stat[axis].status = RCS_ERROR;
         }
      }
      else if (joint->flag & EMCMOT_JOINT_INPOS_BIT && emc_is_tp_done(ps))
      {
         stat[axis].status = RCS_DONE;
      }
      else
      {
         stat[axis].status = RCS_EXEC;
      }
   }    /* for (axis = 0; axis < numAxes; axis++) */
   return EMC_R_OK;
}       /* emcAxisUpdate() */

int emcAxisInit(int axis)
{
   int retval;
 
   DBG("emcAxisInit() axis=%d\n", axis);

   if (axis < 0 || axis >= EMCMOT_MAX_JOINTS)
   {
      BUG("emcAxisInit() failed axis=%d\n", axis);
      return EMC_R_ERROR;
   }

   if ((retval = iniAxis(axis)) != EMC_R_OK)
   {
      BUG("iniAxis() failed ini=%s\n", EMC_INIFILE);
   }

   return retval;
}       /* emcAxisInit() */

/* Called from GUI and INI. */
int emcAxisSetBacklash(int axis, double backlash)
{
   DBG("emcAxisSetBacklash() axis=%d\n", axis);
   if (isnan(backlash))
   {
      BUG("emcAxisSetBacklash() NaN error\n");
      return EMC_R_ERROR;
   }
   emcmotCommand.command = EMCMOT_SET_BACKLASH;
   emcmotCommand.axis = axis;
   emcmotCommand.backlash = backlash;
   return _emc_sio_write_command(&emcmotCommand);
}       /* emcAxisSetBacklash() */

int emcAxisSetMinPositionLimit(int axis, double limit)
{
   DBG("emcAxisSetMinPositionLimit() axis=%d\n", axis);
   if (isnan(limit))
   {
      BUG("emcAxisSetMinPosition() NaN error\n");
      return EMC_R_ERROR;
   }
   emcmotCommand.command = EMCMOT_SET_POSITION_LIMITS;
   emcmotCommand.axis = axis;
   emcmotCommand.maxLimit = saveMaxLimit[axis];
   emcmotCommand.minLimit = limit;
   saveMinLimit[axis] = limit;
   return _emc_sio_write_command(&emcmotCommand);
}       /* emcAxisSetMinPositionLimit() */

int emcAxisSetMaxPositionLimit(int axis, double limit)
{
   DBG("emcAxisSetMaxPositionLimit() axis=%d\n", axis);
   if (isnan(limit))
   {
      BUG("emcAxisSetMaxPosition() NaN error\n");
      return EMC_R_ERROR;
   }
   emcmotCommand.command = EMCMOT_SET_POSITION_LIMITS;
   emcmotCommand.axis = axis;
   emcmotCommand.minLimit = saveMinLimit[axis];
   emcmotCommand.maxLimit = limit;
   saveMaxLimit[axis] = limit;
   return _emc_sio_write_command(&emcmotCommand);
}       /* emcAxisSetMaxPositionLimit() */

int emcAxisSetFerror(int axis, double ferror)
{
   DBG("emcAxisSetFerror() axis=%d\n", axis);
   if (isnan(ferror))
   {
      BUG("emcAxisSetFerror() NaN error\n");
      return EMC_R_ERROR;
   }
   emcmotCommand.command = EMCMOT_SET_MAX_FERROR;
   emcmotCommand.axis = axis;
   emcmotCommand.maxFerror = ferror;
   return _emc_sio_write_command(&emcmotCommand);
}       /* emcAxisSetFerror() */

int emcAxisSetMinFerror(int axis, double ferror)
{
   DBG("emcAxisSetMinFerror() axis=%d\n", axis);
   if (isnan(ferror))
   {
      BUG("emcAxisSetMinFerror() NaN error\n");
      return EMC_R_ERROR;
   }
   emcmotCommand.command = EMCMOT_SET_MIN_FERROR;
   emcmotCommand.axis = axis;
   emcmotCommand.minFerror = ferror;
   return _emc_sio_write_command(&emcmotCommand);
}       /* emcAxisSetMinFerror() */

int emcAxisSetHomingParams(int axis, double home, double offset, double home_final_vel, double search_vel, double latch_vel,
                           int use_index, int ignore_limits, int is_shared, int sequence, int volatile_home)
{
   DBG("emcAxisSetHomingParams() axis=%d\n", axis);

   if (isnan(home) || isnan(offset) || isnan(home_final_vel) || isnan(search_vel) || isnan(latch_vel))
   {
      BUG("emcAxisSetHoming() NaN error\n");
      return EMC_R_ERROR;
   }
   emcmotCommand.command = EMCMOT_SET_HOMING_PARAMS;
   emcmotCommand.axis = axis;
   emcmotCommand.home = home;
   emcmotCommand.offset = offset;
   emcmotCommand.home_final_vel = home_final_vel;
   emcmotCommand.search_vel = search_vel;
   emcmotCommand.latch_vel = latch_vel;
   emcmotCommand.flags = 0;
   emcmotCommand.home_sequence = sequence;
   emcmotCommand.volatile_home = volatile_home;
   if (use_index)
      emcmotCommand.flags |= HOME_USE_INDEX;
   if (ignore_limits)
      emcmotCommand.flags |= HOME_IGNORE_LIMITS;
   if (is_shared)
      emcmotCommand.flags |= HOME_IS_SHARED;

   return _emc_sio_write_command(&emcmotCommand);
}       /* emcAxisSetHomingParams() */

int emcAxisSetMaxVelocity(int axis, double vel)
{
   DBG("emcAxisSetMaxVelocity() axis=%d vel=%0.5f\n", axis, vel);
   if (vel < 0.0)
      vel = 0.0;
   AXIS_MAX_VELOCITY[axis] = vel;
   emcmotCommand.command = EMCMOT_SET_JOINT_VEL_LIMIT;
   emcmotCommand.axis = axis;
   emcmotCommand.vel = vel;
   return _emc_sio_write_command(&emcmotCommand);
}       /* emcAxisSetMaxVelocity() */

int emcAxisSetMaxAcceleration(int axis, double acc)
{
   DBG("emcAxisSetMaxAcceleration() axis=%d acc=%0.5f\n", axis, acc);
   if (acc < 0.0)
      acc = 0.0;
   AXIS_MAX_ACCELERATION[axis] = acc;
   emcmotCommand.command = EMCMOT_SET_JOINT_ACC_LIMIT;
   emcmotCommand.axis = axis;
   emcmotCommand.acc = acc;
   return _emc_sio_write_command(&emcmotCommand);
}       /* emcAxisSetMaxAcceleration() */

int emcAxisSetInputScale(int axis, double scale)
{
   DBG("emcAxisSetInputScale() axis=%d scale=%0.5f\n", axis, scale);
   emcmotCommand.command = EMCMOT_SET_INPUT_SCALE;
   emcmotCommand.axis = axis;
   emcmotCommand.scale = scale;
   return _emc_sio_write_command(&emcmotCommand);
}       /* emcAxisSetInputScale() */

int emcAxisSetStepPin(int axis, int pin)
{
   DBG("emcAxisSetStepPin() axis=%d pin=%d\n", axis, pin);
   if (pin < 1 || pin > 25)
      pin = 0;
   emcmotCommand.command = EMCMOT_SET_STEP_PIN;
   emcmotCommand.axis = axis;
   emcmotCommand.pin = pin;
   return _emc_sio_write_command(&emcmotCommand);
}       /* emcAxisSetStepPin() */

int emcAxisSetDirectionPin(int axis, int pin)
{
   DBG("emcAxisSetDirectionPin() axis=%d pin=%d\n", axis, pin);
   if (pin < 1 || pin > 25)
      pin = 0;
   emcmotCommand.command = EMCMOT_SET_DIRECTION_PIN;
   emcmotCommand.axis = axis;
   emcmotCommand.pin = pin;
   return _emc_sio_write_command(&emcmotCommand);
}       /* emcAxisSetStepPin() */

int emcAxisSetStepPolarity(int axis, int polarity)
{
   DBG("emcAxisSetStepPolarity() axis=%d polarity=%d\n", axis, polarity);
   if (polarity < 0 || polarity > 1)
      polarity = 0;
   emcmotCommand.command = EMCMOT_SET_STEP_POLARITY;
   emcmotCommand.axis = axis;
   emcmotCommand.polarity = polarity;
   return _emc_sio_write_command(&emcmotCommand);
}       /* emcAxisSetStepPolarity() */

int emcAxisSetDirectionPolarity(int axis, int polarity)
{
   DBG("emcAxisSetDirectionPolarity() axis=%d polarity=%d\n", axis, polarity);
   if (polarity < 0 || polarity > 1)
      polarity = 0;
   emcmotCommand.command = EMCMOT_SET_DIRECTION_POLARITY;
   emcmotCommand.axis = axis;
   emcmotCommand.polarity = polarity;
   return _emc_sio_write_command(&emcmotCommand);
}       /* emcAxisSetDirectionPolarity() */

int emcAxisSetAxis(int axis, unsigned char axisType)
{
   if (axis < 0 || axis >= EMCMOT_MAX_JOINTS)
      return EMC_R_ERROR;
   emcmotStatus.axis[axis].axisType = axisType;
   return EMC_R_OK;
}       /* emcAxisSetAxis() */

int emcAxisSetUnits(int axis, double units)
{
   if (axis < 0 || axis >= EMCMOT_MAX_JOINTS)
      return EMC_R_ERROR;
   emcmotStatus.axis[axis].units = units;
   return EMC_R_OK;
}       /* emcAxisSetUnits() */

int emcAxisHome(int axis)
{
   DBG("emcAxisHome() axis=%d\n", axis);
   if (axis < -1 || axis >= EMCMOT_MAX_JOINTS)
      return EMC_R_ERROR;
   emcmotCommand.command = EMCMOT_HOME;
   emcmotCommand.axis = axis;
   return _emc_sio_write_command(&emcmotCommand);
}       /* emcAxisHome() */

int emcAxisUnhome(int axis)
{
   DBG("emcAxisUnhome() axis=%d\n", axis);
   if (axis < -2 || axis >= EMCMOT_MAX_JOINTS)
      return EMC_R_ERROR;
   emcmotCommand.command = EMCMOT_UNHOME;
   emcmotCommand.axis = axis;
   return _emc_sio_write_command(&emcmotCommand);
}       /* emcAxisUnhome() */

int emcAxisJog(int axis, double vel)
{
   DBG("emcAxisJob() axis=%d\n", axis);
   if (axis < 0 || axis >= EMCMOT_MAX_JOINTS)
      return EMC_R_ERROR;

   if (vel > AXIS_MAX_VELOCITY[axis])
      vel = AXIS_MAX_VELOCITY[axis];
   else if (vel < -AXIS_MAX_VELOCITY[axis])
      vel = -AXIS_MAX_VELOCITY[axis];

   emcmotCommand.command = EMCMOT_JOG_CONT;
   emcmotCommand.axis = axis;
   emcmotCommand.vel = vel;
   return _emc_aio_write_command(&emcmotCommand);
}       /* emcAxisJog() */

int emcAxisIncrJog(int axis, double incr, double vel)
{
   DBG("emcAxisIncrJog() axis=%d incr=%0.8f vel=%0.8f\n", axis, incr, vel);
   if (axis < 0 || axis >= EMCMOT_MAX_JOINTS)
      return EMC_R_ERROR;

   if (vel > AXIS_MAX_VELOCITY[axis])
      vel = AXIS_MAX_VELOCITY[axis];
   else if (vel < -AXIS_MAX_VELOCITY[axis])
      vel = -AXIS_MAX_VELOCITY[axis];

   emcmotCommand.command = EMCMOT_JOG_INCR;
   emcmotCommand.axis = axis;
   emcmotCommand.vel = vel;
   emcmotCommand.offset = incr;
   return _emc_aio_write_command(&emcmotCommand);
}       /* emcAxisIncrJog() */

int emcAxisAbsJog(int axis, double pos, double vel)
{
   DBG("emcAxisAbsJog() axis=%d pos=%0.8f vel=%0.8f\n", axis, pos, vel);
   if (axis < 0 || axis >= EMCMOT_MAX_JOINTS)
      return EMC_R_ERROR;

   if (vel > AXIS_MAX_VELOCITY[axis])
      vel = AXIS_MAX_VELOCITY[axis];
   else if (vel < -AXIS_MAX_VELOCITY[axis])
      vel = -AXIS_MAX_VELOCITY[axis];

   emcmotCommand.command = EMCMOT_JOG_ABS;
   emcmotCommand.axis = axis;
   emcmotCommand.vel = vel;
   emcmotCommand.offset = pos;
   return _emc_aio_write_command(&emcmotCommand);
}       /* emcAxisAbsJog() */

int emcAxisActivate(int axis)
{
   DBG("emcAxisActivate() axis=%d\n", axis);
   emcmotCommand.command = EMCMOT_ACTIVATE_JOINT;
   emcmotCommand.axis = axis;
   return _emc_sio_write_command(&emcmotCommand);
}       /* emcAxisActivate() */

int emcAxisHalt(int axis)
{
   emcmot_joint_t *joint;
   DBG("emcAxisHalt()??\n");

   joint = &joints[axis];
   if (GET_JOINT_ACTIVE_FLAG(joint))
   {
      /* if needed, add code here... */
   }

   return EMC_R_OK;
}       /* emcAxisHalt() */

int emcAxisEnable(int axis)
{
   DBG("emcAxisEnable() axis=%d\n", axis);
   emcmotCommand.command = EMCMOT_ENABLE_AMPLIFIER;
   emcmotCommand.axis = axis;
   return _emc_sio_write_command(&emcmotCommand);
}

int emcAxisDisable(int axis)
{
   DBG("emcAxisDisable() axis=%d\n", axis);
   emcmotCommand.command = EMCMOT_DISABLE_AMPLIFIER;
   emcmotCommand.axis = axis;
   return _emc_sio_write_command(&emcmotCommand);
}

int emcAxisAbort(int axis)
{
   DBG("emcAxisAbort() axis=%d\n", axis);
   emcmotCommand.command = EMCMOT_AXIS_ABORT;
   emcmotCommand.axis = axis;
   return _emc_sio_write_command(&emcmotCommand);
}

int emcMotionInit()
{
   emcmot_joint_t *joint;
   int joint_num, n, r;

   DBG("emcMotionInit()\n");

   ZERO_EMC_POSE(emcmotStatus.carte_pos_cmd);
   ZERO_EMC_POSE(emcmotStatus.carte_pos_fb);
   emcmotStatus.vel = DEFAULT_VELOCITY;
   emcmotConfig.limitVel = DEFAULT_VELOCITY;
   emcmotStatus.acc = DEFAULT_ACCELERATION;
   emcmotStatus.feed_scale = 1.0;
   emcmotStatus.spindle_scale = 1.0;
   emcmotStatus.net_feed_scale = 1.0;
   /* adaptive feed is off by default, feed override, spindle 
      override, and feed hold are on */
   emcmotStatus.enables_new = FS_ENABLED | SS_ENABLED | FH_ENABLED;
   emcmotStatus.enables_queued = kinematicsType();
   emcmotStatus.id = 0;
   emcmotStatus.depth = 0;
   emcmotStatus.activeDepth = 0;
   emcmotStatus.paused = 0;
   emcmotStatus.overrideLimitMask = 0;
   emcmotStatus.spindle.speed = 0.0;
   emcmotStatus.motionFlag = 0;
   SET_MOTION_INPOS_FLAG(1);
   SET_MOTION_ENABLE_FLAG(0);
   emcmotStatus.traj.kinematics_type = kinematicsType();
   emcmotSetCycleTime(RTSTEPPER_PERIOD);

    /* init motion emcmotDebug.queue */
   if (tpCreate(&emcmotDebug.queue, DEFAULT_TC_QUEUE_SIZE, emcmotDebug.queueTcSpace) == -1)
   {
      BUG("MOTION: failed to create motion emcmotDebug.queue\n");
      r = EMC_R_ERROR;
      goto bugout;
   }
   tpSetCycleTime(&emcmotDebug.queue, emcmotConfig.trajCycleTime);
   tpSetPos(&emcmotDebug.queue, emcmotStatus.carte_pos_cmd);
   tpSetVmax(&emcmotDebug.queue, emcmotStatus.vel, emcmotStatus.vel);
   tpSetAmax(&emcmotDebug.queue, emcmotStatus.acc);

   emcmotStatus.tail = 0;

   if ((r = emcTrajInit()) != EMC_R_OK)
      goto bugout;

   emcmotConfig.numJoints = emcmotStatus.traj.axes;
   emcmotConfig.kinematics_type = emcmotStatus.traj.kinematics_type;
   emcmotDebug.oldPos = emcmotStatus.carte_pos_cmd;
   ZERO_EMC_POSE(emcmotDebug.oldVel);

   /* init per-joint stuff */
   for (joint_num = 0; joint_num < emcmotStatus.traj.axes; joint_num++)
   {
      /* point to structure for this joint */
      joint = &joints[joint_num];

      /* init the config fields with some "reasonable" defaults" */

      joint->type = 0;
      joint->max_pos_limit = 1.0;
      joint->min_pos_limit = -1.0;
      joint->vel_limit = 1.0;
      joint->acc_limit = 1.0;
      joint->min_ferror = 0.01;
      joint->max_ferror = 1.0;
      joint->home_search_vel = 0.0;
      joint->home_latch_vel = 0.0;
      joint->home_final_vel = -1;
      joint->home_offset = 0.0;
      joint->home = 0.0;
      joint->home_flags = 0;
      joint->home_sequence = -1;
      joint->backlash = 0.0;

      joint->comp.entries = 0;
      joint->comp.entry = &(joint->comp.array[0]);
      /* the compensation code has -DBL_MAX at one end of the table
         and +DBL_MAX at the other so _all_ commanded positions are
         guaranteed to be covered by the table */
      joint->comp.array[0].nominal = -DBL_MAX;
      joint->comp.array[0].fwd_trim = 0.0;
      joint->comp.array[0].rev_trim = 0.0;
      joint->comp.array[0].fwd_slope = 0.0;
      joint->comp.array[0].rev_slope = 0.0;
      for (n = 1; n < EMCMOT_COMP_SIZE + 2; n++)
      {
         joint->comp.array[n].nominal = DBL_MAX;
         joint->comp.array[n].fwd_trim = 0.0;
         joint->comp.array[n].rev_trim = 0.0;
         joint->comp.array[n].fwd_slope = 0.0;
         joint->comp.array[n].rev_slope = 0.0;
      }

      /* init status info */
      joint->flag = 0;
      SET_JOINT_INPOS_FLAG(joint, 1);
      joint->coarse_pos = 0.0;
      joint->pos_cmd = 0.0;
      joint->vel_cmd = 0.0;
      joint->backlash_corr = 0.0;
      joint->backlash_filt = 0.0;
      joint->backlash_vel = 0.0;
      joint->motor_pos_cmd = 0.0;
      joint->motor_pos_fb = 0.0;
      joint->pos_fb = 0.0;
      joint->ferror = 0.0;
      joint->ferror_limit = joint->min_ferror;
      joint->ferror_high_mark = 0.0;

      /* init internal info */
      cubicInit(&(joint->cubic));
      cubicSetInterpolationRate(&(joint->cubic), emcmotConfig.interpolationRate);
      cubicSetSegmentTime(&(joint->cubic), emcmotConfig.servoCycleTime);

      /* init misc other stuff in joint structure */
      joint->big_vel = 10.0 * joint->vel_limit;
      joint->home_state = HOME_IDLE;

      emcAxisInit(joint_num);
   }

   r = EMC_R_OK;

 bugout:
   return r;
}       /* emcMotionInit() */

int emcMotionUpdate(emcmot_status_t * stat)
{
   int axis;
   int error = 0, exec = 0;
//    int dio, aio;

//    DBG("emcMotionUpdate()\n");

   // read the emcmot status
//    if (0 != usrmotReadEmcmotStatus(&emcmotStatus))
//        return 1;

   new_config = 0;
   if (emcmotStatus.config_num != emcmotConfig.config_num)
   {
//        if (0 != usrmotReadEmcmotConfig(&emcmotConfig))
//            return 1;
      emcmotStatus.config_num = emcmotConfig.config_num;        /* dongle change */
      new_config = 1;
   }

   emcAxisUpdate(stat->axis, emcmotStatus.traj.axes);
   emcTrajUpdate(&stat->traj);

   stat->spindle.enabled = emcmotStatus.spindle.speed != 0;
   stat->spindle.speed = emcmotStatus.spindle.speed;
   stat->spindle.brake = emcmotStatus.spindle.brake;
   stat->spindle.direction = emcmotStatus.spindle.direction;

#if 0
   for (dio = 0; dio < EMC_MAX_DIO; dio++)
   {
      stat->synch_di[dio] = emcmotStatus.synch_di[dio];
      stat->synch_do[dio] = emcmotStatus.synch_do[dio];
   }

   for (aio = 0; aio < EMC_MAX_AIO; aio++)
   {
      stat->analog_input[aio] = emcmotStatus.analog_input[aio];
      stat->analog_output[aio] = emcmotStatus.analog_output[aio];
   }
#endif

   for (axis = 0; axis < emcmotStatus.traj.axes; axis++)
   {
      if (stat->axis[axis].status == RCS_ERROR)
      {
         error = 1;
         break;
      }
      if (stat->axis[axis].status == RCS_EXEC)
      {
         exec = 1;
         break;
      }
   }

   // set the status flag
   if (stat->traj.status == RCS_ERROR)
      error = 1;
   else if (stat->traj.status == RCS_EXEC)
      exec = 1;

   if (error)
      stat->status = RCS_ERROR;
   else if (exec)
      stat->status = RCS_EXEC;
   else
      stat->status = RCS_DONE;

   return EMC_R_OK;
}       /* emcMotionUpdate() */

int emcMotionHalt()
{
   int i;

   DBG("emcMotionHalt()\n");
   for (i = 0; i < emcStatus->motion.traj.axes; i++)
      emcAxisHalt(i);

   emcTrajDisable();

   return EMC_R_OK;
}       /* emcMotionHalt() */

int emcMotionAbort()
{
   int i;

   DBG("emcMotionAbort()\n");

   for (i = 0; i < emcStatus->motion.traj.axes; i++)
      emcAxisAbort(i);
   emcTrajAbort();
   return EMC_R_OK;
}       /* emcMotionAbort() */

int emcTrajSetAxes(int axes, int axismask)
{
   DBG("emcTrajSetAxes() axes=%d\n", axes);
   if (axes == 0)
   {
      if (axismask & 256)
         axes = 9;
      else if (axismask & 128)
         axes = 8;
      else if (axismask & 64)
         axes = 7;
      else if (axismask & 32)
         axes = 6;
      else if (axismask & 16)
         axes = 5;
      else if (axismask & 8)
         axes = 4;
      else if (axismask & 4)
         axes = 3;
      else if (axismask & 2)
         axes = 2;
      else if (axismask & 1)
         axes = 1;
   }
   if (axes <= 0 || axes > EMCMOT_MAX_JOINTS || axismask >= (1 << axes))
   {
      BUG("emcTrajSetAxes failing: axes=%d axismask=%x\n", axes, axismask);
      return EMC_R_ERROR;
   }

   emcmotStatus.traj.axes = axes;
   emcmotStatus.traj.axis_mask = axismask;

   return EMC_R_OK;
}       /* emcTrajSetAxes() */

int emcTrajSetUnits(double linearUnits, double angularUnits)
{
   DBG("emcTrajSetUnits() linearUnits=%0.8f angularUnits=%0.8f\n", linearUnits, angularUnits);
   if (linearUnits <= 0.0 || angularUnits <= 0.0)
   {
      return EMC_R_ERROR;
   }

   emcmotStatus.traj.linearUnits = linearUnits;
   emcmotStatus.traj.angularUnits = angularUnits;

   return EMC_R_OK;
}       /* emcTrajSetUnits() */

int emcTrajUpdate(emctraj_status_t * stat)
{
   struct emc_session *ps = &session;
   int enables;

   stat->axes = emcmotStatus.traj.axes;
   stat->axis_mask = emcmotStatus.traj.axis_mask;
   stat->linearUnits = emcmotStatus.traj.linearUnits;
   stat->angularUnits = emcmotStatus.traj.angularUnits;

   stat->mode = emcmotStatus.motionFlag & EMCMOT_MOTION_TELEOP_BIT ? EMC_TRAJ_MODE_TELEOP
      : (emcmotStatus.motionFlag & EMCMOT_MOTION_COORD_BIT ? EMC_TRAJ_MODE_COORD : EMC_TRAJ_MODE_FREE);

   stat->enabled = 0;   /* start at disabled */
   if (emcmotStatus.motionFlag & EMCMOT_MOTION_ENABLE_BIT)
   {
      stat->enabled = 1;
   }

   stat->inpos = emcmotStatus.motionFlag & EMCMOT_MOTION_INPOS_BIT;
   stat->queue = emcmotStatus.depth;
   stat->activeQueue = emcmotStatus.activeDepth;
   stat->queueFull = emcmotStatus.queueFull;
   stat->id = emcmotStatus.id;
   stat->motion_type = emcmotStatus.motionType;
   stat->distance_to_go = emcmotStatus.distance_to_go;
   stat->dtg = emcmotStatus.dtg;
   stat->current_vel = emcmotStatus.current_vel;

   stat->paused = emcmotStatus.paused;
   stat->scale = emcmotStatus.feed_scale;
   stat->spindle_scale = emcmotStatus.spindle_scale;

   stat->position = emcmotStatus.carte_pos_cmd;

   stat->actualPosition = emcmotStatus.carte_pos_fb;

   stat->velocity = emcmotStatus.vel;
   stat->acceleration = emcmotStatus.acc;
   stat->maxAcceleration = emcmotStatus.traj.maxAcceleration;

   if (emcmotStatus.motionFlag & EMCMOT_MOTION_ERROR_BIT)
   {
      stat->status = RCS_ERROR;
   }
   else if (stat->inpos && (stat->queue == 0) && emc_is_tp_done(ps))
   {
      stat->status = RCS_DONE;
   }
   else
   {
      stat->status = RCS_EXEC;
   }

   stat->probedPosition = emcmotStatus.probedPos;

   stat->probeval = emcmotStatus.probeVal;
   stat->probing = emcmotStatus.probing;
   stat->probe_tripped = emcmotStatus.probeTripped;

   if (emcmotStatus.motionFlag & EMCMOT_MOTION_COORD_BIT)
      enables = emcmotStatus.enables_queued;
   else
      enables = emcmotStatus.enables_new;

   stat->feed_override_enabled = enables & FS_ENABLED;
   stat->spindle_override_enabled = enables & SS_ENABLED;
   stat->adaptive_feed_enabled = enables & AF_ENABLED;
   stat->feed_hold_enabled = enables & FH_ENABLED;

   if (new_config)
   {
      stat->cycleTime = emcmotStatus.traj.cycleTime;
      stat->kinematics_type = emcmotStatus.traj.kinematics_type;
      stat->maxVelocity = emcmotStatus.traj.maxVelocity;
   }

   return EMC_R_OK;
}       /* emcTrajUpdate() */

int emcTrajInit()
{
   int retval;

   DBG("emcTrajInit()\n");

   // initialize parameters from ini file
   if ((retval = iniTraj()) != EMC_R_OK)
   {
      BUG("iniTraj() failed ini=%s\n", EMC_INIFILE);
   }
   return retval;
}       /* emcTrajInit() */

int emcTrajAbort()
{
   DBG("emcTrajAbort()\n");
   emcmotCommand.command = EMCMOT_ABORT;
   return _emc_sio_write_command(&emcmotCommand);
}

int emcTrajSetVelocity(double vel, double ini_maxvel)
{
   DBG("emcTrajSetVelocity()\n");
   if (vel < 0.0)
      vel = 0.0;
   else if (vel > TRAJ_MAX_VELOCITY)
      vel = TRAJ_MAX_VELOCITY;

   if (ini_maxvel < 0.0)
      ini_maxvel = 0.0;
   else if (vel > TRAJ_MAX_VELOCITY)
      ini_maxvel = TRAJ_MAX_VELOCITY;
   emcmotCommand.command = EMCMOT_SET_VEL;
   emcmotCommand.vel = vel;
   emcmotCommand.ini_maxvel = ini_maxvel;
   return _emc_sio_write_command(&emcmotCommand);
}       /* emcTrajSetVelocity() */

int emcTrajSetAcceleration(double acc)
{
   DBG("emcTrajSetAcceleration() acc=%0.5f\n", acc);
   if (acc < 0.0)
      acc = 0.0;
   else if (acc > emcmotStatus.traj.maxAcceleration)
      acc = emcmotStatus.traj.maxAcceleration;
   emcmotCommand.command = EMCMOT_SET_ACC;
   emcmotCommand.acc = acc;
   return _emc_sio_write_command(&emcmotCommand);
}       /* emcTrajSetAcceleration() */

int emcTrajSetMaxVelocity(double vel)
{
   DBG("emcTrajSetMaxVelocity() vel=%0.5f\n", vel);
   if (vel < 0.0)
      vel = 0.0;
   emcmotCommand.command = EMCMOT_SET_VEL_LIMIT;
   emcmotCommand.vel = vel;
   return _emc_sio_write_command(&emcmotCommand);
}       /* emcTrajSetMaxVelocity() */

int emcTrajSetMaxAcceleration(double acc)
{
   if (acc < 0.0)
      acc = 0.0;
   emcmotStatus.traj.maxAcceleration = acc;
   return EMC_R_OK;
}       /* emcTrajSetMaxAcceleration() */

int emcTrajSetHome(EmcPose home)
{
   DBG("emcTrajSetHome()\n");
   if (isnan(home.tran.x) || isnan(home.tran.y) || isnan(home.tran.z) ||
       isnan(home.a) || isnan(home.b) || isnan(home.c) || isnan(home.u) || isnan(home.v) || isnan(home.w))
   {
      BUG("emcTrajSetHome() NaN error\n");
      return EMC_R_ERROR;       // ignore it for now, just don't send it
   }

   emcmotCommand.command = EMCMOT_SET_WORLD_HOME;
   emcmotCommand.pos = home;
   return _emc_sio_write_command(&emcmotCommand);
}       /* emcTrajSetHome() */

int emcTrajPause()
{
   DBG("emcTrajPause()\n");
   emcmotCommand.command = EMCMOT_PAUSE;
   return _emc_sio_write_command(&emcmotCommand);
}       /* emcTrajPause() */

int emcTrajStep()
{
   DBG("emcTrajStep()\n");
   emcmotCommand.command = EMCMOT_STEP;
   return _emc_sio_write_command(&emcmotCommand);
}       /* emcTrajStep() */

int emcTrajEnable()
{
   DBG("emcTrajEnable()\n");
   emcmotCommand.command = EMCMOT_ENABLE;
   return _emc_sio_write_command(&emcmotCommand);
}       /* emcTrajEnable() */

int emcTrajDisable()
{
   DBG("emcTrajDisable()\n");
   emcmotCommand.command = EMCMOT_DISABLE;
   return _emc_sio_write_command(&emcmotCommand);
}       /* emcTrajDisable() */

int emcTrajLinearMove(EmcPose end, int type, double vel, double ini_maxvel, double acc)
{
   DBG("emcTrajLinearMove() type=%d x=%0.6f y=%0.6f z=%0.6f a=%0.6f c=%0.6f vel=%0.6f ini_maxvel=%0.6f acc=%0.6f\n", type,
       end.tran.x, end.tran.y, end.tran.z, end.a, end.c, vel, ini_maxvel, acc);

   if (isnan(end.tran.x) || isnan(end.tran.y) || isnan(end.tran.z) ||
       isnan(end.a) || isnan(end.b) || isnan(end.c) || isnan(end.u) || isnan(end.v) || isnan(end.w))
   {
      BUG("emcTrajLinearMove() NaN error\n");
      return EMC_R_OK;                 // ignore it for now, just don't send it
   }

   emcmotCommand.command = EMCMOT_SET_LINE;
   emcmotCommand.pos = end;
   emcmotCommand.id = emcStatus->task.currentLine;
   emcmotCommand.motion_type = type;
   emcmotCommand.vel = vel;
   emcmotCommand.ini_maxvel = ini_maxvel;
   emcmotCommand.acc = acc;

   return _emc_aio_write_command(&emcmotCommand);
} /* emcTrajLinearMove() */

int emcTrajCircularMove(EmcPose end, PmCartesian center, PmCartesian normal, int turn, int type, double vel, double ini_maxvel, double acc)
{
   DBG("emcTrajCircularMove() type=%d\n", type);

   if (isnan(end.tran.x) || isnan(end.tran.y) || isnan(end.tran.z) ||
       isnan(end.a) || isnan(end.b) || isnan(end.c) ||
       isnan(end.u) || isnan(end.v) || isnan(end.w) ||
       isnan(center.x) || isnan(center.y) || isnan(center.z) || isnan(normal.x) || isnan(normal.y) || isnan(normal.z))
   {
      BUG("emcTrajCircularMove() NaN error\n");
      return EMC_R_OK;                 // ignore it for now, just don't send it
   }

   emcmotCommand.command = EMCMOT_SET_CIRCLE;
   emcmotCommand.pos = end;
   emcmotCommand.motion_type = type;
   emcmotCommand.center.x = center.x;
   emcmotCommand.center.y = center.y;
   emcmotCommand.center.z = center.z;
   emcmotCommand.normal.x = normal.x;
   emcmotCommand.normal.y = normal.y;
   emcmotCommand.normal.z = normal.z;
   emcmotCommand.turn = turn;
   emcmotCommand.id = emcStatus->task.currentLine;
   emcmotCommand.vel = vel;
   emcmotCommand.ini_maxvel = ini_maxvel;
   emcmotCommand.acc = acc;

   return _emc_aio_write_command(&emcmotCommand);
} /* emcTrajCircularMove() */

int emcCoolantMistOn()
{
//   emcio_command_t emcioCommand;
//   emcioCommand.type = EMCIO_COOLANT_MIST_ON_COMMAND;
//   return sendCommand(&emcioCommand);
   return emcTaskExecScript(7, 0.0, 0.0, 0);   /* execute m7.py script */
}       /* emcCoolantMistOn() */

int emcCoolantMistOff()
{
//   emcio_command_t emcioCommand;
//   emcioCommand.type = EMCIO_COOLANT_MIST_OFF_COMMAND;
//   return sendCommand(&emcioCommand);
   return emcTaskExecScript(9, 1.0, 0.0, 0);   /* execute m9.py script */
}       /* emcCoolantMistOn() */

int emcCoolantFloodOn()
{
//   emcio_command_t emcioCommand;
//   emcioCommand.type = EMCIO_COOLANT_FLOOD_ON_COMMAND;
//   return sendCommand(&emcioCommand);
   return emcTaskExecScript(8, 0.0, 0.0, 0);   /* execute m8.py script */
}       /* emcCoolantFloodOn() */

int emcCoolantFloodOff()
{
//   emcio_command_t emcioCommand;
//   emcioCommand.type = EMCIO_COOLANT_FLOOD_OFF_COMMAND;
//   return sendCommand(&emcioCommand);
   return emcTaskExecScript(9, 2.0, 0.0, 0);   /* execute m9.py script */
}       /* emcCoolantFloodOff() */

int emcLubeOn()
{
   emcio_command_t emcioCommand;
   emcioCommand.type = EMCIO_LUBE_ON_COMMAND;
   return sendCommand(&emcioCommand);
}       /* emcLubeOn() */

int emcLubeOff()
{
   emcio_command_t emcioCommand;
   emcioCommand.type = EMCIO_AUX_ESTOP_OFF_COMMAND;
   return sendCommand(&emcioCommand);
}       /* emcLubeOff() */

int emcSpindleOff()
{
   DBG("emcSpindleOff()\n");
//   emcmotCommand.command = EMCMOT_SPINDLE_OFF;
//   return _emc_sio_write_command(&emcmotCommand);
   return emcTaskExecScript(5, 0.0, 0.0, 0);         /* execute m5.py script */
}       /* emcSpindleOff() */

#if 0
int emcTaskInit()
{
   stepping = 0;
   steppingWait = 0;
   return EMC_R_OK;
}       /* emcTaskInit() */
#endif

/* Called when the tinypy VM finds a script error. */
static int mcode_exception_cb(int result)
{
   struct emc_session *ps = &session;
   DBG("[%d] mcode_exeception_cb() completed\n", pthread_self());

   pthread_mutex_lock(&ps->mutex);
   ps->mcode_thread_active--;
   if (ps->mcode_thread_active == 0)
      pthread_cond_signal(&ps->mcode_thread_done_cond);
   pthread_mutex_unlock(&ps->mutex);

   pthread_exit(NULL);
   return 0;
}

static void mcode_thread(struct mcode_args_t *ma)
{
   struct emc_session *ps = &session;
   struct tp_vm *tp;
   int argc;
   char *argv[4];
   char sz[LINELEN];
   char sz_p1[16];
   char sz_p2[16];

   pthread_detach(pthread_self());

   snprintf(sz, sizeof(sz), "%s/m%d.py", EMC_PROGRAM_PREFIX, ma->index);
   snprintf(sz_p1, sizeof(sz_p1), "%0.8f", ma->p1);
   sz_p1[sizeof(sz_p1)-1] = 0;
   snprintf(sz_p2, sizeof(sz_p2), "%0.8f", ma->p2);
   sz_p2[sizeof(sz_p2)-1] = 0;
   free(ma);

   DBG("[%d] mcode_thread() started %s\n", pthread_self(), sz);
   argv[0] = (char *)"tinypy";
   argv[1] = sz;
   argv[2] = sz_p1;
   argv[3] = sz_p2;
   argc = 4;

   tp = tp_init(argc, argv, mcode_exception_cb); /* specify script file name via argv */
   rtstepperpy_init(tp);             /* init tinypy rtstepper module */
   tp_call(tp,"py2bc","tinypy", tp_None);   /* execute tinypy script */
   tp_deinit(tp);

   DBG("[%d] mcode_thread() completed %s\n", pthread_self(), sz);

   pthread_mutex_lock(&ps->mutex);
   ps->mcode_thread_active--;
   if (ps->mcode_thread_active == 0)
      pthread_cond_signal(&ps->mcode_thread_done_cond);
   pthread_mutex_unlock(&ps->mutex);

   return;
} /* mcode_thread() */

int emcTaskExecScript(int index, double p1, double p2, unsigned int ticket)
{
   struct emc_session *ps = &session;
   struct mcode_args_t *ma;
   enum EMC_RESULT stat = EMC_R_ERROR;
   FILE *fd;
   char sz[LINELEN];

   /* Make sure we can open the mcode script file. */
   snprintf(sz, sizeof(sz), "%s/m%d.py", EMC_PROGRAM_PREFIX, index);
   if ((fd = fopen(sz, "r")) == NULL)
   {
      BUG("unable to open mcode script %s\n", sz);
      emcOperatorMessage(0, EMC_I18N("unable to open mcode script %s"), sz);
      goto bugout;
   }
   fclose(fd);

   if ((ma = (struct mcode_args_t *)malloc(sizeof(struct mcode_args_t))) == NULL)
      goto bugout;

   DBG("emcTaskExecScript() file=%s p1=%0.8f p2=%0.8f\n", sz, p1, p2);

   ma->index = index;
   ma->p1 = p1;
   ma->p2 = p2;
   ma->ticket = ticket;

   pthread_mutex_lock(&ps->mutex);
   ps->mcode_thread_active++;
   pthread_mutex_unlock(&ps->mutex);
   if (pthread_create(&ps->mcode_thread_tid, NULL, (void *(*)(void *))mcode_thread, (void *)ma) != 0)
   {
      BUG("unable to creat mcode_thread\n");
      goto bugout;      /* bail */
   }

   stat = EMC_R_OK;

bugout:
   return stat;
} /* emcTaskExecScript() */

int emcTaskAbort()
{
//    emcMotionAbort();
   DBG("emcTaskAbort()\n");

   // clear out the pending command
   emcTaskCommand = NULL;
   interp_list.clear();

   // clear out the interpreter state
   emcStatus->task.interpState = EMC_TASK_INTERP_IDLE;
   emcStatus->task.execState = EMC_TASK_EXEC_DONE;
   emcStatus->task.task_paused = 0;
   emcStatus->task.motionLine = 0;
   emcStatus->task.readLine = 0;
   emcStatus->task.currentLine = 0;
   stepping = 0;
   steppingWait = 0;

   // now queue up command to resynch interpreter
   interp_list.append((emc_command_msg_t *) & taskPlanSynchCmd);

   if (taskplanopen)
   {
      emcTaskPlanClose();
      BUG("emcTaskPlanClose() called\n");
   }

   return EMC_R_OK;
}       /* emcTaskAbort() */

int emcTaskUpdate(emctask_status_t * stat)
{
   stat->mode = (enum EMC_TASK_MODE) determineMode();
   stat->state = (enum EMC_TASK_STATE) determineState();

   if (emcStatus->motion.traj.id > 0)
      stat->motionLine = emcStatus->motion.traj.id;     /* line number */

//   char buf[LINELEN];
//   strcpy(stat->file, interp.file(buf, LINELEN));       /* gcode file */

   interp.active_g_codes(&stat->activeGCodes[0]);       /* active codes */
   interp.active_m_codes(&stat->activeMCodes[0]);
   interp.active_settings(&stat->activeSettings[0]);

   stat->optional_stop_state = GET_OPTIONAL_PROGRAM_STOP();
   stat->block_delete_state = GET_BLOCK_DELETE();
   stat->heartbeat++;

   return EMC_R_OK;
}       /* emcTaskUpdate() */

static int emcTaskSetMode(int mode)
{
   int retval;

   switch (mode)
   {
   case EMC_TASK_MODE_MANUAL:
      // go to manual mode
      DBG("emcTaskSetMode(EMC_TASK_MODE_MANUAL)\n");
      emcmotCommand.command = EMCMOT_FREE;
      retval = _emc_sio_write_command(&emcmotCommand);
      mdiOrAuto = EMC_TASK_MODE_AUTO;   // we'll default back to here
      break;
   case EMC_TASK_MODE_MDI:
      // go to mdi mode
      DBG("emcTaskSetMode(EMC_TASK_MODE_MDI)\n");
      emcmotCommand.command = EMCMOT_COORD;
      retval = _emc_sio_write_command(&emcmotCommand);
//      emcTaskAbort();
      interp.synch();
      mdiOrAuto = EMC_TASK_MODE_MDI;
      break;
   case EMC_TASK_MODE_AUTO:
      // go to auto mode
      DBG("emcTaskSetMode(EMC_TASK_MODE_AUTO)\n");
      emcmotCommand.command = EMCMOT_COORD;
      retval = _emc_sio_write_command(&emcmotCommand);
//      emcTaskAbort();
      interp.synch();
      mdiOrAuto = EMC_TASK_MODE_AUTO;
      break;

   default:
      retval = EMC_R_ERROR;
      break;
   }

   return retval;
}       /* emcTaskSetMode() */

int emcTaskSetState(int state)
{
   struct emc_session *ps = &session;
   int i;
   int retval = EMC_R_OK;

   switch (state)
   {
   case EMC_TASK_STATE_OFF:
      DBG("emcTaskSetState(EMC_TASK_STATE_OFF)\n");
      emcMotionAbort();
      // turn the machine servos off-- go into READY state
      //        emcSpindleAbort();
      for (i = 0; i < emcmotStatus.traj.axes; i++)
         emcAxisDisable(i);

      emcTrajDisable();
      emcLubeOff();
      emcTaskAbort();
      emcIoAbort();
      emcSpindleOff();
      emcAxisUnhome(-2);        // only those joints which are volatile_home
      interp.synch();
      break;
   case EMC_TASK_STATE_ON:
      // turn the machine servos on
      DBG("emcTaskSetState(EMC_TASK_STATE_ON)\n");
      emcTrajEnable();
      for (i = 0; i < emcmotStatus.traj.axes; i++)
         emcAxisEnable(i);
      emcLubeOn();
      break;
   case EMC_TASK_STATE_ESTOP_RESET:
      BUG("emcTaskSetState(EMC_TASK_STATE_ESTOP_RESET)\n");
      emcAuxEstopOff();
      //      emcLubeOff();
      //      emcTaskAbort();
      //      emcIoAbort();
      //      emcSpindleOff();
      emcStatus->io.aux.estop = 0;

      if (!rtstepper_is_connected(&ps->dongle))
      {
         if (rtstepper_init(&ps->dongle, emc_io_error_cb) != RTSTEPPER_R_OK)
         {
            emcOperatorMessage(0, EMC_I18N("unable to connect to rt-stepper dongle"));
            emcStatus->io.aux.estop = 1;
         }
         else
         {
            emcOperatorMessage(0, EMC_I18N("sucessfully connected to rt-stepper dongle"));
         }
      }
      rtstepper_clear_abort(&ps->dongle);   /* clear any outstanding abort */

      interp.synch();
      break;
   case EMC_TASK_STATE_ESTOP:
      BUG("emcTaskSetState(EMC_TASK_STATE_ESTOP)\n");
      if (ps->control_cycle_thread_active)
      {
         pthread_cancel(ps->control_cycle_thread_tid); /* kill trajectory planner now */
         ps->control_cycle_thread_active = 0;
      }
      rtstepper_set_abort_wait(&ps->dongle);
      emcMotionAbort();
      emcSpindleOff();
      emcAuxEstopOn();
      for (i = 0; i < emcmotStatus.traj.axes; i++)
         emcAxisDisable(i);
      emcTrajDisable();
      emcLubeOff();
      emcCoolantMistOff();
      emcCoolantFloodOff();
      emcTaskAbort();
      emcIoAbort();
      emcAxisUnhome(-2);        // only those joints which are volatile_home

      // rtstepper dongle change
      emcStatus->io.aux.estop = 1;
      SET_MOTION_INPOS_FLAG(1);
      for (i = 0; i < emcmotStatus.traj.axes; i++)
         SET_JOINT_INPOS_FLAG(&joints[i], 1);

      interp.synch();
      break;
   default:
      retval = EMC_R_ERROR;
      break;
   }

   return retval;
}       /* emcTaskSetState() */

// Used by emcTaskPlan() and by emcTaskExecute(). Issue command immediately to motion controller.
static int emcTaskIssueCommand(emc_command_msg_t * cmd)
{
   emcio_command_t emcioCommand;
   double speed;
   int retval = EMC_R_ERROR;
   int execRetval = 0;
   int mode;
   char *file, *command;

   if (cmd == NULL)
   {
      BUG("emcTaskIssueCommand() null command\n");
      return 0;
   }

   DBG("emcTaskIssueCommand() type=%d cmd=%s execState=%s interp.len=%d  interpState=%s\n", cmd->msg.type, lookup_message(cmd->msg.type),
         lookup_task_exec_state(emcStatus->task.execState), interp_list.len(), lookup_task_interp_state(emcStatus->task.interpState));

   switch (cmd->msg.type)
   {

      // axis commands

   case EMC_AXIS_ABORT_TYPE:
      retval = emcAxisAbort(((emc_axis_cmd_msg_t *) cmd)->axis);
      break;
   case EMC_AXIS_HOME_TYPE:
      retval = emcAxisHome(((emc_axis_cmd_msg_t *) cmd)->axis);
      break;
   case EMC_AXIS_UNHOME_TYPE:
      retval = emcAxisUnhome(((emc_axis_cmd_msg_t *) cmd)->axis);
      break;
   case EMC_AXIS_JOG_TYPE:
      retval = emcAxisJog(((emc_axis_jog_msg_t *) cmd)->axis, ((emc_axis_jog_msg_t *) cmd)->vel);
      break;
   case EMC_AXIS_INCR_JOG_TYPE:
      retval = emcAxisIncrJog(((emc_axis_incr_jog_msg_t *) cmd)->axis, ((emc_axis_incr_jog_msg_t *) cmd)->incr, ((emc_axis_incr_jog_msg_t *) cmd)->vel);
      break;
   case EMC_AXIS_ABS_JOG_TYPE:
      retval = emcAxisAbsJog(((emc_axis_abs_jog_msg_t *) cmd)->axis, ((emc_axis_abs_jog_msg_t *) cmd)->pos, ((emc_axis_abs_jog_msg_t *) cmd)->vel);
      break;
   case EMC_AXIS_SET_BACKLASH_TYPE:
      retval = emcAxisSetBacklash(((emc_axis_set_backlash_msg_t *) cmd)->axis, ((emc_axis_set_backlash_msg_t *) cmd)->backlash);
      break;

      // traj commands

   case EMC_TRAJ_LINEAR_MOVE_TYPE:
      retval = emcTrajLinearMove(((emc_traj_linear_move_msg_t *)cmd)->end, ((emc_traj_linear_move_msg_t *)cmd)->type, 
                                  ((emc_traj_linear_move_msg_t *)cmd)->vel, ((emc_traj_linear_move_msg_t *)cmd)->ini_maxvel, ((emc_traj_linear_move_msg_t *)cmd)->acc);
      break;
   case EMC_TRAJ_CIRCULAR_MOVE_TYPE:
      retval = emcTrajCircularMove(((emc_traj_circular_move_msg_t *)cmd)->end, ((emc_traj_circular_move_msg_t *)cmd)->center, 
                                   ((emc_traj_circular_move_msg_t *)cmd)->normal, ((emc_traj_circular_move_msg_t *)cmd)->turn, 
                                   ((emc_traj_circular_move_msg_t *)cmd)->type, ((emc_traj_circular_move_msg_t *)cmd)->vel, 
                                   ((emc_traj_circular_move_msg_t *)cmd)->ini_maxvel, ((emc_traj_circular_move_msg_t *)cmd)->acc);
      break;
   case EMC_TRAJ_DELAY_TYPE:
      taskExecDelayTimeout = etime() + ((emc_traj_delay_msg_t *)cmd)->delay;
      retval = EMC_R_OK;
      break;
   case EMC_TRAJ_SET_TERM_COND_TYPE:
      emcmotCommand.command = EMCMOT_SET_TERM_COND;
      emcmotCommand.termCond = ((emc_traj_set_term_cond_msg_t *)cmd)->cond == EMC_TRAJ_TERM_COND_STOP ? EMCMOT_TERM_COND_STOP : EMCMOT_TERM_COND_BLEND;
      emcmotCommand.tolerance = ((emc_traj_set_term_cond_msg_t *)cmd)->tolerance;
      retval = _emc_sio_write_command(&emcmotCommand);
      break;
   case EMC_TRAJ_SET_SPINDLESYNC_TYPE:
      emcmotCommand.command = EMCMOT_SET_SPINDLESYNC;
      emcmotCommand.spindlesync = ((emc_traj_set_spindlesync_msg_t *)cmd)->feed_per_revolution;
      emcmotCommand.flags = ((emc_traj_set_spindlesync_msg_t *)cmd)->velocity_mode;
      return _emc_sio_write_command(&emcmotCommand);
      break;
   case EMC_TRAJ_SET_OFFSET_TYPE:
      emcmotCommand.command = EMCMOT_SET_OFFSET;
      emcmotCommand.tool_offset = ((emc_traj_set_offset_msg_t *)cmd)->offset;
      return _emc_sio_write_command(&emcmotCommand);
   case EMC_TRAJ_SET_ROTATION_TYPE:
      emcStatus->task.rotation_xy = ((emc_traj_set_rotation_msg_t *) cmd)->rotation;
      retval = EMC_R_OK;
      break;
   case EMC_TRAJ_SET_ORIGIN_TYPE:
      emcStatus->task.origin = ((emc_traj_set_origin_msg_t *) cmd)->origin;
      retval = EMC_R_OK;
      break;
   case EMC_TRAJ_SET_SCALE_TYPE:
      emcmotCommand.command = EMCMOT_FEED_SCALE;
      emcmotCommand.scale = ((emc_traj_set_scale_msg_t *) cmd)->scale;
      retval = _emc_sio_write_command(&emcmotCommand);
      break;
   case EMC_TRAJ_SET_TELEOP_ENABLE_TYPE:
      if (((emc_traj_set_teleop_enable_msg_t *) cmd)->enable)
         emcmotCommand.command = EMCMOT_TELEOP;
      else
         emcmotCommand.command = EMCMOT_FREE;
      retval = _emc_sio_write_command(&emcmotCommand);
      break;
   case EMC_TRAJ_SET_TELEOP_VECTOR_TYPE:
      emcmotCommand.command = EMCMOT_SET_TELEOP_VECTOR;
      emcmotCommand.pos = ((emc_traj_set_teleop_vector_msg_t *) cmd)->vector;
      retval = _emc_sio_write_command(&emcmotCommand);
      break;

      // IO commands

   case EMC_SPINDLE_SPEED_TYPE:
      return EMC_R_OK;
      break;
   case EMC_SPINDLE_ON_TYPE:
//      emcmotCommand.command = EMCMOT_SPINDLE_ON;
//      emcmotCommand.vel = ((emc_spindle_on_msg_t *) cmd)->speed;
//      emcmotCommand.ini_maxvel = ((emc_spindle_on_msg_t *) cmd)->factor;
//      emcmotCommand.acc = ((emc_spindle_on_msg_t *) cmd)->xoffset;
//      retval = _emc_sio_write_command(&emcmotCommand);
      speed = ((emc_spindle_on_msg_t *)cmd)->speed;
      if (speed >= 0)
         return emcTaskExecScript(3, speed, 0.0, 0);         /* execute m3.py script */
      else
         return emcTaskExecScript(4, speed, 0.0, 0);         /* execute m4.py script */
      break;
   case EMC_SPINDLE_OFF_TYPE:
//      emcmotCommand.command = EMCMOT_SPINDLE_OFF;
//      retval = _emc_sio_write_command(&emcmotCommand);
      return emcTaskExecScript(5, 0.0, 0.0, 0);         /* execute m5.py script */
      break;
   case EMC_SYSTEM_CMD_TYPE:
//      emcmotCommand.command = EMCMOT_SYSTEM_CMD;
//      emcmotCommand.index = ((emc_system_cmd_msg_t *)cmd)->index;
//      emcmotCommand.p_number = ((emc_system_cmd_msg_t *)cmd)->p_number;
//      emcmotCommand.q_number = ((emc_system_cmd_msg_t *)cmd)->q_number;
//      retval = _emc_sio_write_command(&emcmotCommand);
      return emcTaskExecScript(((emc_system_cmd_msg_t *)cmd)->index, ((emc_system_cmd_msg_t *)cmd)->p_number, ((emc_system_cmd_msg_t *)cmd)->q_number, 0);
      break;
   case EMC_MOTION_SET_DOUT_TYPE:
      emcmotCommand.command = EMCMOT_SET_DOUT;
      emcmotCommand.output_num = ((emc_motion_set_dout_msg_t *)cmd)->output_num;
      emcmotCommand.value = ((emc_motion_set_dout_msg_t *) cmd)->value;
      emcmotCommand.sync = ((emc_motion_set_dout_msg_t *) cmd)->sync;
      retval = _emc_sio_write_command(&emcmotCommand);
      break;
   case EMC_MOTION_ENABLE_DIN_ABORT_TYPE:
      emcmotCommand.command = EMCMOT_ENABLE_DIN_ABORT;
      emcmotCommand.input_num = ((emc_motion_din_msg_t *)cmd)->input_num;
      retval = _emc_sio_write_command(&emcmotCommand);
      break;
   case EMC_MOTION_DISABLE_DIN_ABORT_TYPE:
      emcmotCommand.command = EMCMOT_DISABLE_DIN_ABORT;
      emcmotCommand.input_num = ((emc_motion_din_msg_t *)cmd)->input_num;
      retval = _emc_sio_write_command(&emcmotCommand);
      break;
   case EMC_SPINDLE_BRAKE_RELEASE_TYPE:
      emcmotCommand.command = EMCMOT_SPINDLE_BRAKE_RELEASE;
      retval = _emc_sio_write_command(&emcmotCommand);
      break;
   case EMC_SPINDLE_INCREASE_TYPE:
      emcmotCommand.command = EMCMOT_SPINDLE_INCREASE;
      retval = _emc_sio_write_command(&emcmotCommand);
      break;
   case EMC_SPINDLE_DECREASE_TYPE:
      emcmotCommand.command = EMCMOT_SPINDLE_DECREASE;
      retval = _emc_sio_write_command(&emcmotCommand);
      break;
   case EMC_SPINDLE_CONSTANT_TYPE:
      retval = EMC_R_OK;
      break;
   case EMC_SPINDLE_BRAKE_ENGAGE_TYPE:
      emcmotCommand.command = EMCMOT_SPINDLE_BRAKE_ENGAGE;
      retval = _emc_sio_write_command(&emcmotCommand);
      break;
   case EMC_COOLANT_MIST_ON_TYPE:
//      emcioCommand.type = EMCIO_COOLANT_MIST_ON_COMMAND;
//      retval = sendCommand(&emcioCommand);
      retval = emcTaskExecScript(7, 0.0, 0.0, 0);   /* execute m7.py script */
      break;
   case EMC_COOLANT_MIST_OFF_TYPE:
//      emcioCommand.type = EMCIO_COOLANT_MIST_OFF_COMMAND;
//      retval = sendCommand(&emcioCommand);
      retval = emcTaskExecScript(9, 1.0, 0.0, 0);   /* execute m9.py script */
      break;
   case EMC_COOLANT_FLOOD_ON_TYPE:
//      emcioCommand.type = EMCIO_COOLANT_FLOOD_ON_COMMAND;
//      retval = sendCommand(&emcioCommand);
      retval = emcTaskExecScript(8, 0.0, 0.0, 0);   /* execute m8.py script */
      break;
   case EMC_COOLANT_FLOOD_OFF_TYPE:
//      emcioCommand.type = EMCIO_COOLANT_FLOOD_OFF_COMMAND;
//      retval = sendCommand(&emcioCommand);
      retval = emcTaskExecScript(9, 2.0, 0.0, 0);   /* execute m9.py script */
      break;
   case EMC_LUBE_ON_TYPE:
      emcioCommand.type = EMCIO_LUBE_ON_COMMAND;
      retval = sendCommand(&emcioCommand);
      break;
   case EMC_LUBE_OFF_TYPE:
      emcioCommand.type = EMCIO_LUBE_OFF_COMMAND;
      retval = sendCommand(&emcioCommand);
      break;
   case EMC_TOOL_PREPARE_TYPE:
      emcioCommand.type = EMCIO_TOOL_PREPARE_COMMAND;
      retval = sendCommand(&emcioCommand);
      break;
   case EMC_TOOL_LOAD_TYPE:
      emcioCommand.type = EMCIO_TOOL_LOAD_COMMAND;
      retval = sendCommand(&emcioCommand);
      break;
   case EMC_TOOL_LOAD_TOOL_TABLE_TYPE:
      emcioCommand.type = EMCIO_TOOL_LOAD_TOOL_TABLE_COMMAND;
      strcpy(emcioCommand.file, ((emc_tool_load_tool_table_msg_t *) cmd)->file);
      retval = sendCommand(&emcioCommand);
      break;
   case EMC_TOOL_SET_OFFSET_TYPE:
      emcioCommand.type = EMCIO_TOOL_SET_OFFSET_COMMAND;
      emcioCommand.pocket = ((emc_tool_set_offset_msg_t *) cmd)->pocket;
      emcioCommand.toolno = ((emc_tool_set_offset_msg_t *) cmd)->toolno;
      emcioCommand.offset = ((emc_tool_set_offset_msg_t *) cmd)->offset;
      emcioCommand.diameter = ((emc_tool_set_offset_msg_t *) cmd)->diameter;
      emcioCommand.frontangle = ((emc_tool_set_offset_msg_t *) cmd)->frontangle;
      emcioCommand.backangle = ((emc_tool_set_offset_msg_t *) cmd)->backangle;
      emcioCommand.orientation = ((emc_tool_set_offset_msg_t *) cmd)->orientation;
      retval = sendCommand(&emcioCommand);
      break;

      // task commands

   case EMC_TASK_ABORT_TYPE:
      emcTaskAbort();
      emcIoAbort();
//        emcSpindleAbort();
      retval = EMC_R_OK;
      break;
   case EMC_TASK_SET_MODE_TYPE:
      mode = ((emc_task_set_mode_msg_t *) cmd)->mode;

      if (emcStatus->task.mode == EMC_TASK_MODE_AUTO && emcStatus->task.interpState != EMC_TASK_INTERP_IDLE && 
            mode != EMC_TASK_MODE_AUTO && emcStatus->task.execState != EMC_TASK_EXEC_WAITING_FOR_SYSTEM_CMD)
      {
         BUG("Can't switch mode while mode is AUTO and interpreter is not IDLE\n");
         emcOperatorMessage(0, EMC_I18N("Can't switch mode while mode is AUTO and interpreter is not IDLE"));
      }
      else
      { // we can honor the modeswitch
         if (mode == EMC_TASK_MODE_MANUAL && emcStatus->task.mode != EMC_TASK_MODE_MANUAL)
         {
            // leaving auto or mdi mode for manual, abort motion
//            emcTaskAbort();
         }
         retval = emcTaskSetMode(mode);
      }
      break;
   case EMC_TASK_SET_STATE_TYPE:
      retval = emcTaskSetState(((emc_task_set_state_msg_t *) cmd)->state);
      break;

      // interpreter commands

   case EMC_TASK_PLAN_OPEN_TYPE:
      file = ((emc_task_plan_open_msg_t *) cmd)->file;
      retval = emcTaskPlanOpen(file);
      if (retval > INTERP_MIN_ERROR)
      {
         retval = EMC_R_ERROR;
         BUG("can't open %s\n", file);
         emcOperatorMessage(0, EMC_I18N("can't open %s"), file);
      }
      else
      {
         strcpy(emcStatus->task.file, file);
         retval = EMC_R_OK;
      }
      break;
   case EMC_TASK_PLAN_EXECUTE_TYPE:
      stepping = 0;
      steppingWait = 0;
      command = ((emc_task_plan_execute_msg_t *) cmd)->command;  /* mdi command string */
      if (!all_homed() && !no_force_homing)
      {
         BUG("Can't issue MDI command when not homed\n");
         emcOperatorMessage(0, EMC_I18N("Can't issue MDI command when not homed"));
         break;
      }
      if (emcStatus->task.mode != EMC_TASK_MODE_MDI)
      {
         BUG("Must be in MDI mode to issue MDI command\n");
         emcOperatorMessage(0, EMC_I18N("Must be in MDI mode to issue MDI command"));
         break;
      }
      if (command[0] != 0)
      {
         if (emcStatus->task.mode == EMC_TASK_MODE_MDI)
         {
            interp_list.set_line_number(++pseudoMdiLineNumber);
         }
         if ((execRetval = emcTaskPlanExecuteEx(command, pseudoMdiLineNumber)) == INTERP_ENDFILE)
         {
            /* End-of-file, flush execution, signify no more reading until all is done. */
            task_plan_wait = 1;
            /* Resynch the interpreter. */
            interp_list.append((emc_command_msg_t *) & taskPlanSynchCmd);
            retval = EMC_R_OK;
         }
         else if (execRetval <= INTERP_MIN_ERROR)
         {
            retval = EMC_R_OK;
         }
      }
      break;
   case EMC_TASK_PLAN_RUN_TYPE:
      if (!all_homed() && !no_force_homing)
      {
         BUG("Can't run a program when not homed\n");
         emcOperatorMessage(0, EMC_I18N("Can't run a program when not homed"));
         break;
      }
      if (emcStatus->task.file[0] == 0)
      {
         BUG("Can't run, must open program first\n");
         emcOperatorMessage(0, EMC_I18N("Can't run, must open program first"));
         break;
      }
      stepping = 0;
      steppingWait = 0;
      if (!taskplanopen && emcStatus->task.file[0] != 0)
      {
         emcTaskPlanOpen(emcStatus->task.file);
      }
      programStartLine = ((emc_task_plan_run_msg_t *) cmd)->line;
      emcStatus->task.interpState = EMC_TASK_INTERP_READING;
      emcStatus->task.task_paused = 0;
      retval = EMC_R_OK;
      break;
   case EMC_TASK_PLAN_PAUSE_TYPE:
      if (emcStatus->task.file[0] == 0)
      {
         BUG("Can't pause, must open program first\n");
         emcOperatorMessage(0, EMC_I18N("Can't pause, must open program first"));
         break;
      }
      emcmotCommand.command = EMCMOT_PAUSE;
      retval = _emc_sio_write_command(&emcmotCommand);
      if (emcStatus->task.interpState != EMC_TASK_INTERP_PAUSED)
      {
         interpResumeState = emcStatus->task.interpState;
      }
      emcStatus->task.interpState = EMC_TASK_INTERP_PAUSED;
      emcStatus->task.task_paused = 1;
      break;
   case EMC_TASK_PLAN_RESUME_TYPE:
      if (emcStatus->task.file[0] == 0)
      {
         BUG("Can't resume , must open program first\n");
         emcOperatorMessage(0, EMC_I18N("Can't resume, must open program first"));
         break;
      }
      emcmotCommand.command = EMCMOT_RESUME;
      retval = _emc_sio_write_command(&emcmotCommand);
      emcStatus->task.interpState = (enum EMC_TASK_INTERP) interpResumeState;
      emcStatus->task.task_paused = 0;
      stepping = 0;
      steppingWait = 0;
      break;
   case EMC_TASK_PLAN_INIT_TYPE:
      if (emcTaskPlanInit() > INTERP_MIN_ERROR)
         break;
      retval = EMC_R_OK;
      break;
   case EMC_TASK_PLAN_END_TYPE:
      retval = EMC_R_OK;
      break;
   case EMC_TASK_PLAN_SYNCH_TYPE:
      interp.synch();
      retval = EMC_R_OK;
      break;
   case EMC_TASK_PLAN_SET_OPTIONAL_STOP_TYPE:
      SET_OPTIONAL_PROGRAM_STOP(((emc_task_plan_set_optional_stop_msg_t *) cmd)->state);
      retval = EMC_R_OK;
      break;
   default:
      // unrecognized command
      BUG("unknown command type=%d cmd=%s\n", cmd->msg.type, lookup_message(cmd->msg.type));
      retval = EMC_R_OK;        // don't consider this an error
      break;
   }

   if (retval != EMC_R_OK)
   {
      BUG("error executing command type=%d cmd=%s\n", cmd->msg.type, lookup_message(cmd->msg.type));
   }

   return retval;
}       /* emcTaskIssueCommand() */

/* Used by emcTaskExecute(). Check preconditions for commands on the interp_list (non-immediate commands). */
static int emcTaskCheckPreconditions(emc_command_msg_t * cmd)
{
   if (cmd == NULL)
   {
      return EMC_TASK_EXEC_DONE;
   }

   switch (cmd->msg.type)
   {
      // operator messages, if queued, will go out when everything before them is done
   case EMC_OPERATOR_MESSAGE_TYPE:
   case EMC_SYSTEM_CMD_TYPE:
   case EMC_TRAJ_PROBE_TYPE:   // prevent blending of this
   case EMC_TRAJ_RIGID_TAP_TYPE:       //and this
   case EMC_TRAJ_CLEAR_PROBE_TRIPPED_FLAG_TYPE:        // and this
   case EMC_AUX_INPUT_WAIT_TYPE:
      return EMC_TASK_EXEC_WAITING_FOR_MOTION_AND_IO;
      break;

   case EMC_TRAJ_LINEAR_MOVE_TYPE:
   case EMC_TRAJ_CIRCULAR_MOVE_TYPE:
   case EMC_TRAJ_SET_VELOCITY_TYPE:
   case EMC_TRAJ_SET_ACCELERATION_TYPE:
   case EMC_TRAJ_SET_TERM_COND_TYPE:
   case EMC_TRAJ_SET_SPINDLESYNC_TYPE:
   case EMC_TRAJ_SET_FO_ENABLE_TYPE:
   case EMC_TRAJ_SET_FH_ENABLE_TYPE:
   case EMC_TRAJ_SET_SO_ENABLE_TYPE:
      return EMC_TASK_EXEC_WAITING_FOR_IO;
      break;

   case EMC_TRAJ_SET_OFFSET_TYPE:
      // this applies the tool length offset variable after previous motions
   case EMC_TRAJ_SET_ORIGIN_TYPE:
   case EMC_TRAJ_SET_ROTATION_TYPE:
      // this applies the program origin after previous motions
      return EMC_TASK_EXEC_WAITING_FOR_MOTION;
      break;

   case EMC_TOOL_LOAD_TYPE:
   case EMC_TOOL_UNLOAD_TYPE:
   case EMC_COOLANT_MIST_ON_TYPE:
   case EMC_COOLANT_MIST_OFF_TYPE:
   case EMC_COOLANT_FLOOD_ON_TYPE:
   case EMC_COOLANT_FLOOD_OFF_TYPE:
   case EMC_SPINDLE_SPEED_TYPE:
   case EMC_SPINDLE_ON_TYPE:
   case EMC_SPINDLE_OFF_TYPE:
      return EMC_TASK_EXEC_WAITING_FOR_MOTION_AND_IO;
      break;

   case EMC_TOOL_PREPARE_TYPE:
   case EMC_LUBE_ON_TYPE:
   case EMC_LUBE_OFF_TYPE:
      return EMC_TASK_EXEC_WAITING_FOR_IO;
      break;

   case EMC_TOOL_LOAD_TOOL_TABLE_TYPE:
   case EMC_TOOL_SET_OFFSET_TYPE:
      return EMC_TASK_EXEC_WAITING_FOR_MOTION_AND_IO;
      break;

   case EMC_TOOL_SET_NUMBER_TYPE:
      return EMC_TASK_EXEC_WAITING_FOR_IO;
      break;

   case EMC_TASK_PLAN_PAUSE_TYPE:
   case EMC_TASK_PLAN_OPTIONAL_STOP_TYPE:
      /* pause on the interp list is queued, so wait until all are done */
      return EMC_TASK_EXEC_WAITING_FOR_MOTION_AND_IO;
      break;
   case EMC_TASK_PLAN_END_TYPE:
      return EMC_TASK_EXEC_WAITING_FOR_MOTION_AND_IO;
      break;

   case EMC_TASK_PLAN_INIT_TYPE:
   case EMC_TASK_PLAN_RUN_TYPE:
   case EMC_TASK_PLAN_SYNCH_TYPE:
   case EMC_TASK_PLAN_EXECUTE_TYPE:
      return EMC_TASK_EXEC_WAITING_FOR_MOTION_AND_IO;
      break;

   case EMC_TRAJ_DELAY_TYPE:
      return EMC_TASK_EXEC_WAITING_FOR_MOTION_AND_IO;
      break;

   case EMC_MOTION_ADAPTIVE_TYPE:
      return EMC_TASK_EXEC_WAITING_FOR_MOTION;
      break;

   default:
      // unrecognized command
      BUG("preconditions: unrecognized command type=%d cmd=%s\n", cmd->msg.type, lookup_message(cmd->msg.type));
      return EMC_TASK_EXEC_ERROR;
      break;
   }

   return EMC_TASK_EXEC_DONE;
}       /* emcTaskCheckPreconditions() */

/* Used by emcTaskExecute(). Check postconditions for commands on the interp_list (non-immediate commands). */
static int emcTaskCheckPostconditions(emc_command_msg_t * cmd)
{
   if (cmd == NULL)
   {
      return EMC_TASK_EXEC_DONE;
   }

   switch (cmd->msg.type)
   {
   case EMC_OPERATOR_MESSAGE_TYPE:
      return EMC_TASK_EXEC_DONE;
      break;

   case EMC_SYSTEM_CMD_TYPE:
   case EMC_SPINDLE_ON_TYPE:
   case EMC_SPINDLE_OFF_TYPE:
   case EMC_COOLANT_MIST_ON_TYPE:
   case EMC_COOLANT_MIST_OFF_TYPE:
   case EMC_COOLANT_FLOOD_ON_TYPE:
   case EMC_COOLANT_FLOOD_OFF_TYPE:
      return EMC_TASK_EXEC_WAITING_FOR_SYSTEM_CMD; /* tinypy mcode script */
      break;

   case EMC_TRAJ_LINEAR_MOVE_TYPE:
   case EMC_TRAJ_CIRCULAR_MOVE_TYPE:
      return EMC_TASK_EXEC_WAITING_FOR_MOTION;  /* rtstepper dongle change. */
      break;

   case EMC_TRAJ_SET_VELOCITY_TYPE:
   case EMC_TRAJ_SET_ACCELERATION_TYPE:
   case EMC_TRAJ_SET_TERM_COND_TYPE:
   case EMC_TRAJ_SET_SPINDLESYNC_TYPE:
   case EMC_TRAJ_SET_OFFSET_TYPE:
   case EMC_TRAJ_SET_ORIGIN_TYPE:
   case EMC_TRAJ_SET_ROTATION_TYPE:
   case EMC_TRAJ_PROBE_TYPE:
   case EMC_TRAJ_RIGID_TAP_TYPE:
   case EMC_TRAJ_CLEAR_PROBE_TRIPPED_FLAG_TYPE:
   case EMC_TRAJ_SET_TELEOP_ENABLE_TYPE:
   case EMC_TRAJ_SET_TELEOP_VECTOR_TYPE:
   case EMC_TRAJ_SET_FO_ENABLE_TYPE:
   case EMC_TRAJ_SET_FH_ENABLE_TYPE:
   case EMC_TRAJ_SET_SO_ENABLE_TYPE:
      return EMC_TASK_EXEC_DONE;
      break;

   case EMC_TOOL_PREPARE_TYPE:
   case EMC_TOOL_LOAD_TYPE:
   case EMC_TOOL_UNLOAD_TYPE:
   case EMC_TOOL_LOAD_TOOL_TABLE_TYPE:
   case EMC_TOOL_SET_OFFSET_TYPE:
   case EMC_TOOL_SET_NUMBER_TYPE:
   case EMC_SPINDLE_SPEED_TYPE:
   case EMC_LUBE_ON_TYPE:
   case EMC_LUBE_OFF_TYPE:
      return EMC_TASK_EXEC_DONE;
      break;

   case EMC_TASK_PLAN_RUN_TYPE:
   case EMC_TASK_PLAN_PAUSE_TYPE:
   case EMC_TASK_PLAN_END_TYPE:
   case EMC_TASK_PLAN_INIT_TYPE:
   case EMC_TASK_PLAN_SYNCH_TYPE:
   case EMC_TASK_PLAN_EXECUTE_TYPE:
   case EMC_TASK_PLAN_OPTIONAL_STOP_TYPE:
      return EMC_TASK_EXEC_DONE;
      break;

   case EMC_TRAJ_DELAY_TYPE:
   case EMC_AUX_INPUT_WAIT_TYPE:
      return EMC_TASK_EXEC_WAITING_FOR_DELAY;
      break;

   case EMC_MOTION_SET_AOUT_TYPE:
   case EMC_MOTION_SET_DOUT_TYPE:
   case EMC_MOTION_ADAPTIVE_TYPE:
      return EMC_TASK_EXEC_DONE;
      break;

   default:
      // unrecognized command
      BUG("postconditions: unrecognized command type=%d cmd=%s\n", cmd->msg.type, lookup_message(cmd->msg.type));
      return EMC_TASK_EXEC_DONE;
      break;
   }
   return EMC_TASK_EXEC_DONE;   // unreached
}       /* emcTaskCheckPostconditions() */

/*********************************************************************************
 * emcTaskExecute() - executes commands from interp_list.
 */
void emcTaskExecute(void)
{
   struct emc_session *ps = &session;

   switch (emcStatus->task.execState)
   {
#if 0
   case EMC_TASK_EXEC_ERROR:
      // abort everything
      emcTaskAbort();
      emcIoAbort();
      emcSpindleOff();
      retval = EMC_R_ERROR;
      break;
#endif
   case EMC_TASK_EXEC_DONE:
      if (is_stepping_ok())
      {
         if (!emcStatus->motion.traj.queueFull && emcStatus->task.interpState != EMC_TASK_INTERP_PAUSED)
         {
            if (emcTaskCommand == NULL)
            {
               // need a new command
               emcTaskCommand = interp_list.get();
               if (emcTaskCommand != NULL)
               {
                  // interp_list now has line number associated with this-- get it 
                  emcStatus->task.currentLine = interp_list.get_line_number();
                  if (emcStatus->motion.traj.queueFull)
                  {
                     emcStatus->task.execState = EMC_TASK_EXEC_WAITING_FOR_MOTION_QUEUE;
                  }
                  else
                  {
                     emcStatus->task.execState = (enum EMC_TASK_EXEC) emcTaskCheckPreconditions(emcTaskCommand);
                  }
               }
            }
            else
            {
               // have an outstanding command
               if (emcTaskIssueCommand(emcTaskCommand) != EMC_R_OK)
               {
                  emcStatus->task.execState = EMC_TASK_EXEC_ERROR;
               }
               else
               {
                  emcStatus->task.execState = (enum EMC_TASK_EXEC) emcTaskCheckPostconditions(emcTaskCommand);
               }
               emcTaskCommand = NULL;   // reset it
            }
         }
      }
      break;
   case EMC_TASK_EXEC_WAITING_FOR_MOTION_QUEUE:
      if (is_stepping_ok())
      {
         if (!emcStatus->motion.traj.queueFull)
         {
            if (emcTaskCommand != NULL)
            {
               emcStatus->task.execState = (enum EMC_TASK_EXEC) emcTaskCheckPreconditions(emcTaskCommand);
            }
            else
            {
               emcStatus->task.execState = EMC_TASK_EXEC_DONE;
            }
         }
      }
      break;
   case EMC_TASK_EXEC_WAITING_FOR_PAUSE:
      if (is_stepping_ok())
      {
         if (emcStatus->task.interpState != EMC_TASK_INTERP_PAUSED)
         {
            if (emcTaskCommand != NULL)
            {
               if (emcStatus->motion.traj.queue > 0)
               {
                  emcStatus->task.execState = EMC_TASK_EXEC_WAITING_FOR_MOTION_QUEUE;
               }
               else
               {
                  emcStatus->task.execState = (enum EMC_TASK_EXEC) emcTaskCheckPreconditions(emcTaskCommand);
               }
            }
            else
            {
               emcStatus->task.execState = EMC_TASK_EXEC_DONE;
            }
         }
      }
      break;
   case EMC_TASK_EXEC_WAITING_FOR_MOTION:
      if (is_stepping_ok())
      {
         if (emcStatus->motion.status == RCS_ERROR)
         {
            // emcOperatorMessage(0, "error in motion controller");
            emcStatus->task.execState = EMC_TASK_EXEC_ERROR;
         }
         else if (emcStatus->motion.status == RCS_DONE)
         {
            emcStatus->task.execState = EMC_TASK_EXEC_DONE;
         }
      }
      break;
   case EMC_TASK_EXEC_WAITING_FOR_IO:
      if (is_stepping_ok())
      {
         if (emcStatus->io.status == RCS_ERROR)
         {
            // emcOperatorMessage(0, "error in IO controller");
            emcStatus->task.execState = EMC_TASK_EXEC_ERROR;
         }
         else if (emcStatus->io.status == RCS_DONE)
         {
            emcStatus->task.execState = EMC_TASK_EXEC_DONE;
         }
      }
      break;
   case EMC_TASK_EXEC_WAITING_FOR_MOTION_AND_IO:
      if (is_stepping_ok())
      {
         if (emcStatus->motion.status == RCS_ERROR)
         {
            // emcOperatorMessage(0, "error in motion controller");
            emcStatus->task.execState = EMC_TASK_EXEC_ERROR;
         }
         else if (emcStatus->io.status == RCS_ERROR)
         {
            // emcOperatorMessage(0, "error in IO controller");
            emcStatus->task.execState = EMC_TASK_EXEC_ERROR;
         }
         else if (emcStatus->motion.status == RCS_DONE && emcStatus->io.status == RCS_DONE)
         {
            emcStatus->task.execState = EMC_TASK_EXEC_DONE;
         }
      }
      break;
   case EMC_TASK_EXEC_WAITING_FOR_DELAY:
      if (is_stepping_ok())
      {
      // check if delay has passed
      emcStatus->task.delayLeft = taskExecDelayTimeout - etime();
      if (etime() >= taskExecDelayTimeout)
      {
         emcStatus->task.execState = EMC_TASK_EXEC_DONE;
         emcStatus->task.delayLeft = 0;
         if (emcStatus->task.input_timeout != 0)
            emcStatus->task.input_timeout = 1;  // timeout occured
      }
      }
      break;
    case EMC_TASK_EXEC_WAITING_FOR_SYSTEM_CMD:
      if (is_stepping_ok())
      {
         if (ps->mcode_thread_active == 0)
         {
            emcStatus->task.execState = EMC_TASK_EXEC_DONE;
         }
      }
      break;
   default:
      // coding error
      BUG("invalid emcTaskExecute() state=%d\n", emcStatus->task.execState);
      emcStatus->task.execState = EMC_TASK_EXEC_ERROR;
      break;
   }
}       /* emcTaskExecute() */

int emcTaskPlanOpen(const char *file)
{
   emcStatus->task.motionLine = 0;
   emcStatus->task.currentLine = 0;
   emcStatus->task.readLine = 0;

   int retval = interp.open(file);
   if (retval > INTERP_MIN_ERROR)
   {
      print_interp_error(retval);
      return retval;
   }
   taskplanopen = 1;

   DBG("emcTaskPlanOpen() file=%s ret=%d\n", file, retval);
   return retval;
}       /* emcTaskPlanOpen() */

int emcTaskPlanClose()
{
   int retval = interp.close();
   if (retval > INTERP_MIN_ERROR)
   {
      print_interp_error(retval);
   }

   taskplanopen = 0;
   return retval;
}       /* emcTaskPlanClose() */

int emcTaskPlanInit()
{
   int retval, stat = EMC_R_OK;

   DBG("emcTaskPlanInit()\n");
   interp.ini_load(EMC_INIFILE);
   task_plan_wait = 0;

   retval = interp.init();   /* clears interp.file() */
   if (retval > INTERP_MIN_ERROR)
   {
      print_interp_error(retval);
      stat = EMC_R_ERROR;
   }
   else
   {
      if (RS274NGC_STARTUP_CODE[0] != 0)
      {
         retval = interp.execute(RS274NGC_STARTUP_CODE);
         if (retval > INTERP_MIN_ERROR)
         {
            print_interp_error(retval);
            stat = EMC_R_ERROR;
         }
      }
   }

   DBG("emcTaskPlanInit() returned %d\n", stat);

   return stat;
}       /* emcTaskPlanInit() */

int emcTaskPlanExit()
{
   DBG("emcTaskPlanExit()\n");
   return interp.exit();
}

int emcTaskPlanCommand(char *cmd)
{
   char buf[LINELEN];
   strcpy(cmd, interp.command(buf, sizeof(buf)));
   DBG("emcTaskPlanCommand() line=%d: %s\n", emcStatus->task.readLine, cmd);
   return 0;
}       /* emcTaskPlanCommand() */

int emcTaskPlanRead()
{
   int retval = interp.read();

   if (retval == INTERP_FILE_NOT_OPEN)
   {
      if (emcStatus->task.file[0] != 0)
      {
         retval = interp.open(emcStatus->task.file);
         if (retval > INTERP_MIN_ERROR)
         {
            print_interp_error(retval);
         }
         retval = interp.read();
      }
   }
   if (retval > INTERP_MIN_ERROR)
   {
      print_interp_error(retval);
   }

   return retval;
}       /* emcTaskPlanRead() */

static int emcTaskPlanExecute(const char *command)
{
   int inpos = emcStatus->motion.traj.inpos;    // 1 if in position, 0 if not.

   if (command != NULL)
   {    // Command is 0 if in AUTO mode, non-null if in MDI mode.
      // Don't sync if not in position.
      if ((*command != 0) && (inpos))
      {
         interp.synch();
      }
   }
   int retval = interp.execute(command);
   if (retval > INTERP_MIN_ERROR)
      print_interp_error(retval);
   if (command != NULL)
      FINISH();

   DBG("emcTaskPlanExecute() execState=%s interp.len=%d interpState=%s ret=%d\n", lookup_task_exec_state(emcStatus->task.execState), 
        interp_list.len(), lookup_task_interp_state(emcStatus->task.interpState), retval);
   return retval;
}       /* emcTaskPlanExecute() */

static int emcTaskPlanExecuteEx(const char *command, int line_number)
{
   int retval = interp.execute(command, line_number);
   if (retval > INTERP_MIN_ERROR)
      print_interp_error(retval);
   if (command != NULL) // this means MDI
      FINISH();

   DBG("emcTaskPlanExecuteEx() execState=%s interp.len=%d interpState=%s ret=%d\n", lookup_task_exec_state(emcStatus->task.execState), 
        interp_list.len(), lookup_task_interp_state(emcStatus->task.interpState), retval);
   return retval;
}       /* emcTaskPlanExecute() */

/**********************************************************************************
 * emcTaskPlan() - process commands from gui and gcode file.
 */
void emcTaskPlan(emc_command_msg_t *emcCommand)
{
   struct emc_session *ps = &session;
   enum EMC_COMMAND_MSG_TYPE type = EMC_COMMAND_UNUSED;
   int retval = EMC_R_OK;

   if (emcCommand)
   {
       type = emcCommand->msg.type;  /* new command */
       emcStatus->task.planState = EMC_TASK_PLAN_DONE; /* reset state */
   }

   switch (emcStatus->task.state)
   {
   case EMC_TASK_STATE_OFF:
   case EMC_TASK_STATE_ESTOP:
   case EMC_TASK_STATE_ESTOP_RESET:

      // now switch on the mode
      switch (emcStatus->task.mode)
      {
      case EMC_TASK_MODE_MANUAL:
      case EMC_TASK_MODE_AUTO:
      case EMC_TASK_MODE_MDI:

         // now switch on the command
         switch (type)
         {
         case EMC_COMMAND_UNUSED:
            // no command
            break;
         case EMC_QUIT_TYPE:
            ps->command_thread_abort = 1;
            break;

            // immediate commands
         case EMC_AXIS_SET_BACKLASH_TYPE:
         case EMC_AXIS_SET_HOMING_PARAMS_TYPE:
         case EMC_AXIS_DISABLE_TYPE:
         case EMC_AXIS_ENABLE_TYPE:
         case EMC_AXIS_SET_FERROR_TYPE:
         case EMC_AXIS_SET_MIN_FERROR_TYPE:
         case EMC_AXIS_ABORT_TYPE:
         case EMC_AXIS_LOAD_COMP_TYPE:
         case EMC_AXIS_UNHOME_TYPE:
         case EMC_TRAJ_SET_SCALE_TYPE:
         case EMC_TRAJ_SET_MAX_VELOCITY_TYPE:
         case EMC_TRAJ_SET_SPINDLE_SCALE_TYPE:
         case EMC_TRAJ_SET_FO_ENABLE_TYPE:
         case EMC_TRAJ_SET_FH_ENABLE_TYPE:
         case EMC_TRAJ_SET_SO_ENABLE_TYPE:
         case EMC_TRAJ_SET_VELOCITY_TYPE:
         case EMC_TRAJ_SET_ACCELERATION_TYPE:
         case EMC_TASK_INIT_TYPE:
         case EMC_TASK_SET_MODE_TYPE:
         case EMC_TASK_SET_STATE_TYPE:
         case EMC_TASK_PLAN_INIT_TYPE:
         case EMC_TASK_PLAN_OPEN_TYPE:
         case EMC_TASK_PLAN_SET_OPTIONAL_STOP_TYPE:
         case EMC_TASK_PLAN_SET_BLOCK_DELETE_TYPE:
         case EMC_TASK_ABORT_TYPE:
         case EMC_TRAJ_CLEAR_PROBE_TRIPPED_FLAG_TYPE:
         case EMC_TRAJ_PROBE_TYPE:
         case EMC_AUX_INPUT_WAIT_TYPE:
         case EMC_MOTION_SET_DOUT_TYPE:
         case EMC_MOTION_ENABLE_DIN_ABORT_TYPE:
         case EMC_MOTION_DISABLE_DIN_ABORT_TYPE:
         case EMC_MOTION_ADAPTIVE_TYPE:
         case EMC_MOTION_SET_AOUT_TYPE:
         case EMC_TRAJ_RIGID_TAP_TYPE:
         case EMC_TRAJ_SET_TELEOP_ENABLE_TYPE:
            retval = emcTaskIssueCommand(emcCommand);
            break;

            // one case where we need to be in manual mode
         case EMC_AXIS_OVERRIDE_LIMITS_TYPE:
            if (emcStatus->task.mode == EMC_TASK_MODE_MANUAL)
               retval = emcTaskIssueCommand(emcCommand);
            break;

         case EMC_TOOL_LOAD_TOOL_TABLE_TYPE:
         case EMC_TOOL_SET_OFFSET_TYPE:
            // send to IO
            interp_list.append((emc_command_msg_t *) emcCommand);
            // signify no more reading
            task_plan_wait = 1;
            // then resynch interpreter
            interp_list.append((emc_command_msg_t *) & taskPlanSynchCmd);
            break;
         case EMC_TOOL_SET_NUMBER_TYPE:
            // send to IO
            interp_list.append((emc_command_msg_t *) emcCommand);
            // then resynch interpreter
            interp_list.append((emc_command_msg_t *) & taskPlanSynchCmd);
            break;
         default:
            BUG("command (%s) cannot be executed until the machine is out of E-stop and turned on\n", lookup_message(type));
            emcOperatorMessage(0, EMC_I18N("command (%s) cannot be executed until the machine is out of E-stop and turned on"), lookup_message(type));
            retval = EMC_R_ERROR;
            break;
         }      // switch (type)

      default:
         // invalid mode
         break;

      } // switch (mode)

      break;    // case EMC_TASK_STATE_OFF,ESTOP,ESTOP_RESET

   case EMC_TASK_STATE_ON:
      /* we can do everything (almost) when the machine is on, so let's
         switch on the execution mode */
      switch (emcStatus->task.mode)
      {
      case EMC_TASK_MODE_MANUAL:       // ON, MANUAL
         switch (type)
         {
         case EMC_COMMAND_UNUSED:
            // no command
            break;
         case EMC_QUIT_TYPE:
            ps->command_thread_abort = 1;
            break;

            // immediate commands

         case EMC_AXIS_DISABLE_TYPE:
         case EMC_AXIS_ENABLE_TYPE:
         case EMC_AXIS_SET_BACKLASH_TYPE:
         case EMC_AXIS_SET_HOMING_PARAMS_TYPE:
         case EMC_AXIS_SET_FERROR_TYPE:
         case EMC_AXIS_SET_MIN_FERROR_TYPE:
         case EMC_AXIS_SET_MAX_POSITION_LIMIT_TYPE:
         case EMC_AXIS_SET_MIN_POSITION_LIMIT_TYPE:
         case EMC_AXIS_ABORT_TYPE:
         case EMC_AXIS_HALT_TYPE:
         case EMC_AXIS_HOME_TYPE:
         case EMC_AXIS_UNHOME_TYPE:
         case EMC_AXIS_JOG_TYPE:
         case EMC_AXIS_INCR_JOG_TYPE:
         case EMC_AXIS_ABS_JOG_TYPE:
         case EMC_AXIS_OVERRIDE_LIMITS_TYPE:
         case EMC_TRAJ_PAUSE_TYPE:
         case EMC_TRAJ_RESUME_TYPE:
         case EMC_TRAJ_ABORT_TYPE:
         case EMC_TRAJ_SET_SCALE_TYPE:
         case EMC_TRAJ_SET_MAX_VELOCITY_TYPE:
         case EMC_TRAJ_SET_SPINDLE_SCALE_TYPE:
         case EMC_TRAJ_SET_FO_ENABLE_TYPE:
         case EMC_TRAJ_SET_FH_ENABLE_TYPE:
         case EMC_TRAJ_SET_SO_ENABLE_TYPE:
         case EMC_SPINDLE_SPEED_TYPE:
         case EMC_SPINDLE_ON_TYPE:
         case EMC_SPINDLE_OFF_TYPE:
         case EMC_SPINDLE_BRAKE_RELEASE_TYPE:
         case EMC_SPINDLE_BRAKE_ENGAGE_TYPE:
         case EMC_SPINDLE_INCREASE_TYPE:
         case EMC_SPINDLE_DECREASE_TYPE:
         case EMC_SPINDLE_CONSTANT_TYPE:
         case EMC_COOLANT_MIST_ON_TYPE:
         case EMC_COOLANT_MIST_OFF_TYPE:
         case EMC_COOLANT_FLOOD_ON_TYPE:
         case EMC_COOLANT_FLOOD_OFF_TYPE:
         case EMC_LUBE_ON_TYPE:
         case EMC_LUBE_OFF_TYPE:
         case EMC_TASK_SET_MODE_TYPE:
         case EMC_TASK_SET_STATE_TYPE:
         case EMC_TASK_ABORT_TYPE:
         case EMC_TASK_PLAN_PAUSE_TYPE:
         case EMC_TASK_PLAN_RESUME_TYPE:
         case EMC_TASK_PLAN_INIT_TYPE:
         case EMC_TASK_PLAN_SYNCH_TYPE:
         case EMC_TASK_PLAN_SET_OPTIONAL_STOP_TYPE:
         case EMC_TASK_PLAN_SET_BLOCK_DELETE_TYPE:
         case EMC_TASK_PLAN_OPTIONAL_STOP_TYPE:
         case EMC_TRAJ_CLEAR_PROBE_TRIPPED_FLAG_TYPE:
         case EMC_TRAJ_PROBE_TYPE:
         case EMC_AUX_INPUT_WAIT_TYPE:
         case EMC_MOTION_SET_DOUT_TYPE:
         case EMC_MOTION_ENABLE_DIN_ABORT_TYPE:
         case EMC_MOTION_DISABLE_DIN_ABORT_TYPE:
         case EMC_MOTION_SET_AOUT_TYPE:
         case EMC_MOTION_ADAPTIVE_TYPE:
         case EMC_TRAJ_RIGID_TAP_TYPE:
         case EMC_TRAJ_SET_TELEOP_ENABLE_TYPE:
         case EMC_TRAJ_SET_TELEOP_VECTOR_TYPE:
            retval = emcTaskIssueCommand(emcCommand);
            break;

            // queued commands

         case EMC_TASK_PLAN_EXECUTE_TYPE:
            // resynch the interpreter, since we may have moved externally
            interp_list.append((emc_command_msg_t *) & taskPlanSynchCmd);
            // and now call for interpreter execute
            retval = emcTaskIssueCommand(emcCommand);
            break;
         case EMC_TOOL_LOAD_TOOL_TABLE_TYPE:
         case EMC_TOOL_SET_OFFSET_TYPE:
            // send to IO
            interp_list.append((emc_command_msg_t *) emcCommand);
            // signify no more reading
            task_plan_wait = 1;
            // then resynch interpreter
            interp_list.append((emc_command_msg_t *) & taskPlanSynchCmd);
            break;
         case EMC_TOOL_SET_NUMBER_TYPE:
            // send to IO
            interp_list.append((emc_command_msg_t *) emcCommand);
            // then resynch interpreter
            interp_list.append((emc_command_msg_t *) & taskPlanSynchCmd);
            break;

            // otherwise we can't handle it

         default:
            BUG("can't do that (%s) in manual mode\n", lookup_message(type));
            emcOperatorMessage(0, EMC_I18N("can't do that (%s) in manual mode"), lookup_message(type));
            retval = EMC_R_ERROR;
            break;
         }      // switch (type) in ON, MANUAL
         break; // case EMC_TASK_MODE_MANUAL

      case EMC_TASK_MODE_AUTO: // ON, AUTO
         switch (emcStatus->task.interpState)
         {
         case EMC_TASK_INTERP_IDLE:    // ON, AUTO, IDLE
            switch (type)
            {
            case EMC_COMMAND_UNUSED:
               // no command
               break;
            case EMC_QUIT_TYPE:
               ps->command_thread_abort = 1;
               break;

               // immediate commands

            case EMC_AXIS_SET_BACKLASH_TYPE:
            case EMC_AXIS_SET_HOMING_PARAMS_TYPE:
            case EMC_AXIS_SET_FERROR_TYPE:
            case EMC_AXIS_SET_MIN_FERROR_TYPE:
            case EMC_AXIS_UNHOME_TYPE:
            case EMC_TRAJ_PAUSE_TYPE:
            case EMC_TRAJ_RESUME_TYPE:
            case EMC_TRAJ_ABORT_TYPE:
            case EMC_TRAJ_SET_SCALE_TYPE:
            case EMC_TRAJ_SET_MAX_VELOCITY_TYPE:
            case EMC_TRAJ_SET_SPINDLE_SCALE_TYPE:
            case EMC_TRAJ_SET_FO_ENABLE_TYPE:
            case EMC_TRAJ_SET_FH_ENABLE_TYPE:
            case EMC_TRAJ_SET_SO_ENABLE_TYPE:
            case EMC_SPINDLE_SPEED_TYPE:
            case EMC_SPINDLE_ON_TYPE:
            case EMC_SPINDLE_OFF_TYPE:
            case EMC_SPINDLE_BRAKE_RELEASE_TYPE:
            case EMC_SPINDLE_BRAKE_ENGAGE_TYPE:
            case EMC_SPINDLE_INCREASE_TYPE:
            case EMC_SPINDLE_DECREASE_TYPE:
            case EMC_SPINDLE_CONSTANT_TYPE:
            case EMC_MOTION_SET_DOUT_TYPE:
            case EMC_COOLANT_MIST_ON_TYPE:
            case EMC_COOLANT_MIST_OFF_TYPE:
            case EMC_COOLANT_FLOOD_ON_TYPE:
            case EMC_COOLANT_FLOOD_OFF_TYPE:
            case EMC_LUBE_ON_TYPE:
            case EMC_LUBE_OFF_TYPE:
            case EMC_TASK_SET_MODE_TYPE:
            case EMC_TASK_SET_STATE_TYPE:
            case EMC_TASK_ABORT_TYPE:
            case EMC_TASK_PLAN_INIT_TYPE:
            case EMC_TASK_PLAN_OPEN_TYPE:
            case EMC_TASK_PLAN_RUN_TYPE:
            case EMC_TASK_PLAN_EXECUTE_TYPE:
            case EMC_TASK_PLAN_PAUSE_TYPE:
            case EMC_TASK_PLAN_RESUME_TYPE:
            case EMC_TASK_PLAN_SET_OPTIONAL_STOP_TYPE:
            case EMC_TASK_PLAN_SET_BLOCK_DELETE_TYPE:
            case EMC_TASK_PLAN_OPTIONAL_STOP_TYPE:
            case EMC_TRAJ_CLEAR_PROBE_TRIPPED_FLAG_TYPE:
            case EMC_TRAJ_PROBE_TYPE:
            case EMC_AUX_INPUT_WAIT_TYPE:
            case EMC_TRAJ_RIGID_TAP_TYPE:
               retval = emcTaskIssueCommand(emcCommand);
               break;

            case EMC_TASK_PLAN_STEP_TYPE:
               // handles case where first action is to step the program
               taskPlanRunCmd.line = 1; // run from start
               /*! \todo FIXME-- can have GUI set this; send a run instead of a 
                  step */
               retval = emcTaskIssueCommand((emc_command_msg_t *) & taskPlanRunCmd);
               if (retval != EMC_R_OK)
                  break;
               emcTrajPause();
               if (emcStatus->task.interpState != EMC_TASK_INTERP_PAUSED)
               {
                  interpResumeState = emcStatus->task.interpState;
               }
               emcStatus->task.interpState = EMC_TASK_INTERP_PAUSED;
               emcStatus->task.task_paused = 1;
               retval = EMC_R_OK;
               break;

            case EMC_TOOL_LOAD_TOOL_TABLE_TYPE:
            case EMC_TOOL_SET_OFFSET_TYPE:
               // send to IO
               interp_list.append((emc_command_msg_t *) emcCommand);
               // signify no more reading
               task_plan_wait = 1;
               // then resynch interpreter
               interp_list.append((emc_command_msg_t *) & taskPlanSynchCmd);
               break;

               // otherwise we can't handle it
            default:
               BUG("can't do that (%s) in auto mode with the interpreter idle\n", lookup_message(type));
               emcOperatorMessage(0, EMC_I18N("can't do that (%s) in auto mode with the interpreter idle"), lookup_message(type));
               retval = EMC_R_ERROR;
               break;

            }   // switch (type) in ON, AUTO, IDLE

            break;      // EMC_TASK_INTERP_IDLE

         case EMC_TASK_INTERP_READING: // ON, AUTO, READING
            switch (type)
            {
            case EMC_COMMAND_UNUSED:
               // no command
               break;
            case EMC_QUIT_TYPE:
               ps->command_thread_abort = 1;
               break;

               // immediate commands

            case EMC_AXIS_SET_BACKLASH_TYPE:
            case EMC_AXIS_SET_HOMING_PARAMS_TYPE:
            case EMC_AXIS_SET_FERROR_TYPE:
            case EMC_AXIS_SET_MIN_FERROR_TYPE:
            case EMC_AXIS_UNHOME_TYPE:
            case EMC_TRAJ_PAUSE_TYPE:
            case EMC_TRAJ_RESUME_TYPE:
            case EMC_TRAJ_ABORT_TYPE:
            case EMC_TRAJ_SET_SCALE_TYPE:
            case EMC_TRAJ_SET_MAX_VELOCITY_TYPE:
            case EMC_TRAJ_SET_SPINDLE_SCALE_TYPE:
            case EMC_TRAJ_SET_FO_ENABLE_TYPE:
            case EMC_TRAJ_SET_FH_ENABLE_TYPE:
            case EMC_TRAJ_SET_SO_ENABLE_TYPE:
            case EMC_SPINDLE_INCREASE_TYPE:
            case EMC_SPINDLE_DECREASE_TYPE:
            case EMC_SPINDLE_CONSTANT_TYPE:
            case EMC_TASK_PLAN_PAUSE_TYPE:
            case EMC_TASK_PLAN_RESUME_TYPE:
            case EMC_TASK_PLAN_SET_OPTIONAL_STOP_TYPE:
            case EMC_TASK_PLAN_SET_BLOCK_DELETE_TYPE:
            case EMC_TASK_PLAN_OPTIONAL_STOP_TYPE:
            case EMC_TASK_SET_MODE_TYPE:
            case EMC_TASK_SET_STATE_TYPE:
            case EMC_TASK_ABORT_TYPE:
            case EMC_TRAJ_CLEAR_PROBE_TRIPPED_FLAG_TYPE:
            case EMC_TRAJ_PROBE_TYPE:
            case EMC_AUX_INPUT_WAIT_TYPE:
            case EMC_TRAJ_RIGID_TAP_TYPE:
               retval = emcTaskIssueCommand(emcCommand);
               return;
               break;

            case EMC_TASK_PLAN_STEP_TYPE:
               stepping = 1;    // set stepping mode in case it's not
               steppingWait = 0;        // clear the wait
               break;

               // otherwise we can't handle it
            default:
               BUG("can't do that (%s) in auto mode with the interpreter reading\n", lookup_message(type));
               emcOperatorMessage(0, EMC_I18N("can't do that (%s) in auto mode with the interpreter reading"), lookup_message(type));
               retval = EMC_R_ERROR;
               break;

            }   // switch (type) in ON, AUTO, READING

            // handle interp readahead logic
            readahead_reading();
            break;      // EMC_TASK_INTERP_READING

         case EMC_TASK_INTERP_PAUSED:  // ON, AUTO, PAUSED
            switch (type)
            {
            case EMC_COMMAND_UNUSED:
               // no command
               break;
            case EMC_QUIT_TYPE:
               ps->command_thread_abort = 1;
               break;

               // immediate commands

            case EMC_AXIS_SET_BACKLASH_TYPE:
            case EMC_AXIS_SET_HOMING_PARAMS_TYPE:
            case EMC_AXIS_SET_FERROR_TYPE:
            case EMC_AXIS_SET_MIN_FERROR_TYPE:
            case EMC_AXIS_UNHOME_TYPE:
            case EMC_TRAJ_PAUSE_TYPE:
            case EMC_TRAJ_RESUME_TYPE:
            case EMC_TRAJ_ABORT_TYPE:
            case EMC_TRAJ_SET_SCALE_TYPE:
            case EMC_TRAJ_SET_MAX_VELOCITY_TYPE:
            case EMC_TRAJ_SET_SPINDLE_SCALE_TYPE:
            case EMC_TRAJ_SET_FO_ENABLE_TYPE:
            case EMC_TRAJ_SET_FH_ENABLE_TYPE:
            case EMC_TRAJ_SET_SO_ENABLE_TYPE:
            case EMC_SPINDLE_SPEED_TYPE:
            case EMC_SPINDLE_ON_TYPE:
            case EMC_SPINDLE_OFF_TYPE:
            case EMC_SPINDLE_BRAKE_RELEASE_TYPE:
            case EMC_SPINDLE_BRAKE_ENGAGE_TYPE:
            case EMC_SPINDLE_INCREASE_TYPE:
            case EMC_SPINDLE_DECREASE_TYPE:
            case EMC_SPINDLE_CONSTANT_TYPE:
            case EMC_MOTION_SET_DOUT_TYPE:
            case EMC_COOLANT_MIST_ON_TYPE:
            case EMC_COOLANT_MIST_OFF_TYPE:
            case EMC_COOLANT_FLOOD_ON_TYPE:
            case EMC_COOLANT_FLOOD_OFF_TYPE:
            case EMC_LUBE_ON_TYPE:
            case EMC_LUBE_OFF_TYPE:
            case EMC_TASK_SET_MODE_TYPE:
            case EMC_TASK_SET_STATE_TYPE:
            case EMC_TASK_ABORT_TYPE:
            case EMC_TASK_PLAN_EXECUTE_TYPE:
            case EMC_TASK_PLAN_PAUSE_TYPE:
            case EMC_TASK_PLAN_RESUME_TYPE:
            case EMC_TASK_PLAN_SET_OPTIONAL_STOP_TYPE:
            case EMC_TASK_PLAN_SET_BLOCK_DELETE_TYPE:
            case EMC_TASK_PLAN_OPTIONAL_STOP_TYPE:
            case EMC_TRAJ_CLEAR_PROBE_TRIPPED_FLAG_TYPE:
            case EMC_TRAJ_PROBE_TYPE:
            case EMC_AUX_INPUT_WAIT_TYPE:
            case EMC_TRAJ_RIGID_TAP_TYPE:
               retval = emcTaskIssueCommand(emcCommand);
               break;

            case EMC_TASK_PLAN_STEP_TYPE:
               stepping = 1;
               steppingWait = 0;
               if (emcStatus->motion.traj.paused && emcStatus->motion.traj.queue > 0)
               {
                  // there are pending motions paused; step them
                  emcTrajStep();
               }
               else
               {
                  emcStatus->task.interpState = (enum EMC_TASK_INTERP) interpResumeState;
               }
               emcStatus->task.task_paused = 1;
               break;

               // otherwise we can't handle it
            default:
               BUG("can't do that (%s) in auto mode with the interpreter paused\n", lookup_message(type));
               emcOperatorMessage(0, EMC_I18N("can't do that (%s) in auto mode with the interpreter paused"), lookup_message(type));
               retval = EMC_R_ERROR;
               break;
            }   // switch (type) in ON, AUTO, PAUSED

            break;      // EMC_TASK_INTERP_PAUSED

         case EMC_TASK_INTERP_WAITING:  // ON, AUTO, WAITING
            // interpreter ran to end
            // handle input commands
            switch (type)
            {
            case EMC_COMMAND_UNUSED:
               // no command
               break;
            case EMC_QUIT_TYPE:
               ps->command_thread_abort = 1;
               break;

               // immediate commands

            case EMC_AXIS_SET_BACKLASH_TYPE:
            case EMC_AXIS_SET_HOMING_PARAMS_TYPE:
            case EMC_AXIS_SET_FERROR_TYPE:
            case EMC_AXIS_SET_MIN_FERROR_TYPE:
            case EMC_AXIS_UNHOME_TYPE:
            case EMC_TRAJ_PAUSE_TYPE:
            case EMC_TRAJ_RESUME_TYPE:
            case EMC_TRAJ_ABORT_TYPE:
            case EMC_TRAJ_SET_SCALE_TYPE:
            case EMC_TRAJ_SET_MAX_VELOCITY_TYPE:
            case EMC_TRAJ_SET_SPINDLE_SCALE_TYPE:
            case EMC_TRAJ_SET_FO_ENABLE_TYPE:
            case EMC_TRAJ_SET_FH_ENABLE_TYPE:
            case EMC_TRAJ_SET_SO_ENABLE_TYPE:
            case EMC_SPINDLE_INCREASE_TYPE:
            case EMC_SPINDLE_DECREASE_TYPE:
            case EMC_SPINDLE_CONSTANT_TYPE:
            case EMC_MOTION_SET_DOUT_TYPE:
            case EMC_MOTION_ENABLE_DIN_ABORT_TYPE:
            case EMC_MOTION_DISABLE_DIN_ABORT_TYPE:
            case EMC_TASK_PLAN_EXECUTE_TYPE:
            case EMC_TASK_PLAN_PAUSE_TYPE:
            case EMC_TASK_PLAN_RESUME_TYPE:
            case EMC_TASK_PLAN_SET_OPTIONAL_STOP_TYPE:
            case EMC_TASK_PLAN_SET_BLOCK_DELETE_TYPE:
            case EMC_TASK_PLAN_OPTIONAL_STOP_TYPE:
            case EMC_TASK_SET_MODE_TYPE:
            case EMC_TASK_SET_STATE_TYPE:
            case EMC_TASK_ABORT_TYPE:
            case EMC_TRAJ_CLEAR_PROBE_TRIPPED_FLAG_TYPE:
            case EMC_TRAJ_PROBE_TYPE:
            case EMC_AUX_INPUT_WAIT_TYPE:
            case EMC_TRAJ_RIGID_TAP_TYPE:
               retval = emcTaskIssueCommand(emcCommand);
               break;

            case EMC_TASK_PLAN_STEP_TYPE:
               stepping = 1;    // set stepping mode in case it's not
               steppingWait = 0;        // clear the wait
               break;

               // otherwise we can't handle it
            default:
               BUG("can't do that (%s) in auto mode with the interpreter waiting\n", lookup_message(type));
               emcOperatorMessage(0, EMC_I18N("can't do that (%s) in auto mode with the interpreter waiting"), lookup_message(type));
               retval = EMC_R_ERROR;
               break;
            }   // switch (type) in ON, AUTO, WAITING

            // handle interp readahead logic
            readahead_waiting();
            break;      // end of case EMC_TASK_INTERP_WAITING

         default:
            // coding error
            BUG("invalid mode(%d)", emcStatus->task.mode);
            retval = EMC_R_ERROR;
            break;
         }      // switch (mode) in ON, AUTO
         break; // case EMC_TASK_MODE_AUTO

      case EMC_TASK_MODE_MDI:  // ON, MDI
         switch (type)
         {
         case EMC_COMMAND_UNUSED:
            // no command
            break;
         case EMC_QUIT_TYPE:
            ps->command_thread_abort = 1;
            break;

            // immediate commands

         case EMC_AXIS_SET_BACKLASH_TYPE:
         case EMC_AXIS_SET_HOMING_PARAMS_TYPE:
         case EMC_AXIS_SET_FERROR_TYPE:
         case EMC_AXIS_SET_MIN_FERROR_TYPE:
         case EMC_AXIS_UNHOME_TYPE:
         case EMC_TRAJ_SET_SCALE_TYPE:
         case EMC_TRAJ_SET_MAX_VELOCITY_TYPE:
         case EMC_TRAJ_SET_SPINDLE_SCALE_TYPE:
         case EMC_TRAJ_SET_FO_ENABLE_TYPE:
         case EMC_TRAJ_SET_FH_ENABLE_TYPE:
         case EMC_TRAJ_SET_SO_ENABLE_TYPE:
         case EMC_SPINDLE_SPEED_TYPE:
         case EMC_SPINDLE_ON_TYPE:
         case EMC_SPINDLE_OFF_TYPE:
         case EMC_SPINDLE_BRAKE_RELEASE_TYPE:
         case EMC_SPINDLE_BRAKE_ENGAGE_TYPE:
         case EMC_SPINDLE_INCREASE_TYPE:
         case EMC_SPINDLE_DECREASE_TYPE:
         case EMC_SPINDLE_CONSTANT_TYPE:
         case EMC_COOLANT_MIST_ON_TYPE:
         case EMC_COOLANT_MIST_OFF_TYPE:
         case EMC_COOLANT_FLOOD_ON_TYPE:
         case EMC_COOLANT_FLOOD_OFF_TYPE:
         case EMC_LUBE_ON_TYPE:
         case EMC_LUBE_OFF_TYPE:
         case EMC_TASK_SET_MODE_TYPE:
         case EMC_TASK_SET_STATE_TYPE:
         case EMC_TASK_PLAN_INIT_TYPE:
         case EMC_TASK_PLAN_OPEN_TYPE:
         case EMC_TASK_PLAN_EXECUTE_TYPE:
         case EMC_TASK_PLAN_PAUSE_TYPE:
         case EMC_TASK_PLAN_SET_OPTIONAL_STOP_TYPE:
         case EMC_TASK_PLAN_SET_BLOCK_DELETE_TYPE:
         case EMC_TASK_PLAN_RESUME_TYPE:
         case EMC_TASK_PLAN_OPTIONAL_STOP_TYPE:
         case EMC_TASK_ABORT_TYPE:
         case EMC_TRAJ_CLEAR_PROBE_TRIPPED_FLAG_TYPE:
         case EMC_TRAJ_PROBE_TYPE:
         case EMC_AUX_INPUT_WAIT_TYPE:
         case EMC_MOTION_SET_DOUT_TYPE:
         case EMC_MOTION_ENABLE_DIN_ABORT_TYPE:
         case EMC_MOTION_DISABLE_DIN_ABORT_TYPE:
         case EMC_MOTION_SET_AOUT_TYPE:
         case EMC_MOTION_ADAPTIVE_TYPE:
         case EMC_TRAJ_RIGID_TAP_TYPE:
            retval = emcTaskIssueCommand(emcCommand);
            break;
         case EMC_TOOL_LOAD_TOOL_TABLE_TYPE:
         case EMC_TOOL_SET_OFFSET_TYPE:
            // send to IO
            interp_list.append((emc_command_msg_t *) emcCommand);
            // signify no more reading
            task_plan_wait = 1;
            // then resynch interpreter
            interp_list.append((emc_command_msg_t *) & taskPlanSynchCmd);
            break;
            // otherwise we can't handle it
         default:
            BUG("can't do that (%s) in MDI mode\n", lookup_message(type));
            emcOperatorMessage(0, EMC_I18N("can't do that (%s) in MDI mode"), lookup_message(type));
            retval = EMC_R_ERROR;
            break;
         }      // switch (type) in ON, MDI
         break; // case EMC_TASK_MODE_MDI
      default:
         break;
      } // switch (mode)
      break;    // case EMC_TASK_STATE_ON
   default:
      break;
   }    // switch (task.state)

   if (retval != EMC_R_OK)
      emcStatus->task.planState = EMC_TASK_PLAN_ERROR;

}       /* emcTaskPlan() */
