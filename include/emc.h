/********************************************************************
* emc.h - Support commands for EMC2
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
#ifndef _EMC_H
#define _EMC_H

#include <stddef.h>
#include <pthread.h>

#ifndef LINELEN
   #define LINELEN 255
#endif

#include "motion.h"
#include "emc_msg.h"
#include "rtstepper.h"

#if (defined(__WIN32__) || defined(_WINDOWS))
   #define DLL_EXPORT __declspec(dllexport)
#else
   #define DLL_EXPORT __attribute__ ((visibility("default")))
#endif

#define DUMMY_VARIABLE __attribute__((__unused__))

struct mcode_args_t
{
   int index;     /* output0-7 */
   double p1;     /* parameter 1 */
   double p2;    /* parameter 2 */
   unsigned int ticket;
};

struct emc_session
{
   int command_thread_active;
   int command_thread_abort;
   pthread_t command_thread_tid;
   pthread_mutex_t mutex;
   pthread_cond_t event_cond;
   pthread_cond_t command_thread_done_cond;
   pthread_cond_t control_cycle_thread_done_cond;
   pthread_cond_t mcode_thread_done_cond;
   int control_cycle_thread_active;
   pthread_t control_cycle_thread_tid;
   int mcode_thread_active;
   int mcode_thread_abort;
   int mcode_script_active;
   pthread_t mcode_thread_tid;
   struct rtstepper_app_session dongle;
   struct _emc_msg_t head;
};

#include "msg.h"

enum EMC_RESULT
{
   EMC_R_INVALID_INI_KEY = -4,
   EMC_R_INVALID_INI_FILE = -3,
   EMC_R_TIMEOUT = -2,
   EMC_R_ERROR = -1,
   EMC_R_OK = 0,
};

// Localization helper.
#ifndef EMC_I18N
#define EMC_I18N(text) text
#endif

#define container_of(ptr, type, member) ({            \
 const typeof( ((type *)0)->member ) *__mptr = (ptr);    \
 (type *)( (char *)__mptr - offsetof(type,member) );})

#define EMC_AXIS_MAX EMCMOT_MAX_AXIS
#define EMC_MAX_DIO EMCMOT_MAX_DIO
#define EMC_MAX_AIO EMCMOT_MAX_AIO
#define DEFAULT_RS274NGC_STARTUP_CODE ""

/* debug bitflags */
/* Note: these may be hard-code referenced by the GUI (e.g., emcdebug.tcl).
   If you change the assignments here, make sure and reflect that in
   the GUI scripts that use these. Unfortunately there's no easy way to
   get these into Tk automatically */
extern int EMC_DEBUG;
#define EMC_DEBUG_CONFIG            0x00000002
#define EMC_DEBUG_VERSIONS          0x00000008
#define EMC_DEBUG_TASK_ISSUE        0x00000010
#define EMC_DEBUG_NML               0x00000040
#define EMC_DEBUG_MOTION_TIME       0x00000080
#define EMC_DEBUG_INTERP            0x00000100
#define EMC_DEBUG_RCS               0x00000200
#define EMC_DEBUG_INTERP_LIST       0x00000800
#define EMC_DEBUG_ALL               0x7FFFFFFF  /* it's an int for %i to work  */

/* default name of EMC ini file */
#define DEFAULT_EMC_INIFILE EMC2_DEFAULT_INIFILE

#define DEFAULT_EMC_PROGRAM_PREFIX EMC2_DEFAULT_PROGRAM_PREFIX

/* cycle time for emctask, in seconds */
#define DEFAULT_EMC_TASK_CYCLE_TIME 0.100

/* cycle time for emctio, in seconds */
#define DEFAULT_EMC_IO_CYCLE_TIME 0.100

/* default interp len */
#define DEFAULT_EMC_TASK_INTERP_MAX_LEN 1000

/* default name of EMC_TOOL tool table file */
#define DEFAULT_TOOL_TABLE_FILE "tool.tbl"

/* default feed rate, in user units per second */
#define DEFAULT_TRAJ_DEFAULT_VELOCITY 1.0

/* default traverse rate, in user units per second */
#define DEFAULT_TRAJ_MAX_VELOCITY 10.0

/* default axis traverse rate, in user units per second */
#define DEFAULT_AXIS_MAX_VELOCITY 1.0

/* default axis acceleration, in user units per second per second */
#define DEFAULT_AXIS_MAX_ACCELERATION 1.0

// values for EMC_AXIS_SET_AXIS, axisType
enum EmcAxisType
{
   EMC_AXIS_LINEAR = 1,
   EMC_AXIS_ANGULAR = 2,
};

// defs for termination conditions
#define EMC_TRAJ_TERM_COND_STOP  1
#define EMC_TRAJ_TERM_COND_BLEND 2

#define RTSTEPPER_PERIOD 42667  /* 1 / (46875hz / 2) = 0.000042667 */

/* Set the units conversion factor. @see EMC_AXIS_SET_INPUT_SCALE  */
typedef double EmcLinearUnits;
typedef double EmcAngularUnits;

extern char EMC_INIFILE[LINELEN];
extern char EMC_NMLFILE[LINELEN];
extern char RS274NGC_STARTUP_CODE[LINELEN];
extern char EMC_PROGRAM_PREFIX[LINELEN];
extern double EMC_TASK_CYCLE_TIME;
extern double EMC_IO_CYCLE_TIME;
extern int EMC_TASK_INTERP_MAX_LEN;
extern char TOOL_TABLE_FILE[LINELEN];
extern double TRAJ_DEFAULT_VELOCITY;
extern double TRAJ_MAX_VELOCITY;
extern double AXIS_MAX_VELOCITY[EMC_AXIS_MAX];
extern double AXIS_MAX_ACCELERATION[EMC_AXIS_MAX];
extern struct EmcPose TOOL_CHANGE_POSITION;
extern unsigned char HAVE_TOOL_CHANGE_POSITION;
extern struct EmcPose TOOL_HOLDER_CLEAR;
extern unsigned char HAVE_TOOL_HOLDER_CLEAR;
extern const char *USER_HOME_DIR;

//extern int num_axes;
//extern double VELOCITY;
//extern double ACCELERATION;
//extern double MAX_LIMIT;
//extern double MIN_LIMIT;
//extern double MAX_OUTPUT;
//extern double MIN_OUTPUT;
//extern int TC_QUEUE_SIZE;
//extern double MAX_FERROR;
//extern double BACKLASH;

extern struct emc_session session;

extern emc_status_t *emcStatus;
extern emc_command_msg_t *emcCommand;
extern emc_command_msg_t *emcTaskCommand;
extern emcio_status_t emcioStatus;
extern emcmot_command_t emcmotCommand;
extern emcmot_status_t emcmotStatus;
extern emcmot_config_t emcmotConfig;
extern emcmot_debug_t emcmotDebug;
extern emcmot_joint_t joints[EMCMOT_MAX_JOINTS];
extern KINEMATICS_FORWARD_FLAGS fflags;
extern KINEMATICS_INVERSE_FLAGS iflags;

extern const char _gui_tag[];
extern const char _mcd_tag[];
extern const char _ctl_tag[];

#ifdef __cplusplus
extern "C"
{
#endif

   int emcGetArgs(int argc, char *argv[]);
   void emcInitGlobals();
   void esleep(double seconds_to_sleep);
   int emcOperatorMessage(int id, const char *fmt, ...);
   int emc_io_error_cb(int result);

   int emcAxisSetBacklash(int axis, double backlash);
   int emcAxisSetMinPositionLimit(int axis, double limit);
   int emcAxisSetMaxPositionLimit(int axis, double limit);
   int emcAxisSetMotorOffset(int axis, double offset);
   int emcAxisSetFerror(int axis, double ferror);
   int emcAxisSetMinFerror(int axis, double ferror);
   int emcAxisSetHomingParams(int axis, double home, double offset, double home_final_vel, double search_vel,
                              double latch_vel, int use_index, int ignore_limits, int is_shared, int home_sequence, int volatile_home);
   int emcAxisSetMaxVelocity(int axis, double vel);
   int emcAxisSetMaxAcceleration(int axis, double acc);
   int emcAxisSetInputScale(int axis, double scale);
   int emcAxisSetStepPin(int axis, int pin);
   int emcAxisSetDirectionPin(int axis, int pin);
   int emcAxisSetStepPolarity(int axis, int polarity);
   int emcAxisSetDirectionPolarity(int axis, int polarity);
   int emcAxisInit(int axis);
   int emcAxisHalt(int axis);
   int emcAxisAbort(int axis);
   int emcAxisEnable(int axis);
   int emcAxisDisable(int axis);
   int emcAxisHome(int axis);
   int emcAxisUnhome(int axis);
   int emcAxisJog(int axis, double vel);
   int emcAxisIncrJog(int axis, double incr, double vel);
   int emcAxisAbsJog(int axis, double pos, double vel);
   int emcAxisActivate(int axis);
   int emcAxisDeactivate(int axis);
   int emcAxisOverrideLimits(int axis);
   int emcAxisLoadComp(int axis, const char *file, int type);
   int emcAxisUpdate(emcaxis_status_t * stat, int numAxes);
   int emcAxisSetAxis(int axis, unsigned char axisType);
   int emcAxisSetUnits(int axis, double units);
   int emcAxisActivate(int axis);

   int emcTrajSetAxes(int axes, int axismask);
   int emcTrajSetUnits(double linearUnits, double angularUnits);
   int emcTrajSetCycleTime(double cycleTime);
   int emcTrajSetMode(enum EMC_TRAJ_MODE mode);
   int emcTrajSetTeleopVector(EmcPose vel);
   int emcTrajSetVelocity(double vel, double ini_maxvel);
   int emcTrajSetAcceleration(double acc);
   int emcTrajSetMaxVelocity(double vel);
   int emcTrajSetMaxAcceleration(double acc);
   int emcTrajSetScale(double scale);
   int emcTrajSetFOEnable(unsigned char mode);  //feed override enable
   int emcTrajSetFHEnable(unsigned char mode);  //feed hold enable
   int emcTrajSetSpindleScale(double scale);
   int emcTrajSetSOEnable(unsigned char mode);  //spindle speed override enable
   int emcTrajSetAFEnable(unsigned char enable);        //adaptive feed enable
   int emcTrajSetMotionId(int id);
   double emcTrajGetLinearUnits();
   double emcTrajGetAngularUnits();

   int emcTrajInit();
   int emcTrajHalt();
   int emcTrajEnable();
   int emcTrajDisable();
   int emcTrajAbort();
   int emcTrajPause();
   int emcTrajStep();
   int emcTrajResume();
   int emcTrajDelay(double delay);
   int emcTrajLinearMove(EmcPose end, int type, double vel, double ini_maxvel, double acc);
//int emcTrajCircularMove(EmcPose end, PM_CARTESIAN center, PM_CARTESIAN normal,
//      int turn, int type, double vel, double ini_maxvel, double acc);
   int emcTrajSetTermCond(int cond, double tolerance);
//int emcTrajSetSpindleSync(double feed_per_revolution, bool wait_for_index);
   int emcTrajSetOffset(EmcPose tool_offset);
   int emcTrajSetOrigin(EmcPose origin);
   int emcTrajSetRotation(double rotation);
   int emcTrajSetHome(EmcPose home);
   int emcTrajClearProbeTrippedFlag();
   int emcTrajProbe(EmcPose pos, int type, double vel, double ini_maxvel, double acc, unsigned char probe_type);
   int emcAuxInputWait(int index, int input_type, int wait_type, int timeout);
   int emcTrajRigidTap(EmcPose pos, double vel, double ini_maxvel, double acc);
   int emcTrajUpdate(emctraj_status_t * stat);

   int emcMotionInit();
   int emcMotionHalt();
   int emcMotionAbort();
   int emcMotionSetDebug(int debug);
   int emcMotionSetAout(unsigned char index, double start, double end, unsigned char now);
   int emcMotionSetDout(unsigned char index, unsigned char start, unsigned char end, unsigned char now);
   int emcMotionUpdate(emcmot_status_t * stat);

   int emcTaskInit();
   int emcTaskHalt();
   int emcTaskAbort();
   int emcTaskSetState(int state);
   int emcTaskPlanInit();
   int emcTaskPlanExit();
   int emcTaskUpdate(emctask_status_t * stat);
   void emcTaskPlan(emc_command_msg_t *emcCommand);
   void emcTaskExecute(void);
   int emcTaskPlanCommand(char *cmd);
   int emcTaskPlanRead();
   int emcTaskPlanOpen(const char *file);
   int emcTaskPlanClose();

   int emcToolInit();
   int emcToolHalt();
   int emcToolAbort();
   int emcToolPrepare(int tool);
   int emcToolLoad();
   int emcToolUnload();
   int emcToolLoadToolTable(const char *file);
   int emcToolSetOffset(int pocket, int toolno, EmcPose offset, double diameter, double frontangle, double backangle, int orientation);
   int emcToolSetNumber(int number);
   int emcToolUpdate(emctool_status_t * stat);
   int emcToolSetToolTableFile(const char *filename);

   int emcAuxEstopOn();
   int emcAuxEstopOff();
   int emcAuxUpdate(emcaux_status_t * stat);

   int emcSpindleAbort();
   int emcSpindleSpeed(double speed, double factor, double xoffset);
   int emcSpindleOn(double speed, double factor, double xoffset);
   int emcSpindleOff();
   int emcSpindleIncrease();
   int emcSpindleDecrease();
   int emcSpindleConstant();
   int emcSpindleBrakeRelease();
   int emcSpindleBrakeEngage();
   int emcSpindleSetMode(int mode);     //determines if Spindle needs to reset on abort
   int emcSpindleUpdate(emcspindle_status_t * stat);

   int emcCoolantMistOn();
   int emcCoolantMistOff();
   int emcCoolantFloodOn();
   int emcCoolantFloodOff();
   int emcCoolantUpdate(emccoolant_status_t * stat);

   int emcLubeOn();
   int emcLubeOff();
   int emcLubeUpdate(emclube_status_t * stat);

   int emcIoInit();
   int emcIoHalt();
   int emcIoAbort();
   int emcIoSetCycleTime(double cycleTime);
   int emcIoSetDebug(int debug);
   int emcIoUpdate(emcio_status_t * stat);
   void emciocommandHandler(emcio_command_t * emcioCommand);

   void emcmotController(long period);
   void emcmotCommandHandler(emcmot_command_t * emcmotCommand);
   void emcmot_config_change(void);
   int emcmotCheckAllHomed(void);
   void emcmot_refresh_jog_limits(emcmot_joint_t * joint);
   int emcmotSetTrajCycleTime(double secs);
   int emcmotSetServoCycleTime(double secs);
   int emcmotExecScript(int index, double p1, double p2, int type);

   int emcInit();
   int emcHalt();
   int emcAbort();

//   int emcUpdate(emc_status_t * stat);

   DLL_EXPORT void *emc_ui_open(const char *ini_file);
   DLL_EXPORT enum EMC_RESULT emc_ui_close(void *hd);
   DLL_EXPORT enum EMC_RESULT emc_ui_update_status(void *hd);
   DLL_EXPORT enum EMC_RESULT emc_ui_get_operator_message(void *hd, char *buf, int buf_size);
   DLL_EXPORT enum EMC_RESULT emc_ui_operator_message(void *hd, const char *buf);
/* Wait for last command to be received by command thread. */
   DLL_EXPORT enum EMC_RESULT emc_ui_wait_command_received(void *hd, double timeout);
/* Wait for last command to finish executing. */
   DLL_EXPORT enum EMC_RESULT emc_ui_wait_command_done(void *hd, double timeout);
   DLL_EXPORT enum EMC_RESULT emc_ui_wait_io_done(void *hd, double timeout);
   DLL_EXPORT enum EMC_RESULT emc_ui_estop(void *hd);
   DLL_EXPORT enum EMC_RESULT emc_ui_estop_reset(void *hd);
   DLL_EXPORT enum EMC_RESULT emc_ui_machine_on(void *hd);
   DLL_EXPORT enum EMC_RESULT emc_ui_machine_off(void *hd);
   DLL_EXPORT enum EMC_RESULT emc_ui_manual_mode(void *hd);
   DLL_EXPORT enum EMC_RESULT emc_ui_auto_mode(void *hd);
   DLL_EXPORT enum EMC_RESULT emc_ui_mdi_mode(void *hd);
   DLL_EXPORT enum EMC_RESULT emc_ui_mist_on(void *hd);
   DLL_EXPORT enum EMC_RESULT emc_ui_mist_off(void *hd);
   DLL_EXPORT enum EMC_RESULT emc_ui_flood_on(void *hd);
   DLL_EXPORT enum EMC_RESULT emc_ui_flood_off(void *hd);
   DLL_EXPORT enum EMC_RESULT emc_ui_lube_on(void *hd);
   DLL_EXPORT enum EMC_RESULT emc_ui_lube_off(void *hd);
   DLL_EXPORT enum EMC_RESULT emc_ui_spindle_forward(void *hd);
   DLL_EXPORT enum EMC_RESULT emc_ui_spindle_reverse(void *hd);
   DLL_EXPORT enum EMC_RESULT emc_ui_spindle_off(void *hd);
   DLL_EXPORT enum EMC_RESULT emc_ui_tool_table(void *hd, const char *file);
   DLL_EXPORT enum EMC_RESULT emc_ui_tool_offset(void *hd, int toolno, double zoffset, double diameter);
   DLL_EXPORT enum EMC_RESULT emc_ui_override_limits(void *hd, int axis);
   DLL_EXPORT enum EMC_RESULT emc_ui_mdi_cmd(void *hd, const char *mdi);
   DLL_EXPORT enum EMC_RESULT emc_ui_home(void *hd, int axis);
   DLL_EXPORT enum EMC_RESULT emc_ui_unhome(void *hd, int axis);
   DLL_EXPORT enum EMC_RESULT emc_ui_jog_stop(void *hd, int axis);
   DLL_EXPORT enum EMC_RESULT emc_ui_jog_incr(void *hd, int axis, double speed, double incr);
   DLL_EXPORT enum EMC_RESULT emc_ui_jog_abs(void *hd, int axis, double speed, double pos);
   DLL_EXPORT enum EMC_RESULT emc_ui_feed_override(void *hd, double override);
   DLL_EXPORT enum EMC_RESULT emc_ui_plan_init(void *hd);
   DLL_EXPORT enum EMC_RESULT emc_ui_program_open(void *hd, const char *program);
   DLL_EXPORT enum EMC_RESULT emc_ui_program_run(void *hd, int line);
   DLL_EXPORT enum EMC_RESULT emc_ui_program_pause(void *hd);
   DLL_EXPORT enum EMC_RESULT emc_ui_program_resume(void *hd);
   DLL_EXPORT enum EMC_RESULT emc_ui_program_step(void *hd);
   DLL_EXPORT enum EMC_RESULT emc_ui_abort(void *hd);
   DLL_EXPORT enum EMC_RESULT emc_ui_axis_backlash(void *hd, int axis, double backlash);
   DLL_EXPORT enum EMC_RESULT emc_ui_teleop_enable(void *hd, int enable);
   DLL_EXPORT int emc_ui_get_ini_key_value(void *hd, const char *section, const char *key, char *value, int value_size);
   DLL_EXPORT enum EMC_RESULT emc_ui_get_version(const char **ver);
   DLL_EXPORT enum EMC_RESULT emc_ui_dout(void *hd, int output_num, int value, int sync);
   DLL_EXPORT enum EMC_TASK_STATE emc_ui_get_task_state(void *hd);
   DLL_EXPORT enum EMC_TASK_MODE emc_ui_get_task_mode(void *hd);
   DLL_EXPORT enum EMC_DIN_STATE emc_ui_get_din_state(void *hd, int input_num);
   DLL_EXPORT enum EMC_RESULT emc_ui_enable_din_abort(void *hd, int input_num);
   DLL_EXPORT enum EMC_RESULT emc_ui_disable_din_abort(void *hd, int input_num);

#ifdef __cplusplus
}                               /* matches extern "C" at top */
#endif

#endif                          // _COMMAND_H
