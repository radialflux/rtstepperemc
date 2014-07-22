/********************************************************************
  motion.h - Data structures used throughout EMC2

  Author:
  License: GPL Version 2
  System: Linux

  Copyright (c) 2004 All rights reserved

  The terms axis and joint are used inconsistently throughout EMC.
  For all new code, the usages are as follows:

    axis - one of the nine degrees of freedom, x, y, z, a, b, c, u, v, w
        these refer to axes in Cartesian space, which may or
        may not match up with joints (see below). On Cartesian
        machines they do match up, but for hexapods, robots, and
        other non-Cartesian machines they don't.
    joint - one of the physical degrees of freedom of the machine
        these might be linear (leadscrews) or rotary (rotary
        tables, robot arm joints).  There can be any number of
        joints.  The kinematics code is responsible for translating
        from axis space to joint space and back.

  There are three main kinds of data needed by the motion controller

  1) data shared with higher level stuff - commands, status, etc.
  2) data that is local to the motion controller
  3) data shared with lower level stuff - hal pins

  History:

********************************************************************/

#ifndef _MOTION_H
#define _MOTION_H

#include "posemath.h"   /* PmCartesian, PmPose, pmCartMag() */
#include "emcpos.h"     /* EmcPose */
#include "cubic.h"      /* CUBIC_STRUCT, CUBIC_COEFF */
#include "kinematics.h"
#include "emctool.h"

enum RCS_STATUS
{
   UNINITIALIZED_STATUS = -1,
   RCS_DONE = 1,
   RCS_EXEC = 2,
   RCS_ERROR = 3
};

#define EMCMOT_MAX_JOINTS 9
#define EMCMOT_MAX_AXIS 9

#define EMCMOT_MAX_DIO 64
#define EMCMOT_MAX_AIO 16

#define EMCMOT_ERROR_NUM 32     /* how many errors we can queue */
#define EMCMOT_ERROR_LEN 256    /* how long error string can be */

/* initial velocity, accel used for coordinated moves */
#define DEFAULT_VELOCITY 1.0
#define DEFAULT_ACCELERATION 10.0

/* maximum and minimum limit defaults for all axes */
#define DEFAULT_MAX_LIMIT 1000
#define DEFAULT_MIN_LIMIT -1000

/* size of motion queue, a TC_STRUCT is about 512 bytes so this queue is about a megabyte.  */
#define DEFAULT_TC_QUEUE_SIZE 2000

/* max following error */
#define DEFAULT_MAX_FERROR 100

#include "tp.h" /* TP_STRUCT */
#include "tc.h" /* TC_STRUCT, TC_QUEUE_STRUCT */

typedef struct _EMC_TELEOP_DATA
{
   EmcPose currentVel;
   EmcPose currentAccell;
   EmcPose desiredVel;
   EmcPose desiredAccell;
} EMC_TELEOP_DATA;

/* This enum lists all the possible commands */
typedef enum
{
   EMCMOT_ABORT = 1,            /* abort all motion */
   EMCMOT_AXIS_ABORT,   /* abort one axis *///FIXME-AJ: replace command name to EMCMOT_JOINT_ABORT
   EMCMOT_ENABLE,       /* enable servos for active joints */
   EMCMOT_DISABLE,      /* disable servos for active joints */
   EMCMOT_ENABLE_AMPLIFIER,     /* enable amp outputs */
   EMCMOT_DISABLE_AMPLIFIER,    /* disable amp outputs */
   EMCMOT_ENABLE_WATCHDOG,      /* enable watchdog sound, parport */
   EMCMOT_DISABLE_WATCHDOG,     /* enable watchdog sound, parport */
   EMCMOT_ACTIVATE_JOINT,       /* make joint active */
   EMCMOT_DEACTIVATE_JOINT,     /* make joint inactive */

   EMCMOT_PAUSE,        /* pause motion */
   EMCMOT_RESUME,       /* resume motion */
   EMCMOT_STEP, /* resume motion until id encountered */
   EMCMOT_FREE, /* set mode to free (joint) motion */
   EMCMOT_COORD,        /* set mode to coordinated motion */
   EMCMOT_TELEOP,       /* set mode to teleop */

   EMCMOT_SPINDLE_SCALE,        /* set scale factor for spindle speed */
   EMCMOT_SS_ENABLE,    /* enable/disable scaling the spindle speed */
   EMCMOT_FEED_SCALE,   /* set scale factor for feedrate */
   EMCMOT_FS_ENABLE,    /* enable/disable scaling feedrate */
   EMCMOT_FH_ENABLE,    /* enable/disable feed_hold */
   EMCMOT_AF_ENABLE,    /* enable/disable adaptive feedrate */
   EMCMOT_OVERRIDE_LIMITS,      /* temporarily ignore limits until jog done */

   EMCMOT_HOME, /* home a joint or all joints */
   EMCMOT_UNHOME,       /* unhome a joint or all joints */
   EMCMOT_JOG_CONT,     /* continuous jog */
   EMCMOT_JOG_INCR,     /* incremental jog */
   EMCMOT_JOG_ABS,      /* absolute jog */
   EMCMOT_SET_LINE,     /* queue up a linear move */
   EMCMOT_SET_CIRCLE,   /* queue up a circular move */
   EMCMOT_SET_TELEOP_VECTOR,    /* Move at a given velocity but in
                                   world cartesian coordinates, not
                                   in joint space like EMCMOT_JOG_ */

   EMCMOT_CLEAR_PROBE_FLAGS,    /* clears probeTripped flag */
   EMCMOT_PROBE,        /* go to pos, stop if probe trips, record
                           trip pos */
   EMCMOT_RIGID_TAP,    /* go to pos, with sync to spindle speed, 
                           then return to initial pos */

   EMCMOT_SET_POSITION_LIMITS,  /* set the joint position +/- limits */
   EMCMOT_SET_BACKLASH, /* set the joint backlash */
   EMCMOT_SET_MIN_FERROR,       /* minimum following error, input units */
   EMCMOT_SET_MAX_FERROR,       /* maximum following error, input units */
   EMCMOT_SET_VEL,      /* set the velocity for subsequent moves */
   EMCMOT_SET_VEL_LIMIT,        /* set the max vel for all moves (tooltip) */
   EMCMOT_SET_JOINT_VEL_LIMIT,  /* set the max joint vel */
   EMCMOT_SET_JOINT_ACC_LIMIT,  /* set the max joint accel */
   EMCMOT_SET_ACC,      /* set the max accel for moves (tooltip) */
   EMCMOT_SET_TERM_COND,        /* set termination condition (stop, blend) */
   EMCMOT_SET_NUM_AXES, /* set the number of joints *///FIXME-AJ: function needs to get renamed
   EMCMOT_SET_WORLD_HOME,       /* set pose for world home */
   EMCMOT_SET_HOMING_PARAMS,    /* sets joint homing parameters */
   EMCMOT_SET_DEBUG,    /* sets the debug level */
   EMCMOT_SET_DOUT,     /* sets or unsets a DIO, this can be imediate or synched with motion */
   EMCMOT_SET_AOUT,     /* sets or unsets a AIO, this can be imediate or synched with motion */
   EMCMOT_SET_SPINDLESYNC,      /* syncronize motion to spindle encoder */

   EMCMOT_SET_SPINDLE_VEL,      /* set the spindle vel (>0 means forward, <0 means backward) */
   EMCMOT_SPINDLE_ON,   /* start the spindle */
   EMCMOT_SPINDLE_OFF,  /* stop the spindle */
   EMCMOT_SPINDLE_INCREASE,     /* spindle faster */
   EMCMOT_SPINDLE_DECREASE,     /* spindle slower */
   EMCMOT_SPINDLE_BRAKE_ENGAGE, /* engage the spindle brake */
   EMCMOT_SPINDLE_BRAKE_RELEASE,        /* release the spindle brake */
   EMCMOT_SET_MOTOR_OFFSET,     /* set the offset between joint and motor */
   EMCMOT_SET_JOINT_COMP,       /* set a compensation triplet for a joint (nominal, forw., rev.) */
   EMCMOT_SET_OFFSET,   /* set tool offsets */
   EMCMOT_SET_INPUT_SCALE,      /* scale factor for inputs */
   EMCMOT_SET_STEP_PIN, /* rt-stepper DB25 pin number */
   EMCMOT_SET_DIRECTION_PIN,    /* rt-stepper DB25 pin number */
   EMCMOT_SET_STEP_POLARITY,    /* rt-stepper DB25 pin polarity */
   EMCMOT_SET_DIRECTION_POLARITY,    /* rt-stepper DB25 pin polarity */
   EMCMOT_ENABLE_DIN_ABORT,    /* enable rt-stepper input0-2 abort */
   EMCMOT_DISABLE_DIN_ABORT,    /* disable rt-stepper input0-2 abort */
   EMCMOT_SYSTEM_CMD,       /* exec user defined mcode script (m100-m199) */ 
} cmd_code_t;

/* this enum lists the possible results of a command */
typedef enum
{
   EMCMOT_COMMAND_OK = 0,       /* cmd honored */
   EMCMOT_COMMAND_UNKNOWN_COMMAND,      /* cmd not understood */
   EMCMOT_COMMAND_INVALID_COMMAND,      /* cmd can't be handled now */
   EMCMOT_COMMAND_INVALID_PARAMS,       /* bad cmd params */
   EMCMOT_COMMAND_BAD_EXEC      /* error trying to initiate */
} cmd_status_t;

/* termination conditions for queued motions */
#define EMCMOT_TERM_COND_STOP 1
#define EMCMOT_TERM_COND_BLEND 2

/*********************************
       COMMAND STRUCTURE
*********************************/

/* This is the command structure.  There is one of these in shared
   memory, and all commands from higher level code come thru it.
*/
typedef struct _emcmot_command_t
{
   unsigned char head;          /* flag count for mutex detect */
   cmd_code_t command;          /* command code (enum) */
   int commandNum;              /* increment this for new command */
   double motor_offset;         /* offset from joint to motor position */
   double maxLimit;             /* pos value for position limit, output */
   double minLimit;             /* neg value for position limit, output */
   EmcPose pos;                 /* line/circle endpt, or teleop vector */
   PmCartesian center;          /* center for circle */
   PmCartesian normal;          /* normal vec for circle */
   int turn;                    /* turns for circle */
   double vel;                  /* max velocity */
   double ini_maxvel;           /* max velocity allowed by machine
                                   constraints (the ini file) */
   int motion_type;             /* this move is because of traverse, feed, arc, or toolchange */
   double spindlesync;          /* user units per spindle revolution, 0 = no sync */
   double acc;                  /* max acceleration */
   double backlash;             /* amount of backlash */
   int id;                      /* id for motion */
   int termCond;                /* termination condition */
   double tolerance;            /* tolerance for path deviation in CONTINUOUS mode */
   int axis;                    /* which index to use for below *///FIXME-AJ: replace with joint
   double scale;                /* velocity scale or spindle_speed scale arg */
   double offset;               /* input, output, or home offset arg */
   double home;                 /* joint home position */
   double home_final_vel;       /* joint velocity for moving from OFFSET to HOME */
   double search_vel;           /* home search velocity */
   double latch_vel;            /* home latch velocity */
   int flags;                   /* homing config flags, other boolean args */
   int home_sequence;           /* order in homing sequence */
   int volatile_home;           /* joint should get unhomed when we get unhome -2 
                                   (generated by task upon estop, etc) */
   int pin;                     /* DB25 pin */
   int polarity;                /* DB25 pin polarity */
   double minFerror;            /* min following error */
   double maxFerror;            /* max following error */
   int wdWait;                  /* cycle to wait before toggling wd */
   int debug;                   /* debug level, from DEBUG in .ini file */
  //   unsigned char now, out, start, end;  /* these are related to synched AOUT/DOUT. now=wether now or synched, out = which gets set, start=start value, end=end value */
   unsigned char mode;          /* used for turning overrides etc. on/off */
   double comp_nominal, comp_forward, comp_reverse;     /* compensation triplet, nominal, forward, reverse */
   unsigned char probe_type;    /* ~1 = error if probe operation is unsuccessful (ngc default)
                                   |1 = suppress error, report in # instead
                                   ~2 = move until probe trips (ngc default)
                                   |2 = move until probe clears */
   EmcPose tool_offset;         /* TLO */
   unsigned char tail;          /* flag count for mutex detect */
   int index;                   /* user defined mcode */
   double p_number, q_number;   /* user defined mcode parameters */
   int output_num, value, sync;  /* dout parameters */
   int input_num;               /* din parameter */
   int ticket;                  /* 0 = no ticket (no gui response needed) */
} emcmot_command_t;

/* motion flag type */
typedef unsigned short EMCMOT_MOTION_FLAG;

/*
  motion status flag structure-- looks like:

  MSB                             LSB
  v---------------v------------------v
  |   |   |   | T | CE | C | IP | EN |
  ^---------------^------------------^

  where:

  EN is 1 if calculations are enabled, 0 if not
  IP is 1 if all joints in position, 0 if not
  C is 1 if coordinated mode, 0 if in free mode
  CE is 1 if coordinated mode error, 0 if not
  T is 1 if we are in teleop mode.
  */

/* bit masks */
#define EMCMOT_MOTION_ENABLE_BIT      0x0001
#define EMCMOT_MOTION_INPOS_BIT       0x0002
#define EMCMOT_MOTION_COORD_BIT       0x0004
#define EMCMOT_MOTION_ERROR_BIT       0x0008
#define EMCMOT_MOTION_TELEOP_BIT      0x0010

/* joint flag type */
typedef unsigned short EMCMOT_JOINT_FLAG;
/*
  joint status flag structure-- looks like:

  MSB                                                          LSB
  ----------v-----------------v--------------------v-------------------v
  | AF | FE | AH | HD | H | HS | NHL | PHL | - | - | ER | IP | AC | EN |
  ----------^-----------------^--------------------^-------------------^
               

  x = unused

  where:

  EN  is 1 if joint amplifier is enabled, 0 if not
  AC  is 1 if joint is active for calculations, 0 if not
  IP  is 1 if joint is in position, 0 if not (free mode only)
  ER  is 1 if joint has an error, 0 if not

  PHL is 1 if joint is on maximum hardware limit, 0 if not
  NHL is 1 if joint is on minimum hardware limit, 0 if not

  HS  is 1 if joint home switch is tripped, 0 if not
  H   is 1 if joint is homing, 0 if not
  HD  is 1 if joint has been homed, 0 if not
  AH  is 1 if joint is at home position, 0 if not

  FE  is 1 if joint exceeded following error, 0 if not
  AF  is 1 if amplifier is faulted, 0 if not

Suggestion: Split this in to an Error and a Status flag register..
             Then a simple test on each of the two flags can be performed
             rather than testing each bit... Saving on a global per joint
             fault and ready status flag.
  */

/* bit masks */
#define EMCMOT_JOINT_ENABLE_BIT         0x0001
#define EMCMOT_JOINT_ACTIVE_BIT         0x0002
#define EMCMOT_JOINT_INPOS_BIT          0x0004
#define EMCMOT_JOINT_ERROR_BIT          0x0008

#define EMCMOT_JOINT_MAX_HARD_LIMIT_BIT 0x0040
#define EMCMOT_JOINT_MIN_HARD_LIMIT_BIT 0x0080

#define EMCMOT_JOINT_HOME_SWITCH_BIT    0x0100
#define EMCMOT_JOINT_HOMING_BIT         0x0200
#define EMCMOT_JOINT_HOMED_BIT          0x0400

#define EMCMOT_JOINT_AT_HOME_BIT        0x0800

#define EMCMOT_JOINT_FERROR_BIT         0x1000
#define EMCMOT_JOINT_FAULT_BIT          0x2000

/* The terms "teleop", "coord", and "free" are poorly
   documented.  This is my feeble attempt to understand exactly
   what they mean.

   According to Fred, teleop is never used with machine tools,
   although that may not be true for machines with non-trivial
   kinematics.

   "coord", or coordinated mode, means that all the joints are
   synchronized, and move together as commanded by the higher
   level code.  It is the normal mode when machining.  In
   coordinated mode, commands are assumed to be in the cartesean
   reference frame, and if the machine is non-cartesean, the
   commands are translated by the kinematics to drive each
   joint in joint space as needed.

   "free" mode means commands are interpreted in joint space.
   It is used for jogging individual joints, although
   it does not preclude multiple joints moving at once (I think).
   Homing is also done in free mode, in fact machines with
   non-trivial kinematics must be homed before they can go
   into either coord or teleop mode.

   'teleop' is what you probably want if you are 'jogging'
   a hexapod.  The jog commands as implemented by the motion
   controller are joint jogs, which work in free mode.  But
   if you want to jog a hexapod or similar machine along
   one particular cartesean axis, you need to operate more
   than one joint.  That's what 'teleop' is for.

*/

/* compensation structures */
typedef struct
{
   double nominal;              /* nominal (command) position */
   float fwd_trim;              /* correction for forward movement */
   float rev_trim;              /* correction for reverse movement */
   float fwd_slope;             /* slopes between here and next pt */
   float rev_slope;
} emcmot_comp_entry_t;


#define EMCMOT_COMP_SIZE 256
typedef struct
{
   int entries;                 /* number of entries in the array */
   emcmot_comp_entry_t *entry;  /* current entry in array */
   emcmot_comp_entry_t array[EMCMOT_COMP_SIZE + 2];
   /* +2 because array has -HUGE_VAL and +HUGE_VAL entries at the ends */
} emcmot_comp_t;

/* motion controller states */
typedef enum
{
   EMCMOT_MOTION_DISABLED = 0,
   EMCMOT_MOTION_FREE,
   EMCMOT_MOTION_TELEOP,
   EMCMOT_MOTION_COORD
} motion_state_t;

/* states for homing */
typedef enum
{
   HOME_IDLE = 0,
   HOME_START,  // 1
   HOME_INITIAL_BACKOFF_START,  // 2
   HOME_INITIAL_BACKOFF_WAIT,   // 3
   HOME_INITIAL_SEARCH_START,   // 4
   HOME_INITIAL_SEARCH_WAIT,    // 5
   HOME_SET_COARSE_POSITION,    // 6
   HOME_FINAL_BACKOFF_START,    // 7
   HOME_FINAL_BACKOFF_WAIT,     // 8
   HOME_RISE_SEARCH_START,      // 9
   HOME_RISE_SEARCH_WAIT,       // 10
   HOME_FALL_SEARCH_START,      // 11
   HOME_FALL_SEARCH_WAIT,       // 12
   HOME_SET_SWITCH_POSITION,    // 13
   HOME_INDEX_ONLY_START,       // 14
   HOME_INDEX_SEARCH_START,     // 15
   HOME_INDEX_SEARCH_WAIT,      // 16
   HOME_SET_INDEX_POSITION,     // 17
   HOME_FINAL_MOVE_START,       // 18
   HOME_FINAL_MOVE_WAIT,        // 19
   HOME_FINISHED,       // 20
   HOME_ABORT   // 21
} home_state_t;

typedef enum
{
   HOME_SEQUENCE_IDLE = 0,
   HOME_SEQUENCE_START,
   HOME_SEQUENCE_START_JOINTS,
   HOME_SEQUENCE_WAIT_JOINTS,
} home_sequence_state_t;

/* flags for homing */
#define HOME_IGNORE_LIMITS      1
#define HOME_USE_INDEX          2
#define HOME_IS_SHARED          4

/* flags for enabling spindle scaling, feed scaling,
   adaptive feed, and feed hold */
#define SS_ENABLED 0x01
#define FS_ENABLED 0x02
#define AF_ENABLED 0x04
#define FH_ENABLED 0x08

typedef enum
{
   EMC_MOTION_TYPE_TRAVERSE = 1,
   EMC_MOTION_TYPE_FEED,
   EMC_MOTION_TYPE_ARC,
   EMC_MOTION_TYPE_TOOLCHANGE,
   EMC_MOTION_TYPE_PROBING,
} emc_motion_type_t;

typedef struct
{
   /* configuration info - changes rarely */
   int type;                    /* 0 = linear, 1 = rotary */
   double max_pos_limit;        /* upper soft limit on joint pos */
   double min_pos_limit;        /* lower soft limit on joint pos */
   double max_jog_limit;        /* jog limits change when not homed */
   double min_jog_limit;
   double vel_limit;            /* upper limit of joint speed */
   double acc_limit;            /* upper limit of joint accel */
   double min_ferror;           /* zero speed following error limit */
   double max_ferror;           /* max speed following error limit */
   double home_search_vel;      /* dir/spd to look for home switch */
   double home_final_vel;       /* speed to travel from OFFSET to HOME position */
   double home_latch_vel;       /* dir/spd to latch switch/index pulse */
   double home_offset;          /* dir/dist from switch to home point */
   double home;                 /* joint coordinate of home point */
   int home_flags;              /* flags for various homing options */
   int volatile_home;           /* joint should get unhomed when we get unhome -2 
                                   (generated by task upon estop, etc) */
   double backlash;             /* amount of backlash */
   int home_sequence;           /* Order in homing sequence */
   emcmot_comp_t comp;          /* leadscrew correction data */

   /* status info - changes regularly */
   /* many of these need to be made available to higher levels */
   /* they can either be copied to the status struct, or an array of
      joint structs can be made part of the status */
   EMCMOT_JOINT_FLAG flag;      /* see above for bit details */
   double coarse_pos;           /* trajectory point, before interp */
   double pos_cmd;              /* commanded joint position */
   double vel_cmd;              /* comanded joint velocity */
   double backlash_corr;        /* correction for backlash */
   double backlash_filt;        /* filtered backlash correction */
   double backlash_vel;         /* backlash velocity variable */
   double motor_pos_cmd;        /* commanded position, with comp */
   double motor_pos_fb;         /* position feedback, with comp */
   double pos_fb;               /* position feedback, comp removed */
   double ferror;               /* following error */
   double ferror_limit;         /* limit depends on speed */
   double ferror_high_mark;     /* max following error */
   double free_pos_cmd;         /* position command for free mode TP */
   double free_vel_lim;         /* velocity limit for free mode TP */
   int free_tp_enable;          /* if zero, joint stops ASAP */
   int free_tp_active;          /* if non-zero, move in progress */
   int kb_jog_active;           /* non-zero during a keyboard jog */
   int wheel_jog_active;        /* non-zero during a wheel jog */

   /* internal info - changes regularly, not usually accessed from user space */
   CUBIC_STRUCT cubic;          /* cubic interpolator data */

   int on_pos_limit;            /* non-zero if on limit */
   int on_neg_limit;            /* non-zero if on limit */
   double home_sw_pos;          /* latched position of home sw */
   int home_pause_timer;        /* used to delay between homing states */
   int index_enable;            /* current state of index enable pin */

   home_state_t home_state;     /* state machine for homing */
   double motor_offset;         /* diff between internal and motor pos, used
                                   to set position to zero during homing */
   int old_jog_counts;          /* prior value, used for deltas */

   /* stuff moved from the other structs that might be needed (or might not!) */
   double big_vel;              /* used for "debouncing" velocity */
} emcmot_joint_t;

typedef struct
{
   EMCMOT_JOINT_FLAG flag;      /* see above for bit details */
   double pos_cmd;              /* commanded joint position */
   double pos_fb;               /* position feedback, comp removed */
   double vel_cmd;              /* current velocity */
   double ferror;               /* following error */
   double ferror_high_mark;     /* max following error */

   /* Following come from the ini file. */
   double backlash;             /* amount of backlash */
   double max_pos_limit;        /* upper soft limit on joint pos */
   double min_pos_limit;        /* lower soft limit on joint pos */
   double min_ferror;           /* zero speed following error limit */
   double max_ferror;           /* max speed following error limit */
   double home_offset;          /* dir/dist from switch to home point */
} emcmot_joint_status_t;

/*********************************
        CONFIG STRUCTURE
*********************************/

typedef struct _emcmot_config_t
{
   unsigned char head;          /* flag count for mutex detect */

   int config_num;              /* Incremented everytime configuration
                                   changed, should match status.config_num */
   int numJoints;               /* The number of joints in the system (which
                                   must be between 1 and EMCMOT_MAX_JOINTS,
                                   inclusive). Allegedly, holds a copy of the
                                   global num_joints - seems daft to maintain
                                   duplicates ! */

   double trajCycleTime;        /* the rate at which the trajectory loop
                                   runs.... (maybe) */
   double servoCycleTime;       /* the rate of the servo loop - Not the same
                                   as the traj time */

   int interpolationRate;       /* grep control.c for an explanation....
                                   approx line 50 */

   double limitVel;             /* scalar upper limit on vel */
   int kinematics_type;
   int debug;                   /* copy of DEBUG, from .ini file */
   unsigned char tail;          /* flag count for mutex detect */
} emcmot_config_t;

/*********************************
        TRAJECTORY PLANNER STRUCTURE (not debug)
*********************************/

typedef struct emcmot_debug_t
{
   unsigned char head;          /* flag count for mutex detect */

   double tMin, tMax, tAvg;     /* trajectory min, max, avg times */
   double sMin, sMax, sAvg;     /* servo min, max, avg times */
   double nMin, nMax, nAvg;     /* min, max, avg times in DISABLED
                                   mode */
   double yMin, yMax, yAvg;     /* min, max, avg times cycle times
                                   rather than compute */
   double fMin, fMax, fAvg;     /* min, max, avg times frequency */
   double fyMin, fyMax, fyAvg;  /* min, max, avg times frequency
                                   cycle times rather than compute */

   EMC_TELEOP_DATA teleop_data;
   int split;                   /* number of split command reads */

   /* flag for enabling, disabling watchdog; multiple for down-stepping */
   int wdEnabling;
   int wdEnabled;
   int wdWait;
   int wdCount;
   unsigned char wdToggle;

   /* flag that all active axes are homed */
   unsigned char allHomed;

   TP_STRUCT queue;             /* coordinated mode planner */

   TC_STRUCT queueTcSpace[DEFAULT_TC_QUEUE_SIZE + 10];

   EmcPose oldPos;              /* last position, used for vel differencing */
   EmcPose oldVel, newVel;      /* velocities, used for acc differencing */
   EmcPose newAcc;              /* differenced acc */

   int enabling;                /* starts up disabled */
   int coordinating;            /* starts up in free mode */
   int teleoperating;           /* starts up in free mode */

   int overriding;              /* non-zero means we've initiated an joint
                                   move while overriding limits */

   int stepping;
   int idForStep;

   double start_time;
   double running_time;
   double cur_time;
   double last_time;
   unsigned char tail;          /* flag count for mutex detect */
} emcmot_debug_t;

/******************
   Task Structures
******************/

/* Size of certain arrays (same defines as rs274ngc.h) */
#define ACTIVE_G_CODES_ 16
#define ACTIVE_M_CODES_ 10
#define ACTIVE_SETTINGS_ 3

// types for EMC_TASK mode
enum EMC_TASK_MODE
{
   EMC_TASK_MODE_UNUSED = 0,
   EMC_TASK_MODE_MANUAL = 1,
   EMC_TASK_MODE_AUTO = 2,
   EMC_TASK_MODE_MDI = 3
};

// types for EMC_TASK state
enum EMC_TASK_STATE
{
   EMC_TASK_STATE_UNUSED = 0,
   EMC_TASK_STATE_ESTOP = 1,
   EMC_TASK_STATE_ESTOP_RESET = 2,
   EMC_TASK_STATE_OFF = 3,
   EMC_TASK_STATE_ON = 4
};

// types for EMC_TASK execState
enum EMC_TASK_EXEC
{
   EMC_TASK_EXEC_UNUSED = 0,
   EMC_TASK_EXEC_ERROR = 1,
   EMC_TASK_EXEC_DONE = 2,
   EMC_TASK_EXEC_WAITING_FOR_MOTION = 3,
   EMC_TASK_EXEC_WAITING_FOR_MOTION_QUEUE = 4,
   EMC_TASK_EXEC_WAITING_FOR_IO = 5,
   EMC_TASK_EXEC_WAITING_FOR_PAUSE = 6,
   EMC_TASK_EXEC_WAITING_FOR_MOTION_AND_IO = 7,
   EMC_TASK_EXEC_WAITING_FOR_DELAY = 8,
   EMC_TASK_EXEC_WAITING_FOR_SYSTEM_CMD = 9
};

// types for EMC_TASK interpState
enum EMC_TASK_INTERP
{
   EMC_TASK_INTERP_UNUSED = 0,
   EMC_TASK_INTERP_IDLE = 1,
   EMC_TASK_INTERP_READING = 2,
   EMC_TASK_INTERP_PAUSED = 3,
   EMC_TASK_INTERP_WAITING = 4
};

// types for EMC_TASK planState
enum EMC_TASK_PLAN
{
   EMC_TASK_PLAN_UNUSED = 0,
   EMC_TASK_PLAN_ERROR = 1,
   EMC_TASK_PLAN_DONE = 2,
};

// types for motion control
enum EMC_TRAJ_MODE
{
   EMC_TRAJ_MODE_UNUSED = 0,
   EMC_TRAJ_MODE_FREE = 1,      // independent-axis motion,
   EMC_TRAJ_MODE_COORD = 2,     // coordinated-axis motion,
   EMC_TRAJ_MODE_TELEOP = 3     // velocity based world coordinates motion,
};

#include "emc_msg.h"

typedef struct _emctask_status_t        // EMC_TASK_STAT
{
   enum RCS_STATUS status;
   unsigned int echo_serial_number;
   enum EMC_COMMAND_MSG_TYPE command_type;
   enum EMC_TASK_MODE mode;        // EMC_TASK_MODE_MANUAL, etc.
   enum EMC_TASK_STATE state;      // EMC_TASK_STATE_ESTOP, etc.
   enum EMC_TASK_EXEC execState;   // EMC_DONE,WAITING_FOR_MOTION, etc.
   enum EMC_TASK_INTERP interpState;       // EMC_IDLE,READING,PAUSED,WAITING
   enum EMC_TASK_PLAN planState;
   int motionLine;              // line motion is executing-- may lag
   int currentLine;             // line currently executing
   int readLine;                // line interpreter has read to
   int optional_stop_state;     // state of optional stop (== ON means we stop on M1)
   int block_delete_state;      // state of block delete (== ON means we ignore lines starting with "/")
   int input_timeout;           // has a timeout happened on digital input
   char file[LINELEN];
   char command[LINELEN];
   EmcPose origin;              // origin, in user units, currently active
   double rotation_xy;
   EmcPose toolOffset;          // tool offset, in general pose form
   int activeGCodes[ACTIVE_G_CODES_];
   int activeMCodes[ACTIVE_M_CODES_];
   double activeSettings[ACTIVE_SETTINGS_];
   int programUnits;            // CANON_UNITS_INCHES,MM,CM
   unsigned int heartbeat;      /* changes every cycle */

   int interpreter_errcode;     // return value from rs274ngc function 
   // (only useful for new interpreter.)
   int task_paused;             // non-zero means task is paused
   double delayLeft;            // delay time left of G4, M66..
} emctask_status_t;

/******************
   IO Structures
******************/

enum EMCIO_COMMAND
{
   EMCIO_IO_INIT_COMMAND = 1,
   EMCIO_TOOL_INIT_COMMAND,
   EMCIO_TOOL_HALT_COMMAND,
   EMCIO_TOOL_ABORT_COMMAND,
   EMCIO_TOOL_PREPARE_COMMAND,
   EMCIO_TOOL_LOAD_COMMAND,
   EMCIO_TOOL_UNLOAD_COMMAND,
   EMCIO_TOOL_LOAD_TOOL_TABLE_COMMAND,
   EMCIO_TOOL_SET_OFFSET_COMMAND,
   EMCIO_TOOL_SET_NUMBER_COMMAND,
   EMCIO_COOLANT_MIST_ON_COMMAND,
   EMCIO_COOLANT_MIST_OFF_COMMAND,
   EMCIO_COOLANT_FLOOD_ON_COMMAND,
   EMCIO_COOLANT_FLOOD_OFF_COMMAND,
   EMCIO_AUX_ESTOP_ON_COMMAND,
   EMCIO_AUX_ESTOP_OFF_COMMAND,
   EMCIO_AUX_ESTOP_RESET_COMMAND,
   EMCIO_LUBE_ON_COMMAND,
   EMCIO_LUBE_OFF_COMMAND,
   EMCIO_SET_DEBUG_COMMAND,
};

typedef struct
{
   enum EMCIO_COMMAND type;
   int serial_number;
   int tool;
   char file[LINELEN];
   int pocket;
   int toolno;
   EmcPose offset;
   double diameter;
   double frontangle;
   double backangle;
   int orientation;
} emcio_command_t;

typedef struct
{
   int pocketPrepped;           // pocket ready for loading from
   int toolInSpindle;           // tool loaded, 0 is no tool
   struct CANON_TOOL_TABLE toolTable[CANON_POCKETS_MAX];
} emctool_status_t;

typedef struct
{
   int mist;                    // 0 off, 1 on
   int flood;                   // 0 off, 1 on
} emccoolant_status_t;

typedef struct
{
   int estop;                   // non-zero means estopped
} emcaux_status_t;

typedef struct
{
   int on;                      // 0 off, 1 on
   int level;                   // 0 low, 1 okay
} emclube_status_t;

typedef struct _emcio_status_t  // EMC_IO_STAT
{
   enum RCS_STATUS status;
   unsigned int echo_serial_number;
   enum EMC_COMMAND_MSG_TYPE command_type;

   // aggregate of IO-related status classes
   emctool_status_t tool;
   emccoolant_status_t coolant;
   emcaux_status_t aux;
   emclube_status_t lube;
} emcio_status_t;

/******************
   Axis Structures
******************/

typedef struct _emcaxis_status_t        // EMC_AXIS_STAT
{
   enum RCS_STATUS status;
   unsigned int echo_serial_number;

   // configuration parameters
   unsigned char axisType;      // EMC_AXIS_LINEAR, EMC_AXIS_ANGULAR
   double units;                // units per mm, deg for linear, angular
   double backlash;
   double minPositionLimit;
   double maxPositionLimit;
   double maxFerror;
   double minFerror;

   // dynamic status
   double ferrorCurrent;        // current following error
   double ferrorHighMark;       // magnitude of max following error

   double output;               // commanded output position
   double input;                // current input position
   double velocity;             // current velocity
   unsigned char inpos;         // non-zero means in position
   unsigned char homing;        // non-zero means homing
   unsigned char homed;         // non-zero means has been homed
   unsigned char fault;         // non-zero means axis amp fault
   unsigned char enabled;       // non-zero means enabled
   unsigned char minSoftLimit;  // non-zero means min soft limit exceeded
   unsigned char maxSoftLimit;  // non-zero means max soft limit exceeded
   unsigned char minHardLimit;  // non-zero means min hard limit exceeded
   unsigned char maxHardLimit;  // non-zero means max hard limit exceeded
   unsigned char overrideLimits;        // non-zero means limits are overridden
} emcaxis_status_t;

/*******************
   Motion Structures
********************/

typedef struct _emctraj_status_t        // EMC_TRAJ_STAT
{
   enum RCS_STATUS status;
   unsigned int echo_serial_number;

   double linearUnits;          // units per mm
   double angularUnits;         // units per degree
   double cycleTime;            // cycle time, in seconds
   int axes;                    // maximum axis number
   int axis_mask;               // mask of axes actually present
   enum EMC_TRAJ_MODE mode;        // EMC_TRAJ_MODE_FREE,
   // EMC_TRAJ_MODE_COORD
   int enabled;                 // non-zero means enabled

   int inpos;                   // non-zero means in position
   int queue;                   // number of pending motions, counting
   // current
   int activeQueue;             // number of motions blending
   int queueFull;               // non-zero means can't accept another motion
   int id;                      // id of the currently executing motion
   int paused;                  // non-zero means motion paused
   double scale;                // velocity scale factor
   double spindle_scale;        // spindle velocity scale factor

   EmcPose position;            // current commanded position
   EmcPose actualPosition;      // current actual position, from forward kins
   double velocity;             // system velocity, for subsequent motions
   double acceleration;         // system acceleration, for subsequent
   // motions
   double maxVelocity;          // max system velocity
   double maxAcceleration;      // system acceleration

   EmcPose probedPosition;      // last position where probe was tripped.
   int probe_tripped;           // Has the probe been tripped since the last
   // clear.
   int probing;                 // Are we currently looking for a probe
   // signal.
   int probeval;                // Current value of probe input.
   int kinematics_type;         // identity=1,serial=2,parallel=3,custom=4
   int motion_type;
   double distance_to_go;       // in current move
   EmcPose dtg;
   double current_vel;          // in current move
   int feed_override_enabled;
   int spindle_override_enabled;
   int adaptive_feed_enabled;
   int feed_hold_enabled;
} emctraj_status_t;

typedef struct _emcspindle_status_t
{
   double speed;                // spindle speed in RPMs
   int direction;               // 0 stopped, 1 forward, -1 reverse
   int brake;                   // 0 released, 1 engaged
   int increasing;              // 1 increasing, -1 decreasing, 0 neither
   int enabled;                 // non-zero means enabled
   double css_factor;
   double xoffset;
} emcspindle_status_t;

enum EMC_DIN_STATE
{
   EMC_DIN_FALSE = 0,
   EMC_DIN_TRUE = 1,
};

typedef struct _emcdout_status_t
{
   int active;         // 0=no, 1=yes
   int output_num;     // output0-7
   int value;        // 0=false, 1=true
   int sync;         // 0=output immediately, 1=output with move command
} emcdout_status_t;

typedef struct _emcmot_status_t
{
   enum RCS_STATUS status;

   // aggregate of motion-related status classes
   emctraj_status_t traj;
   emcaxis_status_t axis[EMCMOT_MAX_AXIS];  /* used by emcStatus but not emcmotStatus */

   unsigned char head;          /* flag count for mutex detect */

   /* these three are updated only when a new command is handled */
   cmd_code_t commandEcho;      /* echo of input command */
   int commandNumEcho;          /* echo of input command number */
   cmd_status_t commandStatus;  /* result of most recent command */

   /* these are config info, updated when a command changes them */
   double feed_scale;           /* velocity scale factor for all motion */
   double spindle_scale;        /* velocity scale factor for spindle speed */
   unsigned char enables_new;   /* flags for FS, SS, etc */

   /* the rest are updated every cycle */
   double net_feed_scale;       /* net scale factor for all motion */
   double net_spindle_scale;    /* net scale factor for spindle */
   unsigned char enables_queued;        /* flags for FS, SS, etc */

   motion_state_t motion_state; /* operating state: FREE, COORD, etc. */
   EMCMOT_MOTION_FLAG motionFlag;       /* see above for bit details */
   EmcPose carte_pos_cmd;       /* commanded Cartesian position */
   int carte_pos_cmd_ok;        /* non-zero if command is valid */
   EmcPose carte_pos_fb;        /* actual Cartesian position */
   int carte_pos_fb_ok;         /* non-zero if feedback is valid */
   EmcPose world_home;          /* cartesean coords of home position */
   int homing_active;           /* non-zero if any joint is homing */
   home_sequence_state_t homingSequenceState;
   emcmot_joint_status_t joint_status[EMCMOT_MAX_JOINTS];       /* all joint status data */

   int on_soft_limit;           /* non-zero if any joint is on soft limit */

   int probeVal;                /* debounced value of probe input */

   int probeTripped;            /* Has the probe signal changed since start
                                   of probe command? */
   int probing;                 /* Currently looking for a probe signal? */
   unsigned char probe_type;
   EmcPose probedPos;           /* Axis positions stored as soon as possible
                                   after last probeTripped */
   int spindle_index_enable;    /* hooked to a canon encoder index-enable */
   int spindleSync;             /* we are doing spindle-synced motion */
   double spindleRevs;          /* position of spindle in revolutions */
   double spindleSpeedIn;       /* velocity of spindle in revolutions per minute */
   int spindle_is_atspeed;      /* hal input */
   int atspeed_next_feed;       /* at next feed move, wait for spindle to be at speed  */
   unsigned char tail;          /* flag count for mutex detect */
   unsigned int heartbeat;      /* changes every cycle */

   int config_num;              /* incremented whenever configuration changed. */
   int id;                      /* id for executing motion */
   int depth;                   /* motion queue depth */
   int activeDepth;             /* depth of active blend elements */
   int queueFull;               /* Flag to indicate the tc queue is full */
   int paused;                  /* Flag to signal motion paused */
   int overrideLimitMask;       /* non-zero means one or more limits ignored */
   /* 1 << (joint-num*2) = ignore neg limit */
   /* 2 << (joint-num*2) = ignore pos limit */

   /* static status-- only changes upon input commands, e.g., config */
   double vel;                  /* scalar max vel */
   double acc;                  /* scalar max accel */

   int motionType;
   double distance_to_go;       /* in this move */
   EmcPose dtg;
   double current_vel;
   double requested_vel;
   unsigned int tcqlen;
   EmcPose tool_offset;

   emcspindle_status_t spindle;
   emcdout_status_t dout;

   int synch_di[EMCMOT_MAX_DIO];        // motion inputs queried by interp
   int synch_do[EMCMOT_MAX_DIO];        // motion outputs queried by interp
   double analog_input[EMCMOT_MAX_AIO]; //motion analog inputs queried by interp
   double analog_output[EMCMOT_MAX_AIO];        //motion analog outputs queried by interp
   int debug;                   // copy of EMC_DEBUG global
} emcmot_status_t;

/* macros for reading, writing bit flags */

/* motion flags */
#define GET_MOTION_ERROR_FLAG() (emcmotStatus.motionFlag & EMCMOT_MOTION_ERROR_BIT ? 1 : 0)
#define SET_MOTION_ERROR_FLAG(fl) if (fl) emcmotStatus.motionFlag |= EMCMOT_MOTION_ERROR_BIT; else emcmotStatus.motionFlag &= ~EMCMOT_MOTION_ERROR_BIT;
#define GET_MOTION_COORD_FLAG() (emcmotStatus.motionFlag & EMCMOT_MOTION_COORD_BIT ? 1 : 0)
#define SET_MOTION_COORD_FLAG(fl) if (fl) emcmotStatus.motionFlag |= EMCMOT_MOTION_COORD_BIT; else emcmotStatus.motionFlag &= ~EMCMOT_MOTION_COORD_BIT;
#define GET_MOTION_TELEOP_FLAG() (emcmotStatus.motionFlag & EMCMOT_MOTION_TELEOP_BIT ? 1 : 0)
#define SET_MOTION_TELEOP_FLAG(fl) if (fl) emcmotStatus.motionFlag |= EMCMOT_MOTION_TELEOP_BIT; else emcmotStatus.motionFlag &= ~EMCMOT_MOTION_TELEOP_BIT;
#define GET_MOTION_INPOS_FLAG() (emcmotStatus.motionFlag & EMCMOT_MOTION_INPOS_BIT ? 1 : 0)
#define SET_MOTION_INPOS_FLAG(fl) if (fl) emcmotStatus.motionFlag |= EMCMOT_MOTION_INPOS_BIT; else emcmotStatus.motionFlag &= ~EMCMOT_MOTION_INPOS_BIT;
#define GET_MOTION_ENABLE_FLAG() (emcmotStatus.motionFlag & EMCMOT_MOTION_ENABLE_BIT ? 1 : 0)
#define SET_MOTION_ENABLE_FLAG(fl) if (fl) emcmotStatus.motionFlag |= EMCMOT_MOTION_ENABLE_BIT; else emcmotStatus.motionFlag &= ~EMCMOT_MOTION_ENABLE_BIT;

/* joint flags */
#define GET_JOINT_ENABLE_FLAG(joint) ((joint)->flag & EMCMOT_JOINT_ENABLE_BIT ? 1 : 0)
#define SET_JOINT_ENABLE_FLAG(joint,fl) if (fl) (joint)->flag |= EMCMOT_JOINT_ENABLE_BIT; else (joint)->flag &= ~EMCMOT_JOINT_ENABLE_BIT;
#define GET_JOINT_ACTIVE_FLAG(joint) ((joint)->flag & EMCMOT_JOINT_ACTIVE_BIT ? 1 : 0)
#define SET_JOINT_ACTIVE_FLAG(joint,fl) if (fl) (joint)->flag |= EMCMOT_JOINT_ACTIVE_BIT; else (joint)->flag &= ~EMCMOT_JOINT_ACTIVE_BIT;
#define GET_JOINT_INPOS_FLAG(joint) ((joint)->flag & EMCMOT_JOINT_INPOS_BIT ? 1 : 0)
#define SET_JOINT_INPOS_FLAG(joint,fl) if (fl) (joint)->flag |= EMCMOT_JOINT_INPOS_BIT; else (joint)->flag &= ~EMCMOT_JOINT_INPOS_BIT;
#define GET_JOINT_ERROR_FLAG(joint) ((joint)->flag & EMCMOT_JOINT_ERROR_BIT ? 1 : 0)
#define SET_JOINT_ERROR_FLAG(joint,fl) if (fl) (joint)->flag |= EMCMOT_JOINT_ERROR_BIT; else (joint)->flag &= ~EMCMOT_JOINT_ERROR_BIT;
#define GET_JOINT_PHL_FLAG(joint) ((joint)->flag & EMCMOT_JOINT_MAX_HARD_LIMIT_BIT ? 1 : 0)
#define SET_JOINT_PHL_FLAG(joint,fl) if (fl) (joint)->flag |= EMCMOT_JOINT_MAX_HARD_LIMIT_BIT; else (joint)->flag &= ~EMCMOT_JOINT_MAX_HARD_LIMIT_BIT;
#define GET_JOINT_NHL_FLAG(joint) ((joint)->flag & EMCMOT_JOINT_MIN_HARD_LIMIT_BIT ? 1 : 0)
#define SET_JOINT_NHL_FLAG(joint,fl) if (fl) (joint)->flag |= EMCMOT_JOINT_MIN_HARD_LIMIT_BIT; else (joint)->flag &= ~EMCMOT_JOINT_MIN_HARD_LIMIT_BIT;
#define GET_JOINT_HOME_SWITCH_FLAG(joint) ((joint)->flag & EMCMOT_JOINT_HOME_SWITCH_BIT ? 1 : 0)
#define SET_JOINT_HOME_SWITCH_FLAG(joint,fl) if (fl) (joint)->flag |= EMCMOT_JOINT_HOME_SWITCH_BIT; else (joint)->flag &= ~EMCMOT_JOINT_HOME_SWITCH_BIT;
#define GET_JOINT_HOMING_FLAG(joint) ((joint)->flag & EMCMOT_JOINT_HOMING_BIT ? 1 : 0)
#define SET_JOINT_HOMING_FLAG(joint,fl) if (fl) (joint)->flag |= EMCMOT_JOINT_HOMING_BIT; else (joint)->flag &= ~EMCMOT_JOINT_HOMING_BIT;
#define GET_JOINT_HOMED_FLAG(joint) ((joint)->flag & EMCMOT_JOINT_HOMED_BIT ? 1 : 0)
#define SET_JOINT_HOMED_FLAG(joint,fl) if (fl) (joint)->flag |= EMCMOT_JOINT_HOMED_BIT; else (joint)->flag &= ~EMCMOT_JOINT_HOMED_BIT;
#define GET_JOINT_AT_HOME_FLAG(joint) ((joint)->flag & EMCMOT_JOINT_AT_HOME_BIT ? 1 : 0)
#define SET_JOINT_AT_HOME_FLAG(joint,fl) if (fl) (joint)->flag |= EMCMOT_JOINT_AT_HOME_BIT; else (joint)->flag &= ~EMCMOT_JOINT_AT_HOME_BIT;
#define GET_JOINT_FERROR_FLAG(joint) ((joint)->flag & EMCMOT_JOINT_FERROR_BIT ? 1 : 0)
#define SET_JOINT_FERROR_FLAG(joint,fl) if (fl) (joint)->flag |= EMCMOT_JOINT_FERROR_BIT; else (joint)->flag &= ~EMCMOT_JOINT_FERROR_BIT;
#define GET_JOINT_FAULT_FLAG(joint) ((joint)->flag & EMCMOT_JOINT_FAULT_BIT ? 1 : 0)
#define SET_JOINT_FAULT_FLAG(joint,fl) if (fl) (joint)->flag |= EMCMOT_JOINT_FAULT_BIT; else (joint)->flag &= ~EMCMOT_JOINT_FAULT_BIT;

/***********************
   Top Level Status Structures
************************/

typedef struct
{
   enum RCS_STATUS status;
   unsigned int echo_serial_number;
   enum EMC_COMMAND_MSG_TYPE command_type;

   /* top level status */
   emctask_status_t task;       // EMC_TASK_STAT

   /* subordinate status */
   emcmot_status_t motion;      // EMC_MOTION_STAT
   emcio_status_t io;           // EMC_IO_STAT
} emc_status_t;                 // EMC_STAT

#endif /* MOTION_H */
