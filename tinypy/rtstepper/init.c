/*****************************************************************************\

  init.c - tinypy module for rtstepperemc

  (c) 2013 Copyright Eckler Software

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
#include "tinypy.h"
#include "emc.h"
#include "bug.h"

static tp_obj r_timeout;
static tp_obj r_error;
static tp_obj r_ok;

static tp_obj state_unused;
static tp_obj state_estop;
static tp_obj state_estop_reset;
static tp_obj state_off;
static tp_obj state_on;

static tp_obj mode_unused;
static tp_obj mode_manual;
static tp_obj mode_auto;
static tp_obj mode_mdi;

static tp_obj output0;
static tp_obj output1;
static tp_obj output2;
static tp_obj output3;
static tp_obj output4;
static tp_obj output5;
static tp_obj output6;
static tp_obj output7;

static tp_obj _dout(struct tp_vm *tp)
{
   enum EMC_RESULT stat;
   tp_obj self = tp_get(tp, tp->modules, tp_string("rtstepper"));
   struct emc_session *ps = tp_get(tp, self, tp_string("__session")).data.val;
   enum RTSTEPPER_GPO_BIT num = TP_NUM();
   int value = TP_NUM();
   stat = emc_ui_dout(ps, num, value, 0);
   return tp_number(stat);
}

static tp_obj _get_ini_key_value(struct tp_vm *tp)
{
   char buf[LINELEN];
   tp_obj self = tp_get(tp, tp->modules, tp_string("rtstepper"));
   struct emc_session *ps = tp_get(tp, self, tp_string("__session")).data.val;
   const char *section = TP_STR();
   const char *key = TP_STR();
   int len = emc_ui_get_ini_key_value(ps, section, key, buf, sizeof(buf)); 
   if (len < 0)
      len = 0;
   tp_obj r = tp_string_t(tp, len);  /* allocate buf */
   strcpy(r.string.info->s, buf); 
   return tp_track(tp, r);  /* track buf recovery */
}

static tp_obj _jog_abs(struct tp_vm *tp)
{
   enum EMC_RESULT stat;
   tp_obj self = tp_get(tp, tp->modules, tp_string("rtstepper"));
   struct emc_session *ps = tp_get(tp, self, tp_string("__session")).data.val;
   int axis = TP_NUM();
   double speed = TP_NUM();
   double pos = TP_NUM();
   stat = emc_ui_jog_abs(ps, axis, speed, pos);  
   return tp_number(stat);
}

static tp_obj _wait_command_done(struct tp_vm *tp)
{
   enum EMC_RESULT stat;
   tp_obj self = tp_get(tp, tp->modules, tp_string("rtstepper"));
   struct emc_session *ps = tp_get(tp, self, tp_string("__session")).data.val;
   double timout = TP_NUM();
   stat = emc_ui_wait_command_done(ps, timout);
   return tp_number(stat);
}

static tp_obj _wait_io_done(struct tp_vm *tp)
{
   enum EMC_RESULT stat;
   tp_obj self = tp_get(tp, tp->modules, tp_string("rtstepper"));
   struct emc_session *ps = tp_get(tp, self, tp_string("__session")).data.val;
   double timout = TP_NUM();
   stat = emc_ui_wait_io_done(ps, timout);
   return tp_number(stat);
}

static tp_obj _get_command_state(struct tp_vm *tp)
{
   enum EMC_TASK_STATE state;
   tp_obj self = tp_get(tp, tp->modules, tp_string("rtstepper"));
   struct emc_session *ps = tp_get(tp, self, tp_string("__session")).data.val;
   state = emc_ui_get_task_state(ps);  
   return tp_number(state);
}

static tp_obj _get_command_mode(struct tp_vm *tp)
{
   enum EMC_TASK_MODE mode;
   tp_obj self = tp_get(tp, tp->modules, tp_string("rtstepper"));
   struct emc_session *ps = tp_get(tp, self, tp_string("__session")).data.val;
   mode = emc_ui_get_task_mode(ps);  
   return tp_number(mode);
}

static tp_obj _manual_mode(struct tp_vm *tp)
{
   enum EMC_RESULT stat;
   tp_obj self = tp_get(tp, tp->modules, tp_string("rtstepper"));
   struct emc_session *ps = tp_get(tp, self, tp_string("__session")).data.val;
   stat = emc_ui_manual_mode(ps);
   return tp_number(stat);
}

static tp_obj _auto_mode(struct tp_vm *tp)
{
   enum EMC_RESULT stat;
   tp_obj self = tp_get(tp, tp->modules, tp_string("rtstepper"));
   struct emc_session *ps = tp_get(tp, self, tp_string("__session")).data.val;
   stat = emc_ui_auto_mode(ps);
   return tp_number(stat);
}

static tp_obj _mdi_mode(struct tp_vm *tp)
{
   enum EMC_RESULT stat;
   tp_obj self = tp_get(tp, tp->modules, tp_string("rtstepper"));
   struct emc_session *ps = tp_get(tp, self, tp_string("__session")).data.val;
   stat = emc_ui_mdi_mode(ps);
   return tp_number(stat);
}

static tp_obj _get_din_state(struct tp_vm *tp)
{
   enum EMC_DIN_STATE state;
   tp_obj self = tp_get(tp, tp->modules, tp_string("rtstepper"));
   struct emc_session *ps = tp_get(tp, self, tp_string("__session")).data.val;
   int input_num = TP_NUM();
   state = emc_ui_get_din_state(ps, input_num);  
   return tp_number(state);
}

static tp_obj _enable_din_abort(struct tp_vm *tp)
{
   enum EMC_RESULT stat;
   tp_obj self = tp_get(tp, tp->modules, tp_string("rtstepper"));
   struct emc_session *ps = tp_get(tp, self, tp_string("__session")).data.val;
   int input_num = TP_NUM();
   stat = emc_ui_enable_din_abort(ps, input_num);
   return tp_number(stat);
}

static tp_obj _disable_din_abort(struct tp_vm *tp)
{
   enum EMC_RESULT stat;
   tp_obj self = tp_get(tp, tp->modules, tp_string("rtstepper"));
   struct emc_session *ps = tp_get(tp, self, tp_string("__session")).data.val;
   int input_num = TP_NUM();
   stat = emc_ui_disable_din_abort(ps, input_num);
   return tp_number(stat);
}

static tp_obj _home(struct tp_vm *tp)
{
   enum EMC_RESULT stat;
   tp_obj self = tp_get(tp, tp->modules, tp_string("rtstepper"));
   struct emc_session *ps = tp_get(tp, self, tp_string("__session")).data.val;
   int axis = TP_NUM();
   stat = emc_ui_home(ps, axis);
   return tp_number(stat);
}

static tp_obj _estop_reset(struct tp_vm *tp)
{
   enum EMC_RESULT stat;
   tp_obj self = tp_get(tp, tp->modules, tp_string("rtstepper"));
   struct emc_session *ps = tp_get(tp, self, tp_string("__session")).data.val;
   stat = emc_ui_estop_reset(ps);
   return tp_number(stat);
}

static tp_obj _machine_on(struct tp_vm *tp)
{
   enum EMC_RESULT stat;
   tp_obj self = tp_get(tp, tp->modules, tp_string("rtstepper"));
   struct emc_session *ps = tp_get(tp, self, tp_string("__session")).data.val;
   stat = emc_ui_machine_on(ps);
   return tp_number(stat);
}

static tp_obj _estop(struct tp_vm *tp)
{
   enum EMC_RESULT stat;
   tp_obj self = tp_get(tp, tp->modules, tp_string("rtstepper"));
   struct emc_session *ps = tp_get(tp, self, tp_string("__session")).data.val;
   stat = emc_ui_estop(ps);
   return tp_number(stat);
}

static tp_obj _abort(struct tp_vm *tp)
{
   enum EMC_RESULT stat;
   tp_obj self = tp_get(tp, tp->modules, tp_string("rtstepper"));
   struct emc_session *ps = tp_get(tp, self, tp_string("__session")).data.val;
   stat = emc_ui_abort(ps);
   return tp_number(stat);
}

static tp_obj _operator_message(struct tp_vm *tp)
{
   enum EMC_RESULT stat;
   tp_obj self = tp_get(tp, tp->modules, tp_string("rtstepper"));
   struct emc_session *ps = tp_get(tp, self, tp_string("__session")).data.val;
   const char *msg = TP_STR();
   stat = emc_ui_operator_message(ps, msg);
   DBG("%s", msg);
   return tp_number(stat);
}

static tp_obj _log(struct tp_vm *tp)
{
   const char *msg = TP_STR();
   BUG("%s", msg);
   return tp_None;
}

void rtstepperpy_init(struct tp_vm *tp)
{
    struct emc_session *ps = &session;
    tp_obj rtstepper_mod = tp_dict(tp);

    /* bind rtstepper result codes */
    r_timeout = tp_number(EMC_R_TIMEOUT);
    r_error = tp_number(EMC_R_ERROR);
    r_ok = tp_number(EMC_R_OK);
    tp_set(tp, rtstepper_mod, tp_string("R_TIMEOUT"), r_timeout);
    tp_set(tp, rtstepper_mod, tp_string("R_ERROR"), r_error);
    tp_set(tp, rtstepper_mod, tp_string("R_OK"), r_ok);

    /* command state codes */
    state_unused = tp_number(EMC_TASK_STATE_UNUSED);
    state_estop = tp_number(EMC_TASK_STATE_ESTOP);
    state_estop_reset = tp_number(EMC_TASK_STATE_ESTOP_RESET);
    state_off = tp_number(EMC_TASK_STATE_OFF);
    state_on = tp_number(EMC_TASK_STATE_ON);
    tp_set(tp, rtstepper_mod, tp_string("STATE_UNUSED"), state_unused);
    tp_set(tp, rtstepper_mod, tp_string("STATE_ESTOP"), state_estop);
    tp_set(tp, rtstepper_mod, tp_string("STATE_ESTOP_RESET"), state_estop_reset);
    tp_set(tp, rtstepper_mod, tp_string("STATE_OFF"), state_off);
    tp_set(tp, rtstepper_mod, tp_string("STATE_ON"), state_on);

    /* command mode codes */
    mode_unused = tp_number(EMC_TASK_MODE_UNUSED);
    mode_manual = tp_number(EMC_TASK_MODE_MANUAL);
    mode_auto = tp_number(EMC_TASK_MODE_AUTO);
    mode_mdi = tp_number(EMC_TASK_MODE_MDI);
    tp_set(tp, rtstepper_mod, tp_string("MODE_UNUSED"), mode_unused);
    tp_set(tp, rtstepper_mod, tp_string("MODE_MANUAL"), mode_manual);
    tp_set(tp, rtstepper_mod, tp_string("MODE_AUTO"), mode_auto);
    tp_set(tp, rtstepper_mod, tp_string("MODE_MDI"), mode_mdi);

    /* bind OUTPUTn definitions */
    output0 = tp_number(0);
    output1 = tp_number(1);
    output2 = tp_number(2);
    output3 = tp_number(3);
    output4 = tp_number(4);
    output5 = tp_number(5);
    output6 = tp_number(6);
    output7 = tp_number(7);
    tp_set(tp, rtstepper_mod, tp_string("OUTPUT0"), output0);
    tp_set(tp, rtstepper_mod, tp_string("OUTPUT1"), output1);
    tp_set(tp, rtstepper_mod, tp_string("OUTPUT2"), output2);
    tp_set(tp, rtstepper_mod, tp_string("OUTPUT3"), output3);
    tp_set(tp, rtstepper_mod, tp_string("OUTPUT4"), output4);
    tp_set(tp, rtstepper_mod, tp_string("OUTPUT5"), output5);
    tp_set(tp, rtstepper_mod, tp_string("OUTPUT6"), output6);
    tp_set(tp, rtstepper_mod, tp_string("OUTPUT7"), output7);

    /* bind rtstepper functions to rtstepper module */
    tp_set(tp, rtstepper_mod, tp_string("dout"), tp_fnc(tp, _dout));
    tp_set(tp, rtstepper_mod, tp_string("get_ini_key_value"), tp_fnc(tp, _get_ini_key_value));
    tp_set(tp, rtstepper_mod, tp_string("jog_abs"), tp_fnc(tp, _jog_abs));
    tp_set(tp, rtstepper_mod, tp_string("wait_command_done"), tp_fnc(tp, _wait_command_done));
    tp_set(tp, rtstepper_mod, tp_string("wait_io_done"), tp_fnc(tp, _wait_io_done));
    tp_set(tp, rtstepper_mod, tp_string("get_command_state"), tp_fnc(tp, _get_command_state));
    tp_set(tp, rtstepper_mod, tp_string("get_command_mode"), tp_fnc(tp, _get_command_mode));
    tp_set(tp, rtstepper_mod, tp_string("manual_mode"), tp_fnc(tp, _manual_mode));
    tp_set(tp, rtstepper_mod, tp_string("auto_mode"), tp_fnc(tp, _auto_mode));
    tp_set(tp, rtstepper_mod, tp_string("mdi_mode"), tp_fnc(tp, _mdi_mode));
    tp_set(tp, rtstepper_mod, tp_string("home"), tp_fnc(tp, _home));
    tp_set(tp, rtstepper_mod, tp_string("get_din_state"), tp_fnc(tp, _get_din_state));
    tp_set(tp, rtstepper_mod, tp_string("enable_din_abort"), tp_fnc(tp, _enable_din_abort));
    tp_set(tp, rtstepper_mod, tp_string("disable_din_abort"), tp_fnc(tp, _disable_din_abort));
    tp_set(tp, rtstepper_mod, tp_string("estop_reset"), tp_fnc(tp, _estop_reset));
    tp_set(tp, rtstepper_mod, tp_string("machine_on"), tp_fnc(tp, _machine_on));
    tp_set(tp, rtstepper_mod, tp_string("estop"), tp_fnc(tp, _estop));
    tp_set(tp, rtstepper_mod, tp_string("abort"), tp_fnc(tp, _abort));
    tp_set(tp, rtstepper_mod, tp_string("log"), tp_fnc(tp, _log));
    tp_set(tp, rtstepper_mod, tp_string("operator_message"), tp_fnc(tp, _operator_message));

    /* bind special attributes to rtstepper module */
    tp_set(tp, rtstepper_mod, tp_string("__doc__"), 
    tp_string("This module is always available. It provides access to the\n"
                "rtstepperemc functions which control the rtstepper dongle."));
    tp_set(tp, rtstepper_mod, tp_string("__name__"), tp_string("rtstepper"));
    tp_set(tp, rtstepper_mod, tp_string("__file__"), tp_string(__FILE__));
    tp_set(tp, rtstepper_mod, tp_string("__session"), tp_data(tp, 0, ps));

    /* bind to tinypy modules[] */
    tp_set(tp, tp->modules, tp_string("rtstepper"), rtstepper_mod);
}

