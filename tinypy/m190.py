#
# m190.py - m190 mcode script home specified axis using limit switch.
# 
#     WARNING!!! do NOT run this script without limits switches
#
# inputs:
#  path = ARGV[0]   # this python file name
#  din = ARGV[1]   # digital input number (0 = INPUT0, 1 = INPUT1, 2 = INPUT2)
#  axis = ARGV[2]   # axis number (0 = x, 1 = y, 2 = z)
#
# returns:
#  emc.h result code
#
# example:
#  m190
#
import rtstepper

state_map = ["EMC_TASK_STATE_UNUSED", "EMC_TASK_STATE_ESTOP", "EMC_TASK_STATE_ESTOP_RESET", "EMC_TASK_STATE_OFF", "EMC_TASK_STATE_ON"]
din_map = ["INPUT0_ABORT", "INPUT1_ABORT", "INPUT2_ABORT"]
axis_section = ["axis_0", "axis_1", "axis_2"]
axis_limit = ["min_limit", "max_limit", "min_limit"]

def main():

   din = int(ARGV[1])
   axis = int(ARGV[2])

   if (din < 0 or din > len(din_map)):
      rtstepper.operator_message("invalid din=" + str(din))
      rtstepper.abort()
      return  # bail

   if (axis < 0 or axis > len(axis_section)):
      rtstepper.operator_message("invalid axis=" + str(axis))
      rtstepper.abort()
      return  # bail

   # Safety check, see if INPUTx will trigger estop in .ini file.
   key = din_map[din]
   val = rtstepper.get_ini_key_value("task", key)
   if (val=="0"):
      rtstepper.operator_message("invalid " + key + "=" + val)
      rtstepper.abort()
      return  # bail

   # Get soft limit for this axis.
   dist = float(rtstepper.get_ini_key_value(axis_section[axis], axis_limit[axis]))

   # Get rapid feed rate.
   speed = float(rtstepper.get_ini_key_value(axis_section[axis], "max_velocity"))

   # Convert units/second to units/minute. */
   speed = speed * 60

   # Set manual mode.
   old_mode = rtstepper.get_command_mode()
   rtstepper.manual_mode()

   # Move to the soft limit, assumes limit switch is before the soft limit.
   rtstepper.jog_abs(axis, speed, dist)
   rtstepper.wait_io_done(0.0)

   # Make sure we hit estop.
   if (rtstepper.get_command_state() != rtstepper.STATE_ESTOP):
      rtstepper.operator_message("invalid state: act=" + state_map[rtstepper.get_command_state()] + " exp=" + state_map[rtstepper.STATE_ESTOP])
      if (old_mode == rtstepper.MODE_MDI):
         rtstepper.mdi_mode()
      else:
         rtstepper.auto_mode()
      rtstepper.abort()
      return  # bail

   # Disable INPUTx estop trigger.
   rtstepper.disable_din_abort(din)

   # Reset estop.
   rtstepper.estop_reset()
   rtstepper.machine_on()

   # Zero the axis.
   rtstepper.home(axis);

   # Move to "home" position.
   dist = float(rtstepper.get_ini_key_value(axis_section[axis], "home"))
   rtstepper.jog_abs(axis, speed, dist)
   rtstepper.wait_io_done(0.0)

   # Zero the axis.
   rtstepper.home(axis);

   # Enable INPUTx estop trigger.
   rtstepper.enable_din_abort(din)

   # Restore mode.
   if (old_mode == rtstepper.MODE_MDI):
      rtstepper.mdi_mode()
   else:
      rtstepper.auto_mode()

   return

return main()
