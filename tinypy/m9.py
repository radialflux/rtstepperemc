#
# m9.py - m9 mcode script turns coolant off.
# 
# inputs:
#  path = ARGV[0]   # this python file name
#  coolant = ARGV[1]   # 1 = mist, 2 = flood
#
# returns:
#  emc.h result code
#
# example:
#  m9
#
import rtstepper

def main():
   if ARGV[1] = 1:
      return rtstepper.dout(rtstepper.OUTPUT7, False)
#   else:
#      return rtstepper.dout(rtstepper.OUTPUT7, False)

return main()
