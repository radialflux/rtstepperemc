#
# m5.py - m5 mcode script turns spindle off.
# 
# inputs:
#  path = ARGV[0]   # this python file name
#
# returns:
#  emc.h result code
#
# example:
#  m5
#
import rtstepper

def main():
   return rtstepper.dout(rtstepper.OUTPUT6, False)

return main()
