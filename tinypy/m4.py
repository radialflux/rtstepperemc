#
# m4.py - m4 mcode script turns spindle on counter clockwise.
# 
# inputs:
#  path = ARGV[0]   # this python file name
#  rpm  = ARGV[1]   # S parameter (spindle speed)
#
# returns:
#  emc.h result code
#
# example:
#  m4 s250.0
#
import rtstepper

def main():
#    print("rpm:", ARGV[1])
    return rtstepper.dout(rtstepper.OUTPUT6, True)

return main()
