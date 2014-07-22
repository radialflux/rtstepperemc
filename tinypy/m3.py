#
# m3.py - m3 mcode script turns spindle on clockwise.
# 
# inputs:
#  path = ARGV[0]   # this python file name
#  rpm  = ARGV[1]   # S parameter (spindle speed)
#
# returns:
#  emc.h result code
#
# example:
#  m3 s250.0
#
import rtstepper

def main():
#    print("rpm:", ARGV[1])
    return rtstepper.dout(rtstepper.OUTPUT6, True)

return main()
