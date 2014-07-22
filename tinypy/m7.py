#
# m7.py - m7 mcode script turns coolant mist on.
# 
# inputs:
#  path = ARGV[0]   # this python file name
#
# returns:
#  emc.h result code
#
# example:
#  m7
#
import rtstepper

def main():
    return rtstepper.dout(rtstepper.OUTPUT7, True)

return main()
