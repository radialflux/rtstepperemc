#
# m8.py - m8 mcode script turns coolant flood on.
# 
# inputs:
#  path = ARGV[0]   # this python file name
#
# returns:
#  emc.h result code
#
# example:
#  m8
#
import rtstepper

def main():
    return rtstepper.dout(rtstepper.OUTPUT7, True)

return main()
