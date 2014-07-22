#ifndef _BUG_H
#define _BUG_H

#include "rtstepper.h"

//#define DEBUG_RTSTEPPER_EMC

#define _STRINGIZE(x) #x
#define STRINGIZE(x) _STRINGIZE(x)

#define BUG(args...) rtstepper_syslog(__FILE__ " " STRINGIZE(__LINE__) ": " args)

#ifdef DEBUG_RTSTEPPER_EMC
#define DBG(args...) rtstepper_syslog(__FILE__ " " STRINGIZE(__LINE__) ": " args)
#else
#define DBG(args...)
#endif

#endif /* _BUG_H */
