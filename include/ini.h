/********************************************************************
* ini.h - INI file support for EMC2
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author:
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2004 All rights reserved.
*
* Last change:
********************************************************************/
#ifndef _INI_H
#define _INI_H

#include "emc.h"        // EMC_AXIS_STAT

#ifdef __cplusplus
extern "C"
{
#endif

   int iniAxis(int axis);
   int iniTraj(void);
   int iniTool(void);
   int iniTask(void);
   int iniGetKeyValue(const char *section, const char *key, char *value, int value_size);

#ifdef __cplusplus
}                               /* matches extern "C" at top */
#endif

#endif                          /* _INI_H */
