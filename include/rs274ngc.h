/********************************************************************
* Description: rs274ngc.hh
*
*   Derived from a work by Thomas Kramer
*
* Author:
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2004 All rights reserved.
*
********************************************************************/
#ifndef RS274NGC_HH
#define RS274NGC_HH

/* Size of certain arrays */
#define ACTIVE_G_CODES 16
#define ACTIVE_M_CODES 10
#define ACTIVE_SETTINGS 3

/**********************/
/* INCLUDE DIRECTIVES */
/**********************/

#include <stdio.h>
#include "canon.h"
#include "emc.h"
#include "bug.h"

typedef struct setup_struct setup;
#ifndef JAVA_DIAG_APPLET
typedef setup *setup_pointer;
#endif
typedef struct block_struct block;
#ifndef JAVA_DIAG_APPLET
typedef block *block_pointer;
#endif

typedef bool ON_OFF;

// Declare class so that we can use it in the typedef.
class Interp;
typedef int (Interp::*read_function_pointer) (char *, int *, block_pointer, double *);

#undef DEBUG_EMC

#define _logDebug(level, fmt, args...)          \
    do {                                        \
        if (level < _setup.loggingLevel) {      \
            doLog(                              \
                "%02d(%d):%s:%d -- " fmt "\n",  \
                level,                          \
                getpid(),                       \
                __FILE__,                       \
                __LINE__ ,                      \
                ## args                         \
            );                                  \
        }                                       \
    } while(0)

//#define logDebug(fmt, args...) _logDebug(0, fmt, ## args)

//#define DEBUG_RS274NGC

#ifdef DEBUG_RS274NGC
#define logDebug DBG
#else
#define logDebug(args...)
#endif

#endif
