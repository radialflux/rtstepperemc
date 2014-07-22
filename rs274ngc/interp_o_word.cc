/********************************************************************
* Description: interp_o_word.cc
*
*
* Author: Kenneth Lerman
* License: GPL Version 2
* System: Linux
*    
* Copyright 2005 All rights reserved.
*
* Last change:
********************************************************************/

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <ctype.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include "rs274ngc.h"
#include "rs274ngc_return.h"
#include "interp_return.h"
#include "interp_internal.h"
#include "rs274ngc_interp.h"

//========================================================================
// Functions for control stuff (O-words)
//========================================================================

/*
  Given the root of a directory tree and a file name,
  find the path to the file, if any.
*/

int Interp::findFile( // ARGUMENTS
		     char *direct,  // the directory to start looking in
		     char *target,  // the name of the file to find
		     char *foundFileDirect) // where to store the result
{
    FILE *file;
    DIR *aDir;
    struct dirent *aFile;
    char targetPath[PATH_MAX+1];

    snprintf(targetPath, PATH_MAX, "%s/%s", direct, target);

    file = fopen(targetPath, "r");

    if(file)
    {
        strncpy(foundFileDirect, direct, PATH_MAX);
        fclose(file);
        return INTERP_OK;
    }

    aDir = opendir(direct);

    if(!aDir)
    {
	ERS(NCE_FILE_NOT_OPEN);
    }

    while((aFile = readdir(aDir)))
    {
        if((0 != strcmp(aFile->d_name, "..")) && (0 != strcmp(aFile->d_name, ".")))
        {
            char path[PATH_MAX+1];
            snprintf(path, PATH_MAX, "%s/%s", direct, aFile->d_name);
            if(INTERP_OK == findFile(path, target, foundFileDirect))
            {
	        closedir(aDir);
                return INTERP_OK;
            }
        }
    }
    closedir(aDir);
    ERS(NCE_FILE_NOT_OPEN);
}


/************************************************************************/
/*
   In the long run, this function will use a hash table or other
   fast data structure
*/
int Interp::control_save_offset( /* ARGUMENTS                   */
 int line,                   /* (o-word) line number        */
 block_pointer block,        /* pointer to a block of RS274/NGC instructions */
 setup_pointer settings)     /* pointer to machine settings */
{
  int index;

  logDebug("Entered:Interp::control_save_offset for o_name:|%s|", block->o_name);

  if(control_find_oword(block, settings, &index) == INTERP_OK)
  {
      // already exists
      ERS("File:%s line:%d redefining sub: o|%s| already defined in file:%s",
               settings->filename, settings->sequence_number,
	       block->o_name,
               settings->oword_offset[index].filename);
      //return INTERP_OK;
  }

  CHKS((settings->oword_labels >= INTERP_OWORD_LABELS),
      NCE_TOO_MANY_OWORD_LABELS);


  index = settings->oword_labels++;
  //logDebug("index: %d offset: %ld", index, block->offset);

  //  settings->oword_offset[index].o_word = line;
  settings->oword_offset[index].o_word_name = strdup(block->o_name);
  settings->oword_offset[index].type = block->o_type;
  settings->oword_offset[index].offset = block->offset;
  settings->oword_offset[index].filename = strdup(settings->filename);
  settings->oword_offset[index].repeat_count = -1;

  // the sequence number has already been bumped, so save
  // the proper value
  settings->oword_offset[index].sequence_number =
    settings->sequence_number - 1;

  return INTERP_OK;
}

int Interp::control_find_oword( /* ARGUMENTS                       */
  block_pointer block,      /* pointer to block */
  setup_pointer settings,   /* pointer to machine settings      */
  int *o_index)             /* the index of o-word (returned) */
{
  int i;

  logDebug("Entered:Interp::control_find_oword\n");
  for(i=0; i<settings->oword_labels; i++)
    {
      if(0 == strcmp(settings->oword_offset[i].o_word_name, block->o_name))
	{
	  *o_index = i;
	  logDebug("Found oword[%d]: |%s|", i, block->o_name);
	  return INTERP_OK;
	}
    }
  logDebug("Unknown oword name: |%s|", block->o_name);
  ERS(NCE_UNKNOWN_OWORD_NUMBER);
}

//
// TESTME!!! MORE THOROUGHLY !!!KL
//
// In the past, calls had to be to predefined subs
//
// Now they don't. Do things in the following sequence:
// 1 -- if o_word is already defined, just go back to it, else
// 2 -- if there is a file with the name of the o_word,
//             open it and start skipping (as in 3, below)
// 3 -- skip to the o_word (will be an error if not found)
//
int Interp::control_back_to( /* ARGUMENTS                       */
 block_pointer block, // pointer to block
 setup_pointer settings)   /* pointer to machine settings      */
{
  int i;
  char newFileName[PATH_MAX+1];
  char foundPlace[PATH_MAX+1];
  char tmpFileName[PATH_MAX+1];
  FILE *newFP;

  foundPlace[0] = 0;
  logDebug("Entered:Interp::control_back_to\n");
  for(i=0; i<settings->oword_labels; i++)
    {
      // if(settings->oword_offset[i].o_word == line)
      if(0 == strcmp(settings->oword_offset[i].o_word_name, block->o_name))
	{
          if(settings->file_pointer == NULL)
          {
            ERS(NCE_FILE_NOT_OPEN);
          }
          if(0 != strcmp(settings->filename,
                         settings->oword_offset[i].filename))
          {
              // open the new file...

              newFP = fopen(settings->oword_offset[i].filename, "r");

              // set the line number
              settings->sequence_number = 0;

              strcpy(settings->filename, settings->oword_offset[i].filename);

              if(newFP)
              {
                  // close the old file...
                  fclose(settings->file_pointer);
                  settings->file_pointer = newFP;
              }
              else
              {
                  logDebug("Unable to open file: %s", settings->filename);
                  ERS(NCE_UNABLE_TO_OPEN_FILE,settings->filename);
              }
          }
	  fseek(settings->file_pointer,
		settings->oword_offset[i].offset, SEEK_SET);

	  settings->sequence_number =
	    settings->oword_offset[i].sequence_number;

	  return INTERP_OK;
	}
    }

  // NO o_word found

  // look for a new file
  logDebug("settings->program_prefix:%s:", settings->program_prefix);
  sprintf(tmpFileName, "%s.ngc", block->o_name);

  // first look in the prefix place

  sprintf(newFileName, "%s/%s", settings->program_prefix, tmpFileName);

  newFP = fopen(newFileName, "r");
  logDebug("fopen: |%s|", newFileName);

  // if not found, search the wizard tree
  if(!newFP)
  {
      int ret;
      ret = findFile(settings->wizard_root, tmpFileName, foundPlace);

      if(INTERP_OK == ret)
      {
	  // create the long name
          sprintf(newFileName, "%s/%s",
		  foundPlace, tmpFileName);
          newFP = fopen(newFileName, "r");
      }
  }

  if(newFP)
  {
      logDebug("fopen: |%s| OK", newFileName);

      // close the old file...
      fclose(settings->file_pointer);
      settings->file_pointer = newFP;

      strcpy(settings->filename, newFileName);
  }
  else
  {
     logDebug("fopen: |%s| failed CWD:|%s|", newFileName,
              get_current_dir_name());
     ERS(NCE_UNABLE_TO_OPEN_FILE,tmpFileName);
  }

  if(settings->skipping_o)free(settings->skipping_o);
  settings->skipping_o = strdup(block->o_name); // start skipping

  if(settings->skipping_to_sub)free(settings->skipping_to_sub);
  settings->skipping_to_sub = strdup(block->o_name); // start skipping

  settings->skipping_start = settings->sequence_number;
  //ERS(NCE_UNKNOWN_OWORD_NUMBER);
  return INTERP_OK;
}

/************************************************************************/
/* convert_control_functions

Returned Value: int (INTERP_OK)
Side effects:
   Changes the flow of control.

Called by: execute_block

Calls: control_skip_to
       control_back_to
       control_save_offset
*/

int Interp::convert_control_functions( /* ARGUMENTS           */
 block_pointer block,      /* pointer to a block of RS274/NGC instructions */
 setup_pointer settings)   /* pointer to machine settings                  */
{
  int status;
  int index;
  int i;

  logDebug("convert_control_functions");

  // if there is an oword, must get the block->o_number
  // !!!KL
  if(block->o_name)
    {
      control_find_oword(block, settings, &(block->o_number));
    }
  else
    {
      block->o_number = 0;
    }

  // must skip if skipping
  if(settings->skipping_o && (0!=strcmp(settings->skipping_o, block->o_name)))
  {
      logDebug("skipping to line: |%s|", settings->skipping_o);
      return INTERP_OK;
  }

  if(settings->skipping_to_sub && (block->o_type != O_sub))
  {
      logDebug("skipping to sub: |%s|", settings->skipping_to_sub);
      return INTERP_OK;
  }

  logDebug("o_type:%d", block->o_type);
  switch(block->o_type)
    {
    case O_none:
      // not an error because we use this to signal that we
      // are not evaluating functions
      break;

    case O_sub:
      // if the level is not zero, this is a call
      // not the definition
      // if we were skipping, no longer
      if(settings->skipping_o)
      {
          logDebug("sub(o_|%s|) was skipping to here", settings->skipping_o);

          // skipping to a sub means that we must define this now
	  CHP(control_save_offset(block->o_number, block, settings));
      }

      if(settings->skipping_o)
	{
	  logDebug("no longer skipping to:|%s|", settings->skipping_o);
	  free(settings->skipping_o);
          settings->skipping_o = 0; // this IS our block number
	}
      if(settings->skipping_to_sub)
	{
	  free(settings->skipping_to_sub);
          settings->skipping_to_sub = 0; // this IS our block number
	}
      if(settings->call_level != 0)
	{
	  logDebug("call:%f:%f:%f",
		   settings->parameters[1],
		   settings->parameters[2],
		   settings->parameters[3]);
	}
      else
	{
	  logDebug("started a subroutine defn");
	  // a definition
	  CHKS((settings->defining_sub == 1), NCE_NESTED_SUBROUTINE_DEFN);
	  CHP(control_save_offset(block->o_number, block, settings));
          if(settings->skipping_o)free(settings->skipping_o);
          settings->skipping_o = strdup(block->o_name); // start skipping

          settings->skipping_start = settings->sequence_number;
	  settings->defining_sub = 1;
	  if(settings->sub_name)free(settings->sub_name);
	  settings->sub_name = strdup(block->o_name);
	  logDebug("will now skip to: |%s| %d", settings->sub_name,
		   block->o_number);
	}
      break;
    case O_endsub:
      // if level is not zero, in a call
      // otherwise in a defn
      // if we were skipping, no longer
      if(settings->skipping_o)
	{
	  logDebug("case O_endsub -- no longer skipping to:|%s|",
	       settings->skipping_o);
	  free(settings->skipping_o);
          settings->skipping_o = 0;
	}
      if(settings->call_level != 0)
	{
	  // in a call -- must do a return
          // restore old values of parameters
          // restore file position from context

          free_named_parameters(settings->call_level, settings);
	  if(settings->sub_context[settings->call_level].subName)
	    {
	      free(settings->sub_context[settings->call_level].subName);
	      settings->sub_context[settings->call_level].subName = 0;
	    }
          settings->call_level--;

          for(i=0; i<INTERP_SUB_PARAMS; i++)
	    {
              settings->parameters[i+INTERP_FIRST_SUBROUTINE_PARAM] =
	        settings->sub_context[settings->call_level].saved_params[i];
	    }

	  logDebug("seeking to: %ld",
		   settings->sub_context[settings->call_level].position);

          if(settings->file_pointer == NULL)
          {
            ERS(NCE_FILE_NOT_OPEN);
          }

          //!!!KL must open the new file, if changed

          if(0 != strcmp(settings->filename,
                         settings->sub_context[settings->call_level].filename))
          {
              fclose(settings->file_pointer);
              settings->file_pointer = 
              fopen(settings->sub_context[settings->call_level].filename, "r");

              strcpy(settings->filename,
                     settings->sub_context[settings->call_level].filename);
          }
          
	  fseek(settings->file_pointer,
		settings->sub_context[settings->call_level].position,
		SEEK_SET);

	  settings->sequence_number =
	    settings->sub_context[settings->call_level].sequence_number;

	  if(settings->sub_name)
	    {
	      free(settings->sub_name);
	      settings->sub_name = 0;
	    }

	  if(settings->sub_context[settings->call_level].subName)
	    {
	      settings->sub_name =
		strdup(settings->sub_context[settings->call_level].subName);
	    }
	  else
	    {
	      settings->sub_name = 0;
	    }
	}
      else
	{
	  // a definition
	  CHKS((settings->defining_sub != 1), NCE_NOT_IN_SUBROUTINE_DEFN);
	  // no longer skipping or defining
          if(settings->skipping_o)
	    {
	      logDebug("case O_endsub in defn -- no longer skipping to:|%s|",
		   settings->skipping_o);
	      free(settings->skipping_o);
              settings->skipping_o = 0;
	    }
	  settings->defining_sub = 0;
	  if(settings->sub_name)free(settings->sub_name);
	  settings->sub_name = 0;
	}
      break;
    case O_call:
      // copy parameters from context
      // save old values of parameters
      // save current file position in context
      // if we were skipping, no longer
      if(settings->skipping_o)
	{
	  logDebug("case O_call -- no longer skipping to:|%s|",
	       settings->skipping_o);
	  free(settings->skipping_o);
          settings->skipping_o = 0;
	}
      if(settings->call_level >= INTERP_SUB_ROUTINE_LEVELS)
	{
	  ERS(NCE_TOO_MANY_SUBROUTINE_LEVELS);
	}

      for(i=0; i<INTERP_SUB_PARAMS; i++)
	{
	  settings->sub_context[settings->call_level].saved_params[i] =
	    settings->parameters[i+INTERP_FIRST_SUBROUTINE_PARAM];

	  settings->parameters[i+INTERP_FIRST_SUBROUTINE_PARAM] =
	    block->params[i];

	}

        if(settings->file_pointer == NULL)
          {
            ERS(NCE_FILE_NOT_OPEN);
          }
        settings->sub_context[settings->call_level].position =
	    ftell(settings->file_pointer);
        if(settings->sub_context[settings->call_level].filename)
          {
              // if there is a string here, free it
              free(settings->sub_context[settings->call_level].filename);
          }
        // save the previous filename
	logDebug("Duping |%s|", settings->filename);
        settings->sub_context[settings->call_level].filename =
              strdup(settings->filename);

	settings->sub_context[settings->call_level].sequence_number
	    = settings->sequence_number;

      logDebug("(in call)set params[%d] return file:%s offset:%ld",
	       settings->call_level,
	       settings->sub_context[settings->call_level].filename,
	       settings->sub_context[settings->call_level].position);

      settings->call_level++;

      // set the new subName
      // !!!KL do we need to free old subName?
      settings->sub_context[settings->call_level].subName =
	strdup(block->o_name);

      if (control_back_to(block,settings) == INTERP_ERROR) {
          ERS(NCE_UNABLE_TO_OPEN_FILE,block->o_name);
          return INTERP_ERROR;
      }
      break;
    case O_do:
      // if we were skipping, no longer
      if(settings->skipping_o)free(settings->skipping_o);
      settings->skipping_o = 0;
      // save the loop point
      // we hit this again on loop back -- so test first
      if(INTERP_OK != control_find_oword(block, settings, &index))
      {
          CHP(control_save_offset(block->o_number, block, settings));
      }
      break;

    case O_repeat:
      // if we were skipping, no longer
      if(settings->skipping_o)free(settings->skipping_o);
      settings->skipping_o = 0;
      status = control_find_oword(block, settings, &index);

      // test if not already seen OR
      // if seen and this is a repeat
      if((status != INTERP_OK) ||
	 (settings->oword_offset[index].type == block->o_type))	{
	  // this is the beginning of a 'repeat' loop
	  // add it to the table if not already there
	  if(status != INTERP_OK)
              CHP(control_save_offset(block->o_number, block, settings));

          // note the repeat count.  it should only be calculated at the
          // start of the repeat loop.
          control_find_oword(block, settings, &index);
          if(settings->oword_offset[index].repeat_count == -1)
              settings->oword_offset[index].repeat_count = 
                  round_to_int(settings->test_value);

	  // are we still repeating?
	  if(settings->oword_offset[index].repeat_count > 0) {
	      // execute forward
	      logDebug("executing forward: [%s] in 'repeat' test value-- %g",
		       block->o_name, settings->test_value);
              // one less repeat remains
              settings->oword_offset[index].repeat_count--;
          } else {
	      // skip forward
	      logDebug("skipping forward: [%s] in 'repeat'",
		       block->o_name);
              if(settings->skipping_o)free(settings->skipping_o);
	      settings->skipping_o = strdup(block->o_name);
              settings->skipping_start = settings->sequence_number;
              // cause the repeat count to be recalculated
	      // if we do this loop again
              settings->oword_offset[index].repeat_count = -1;
          }
      }
      break;

    case O_while:
      // if we were skipping, no longer
      if(settings->skipping_o)free(settings->skipping_o);
      settings->skipping_o = 0;
      status = control_find_oword(block, settings, &index);

      // test if not already seen OR
      // if seen and this is a while (alternative is that it is a do)
      if((status != INTERP_OK) ||
	 (settings->oword_offset[index].type == block->o_type))
	{
	  // this is the beginning of a 'while' loop

	  // add it to the table if not already there
	  if(status != INTERP_OK)
	    {
               CHP(control_save_offset(block->o_number, block, settings));
	    }

	  // test the condition

	  if(settings->test_value != 0.0)
	    {
	      // true
	      // execute forward
	      logDebug("executing forward: [%s] in 'while'",
		       block->o_name);
	    }
	  else
	    {
	      // false
	      // skip forward
	      logDebug("skipping forward: [%s] in 'while'",
		       block->o_name);
              if(settings->skipping_o)free(settings->skipping_o);
	      settings->skipping_o = strdup(block->o_name);
              settings->skipping_start = settings->sequence_number;
	    }
	}
      else
	{
	  // this is the end of a 'do'
	  // test the condition
	  if(settings->test_value != 0.0)
	    {
	      // true
	      // loop on back
	      logDebug("looping back to: [%s] in 'do while'",
		       block->o_name);
              CHP(control_back_to(block, settings));
	    }
	  else
	    {
	      // false
	      logDebug("not looping back to: [%s] in 'do while'",
		       block->o_name);
	    }
	}
      
      break;

    case O_if:
      if(settings->test_value != 0.0)
	{
	  //true
	  logDebug("executing forward: [%s] in 'if'",
	      block->o_name);
          if(settings->skipping_o)free(settings->skipping_o);
          settings->skipping_o = 0;
	  settings->executed_if = 1;
	}
      else
	{
	  //false
          logDebug("skipping forward: [%s] in 'if'",
	      block->o_name);
          if(settings->skipping_o)free(settings->skipping_o);
          settings->skipping_o = strdup(block->o_name);
          settings->skipping_start = settings->sequence_number;
	  settings->executed_if = 0;
	}
      break;

    case O_elseif:
      if((settings->skipping_o) &&
	 (0 != strcmp(settings->skipping_o, block->o_name)))
	{
	  //!!!KL -- the if conditions here say that we were skipping
	  //!!!KL but that the target o_name is not ours.
	  //!!!KL so we should continue skipping -- that's not what
	  //!!!KL the code below says.
#if 0
	  // we were not skipping -- start skipping
          logDebug("start skipping forward: [%s] in 'elseif'",
	      block->o_name);
          if(settings->skipping_o)free(settings->skipping_o);
	  settings->skipping_o = strdup(block->o_name);
          settings->skipping_start = settings->sequence_number;
          return INTERP_OK;
#else
	  // we were skipping -- continue skipping
          logDebug("continue skipping forward: [%s] in 'elseif'",
	      block->o_name);
          return INTERP_OK;
#endif
	}

      // we were skipping
      // but were we ever not skipping
      if(settings->executed_if)
	{
	  // we have already executed, keep on skipping
          logDebug("already executed, continue  "
		   "skipping forward: [%s] in 'elseif'",
	      block->o_name);
	  //settings->skipping_0 = block->o_number;
          if(settings->skipping_o)free(settings->skipping_o);
          settings->skipping_o = strdup(block->o_name);
          settings->skipping_start = settings->sequence_number;
          return INTERP_OK;
	}
      
      if(settings->test_value != 0.0)
	{
	  //true -- start executing
          logDebug("start executing forward: [%s] in 'elseif'",
	      block->o_name);
         if(settings->skipping_o)free(settings->skipping_o);
	  settings->skipping_o = 0;
	  settings->executed_if = 1;
	}
      else
	{
	  //false
          logDebug("continue skipping forward: [%s] in 'elseif'",
	      block->o_name);
	}
      break;

    case O_else:
      // were we ever not skipping
      if(settings->executed_if)
	{
	  // we have already executed, skip
          logDebug("already executed, "
		   "skipping forward: [%s] in 'else'",
	      block->o_name);
          if(settings->skipping_o)free(settings->skipping_o);
	  settings->skipping_o = strdup(block->o_name);
          settings->skipping_start = settings->sequence_number;
          return INTERP_OK;
	}

      if((settings->skipping_o) &&
	 (0 == strcmp(settings->skipping_o, block->o_name)))
	{
	  // we were skipping so stop skipping
          logDebug("stop skipping forward: [%s] in 'else'",
	      block->o_name);
          settings->executed_if = 1;
         if(settings->skipping_o)free(settings->skipping_o);
	  settings->skipping_o = 0;
	}
      else
	{
          // we were not skipping -- so skip
          logDebug("start skipping forward: [%s] in 'else'",
	      block->o_name);
	}
      break;

    case O_endif:
      // stop skipping if we were
      if(settings->skipping_o)free(settings->skipping_o);
      settings->skipping_o = 0;
      logDebug("stop skipping forward: [%s] in 'endif'",
	      block->o_name);
      // the KEY -- outside if clearly must have executed
      // or this would not have executed
      settings->executed_if = 1;
      break;

    case O_break:
      // start skipping
      if(settings->skipping_o)free(settings->skipping_o);
      settings->skipping_o = strdup(block->o_name);
      settings->skipping_start = settings->sequence_number;
      //settings->doing_break = 1;
      logDebug("start skipping forward: [%s] in 'break'",
	      block->o_name);
      break;

    case O_continue:

      // if already skipping, do nothing
      if((settings->skipping_o) &&
	 (0 == strcmp(settings->skipping_o, block->o_name)))
	{
	  logDebug("already skipping: [%s] in 'continue'",
		   block->o_name);
	  return INTERP_OK;
	}
      // start skipping
      if(settings->skipping_o)free(settings->skipping_o);
      settings->skipping_o = strdup(block->o_name);
      settings->skipping_start = settings->sequence_number;
      settings->doing_continue = 1;
      logDebug("start skipping forward: [%s] in 'continue'",
	      block->o_name);
      break;

    case O_endrepeat:
    case O_endwhile:
      // end of a while loop
      logDebug("endwhile: skipping_o:%s", settings->skipping_o);
      if((settings->skipping_o) &&
	 (0 == strcmp(settings->skipping_o, block->o_name)))
	{
	  // we were skipping, so this is the end
          if(settings->skipping_o)free(settings->skipping_o);
	  settings->skipping_o = 0;

	  if(settings->doing_continue)
	    {
	      settings->doing_continue = 0;

  	      // loop on back
	      logDebug("looping back (continue) to: [%s] in while/repeat",
		   block->o_name);
	      CHP(control_back_to(block, settings));
	    }
	  else
	    {
	      // not doing continue, we are done
	      logDebug("falling thru the complete while/repeat: [%s]",
		   block->o_name);
	      return INTERP_OK;
	    }
	}
      else
	{
	  // loop on back
	  logDebug("looping back to: [%s] in 'endwhile/endrepeat'",
		   block->o_name);
	  CHP(control_back_to(block, settings));
	}
      break;

    case O_return:
      if(settings->call_level == 0)
      {
          // this is in the definition
          break;
      }

      // if we were skipping, no longer
      if(settings->skipping_o)free(settings->skipping_o);
      settings->skipping_o = 0;

      // in a call -- must do a return
      // restore old values of parameters
      // restore file position from context

      free_named_parameters(settings->call_level, settings);
      if(settings->sub_context[settings->call_level].subName)
	{
	  free(settings->sub_context[settings->call_level].subName);
	}

      settings->call_level--;

      for(i=0; i<INTERP_SUB_PARAMS; i++)
	{
	  settings->parameters[i + INTERP_FIRST_SUBROUTINE_PARAM] =
	    settings->sub_context[settings->call_level].saved_params[i];
	}

      logDebug("seeking to: %ld",
	       settings->sub_context[settings->call_level].position);

      if(settings->file_pointer == NULL)
      {
        ERS(NCE_FILE_NOT_OPEN);
      }

      //!!!KL must open the new file, if changed
      fseek(settings->file_pointer,
	    settings->sub_context[settings->call_level].position, SEEK_SET);

      settings->sequence_number =
	settings->sub_context[settings->call_level].sequence_number;

      break;

    default:
      // FIXME !!!KL should probably be an error
      break;
    }
    return INTERP_OK;
}
//========================================================================
// End of functions for control stuff (O-words)
//========================================================================
