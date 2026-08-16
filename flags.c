/* flags.c -- Everything about flags except the `set' command.  That
   is in builtins.c */

/* Copyright (C) 1987-2021 Free Software Foundation, Inc.

   This file is part of GNU Bash, the Bourne Again SHell.

   Bash is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Bash is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with Bash.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "config.h"
#if defined (HAVE_UNISTD_H)
#  include <unistd.h>
#endif

#include "shell.h"
#include "execute_cmd.h"
#include "flags.h"

#if defined (BANG_HISTORY)
#  include "bashhist.h"
#endif

#if defined (JOB_CONTROL)
extern int set_job_control PARAMS((int));
#endif

/* **************************************************************** */
/*								    */
/*			The Standard sh Flags.			    */
/*								    */
/* **************************************************************** */

/* Non-zero means automatically mark variables which are modified or created
   as auto export variables. */
__thread int mark_modified_vars = 0;

/* Non-zero causes asynchronous job notification.  Otherwise, job state
   notification only takes place just before a primary prompt is printed. */
__thread int asynchronous_notification = 0;

/* Non-zero means exit immediately if a command exits with a non-zero
   exit status.  The first is what controls set -e; the second is what
   bash uses internally. */
__thread int errexit_flag = 0;
__thread int exit_immediately_on_error = 0;

/* Non-zero means disable filename globbing. */
__thread int disallow_filename_globbing = 0;

/* Non-zero means that all keyword arguments are placed into the environment
   for a command, not just those that appear on the line before the command
   name. */
__thread int place_keywords_in_env = 0;

/* Non-zero means read commands, but don't execute them.  This is useful
   for debugging shell scripts that should do something hairy and possibly
   destructive. */
__thread int read_but_dont_execute = 0;

/* Non-zero means end of file is after one command. */
__thread int just_one_command = 0;

/* Non-zero means don't overwrite existing files while doing redirections. */
__thread int noclobber = 0;

/* Non-zero means trying to get the value of $i where $i is undefined
   causes an error, instead of a null substitution. */
__thread int unbound_vars_is_error = 0;

/* Non-zero means type out input lines after you read them. */
__thread int echo_input_at_read = 0;
__thread int verbose_flag = 0;

/* Non-zero means type out the command definition after reading, but
   before executing. */
__thread int echo_command_at_execute = 0;

/* Non-zero means turn on the job control features. */
__thread int jobs_m_flag = 0;

/* Non-zero means this shell is interactive, even if running under a
   pipe. */
__thread int forced_interactive = 0;

/* By default, follow the symbolic links as if they were real directories
   while hacking the `cd' command.  This means that `cd ..' moves up in
   the string of symbolic links that make up the current directory, instead
   of the absolute directory.  The shell variable `nolinks' also controls
   this flag. */
__thread int no_symbolic_links = 0;

/* **************************************************************** */
/*								    */
/*		     Non-Standard Flags Follow Here.		    */
/*								    */
/* **************************************************************** */

#if 0
/* Non-zero means do lexical scoping in the body of a FOR command. */
int lexical_scoping = 0;
#endif

/* Non-zero means look up and remember command names in a hash table, */
__thread int hashing_enabled = 1;

#if defined (BANG_HISTORY)
/* Non-zero means that we are doing history expansion.  The default.
   This means !22 gets the 22nd line of history. */
__thread int history_expansion = HISTEXPAND_DEFAULT;
__thread int histexp_flag = 0;
#endif /* BANG_HISTORY */

/* Non-zero means that we allow comments to appear in interactive commands. */
__thread int interactive_comments = 1;

#if defined (RESTRICTED_SHELL)
/* Non-zero means that this shell is `restricted'.  A restricted shell
   disallows: changing directories, command or path names containing `/',
   unsetting or resetting the values of $PATH and $SHELL, and any type of
   output redirection. */
__thread int restricted = 0;		/* currently restricted */
__thread int restricted_shell = 0;	/* shell was started in restricted mode. */
#endif /* RESTRICTED_SHELL */

/* Non-zero means that this shell is running in `privileged' mode.  This
   is required if the shell is to run setuid.  If the `-p' option is
   not supplied at startup, and the real and effective uids or gids
   differ, disable_priv_mode is called to relinquish setuid status. */
__thread int privileged_mode = 0;

#if defined (BRACE_EXPANSION)
/* Zero means to disable brace expansion: foo{a,b} -> fooa foob */
__thread int brace_expansion = 1;
#endif

/* Non-zero means that shell functions inherit the DEBUG trap. */
__thread int function_trace_mode = 0;

/* Non-zero means that shell functions inherit the ERR trap. */
__thread int error_trace_mode = 0;

/* Non-zero means that the rightmost non-zero exit status in a pipeline
   is the exit status of the entire pipeline.  If each processes exits
   with a 0 status, the status of the pipeline is 0. */
__thread int pipefail_opt = 0;

/* **************************************************************** */
/*								    */
/*			The Flags ALIST.			    */
/*								    */
/* **************************************************************** */

__thread struct flags_alist shell_flags[] = {
  /* Standard sh flags. */
  { 'a', 0 },
#if defined (JOB_CONTROL)
  { 'b', 0 },
#endif /* JOB_CONTROL */
  { 'e', 0 },
  { 'f', 0 },
  { 'h', 0 },
  { 'i', 0 },
  { 'k', 0 },
#if defined (JOB_CONTROL)
  { 'm', 0 },
#endif /* JOB_CONTROL */
  { 'n', 0 },
  { 'p', 0 },
#if defined (RESTRICTED_SHELL)
  { 'r', 0 },
#endif /* RESTRICTED_SHELL */
  { 't', 0 },
  { 'u', 0 },
  { 'v', 0 },
  { 'x', 0 },

  /* New flags that control non-standard things. */
#if 0
  { 'l', 0 },
#endif
#if defined (BRACE_EXPANSION)
  { 'B', 0 },
#endif
  { 'C', 0 },
  { 'E', 0 },
#if defined (BANG_HISTORY)
  { 'H', 0 },
#endif /* BANG_HISTORY */
  { 'P', 0 },
  { 'T', 0 },
  {0, (int *)NULL}
};

/* iSH-AOK: shell_flags[] pointed at variables that are thread-local now,
   so its addresses are no longer compile-time constants. The entries
   below carry the same #if lines as the table, which is what keeps the
   indices in step with it. Generated by tools/bash-tls-fix-tables.py. */
void
aok_fix_shell_flags ()
{
  __thread static __thread int aok_done;
  int aok_i = 0;

  if (aok_done)
    return;
  aok_done = 1;

  shell_flags[aok_i].value = &mark_modified_vars;
  aok_i++;
#if defined (JOB_CONTROL)
  shell_flags[aok_i].value = &asynchronous_notification;
  aok_i++;
#endif /* JOB_CONTROL */
  shell_flags[aok_i].value = &errexit_flag;
  aok_i++;
  shell_flags[aok_i].value = &disallow_filename_globbing;
  aok_i++;
  shell_flags[aok_i].value = &hashing_enabled;
  aok_i++;
  shell_flags[aok_i].value = &forced_interactive;
  aok_i++;
  shell_flags[aok_i].value = &place_keywords_in_env;
  aok_i++;
#if defined (JOB_CONTROL)
  shell_flags[aok_i].value = &jobs_m_flag;
  aok_i++;
#endif /* JOB_CONTROL */
  shell_flags[aok_i].value = &read_but_dont_execute;
  aok_i++;
  shell_flags[aok_i].value = &privileged_mode;
  aok_i++;
#if defined (RESTRICTED_SHELL)
  shell_flags[aok_i].value = &restricted;
  aok_i++;
#endif /* RESTRICTED_SHELL */
  shell_flags[aok_i].value = &just_one_command;
  aok_i++;
  shell_flags[aok_i].value = &unbound_vars_is_error;
  aok_i++;
  shell_flags[aok_i].value = &verbose_flag;
  aok_i++;
  shell_flags[aok_i].value = &echo_command_at_execute;
  aok_i++;
#if 0
  shell_flags[aok_i].value = &lexical_scoping;
  aok_i++;
#endif
#if defined (BRACE_EXPANSION)
  shell_flags[aok_i].value = &brace_expansion;
  aok_i++;
#endif
  shell_flags[aok_i].value = &noclobber;
  aok_i++;
  shell_flags[aok_i].value = &error_trace_mode;
  aok_i++;
#if defined (BANG_HISTORY)
  shell_flags[aok_i].value = &histexp_flag;
  aok_i++;
#endif /* BANG_HISTORY */
  shell_flags[aok_i].value = &no_symbolic_links;
  aok_i++;
  shell_flags[aok_i].value = &function_trace_mode;
  aok_i++;
  aok_i++;
}


#define NUM_SHELL_FLAGS (sizeof (shell_flags) / sizeof (struct flags_alist))

__thread char optflags[NUM_SHELL_FLAGS+4] = { '+' };

int *
find_flag (name)
     int name;
{
  int i;
  for (i = 0; shell_flags[i].name; i++)
    {
      if (shell_flags[i].name == name)
	return (shell_flags[i].value);
    }
  return (FLAG_UNKNOWN);
}

/* Change the state of a flag, and return it's original value, or return
   FLAG_ERROR if there is no flag FLAG.  ON_OR_OFF must be either
   FLAG_ON or FLAG_OFF. */
int
change_flag (flag, on_or_off)
  int flag;
  int on_or_off;
{
  int *value, old_value;

#if defined (RESTRICTED_SHELL)
  /* Don't allow "set +r" in a shell which is `restricted'. */
  if (restricted && flag == 'r' && on_or_off == FLAG_OFF)
    return (FLAG_ERROR);
#endif /* RESTRICTED_SHELL */

  value = find_flag (flag);

  if ((value == (int *)FLAG_UNKNOWN) || (on_or_off != FLAG_ON && on_or_off != FLAG_OFF))
    return (FLAG_ERROR);

  old_value = *value;
  *value = (on_or_off == FLAG_ON) ? 1 : 0;

  /* Special cases for a few flags. */
  switch (flag)
    {
#if defined (BANG_HISTORY)
    case 'H':
      history_expansion = histexp_flag;
      if (on_or_off == FLAG_ON)
	bash_initialize_history ();
      break;
#endif

#if defined (JOB_CONTROL)
    case 'm':
      set_job_control (on_or_off == FLAG_ON);
      break;
#endif /* JOB_CONTROL */

    case 'e':
      if (builtin_ignoring_errexit == 0)
	exit_immediately_on_error = errexit_flag;
      break;

    case 'n':
      if (interactive_shell)
	read_but_dont_execute = 0;
      break;

    case 'p':
      if (on_or_off == FLAG_OFF)
	disable_priv_mode ();
      break;

#if defined (RESTRICTED_SHELL)
    case 'r':
      if (on_or_off == FLAG_ON && shell_initialized)
	maybe_make_restricted (shell_name);
      break;
#endif

    case 'v':
      echo_input_at_read = verbose_flag;
      break;
    }

  return (old_value);
}

/* Return a string which is the names of all the currently
   set shell flags. */
char *
which_set_flags ()
{
  char *temp;
  int i, string_index;

  temp = (char *)xmalloc (1 + NUM_SHELL_FLAGS + read_from_stdin + want_pending_command);
  for (i = string_index = 0; shell_flags[i].name; i++)
    if (*(shell_flags[i].value))
      temp[string_index++] = shell_flags[i].name;

  if (want_pending_command)
    temp[string_index++] = 'c';
  if (read_from_stdin)
    temp[string_index++] = 's';

  temp[string_index] = '\0';
  return (temp);
}

char *
get_current_flags ()
{
  char *temp;
  int i;

  temp = (char *)xmalloc (1 + NUM_SHELL_FLAGS);
  for (i = 0; shell_flags[i].name; i++)
    temp[i] = *(shell_flags[i].value);
  temp[i] = '\0';
  return (temp);
}

void
set_current_flags (bitmap)
     const char *bitmap;
{
  int i;

  if (bitmap == 0)
    return;
  for (i = 0; shell_flags[i].name; i++)
    *(shell_flags[i].value) = bitmap[i];
}

void
reset_shell_flags ()
{
  mark_modified_vars = disallow_filename_globbing = 0;
  place_keywords_in_env = read_but_dont_execute = just_one_command = 0;
  noclobber = unbound_vars_is_error = 0;
  echo_command_at_execute = jobs_m_flag = forced_interactive = 0;
  no_symbolic_links = 0;
  privileged_mode = pipefail_opt = 0;

  error_trace_mode = function_trace_mode = 0;

  exit_immediately_on_error = errexit_flag = 0;
  echo_input_at_read = verbose_flag = 0;

  hashing_enabled = interactive_comments = 1;

#if defined (JOB_CONTROL)
  asynchronous_notification = 0;
#endif

#if defined (BANG_HISTORY)
  histexp_flag = 0;
#endif

#if defined (BRACE_EXPANSION)
  brace_expansion = 1;
#endif

#if defined (RESTRICTED_SHELL)
  restricted = 0;
#endif
}

void
initialize_flags ()
{
  register int i;

  for (i = 0; shell_flags[i].name; i++)
    optflags[i+1] = shell_flags[i].name;
  optflags[++i] = 'o';
  optflags[++i] = ';';
  optflags[i+1] = '\0';
}
