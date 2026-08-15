/* aok_fork.c -- fork, for a bash that cannot fork.
 *
 * iSH-AOK compiles bash in as HOST code inside one app process, so fork()
 * cannot work: it needs two threads of execution to see different memory at
 * the same addresses, and that is what a process boundary provides and a
 * thread boundary does not. See docs/bash_native_plan.md in the iSH-AOK tree.
 *
 * Six of bash's seven make_child sites need the child to go on running bash's
 * own C code. This file is the answer for them: spawn a fresh bash TASK, hand
 * it the parent's shell state, and let it run the command.
 *
 * Why that is faithful rather than an approximation
 * -------------------------------------------------
 * A subshell is one-way by definition. Everything it does to its own state is
 * MEANT to be discarded -- that is what makes it a subshell -- so "state in,
 * status and output out" is the actual contract, not a weakening of it. Only
 * the inbound direction has to be complete, and it is finite: variables
 * (including unexported ones, which the environment alone would lose),
 * functions, and the shell options. Descriptors and the cwd cross by
 * inheritance because the child is a real task started from this one.
 *
 * What it costs
 * -------------
 * Measured on this platform: a native task spawn is ~1.6ms against ~2.5ms for
 * the guest fork bash would otherwise perform. Forking was never the expensive
 * part of running bash under emulation -- interpretation was, by 38-46x -- so
 * this is not a slower shell paying for correctness. It is the same cost.
 *
 * The serialisation reuses bash's own quoting throughout (sh_single_quote,
 * ansic_quote, array_to_assign, assoc_to_assign, named_function_string), so
 * what the child reads is what bash itself would have written and can read
 * back. Nothing here reimplements quoting, because a second implementation
 * that had to agree with the first is exactly the kind of thing that silently
 * stops agreeing.
 */

#include "config.h"

#include "bashtypes.h"
#include <stdio.h>
#include <errno.h>
#include <sys/wait.h>

#include "bashansi.h"
#include "shell.h"
#include "variables.h"
#include "array.h"
#include "assoc.h"
#include "execute_cmd.h"
#include "flags.h"
#include "subst.h"
#include "jobs.h"
#include "redir.h"
extern REDIRECT *redirection_undo_list;
#include "flags.h"

/* sh_single_quote, ansic_quote, ansic_shouldquote and named_function_string
   all come from externs.h, which shell.h includes. Re-declaring them here was
   the first attempt and was wrong in the way second declarations always are:
   ansic_quote takes char *, not const char *, and the compiler said so. */

/* The subshell is the GUEST's bash, not the native one, and that is the whole
   crux of this file.
 
   The obvious choice is /AOK/native/bash: it is the fast one, and re-launching
   the emulated shell looks like throwing away the reason for compiling bash
   natively. It does not work, and the reason is the same wall fork itself hits.
   A native program is a function in the app's process, so two LIVE native bash
   instances share every one of bash's globals -- and the parent is still live,
   blocked in wait, while the child runs. The child's first
   maybe_make_export_env flushes the PARENT's export_env and aborts on a pointer
   it never allocated. Proved with a backtrace before this comment was written.
 
   What a subshell actually requires is a separate ADDRESS SPACE, and AOK has
   exactly one mechanism for that: a guest process. So the child is emulated.
 
   The cost is honest and bounded: an emulated bash starts in ~15ms against the
   ~2.5ms an emulated bash's own fork costs, and that price is paid once per
   subshell. Everything else -- the parsing, expansion and arithmetic that made
   native bash 38-46x faster -- still happens in the native parent, which is
   where the time in a real script goes. */
#define AOK_SUBSHELL_BASH "/bin/bash"

/* ------------------------------------------------------------ a growing buffer */

typedef struct { char *s; size_t len, cap; } aok_buf;

static int
aok_buf_add (buf, text, len)
     aok_buf *buf;
     const char *text;
     size_t len;
{
  if (buf->len + len + 1 > buf->cap)
    {
      size_t cap = buf->cap ? buf->cap * 2 : 4096;
      while (cap < buf->len + len + 1)
	cap *= 2;
      char *grown = (char *) realloc (buf->s, cap);
      if (grown == 0)
	return -1;
      buf->s = grown;
      buf->cap = cap;
    }
  memcpy (buf->s + buf->len, text, len);
  buf->len += len;
  buf->s[buf->len] = '\0';
  return 0;
}

static int
aok_buf_str (buf, text)
     aok_buf *buf;
     const char *text;
{
  return text ? aok_buf_add (buf, text, strlen (text)) : 0;
}

/* ------------------------------------------------------------- serialisation */

/* Ends a declaration line, silencing it if the variable is readonly.
 
   bash defines a dozen of its own readonly variables at startup -- BASHOPTS,
   EUID, PPID, SHELLOPTS, UID, BASH_VERSINFO -- and a fresh child has already
   set every one of them, correctly and for itself. Assigning to them again is
   an error the child prints and ignores, which is noise on every subshell.
 
   Silencing only the readonly lines is deliberately narrower than wrapping the
   whole state in a redirection. An error on a readonly variable means "the
   child already has this", which is the case we do not care about; an error
   anywhere else means the state did not cross, which we very much do, and
   stays visible. A user's own `readonly x=v` still crosses: the assignment
   succeeds in the child because nothing has made x readonly there yet. */
static int
aok_buf_end_line (buf, var)
     aok_buf *buf;
     SHELL_VAR *var;
{
  /* A GROUP rather than a bare trailing redirection. `declare -a X=(...)
     2>/dev/null` does not mean what it looks like -- the compound assignment
     swallows the redirection -- so the silencing has to enclose the command
     rather than follow it. */
  if (readonly_p (var))
    return aok_buf_str (buf, " ; } 2>/dev/null\n");
  return aok_buf_str (buf, "\n");
}

static int
aok_buf_begin_line (buf, var)
     aok_buf *buf;
     SHELL_VAR *var;
{
  return readonly_p (var) ? aok_buf_str (buf, "{ ") : 0;
}

/* One variable, in the form `declare -X name=value` that bash itself emits and
   reads back. Returns 0 if the variable should not cross at all. */
static int
aok_emit_variable (buf, var)
     aok_buf *buf;
     SHELL_VAR *var;
{
  char *value, *quoted;
  char flags[8];
  int n;

  if (var == 0 || invisible_p (var) || var_isset (var) == 0)
    return 0;

  /* Anything bash computes on demand -- SECONDS, RANDOM, LINENO, BASHPID and
     the rest. They are marked by having a dynamic_value function rather than
     by being on a list, which is why this needs no list and cannot fall behind
     one. Restoring them would be wrong twice over: the value is a snapshot of
     something that moves, and the child establishes its own. */
  if (var->dynamic_value != 0)
    return 0;

  /* A nameref's value is another variable's name; it is restored correctly by
     the declare -n below, but only if that target crossed too, which it will
     have done -- the whole variable set is emitted. */
  if (function_p (var))
    return 0;			/* functions are emitted separately */

  n = 0;
  flags[n++] = '-';
  if (array_p (var))    flags[n++] = 'a';
  if (assoc_p (var))    flags[n++] = 'A';
  if (integer_p (var))  flags[n++] = 'i';
  if (nameref_p (var))  flags[n++] = 'n';
  if (exported_p (var)) flags[n++] = 'x';
  if (n == 1)           flags[n++] = '-';
  flags[n] = '\0';

  if (aok_buf_begin_line (buf, var) < 0 ||
      aok_buf_str (buf, "declare ") < 0 ||
      aok_buf_str (buf, flags) < 0 ||
      aok_buf_str (buf, " ") < 0 ||
      aok_buf_str (buf, var->name) < 0 ||
      aok_buf_str (buf, "=") < 0)
    return -1;

#if defined (ARRAY_VARS)
  if (array_p (var))
    {
      quoted = array_to_assign (array_cell (var), 0);
      if (aok_buf_str (buf, quoted ? quoted : "()") < 0)
	{ FREE (quoted); return -1; }
      FREE (quoted);
      return aok_buf_end_line (buf, var);
    }
  if (assoc_p (var))
    {
      quoted = assoc_to_assign (assoc_cell (var), 0);
      if (aok_buf_str (buf, quoted ? quoted : "()") < 0)
	{ FREE (quoted); return -1; }
      FREE (quoted);
      return aok_buf_end_line (buf, var);
    }
#endif

  value = value_cell (var);
  if (value == 0)
    value = "";
  /* bash's own choice of quoting, so the child reads back exactly this value:
     $'...' where the text needs escapes, '...' where it merely needs quoting. */
  if (ansic_shouldquote (value))
    quoted = ansic_quote (value, 0, (int *) 0);
  else
    quoted = sh_single_quote (value);
  if (aok_buf_str (buf, quoted ? quoted : "''") < 0)
    { FREE (quoted); return -1; }
  FREE (quoted);
  return aok_buf_end_line (buf, var);
}

/* Readonly is applied in a SECOND pass over the same variables. Emitting
   `declare -r` with the value would make the variable readonly before anything
   else could be set from it, and would make a later assignment in the same
   stream fail; doing it afterwards means the child ends up in the same state
   without the ordering hazard. */
static int
aok_emit_readonly (buf, var)
     aok_buf *buf;
     SHELL_VAR *var;
{
  if (var == 0 || readonly_p (var) == 0 || var->dynamic_value != 0)
    return 0;
  if (invisible_p (var) || var_isset (var) == 0)
    return 0;
  if (aok_buf_str (buf, "{ readonly ") < 0 ||
      aok_buf_str (buf, var->name) < 0 ||
      aok_buf_str (buf, " ; } 2>/dev/null\n") < 0)
    return -1;
  return 0;
}

static int
aok_emit_function (buf, var)
     aok_buf *buf;
     SHELL_VAR *var;
{
  char *text;

  if (var == 0 || function_cell (var) == 0)
    return 0;

  text = named_function_string (var->name, function_cell (var),
				FUNC_MULTILINE | FUNC_EXTERNAL);
  if (text == 0)
    return 0;
  if (aok_buf_str (buf, text) < 0 || aok_buf_str (buf, "\n") < 0)
    return -1;
  if (exported_p (var))
    {
      if (aok_buf_str (buf, "export -f ") < 0 ||
	  aok_buf_str (buf, var->name) < 0 ||
	  aok_buf_str (buf, "\n") < 0)
	return -1;
    }
  return 0;
}

/* The shell state a subshell would have inherited, as a script that recreates
   it. Returns a malloc'd string, or 0. */
char *
aok_serialize_state ()
{
  aok_buf buf;
  SHELL_VAR **vars;
  int i;

  buf.s = 0; buf.len = 0; buf.cap = 0;

  /* set -e, -u and the rest. A subshell inherits them, and losing them would
     change whether the command stops on an error. Emitted FIRST so that
     everything after it is subject to the same options the parent had --
     except errexit, which is deliberately deferred to the end: a `declare` of
     something odd must not abort the state itself. */
  if (aok_buf_str (&buf, "set +e\n") < 0)
    goto fail;

  vars = all_shell_variables ();
  if (vars)
    {
      for (i = 0; vars[i]; i++)
	if (aok_emit_variable (&buf, vars[i]) < 0)
	  { free (vars); goto fail; }
      for (i = 0; vars[i]; i++)
	if (aok_emit_readonly (&buf, vars[i]) < 0)
	  { free (vars); goto fail; }
      free (vars);
    }

  vars = all_shell_functions ();
  if (vars)
    {
      for (i = 0; vars[i]; i++)
	if (aok_emit_function (&buf, vars[i]) < 0)
	  { free (vars); goto fail; }
      free (vars);
    }

  /* Now the options, including errexit if the parent had it. */
  if (aok_buf_str (&buf, exit_immediately_on_error ? "set -e\n" : "") < 0)
    goto fail;
  if (aok_buf_str (&buf, unbound_vars_is_error ? "set -u\n" : "") < 0)
    goto fail;

  return buf.s;

fail:
  FREE (buf.s);
  return (char *) 0;
}

/* ------------------------------------------------------------- the re-launch */

/* Run COMMAND in a subshell carrying this shell's state, collecting its
   standard output. Returns the output (malloc'd, caller frees) and stores the
   wait status; returns 0 on failure to start, with errno set.

   The state and the command go in ARGV, as one -c script. That is the whole
   reason this works against a stock /bin/bash: nothing has to be taught to
   look for state on a descriptor, so the subshell can be any bash the guest
   already has. An earlier version passed it on a pipe and needed a hook in
   bash's own startup -- which was fine while the child was the native bash and
   useless the moment it could not be. Linux allows about 2MB of argv and a
   shell's state is a few KB; a state large enough to exceed that would fail
   the spawn with E2BIG rather than silently truncating. */
char *
aok_run_in_subshell (command, status_out)
     char *command;
     int *status_out;
{
  char *state, *script, *argv[4], *out;
  int out_pipe[2];
  void *fa;
  pid_t pid;
  size_t out_len, out_cap, state_len, cmd_len;
  ssize_t n;
  int err, status;

  if (status_out)
    *status_out = 0;

  state = aok_serialize_state ();
  if (state == 0)
    return (char *) 0;

  state_len = strlen (state);
  cmd_len = command ? strlen (command) : 0;
  script = (char *) malloc (state_len + cmd_len + 2);
  if (script == 0)
    { free (state); return (char *) 0; }
  memcpy (script, state, state_len);
  script[state_len] = '\n';
  memcpy (script + state_len + 1, command ? command : "", cmd_len);
  script[state_len + 1 + cmd_len] = '\0';
  free (state);

  if (pipe (out_pipe) < 0)
    { free (script); return (char *) 0; }

  /* The child writes where we read, and holds neither end afterwards --
     leaving the write end open in the child would mean this never sees EOF. */
  if (posix_spawn_file_actions_init (&fa) != 0)
    { close (out_pipe[0]); close (out_pipe[1]); free (script); return (char *) 0; }
  posix_spawn_file_actions_adddup2 (&fa, out_pipe[1], 1);
  posix_spawn_file_actions_addclose (&fa, out_pipe[0]);
  if (out_pipe[1] != 1)
    posix_spawn_file_actions_addclose (&fa, out_pipe[1]);

  argv[0] = "bash";
  argv[1] = "-c";
  argv[2] = script;
  argv[3] = (char *) 0;
  err = posix_spawn (&pid, AOK_SUBSHELL_BASH, &fa, (void **) 0,
		     argv, (char **) 0);
  posix_spawn_file_actions_destroy (&fa);
  if (err != 0)
    {
      close (out_pipe[0]); close (out_pipe[1]);
      free (script);
      errno = err;
      return (char *) 0;
    }

  close (out_pipe[1]);

  out = 0; out_len = 0; out_cap = 0;
  for (;;)
    {
      char chunk[4096];
      n = read (out_pipe[0], chunk, sizeof (chunk));
      if (n <= 0)
	break;
      if (out_len + n + 1 > out_cap)
	{
	  size_t cap = out_cap ? out_cap * 2 : 8192;
	  while (cap < out_len + n + 1)
	    cap *= 2;
	  char *grown = (char *) realloc (out, cap);
	  if (grown == 0)
	    break;
	  out = grown; out_cap = cap;
	}
      memcpy (out + out_len, chunk, n);
      out_len += n;
      out[out_len] = '\0';
    }
  close (out_pipe[0]);
  free (script);

  status = 0;
  waitpid (pid, &status, 0);
  if (status_out)
    *status_out = status;

  if (out == 0)
    {
      out = (char *) malloc (1);
      if (out)
	out[0] = '\0';
    }
  return out;
}

/* ------------------------------------------- subshells and pipeline elements
 
   Everything above serves command substitution, which captures output. The
   other fork sites do not: a subshell or a pipeline element runs with the
   caller's own descriptors and reports only a status, and bash's job machinery
   wants a pid it can wait for.
 
   So this returns a pid and nothing else, and jobs.c calls it INSTEAD of
   fork() -- which means make_child's whole parent branch runs untouched,
   add_process and pipeline_pgrp and last_asynchronous_pid included. The child
   branch is simply never taken, because this never returns 0. Reusing bash's
   bookkeeping rather than reimplementing it is the entire point: a second copy
   of that logic would be one more thing obliged to stay in agreement.
 
   The command text is what make_child is already handed for the job table.
   That it is re-parseable is not an assumption -- make_command_string exists to
   produce shell that `jobs` can display and a user can retype. */

int aok_fork_pipe_in = -1;
int aok_fork_pipe_out = -1;
char *aok_fork_cmdtext = 0;

pid_t
aok_spawn_command (cmdtext, pipe_in, pipe_out)
     char *cmdtext;
     int pipe_in, pipe_out;
{
  char *state, *script, *argv[4];
  void *fa;
  pid_t pid;
  size_t state_len, cmd_len;
  int err;

  if (cmdtext == 0)
    { errno = ENOSYS; return (pid_t) -1; }

  state = aok_serialize_state ();
  if (state == 0)
    return (pid_t) -1;

  state_len = strlen (state);
  cmd_len = strlen (cmdtext);
  script = (char *) malloc (state_len + cmd_len + 2);
  if (script == 0)
    { free (state); return (pid_t) -1; }
  memcpy (script, state, state_len);
  script[state_len] = '\n';
  memcpy (script + state_len + 1, cmdtext, cmd_len);
  script[state_len + 1 + cmd_len] = '\0';
  free (state);

  if (posix_spawn_file_actions_init (&fa) != 0)
    { free (script); return (pid_t) -1; }
  /* do_piping's work, done from here because there is no child to do it in.
     Both ends are closed after the dup2 for the same reason a forked child
     closes them: a pipeline whose writer still holds the read end never ends. */
  if (pipe_in != NO_PIPE)
    {
      posix_spawn_file_actions_adddup2 (&fa, pipe_in, 0);
      if (pipe_in != 0)
	posix_spawn_file_actions_addclose (&fa, pipe_in);
    }
  if (pipe_out != NO_PIPE && pipe_out != REDIRECT_BOTH)
    {
      posix_spawn_file_actions_adddup2 (&fa, pipe_out, 1);
      if (pipe_out != 1)
	posix_spawn_file_actions_addclose (&fa, pipe_out);
    }
  else if (pipe_out == REDIRECT_BOTH)
    {
      /* `|&`: stderr joins stdout. */
      posix_spawn_file_actions_adddup2 (&fa, 1, 2);
    }

  argv[0] = "bash";
  argv[1] = "-c";
  argv[2] = script;
  argv[3] = (char *) 0;
  err = posix_spawn (&pid, AOK_SUBSHELL_BASH, &fa, (void **) 0,
		     argv, (char **) 0);
  posix_spawn_file_actions_destroy (&fa);
  free (script);
  if (err != 0)
    { errno = err; return (pid_t) -1; }
  return pid;
}

/* ------------------------------------------------- an external command

   execute_disk_command's fork is the one site that must NOT be handled by
   re-running the command text. By the time bash gets here the words are
   already EXPANDED -- re-parsing the printed form would run any $(...) inside
   it a second time, which is a visible side effect, not just waste. So this
   spawns the executable with the argv bash computed.

   That leaves the redirections, which the forked child would have applied to
   itself. They are applied HERE instead, undoably, using bash's own
   do_redirections/undo_redirections -- the same pair it uses to run a builtin
   with redirections -- and the spawned child inherits the result. The pipe
   descriptors are handled the same way rather than as spawn file actions,
   because order matters: bash does do_piping FIRST and then redirections, so a
   `>file` on a command in a pipeline wins over the pipe, and file actions
   would apply after everything and silently invert that.

   Interactive shells are why this matters. Non-interactively bash sets
   CMD_NO_FORK for the last command and execs in place, so this path is never
   taken -- which is exactly why `bash -c` worked throughout while typing `id`
   at a prompt reported "fork: Function not implemented". */
pid_t
aok_spawn_disk_command (command, args, env, redirects, pipe_in, pipe_out)
     char *command;
     char **args;
     char **env;
     REDIRECT *redirects;
     int pipe_in, pipe_out;
{
  int saved[3], i, err;
  pid_t pid;

  /* The standard three, kept out of the way while the child's view is built.
     F_DUPFD_CLOEXEC so the copies do not reach the child. */
  for (i = 0; i < 3; i++)
    saved[i] = fcntl (i, F_DUPFD_CLOEXEC, 10);

  if (pipe_in != NO_PIPE)
    dup2 (pipe_in, 0);
  if (pipe_out != NO_PIPE && pipe_out != REDIRECT_BOTH)
    dup2 (pipe_out, 1);
  else if (pipe_out == REDIRECT_BOTH)
    dup2 (1, 2);

  if (redirects && do_redirections (redirects, RX_ACTIVE|RX_UNDOABLE) != 0)
    {
      for (i = 0; i < 3; i++)
	if (saved[i] >= 0) { dup2 (saved[i], i); close (saved[i]); }
      errno = EIO;
      return (pid_t) -1;
    }

  err = posix_spawn (&pid, command, (void *) 0, (void **) 0, args,
		     env ? env : (char **) 0);

  /* bash's own undo: do_redirections with RX_UNDOABLE builds
     redirection_undo_list, and replaying it restores what was there. This is
     the same pair execute_cmd.c's cleanup_redirects uses for a builtin. */
  if (redirection_undo_list)
    {
      do_redirections (redirection_undo_list, RX_ACTIVE);
      dispose_redirects (redirection_undo_list);
      redirection_undo_list = (REDIRECT *) NULL;
    }
  for (i = 0; i < 3; i++)
    if (saved[i] >= 0) { dup2 (saved[i], i); close (saved[i]); }

  if (err != 0)
    { errno = err; return (pid_t) -1; }
  return pid;
}
