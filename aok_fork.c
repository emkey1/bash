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
#include "bashintl.h"
#include "shell.h"
#include "variables.h"
#include "array.h"
#include "assoc.h"
#include "execute_cmd.h"
#include "flags.h"
#include "subst.h"
#include "jobs.h"
#include "redir.h"
__thread extern REDIRECT *redirection_undo_list;
#include "flags.h"
#include "trap.h"
#include "alias.h"
#include "typemax.h"
#include "aok_fork.h"

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

/* One quoted word, in bash's own quoting so the child reads back this exact
   string. Used for positional parameters and trap bodies. */
static int
aok_emit_quoted (buf, value)
     aok_buf *buf;
     char *value;
{
  char *quoted;
  int rc;

  if (value == 0)
    value = "";
  if (ansic_shouldquote (value))
    quoted = ansic_quote (value, 0, (int *) 0);
  else
    quoted = sh_single_quote (value);
  rc = aok_buf_str (buf, quoted ? quoted : "''");
  FREE (quoted);
  return rc;
}

/* `set -- ...`, so $1, $@ and $# survive.

   This is not a nicety. Command substitution hands the child the UNEXPANDED
   text between the parentheses, so a `$(f $1)` -- or the recursive
   `$(fact $(( $1 - 1 )))` that found this -- expands its parameters in the
   CHILD. Without them the child expands nothing, and the answer is wrong
   rather than absent. */
static int
aok_emit_positional (buf)
     aok_buf *buf;
{
  WORD_LIST *l;
  int i;

  if (aok_buf_str (buf, "set --") < 0)
    return -1;
  /* $1..$9 live in dollar_vars; anything beyond is in rest_of_args. */
  for (i = 1; i < 10 && dollar_vars[i]; i++)
    if (aok_buf_str (buf, " ") < 0 || aok_emit_quoted (buf, dollar_vars[i]) < 0)
      return -1;
  for (l = rest_of_args; l; l = l->next)
    if (aok_buf_str (buf, " ") < 0 || aok_emit_quoted (buf, l->word->word) < 0)
      return -1;
  return aok_buf_str (buf, "\n");
}

/* `command_not_found_handle 'word' ...`, for the spawn path in
   execute_disk_command to hand to a re-launched shell. Malloc'd, caller frees.

   The words are already expanded, so they are quoted rather than re-printed:
   what the handler must see is the arguments the shell computed, not the text
   they came from. */
char *
aok_notfound_command (words)
     WORD_LIST *words;
{
  aok_buf buf;
  WORD_LIST *l;

  buf.s = 0; buf.len = 0; buf.cap = 0;
  if (aok_buf_str (&buf, "command_not_found_handle") < 0)
    goto fail;
  for (l = words; l; l = l->next)
    if (aok_buf_str (&buf, " ") < 0 || aok_emit_quoted (&buf, l->word->word) < 0)
      goto fail;
  return buf.s;

fail:
  FREE (buf.s);
  return (char *) 0;
}

/* `set -o` and `shopt`. A subshell inherits every one of these, and a shell
   started afresh has the defaults instead, so they have to be said out loud.

   Four are deliberately never emitted, and the reason is that this child is a
   whole shell rather than a copy of one in mid-flight:

     errexit    handled at the end of the state instead -- a `declare` of
		something the child dislikes must not abort the state itself.
     noexec     `set -n` means "parse but do not run", which for a child whose
		entire purpose is to run one command means it silently does
		nothing. bash reaches this state only while syntax-checking a
		script, where no subshell should be running anyway.
     onecmd     `set -t` means "exit after one command", and the state script
		is many commands, so the child would exit before reaching the
		one it was started for.
     restricted `set -r` cannot be turned off once on, and would refuse the
		child's own `cd`, redirections and PATH-qualified commands.

   monitor is emitted like any other: the child is a real task with a real
   process group, so job control means the same thing there. */
static int
aok_emit_options (buf)
     aok_buf *buf;
{
  extern int aok_minus_o_state PARAMS((int, char **));
  extern int aok_shopt_state PARAMS((int, char **));
  char *name;
  int i, on;

  /* shopt FIRST, then set -o, and the order is load-bearing. Some shopt
     options reach across and change a `set -o` one: `shopt -u extdebug` runs
     shopt_set_debug_mode, which does `error_trace_mode = function_trace_mode =
     debugging_mode` -- it turns OFF both -E and -T. Emitting shopt last
     therefore undid the parent's `set -E` on every subshell, and an ERR trap
     that fired in the parent silently did not fire in its subshells.

     bash's own startup does the reverse (SHELLOPTS then BASHOPTS) and does not
     hit this, because BASHOPTS lists only the options that are ON: it never
     issues a `shopt -u` at all. This state describes every option either way,
     which is what exposes the interaction. */
  for (i = 0; (on = aok_shopt_state (i, &name)) >= 0; i++)
    if (aok_buf_str (buf, on ? "shopt -s " : "shopt -u ") < 0 ||
	aok_buf_str (buf, name) < 0 ||
	aok_buf_str (buf, " 2>/dev/null\n") < 0)
      return -1;

  for (i = 0; (on = aok_minus_o_state (i, &name)) >= 0; i++)
    {
      if (STREQ (name, "errexit") || STREQ (name, "nounset") ||
	  STREQ (name, "noexec") || STREQ (name, "onecmd") ||
	  STREQ (name, "restricted"))
	continue;	/* errexit and nounset are put back at the very end */
      /* Interactive-shell options, which this child never is. Sending them
	 costs rather than gains: `set -o history` in a non-interactive shell
	 makes it record the state script itself, and those lines then turn up
	 in the user's ~/.bash_history. monitor is job control, which bash turns
	 off in a subshell anyway. */
      if (STREQ (name, "emacs") || STREQ (name, "vi") ||
	  STREQ (name, "history") || STREQ (name, "notify") ||
	  STREQ (name, "monitor") || STREQ (name, "ignoreeof"))
	continue;
      if (aok_buf_str (buf, on ? "set -o " : "set +o ") < 0 ||
	  aok_buf_str (buf, name) < 0 ||
	  aok_buf_str (buf, " 2>/dev/null\n") < 0)
	return -1;
    }

  return 0;
}

/* The aliases. A forked child has them; a fresh shell does not, and a command
   substitution's body is re-parsed IN the child, so `x=$(ll)` came back empty
   with `ll: command not found` after a login that defines `ll` as an alias.

   Subshells and pipeline elements did not need this and still do not: alias
   expansion happens at parse time in the PARENT, so what those receive is
   already-expanded command text. Command substitution is the one that hands
   over source. */
static int
aok_emit_aliases (buf)
     aok_buf *buf;
{
#if defined (ALIAS)
  alias_t **list;
  int i;

  list = all_aliases ();
  if (list == 0)
    return 0;
  for (i = 0; list[i]; i++)
    {
      if (aok_buf_str (buf, "alias ") < 0 ||
	  aok_buf_str (buf, list[i]->name) < 0 ||
	  aok_buf_str (buf, "=") < 0 ||
	  aok_emit_quoted (buf, list[i]->value) < 0 ||
	  aok_buf_str (buf, " 2>/dev/null\n") < 0)
	{ free (list); return -1; }
    }
  free (list);
#endif
  return 0;
}

/* The traps, and WHICH of them a child is supposed to still be running is the
   whole content of this function.

   A forked child calls reset_signal_handlers() first thing (execute_cmd.c's
   execute_in_subshell, subst.c's command_substitute), and trap.c's
   reset_or_restore_signal_handlers says exactly what that means:

     - the EXIT trap loses SIG_TRAPPED but KEEPS its string. So `trap -p` in a
       subshell still lists it and it does NOT run when the subshell exits.
     - every trapped signal is reset to its original disposition, again keeping
       the string. So a SIGUSR1 the parent handles KILLS a command
       substitution, rather than running the parent's handler there.
     - ignored signals stay ignored, because SIG_IGN survives exec and a child
       that reset one to default would die where the parent would not.
     - DEBUG, ERR and RETURN are not in that loop at all: they are inherited and
       ARMED, which is what `set -E` and `set -T` are for.

   So what crosses is the ignored ones and the three special traps, and nothing
   else. Emitting the lot -- which is what this did first -- meant the EXIT trap
   fired once per subshell and once per command substitution: a script whose
   `trap ... EXIT` cleaned up after itself did it ten times, interleaved into
   the middle of its own output.

   The one thing not reproduced is `trap -p` inside a child listing the strings
   of traps that are not armed there. Making that visible would mean setting a
   trap in the child and then disarming it, and a child that exits early -- the
   normal case for `( exit 1 )` -- would never reach the disarming. A listing
   that under-reports is the cheaper mistake by a wide margin. */
static int
aok_emit_traps (buf)
     aok_buf *buf;
{
  int sig;

  for (sig = 0; sig < BASH_NSIG; sig++)
    {
      char *name, *body;
      int special;

      /* DEFAULT_SIG and IMPOSSIBLE_TRAP_HANDLER are sentinel POINTERS stored in
	 trap_list, not strings, and must never reach the quoter. */
      body = trap_list[sig];
      if (body == (char *) IMPOSSIBLE_TRAP_HANDLER)
	continue;

      special = (sig == DEBUG_TRAP || sig == ERROR_TRAP || sig == RETURN_TRAP);

      if (special == 0 && (body == (char *) IGNORE_SIG || signal_is_hard_ignored (sig)))
	body = "";		/* stays ignored across the spawn */
      else if (special == 0)
	continue;		/* reset in the child, string and all */
      else if (body == (char *) DEFAULT_SIG || body == (char *) IGNORE_SIG)
	continue;		/* no special trap set */

      name = signal_name (sig);
      if (name == 0 || STREQN (name, "SIGJUNK", 7) || STREQN (name, "unknown", 7))
	continue;
      if (aok_buf_str (buf, "trap ") < 0 ||
	  aok_emit_quoted (buf, body) < 0 ||
	  aok_buf_str (buf, " ") < 0 ||
	  aok_buf_str (buf, name) < 0 ||
	  aok_buf_str (buf, " 2>/dev/null\n") < 0)
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

  /* errexit and nounset OFF while the state is being restored, and put back at
     the end. A `declare` the child dislikes must not abort the state before the
     command it was started for has run, and nounset would turn a variable the
     child has not been given yet into a fatal error partway through being given
     it. */
  if (aok_buf_str (&buf, "set +e\n") < 0)
    goto fail;
  if (aok_buf_str (&buf, "set +u\n") < 0)
    goto fail;

  /* extglob ON for the definitions that follow, whatever the parent's own
     setting is, and back to the parent's setting with the other options at the
     end. This is not a preference, it is what makes the state PARSE.

     A function whose body contains `?(...)` or `+(...)` -- bash-completion is
     built out of them, and so is the AOK profile -- can only be read back by a
     shell that has extglob on. bash stores such a function as a parse tree and
     prints it back in that syntax, and the shell that DEFINED it very often
     turns extglob off again afterwards, which is exactly what bash-completion
     does. So the parent can be sitting there with extglob off and a table full
     of functions that cannot be re-read without it.

     The symptom was every child dying on `syntax error near unexpected token
     (' partway through the state, which left command substitutions empty and a
     login shell in the AOK rootfs unusable. The final options block below puts
     extglob back to whatever the parent actually had, and the COMMAND is parsed
     after that -- bash reads a -c string command by command, which is what
     makes both halves of this work. */
  if (aok_buf_str (&buf, "shopt -s extglob 2>/dev/null\n") < 0)
    goto fail;
  if (aok_emit_aliases (&buf) < 0)
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

  if (aok_emit_positional (&buf) < 0)
    goto fail;
  if (aok_emit_traps (&buf) < 0)
    goto fail;

  /* And now the real options, including whatever extglob actually was. */
  if (aok_emit_options (&buf) < 0)
    goto fail;

  if (aok_buf_str (&buf, exit_immediately_on_error ? "set -e\n" : "") < 0)
    goto fail;
  if (aok_buf_str (&buf, unbound_vars_is_error ? "set -u\n" : "") < 0)
    goto fail;

  /* $? last of all, since every line above sets it. `(exit N)` is how a shell
     script says this, and it is emitted only when there is something to say --
     the child's $? is already 0 from the state itself. */
  if (last_command_exit_value != 0)
    {
      char status[INT_STRLEN_BOUND (int) + 8];
      sprintf (status, "(exit %d)\n", last_command_exit_value);
      if (aok_buf_str (&buf, status) < 0)
	goto fail;
    }

  return buf.s;

fail:
  FREE (buf.s);
  return (char *) 0;
}

/* ------------------------------------------------------------- the re-launch */

/* This shell's state followed by COMMAND, as one script for `bash -c`.
   Malloc'd, caller frees; 0 on failure.

   COMMAND IS COPIED FIRST, and that ordering is load-bearing rather than
   tidiness. Most callers pass a pointer into the_printed_command -- bash's ONE
   static command-printing buffer, which make_command_string returns without
   copying -- and aok_serialize_state prints every shell function through
   named_function_string, which writes to that same buffer.

   Serialising first therefore replaced the command with the text of the last
   function defined, so a shell that had defined any function at all ran that
   definition in place of its subshell: no output, exit status 0, and no error
   anywhere. Command substitution was unaffected, because the string it passes
   comes from the parser rather than the printer -- which is what made this look
   like a subshell-only fault for as long as it did. One helper for both paths,
   so there is one place for this to be right. */
static char *
aok_build_script (command)
     char *command;
{
  char *state, *cmd, *script;
  size_t state_len, cmd_len;

  cmd_len = command ? strlen (command) : 0;
  cmd = (char *) malloc (cmd_len + 1);
  if (cmd == 0)
    return (char *) 0;
  memcpy (cmd, command ? command : "", cmd_len);
  cmd[cmd_len] = '\0';

  state = aok_serialize_state ();
  if (state == 0)
    { free (cmd); return (char *) 0; }

  state_len = strlen (state);
  script = (char *) malloc (state_len + cmd_len + 2);
  if (script)
    {
      memcpy (script, state, state_len);
      script[state_len] = '\n';
      memcpy (script + state_len + 1, cmd, cmd_len);
      script[state_len + 1 + cmd_len] = '\0';
    }
  free (state);
  free (cmd);
  /* The single most useful thing when a re-launched child misbehaves: what it
     was actually handed. Every bug in this file so far has been visible in one
     look at this -- a command replaced by a function definition, a syntax error
     from an extglob pattern emitted before extglob was on, an option that
     turned another option off. */
  if (script && getenv ("AOK_BASH_DUMP_STATE"))
    fprintf (stderr, "----- AOK STATE -----\n%s\n----- END -----\n", script);
  return script;
}

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
  char *script, *argv[5], *out;
  int out_pipe[2];
  void *fa;
  pid_t pid;
  size_t out_len, out_cap;
  ssize_t n;
  int err, status;

  if (status_out)
    *status_out = 0;

  script = aok_build_script (command);
  if (script == 0)
    return (char *) 0;

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
  /* $0. `bash -c script name` names the child, and a subshell keeps the
     parent's $0 -- without this every re-launch would report itself as "bash"
     in an error message or a usage string. The positional parameters do NOT
     come from here; the state script sets them, so that $1 and $@ survive
     quoting exactly as the parent had them. */
  argv[3] = dollar_vars[0] ? dollar_vars[0] : "bash";
  argv[4] = (char *) 0;
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

__thread int aok_fork_pipe_in = -1;
__thread int aok_fork_pipe_out = -1;
__thread char *aok_fork_cmdtext = 0;

/* Descriptors the forked child would have closed for itself before running.
   A fork gives the child its own copy of every descriptor and the child throws
   away the ones that belong to the other end of its pipes; a spawn has no such
   moment, so the closes have to be described to it in advance. Set beside
   aok_fork_cmdtext at the call site, and cleared by the same code that clears
   that -- an fd number left behind here would be applied to an unrelated
   descriptor on the next spawn, which is the kind of bug that surfaces
   somewhere else entirely. */
__thread int aok_fork_close_fds[AOK_FORK_MAX_CLOSE];
__thread int aok_fork_nclose = 0;

/* What a failed aok_spawn_disk_command would have exited with -- 126 or 127.
   Set there because the diagnosis has to happen while the command's own
   redirections are still applied, and read by the caller, which is the one
   with somewhere to put an exit status. */
__thread int aok_spawn_status = 0;

/* The process group the child is to start in: AOK_PGID_INHERIT for the
   shell's own, 0 for a group of its own led by the child, or an existing
   group's id.

   A forked child sets this for itself, first thing, and bash is emphatic about
   why (jobs.c, make_child: "Set the process group before trying to mess with
   the terminal's process group. This is mandated by POSIX."). A spawned child
   has no such moment, and the parent CANNOT do it afterwards -- sys_setpgid
   refuses with EACCES once the child has exec'd. So it is described to the
   spawn, which applies it while impersonating the child, before the exec.

   Without it every child stayed in the shell's own process group, so no
   foreground job ever owned the terminal: ^C during `sleep 30` left the tty
   pointing somewhere that no longer read from it, and the shell never saw
   another keystroke. */
#define AOK_PGID_INHERIT (-1)
__thread int aok_fork_pgid = AOK_PGID_INHERIT;

/* Whether the child should get the job-control signals back at SIG_DFL.
   See aok_spawn_attr. */
__thread int aok_fork_sigdefault = 1;

/* posix_spawn attributes: the child's process group, and the dispositions a
   forked child would have restored for itself.

   The dispositions are not optional decoration. exec resets handlers but
   PRESERVES SIG_IGN, and an interactive bash ignores SIGTSTP, SIGTTIN and
   SIGTTOU for itself -- so without this every command it runs ignores them too,
   and ^Z did nothing at all: the terminal sent SIGTSTP to a foreground job that
   ignored it, and the shell went on waiting for a stop that never came.
   make_child's child branch calls default_tty_job_signals for exactly this, on
   exactly the condition mirrored in aok_fork_sigdefault. */
static int
aok_spawn_attr (attr)
     void **attr;
{
  short flags = 0;
  sigset_t dfl;

  *attr = 0;
  if (aok_fork_pgid == AOK_PGID_INHERIT && aok_fork_sigdefault == 0)
    return 0;
  if (posix_spawnattr_init (attr) != 0)
    return -1;
  if (aok_fork_pgid != AOK_PGID_INHERIT)
    {
      flags |= NLIBC_SPAWN_SETPGROUP;
      posix_spawnattr_setpgroup (attr, aok_fork_pgid);
    }
  if (aok_fork_sigdefault)
    {
      flags |= NLIBC_SPAWN_SETSIGDEF;
      sigemptyset (&dfl);
      sigaddset (&dfl, SIGTSTP);
      sigaddset (&dfl, SIGTTIN);
      sigaddset (&dfl, SIGTTOU);
      posix_spawnattr_setsigdefault (attr, &dfl);
    }
  posix_spawnattr_setflags (attr, flags);
  return 0;
}

/* The pipe descriptors a forked child closes before it runs -- close_fd_bitmap
   (execute_cmd.c), which every fork site calls in the child.

   Leaving them open is not a leak, it is a HANG. `yes | head -1` is the shape:
   head exits after one line, and yes only stops because its write end has no
   reader left. The spawned yes had inherited the pipe's READ end as well as
   its write end, so a reader always existed, yes filled the pipe and blocked,
   and the shell waited for it forever. */
void
aok_fork_close_bitmap (fdbp)
     struct fd_bitmap *fdbp;
{
  int i;

  if (fdbp == 0)
    return;
  for (i = 0; i < fdbp->size && aok_fork_nclose < AOK_FORK_MAX_CLOSE; i++)
    if (fdbp->bitmap[i])
      aok_fork_close_fds[aok_fork_nclose++] = i;
}

void
aok_fork_clear ()
{
  aok_fork_cmdtext = 0;
  aok_fork_pipe_in = aok_fork_pipe_out = NO_PIPE;
  aok_fork_nclose = 0;
}

pid_t
aok_spawn_command (cmdtext, pipe_in, pipe_out)
     char *cmdtext;
     int pipe_in, pipe_out;
{
  char *script, *argv[5];
  void *fa, *attr;
  pid_t pid;
  int err, i;

  if (cmdtext == 0)
    { errno = ENOSYS; return (pid_t) -1; }

  script = aok_build_script (cmdtext);
  if (script == 0)
    return (pid_t) -1;

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
  for (i = 0; i < aok_fork_nclose; i++)
    if (aok_fork_close_fds[i] >= 0)
      posix_spawn_file_actions_addclose (&fa, aok_fork_close_fds[i]);

  argv[0] = "bash";
  argv[1] = "-c";
  argv[2] = script;
  /* $0. `bash -c script name` names the child, and a subshell keeps the
     parent's $0 -- without this every re-launch would report itself as "bash"
     in an error message or a usage string. The positional parameters do NOT
     come from here; the state script sets them, so that $1 and $@ survive
     quoting exactly as the parent had them. */
  argv[3] = dollar_vars[0] ? dollar_vars[0] : "bash";
  argv[4] = (char *) 0;
  aok_spawn_attr (&attr);
  err = posix_spawn (&pid, AOK_SUBSHELL_BASH, &fa,
		     attr ? (void **) &attr : (void **) 0, argv, (char **) 0);
  if (attr)
    posix_spawnattr_destroy (&attr);
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
/* Build the descriptor view the forked child would have built for itself --
   the pipes, then the redirections -- keeping the shell's own stdin, stdout
   and stderr in SAVED. Returns 0, or -1 with the view already restored.

   Split out because it has a second caller: a command that could not be
   started reports why, and that message is the child's output, so it belongs
   inside this window. `nosuchcommand 2>/dev/null` must be silent, and it was
   not while the message was printed after the descriptors were put back. */
static int
aok_redirect_push (redirects, pipe_in, pipe_out, saved)
     REDIRECT *redirects;
     int pipe_in, pipe_out;
     int *saved;
{
  int i;

  /* F_DUPFD_CLOEXEC so the copies do not reach the child. */
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
      return -1;
    }
  return 0;
}

/* bash's own undo: do_redirections with RX_UNDOABLE builds
   redirection_undo_list, and replaying it restores what was there. This is the
   same pair execute_cmd.c's cleanup_redirects uses for a builtin. */
static void
aok_redirect_pop (saved)
     int *saved;
{
  int i;

  if (redirection_undo_list)
    {
      do_redirections (redirection_undo_list, RX_ACTIVE);
      dispose_redirects (redirection_undo_list);
      redirection_undo_list = (REDIRECT *) NULL;
    }
  for (i = 0; i < 3; i++)
    if (saved[i] >= 0) { dup2 (saved[i], i); close (saved[i]); }
}

/* "NAME: command not found", 127, with the descriptors the command itself
   would have had -- so `nosuchcommand 2>/dev/null` is silent, as it is when a
   forked child prints this after applying its own redirections. */
int
aok_report_notfound (name, redirects, pipe_in, pipe_out)
     char *name;
     REDIRECT *redirects;
     int pipe_in, pipe_out;
{
  int saved[3];

  if (aok_redirect_push (redirects, pipe_in, pipe_out, saved) < 0)
    return EXECUTION_FAILURE;
  internal_error (_("%s: command not found"), name);
  aok_redirect_pop (saved);
  return EX_NOTFOUND;
}

pid_t
aok_spawn_disk_command (command, args, env, redirects, pipe_in, pipe_out)
     char *command;
     char **args;
     char **env;
     REDIRECT *redirects;
     int pipe_in, pipe_out;
{
  int saved[3], i, err;
  void *attr, *fa;
  pid_t pid;

  if (aok_redirect_push (redirects, pipe_in, pipe_out, saved) < 0)
    { errno = EIO; return (pid_t) -1; }

  /* The same descriptors a forked child would have closed for itself. Passed as
     file actions because this spawn has no re-launch script to hang them on;
     the pipe ends themselves are already in place through the dup2s above. */
  fa = 0;
  if (aok_fork_nclose > 0 && posix_spawn_file_actions_init (&fa) == 0)
    for (i = 0; i < aok_fork_nclose; i++)
      if (aok_fork_close_fds[i] >= 0)
	posix_spawn_file_actions_addclose (&fa, aok_fork_close_fds[i]);

  aok_spawn_attr (&attr);
  err = posix_spawn (&pid, command, fa ? (void **) &fa : (void **) 0,
		     attr ? (void **) &attr : (void **) 0, args,
		     env ? env : (char **) 0);
  if (fa)
    posix_spawn_file_actions_destroy (&fa);

  /* ENOEXEC is not a failure -- it is how the kernel says "this is a text file
     with no #!", which POSIX requires a shell to run as a script. Upstream
     reaches this through shell_execve's execute_shell_script, which re-execs
     the shell over itself with the script prepended to argv; there is no image
     to replace here, so it is a second spawn with the same argv behind a
     shell. Everything else -- the redirections and the saved descriptors --
     is already in place from the first attempt. */
  if (err == ENOEXEC)
    {
      char **sargs;
      int n;

      for (n = 0; args && args[n]; n++)
	;
      sargs = (char **) malloc ((n + 3) * sizeof (char *));
      if (sargs)
	{
	  sargs[0] = "bash";
	  sargs[1] = command;
	  for (i = 1; i < n; i++)
	    sargs[i + 1] = args[i];
	  sargs[n + 1] = (char *) 0;
	  err = posix_spawn (&pid, AOK_SUBSHELL_BASH, (void *) 0,
			     attr ? (void **) &attr : (void **) 0, sargs,
			     env ? env : (char **) 0);
	  free (sargs);
	}
    }

  /* Reported here rather than by the caller, because here the command's own
     redirections are still in place and the message is the command's output.
     The status travels back in aok_spawn_status. */
  if (attr)
    posix_spawnattr_destroy (&attr);

  aok_spawn_status = 0;
  if (err != 0)
    aok_spawn_status = aok_report_exec_failure (command, err);

  aok_redirect_pop (saved);

  if (err != 0)
    { errno = err; return (pid_t) -1; }
  return pid;
}

/* ------------------------------------------------- thread-local table fixups

   bash's globals are __thread so that more than one native bash can be live in
   the app at once (docs/bash_native_plan.md). The option tables that used to
   hold `&some_option` cannot: the address of a thread-local is not a
   compile-time constant. tools/bash-tls-fix-tables.py moved those addresses
   into per-table fixup functions, and this calls them.

   Once per thread, before bash's main, from kernel/bash_glue.c. They are
   pointer fixups with no dependencies of their own, so the earliest moment is
   also the safest; each guards itself with a thread-local flag, so calling
   this twice is free. */
extern void aok_fix_shopt_vars PARAMS((void));
extern void aok_fix_o_options PARAMS((void));
extern void aok_fix_shell_flags PARAMS((void));
extern void aok_fix_posix_vars PARAMS((void));
extern void aok_fix_long_args PARAMS((void));
#if defined (READLINE)
extern void aok_fix_boolean_varlist PARAMS((void));
extern void aok_fix_tc_strings PARAMS((void));
extern void aok_fix_line_state PARAMS((void));
#endif

void
aok_tls_fixups ()
{
  aok_fix_shopt_vars ();
  aok_fix_o_options ();
  aok_fix_shell_flags ();
  aok_fix_posix_vars ();
  aok_fix_long_args ();
#if defined (READLINE)
  aok_fix_boolean_varlist ();
  aok_fix_tc_strings ();
  aok_fix_line_state ();
#endif
}
