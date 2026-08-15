/* aok_fork.h -- the fork replacement's interface to the rest of bash.
 *
 * See aok_fork.c for what this is and why bash needs it under iSH-AOK. The
 * declarations live in a header rather than beside each use because there are
 * now six call sites, and a second declaration that disagrees with the first
 * is a class of bug worth spending a header to make impossible.
 *
 * Everything here is guarded by AOK_NATIVE_FORK, which the iSH-AOK build
 * defines and a plain build of this tree does not, so the vendor sources still
 * compile as bash.
 */

#if !defined (_AOK_FORK_H_)
#define _AOK_FORK_H_

#if defined (AOK_NATIVE_FORK)

#include "stdc.h"

/* Enough for the descriptors any one site hands over: process substitution
   passes one, a coprocess two. */
#define AOK_FORK_MAX_CLOSE 4

/* The command the spawned child is to run, in bash's own printed form, plus
   the descriptors it is to be started with. Set these immediately before
   make_child(); jobs.c reads them there and clears them again. */
extern char *aok_fork_cmdtext;
extern int aok_fork_pipe_in, aok_fork_pipe_out;
extern int aok_fork_close_fds[AOK_FORK_MAX_CLOSE];
extern int aok_fork_nclose;

extern void aok_fork_clear PARAMS((void));

/* Spawn a bash task carrying this shell's state, with the descriptors above.
   Returns a pid, never 0 -- there is no child branch to return into. */
extern pid_t aok_spawn_command PARAMS((char *, int, int));

/* Command substitution: run COMMAND and give back what it wrote to stdout. */
extern char *aok_run_in_subshell PARAMS((char *, int *));

/* The one site whose child execs: spawn ARGS directly, with REDIRECTS applied
   around the spawn rather than in a child that does not exist. */
extern pid_t aok_spawn_disk_command PARAMS((char *, char **, char **, REDIRECT *, int, int));

/* The shell state a subshell would have inherited, as a script. */
extern char *aok_serialize_state PARAMS((void));

/* jobs.c: record a spawned child in the job table as make_child would have. */
extern void aok_register_spawned PARAMS((char *, pid_t, int));

/* `command_not_found_handle` plus the expanded words, for a re-launch. */
extern char *aok_notfound_command PARAMS((WORD_LIST *));

/* execute_cmd.c: why an exec failed, and the 126/127 that goes with it. */
extern int aok_report_exec_failure PARAMS((char *, int));

/* "NAME: command not found", with the command's own redirections applied. */
extern int aok_report_notfound PARAMS((char *, REDIRECT *, int, int));

/* What a failed aok_spawn_disk_command would have exited with. */
extern int aok_spawn_status;

/* The process group to start the next spawned child in: -1 for the shell's
   own, 0 for a group of its own led by the child, or an existing group. A
   forked child sets this for itself; a spawned one has to be told. */
extern int aok_fork_pgid;

/* Whether the spawned child gets SIGTSTP/SIGTTIN/SIGTTOU back at SIG_DFL, as
   make_child's child branch does with default_tty_job_signals. */
extern int aok_fork_sigdefault;

/* Re-entry. bash is a function here, called again for each `bash`, so every
   global holds what the last invocation left in it; these put the four files
   with state that outlives a shell back to where a fresh process would be.
   Called from shell_reinitialize -- see the block there, and
   docs/bash_native_reentry.md for the audit. */
extern void aok_reinit_jobs PARAMS((void));
extern void aok_reinit_subst PARAMS((void));
extern void aok_reinit_execute PARAMS((void));
extern void aok_reinit_signals PARAMS((void));
extern void aok_reinit_variables PARAMS((void));
extern void aok_reinit_parser PARAMS((void));
extern void aok_reinit_input PARAMS((void));
extern void aok_reinit_aliases PARAMS((void));
extern void aok_reinit_execignore PARAMS((void));
extern void aok_reinit_groups PARAMS((void));
extern void aok_reinit_history PARAMS((void));
extern void aok_reinit_fignore PARAMS((void));

#endif /* AOK_NATIVE_FORK */

#endif /* _AOK_FORK_H_ */
