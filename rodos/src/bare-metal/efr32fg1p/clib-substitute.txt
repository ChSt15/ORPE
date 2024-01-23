/*
 * Clib substitute template
 *
 *  Created on: 13.03.2012
 *  Michael Ruffer
 */


extern "C" {

/* newlib stubs
 * The following function are necessary for newlib. It is very important
 * that there are valid implementations for _sbrk_r and sbrk. These
 * functions are used for memory allocation.
 * For more informations about syscalls visit:
 * http://sourceware.org/newlib/libc.html#Syscalls
 * http://sourceware.org/newlib/libc.html#Reentrant-Syscalls
 *
 */

int    _close(int file)                             { return -1; }
int    _execve(char *name, char **argv, char **env) { return -1; }
int    _fork(void)                                  { return -1; }
int    _fstat(int file, struct stat *st)            { return 0; }
int    _getpid(void)                                { return 1; }
int    _isatty(int file)                            { return 1; }
int    _kill(int pid, int sig)                      { return -1; }
int    _link(char *old, char *pNew)                 { return -1; }
int    _lseek(int file, int ptr, int dir)           { return 0; }
int    _open(const char *name, int flags, int mode) { return -1; }
int    _read(int file, char *ptr, int len)          { return 0; }
int    _write(int file, char *ptr, int len)         { return len; }
void*  _sbrk(int incr)                              { return 0; }

} // end extern "C"

