
#include <stdint.h>
#include <stddef.h>
#include <reent.h>
#include "usart.h"

#include "delay.h"

#include "fs.hpp"

extern fs filesys;

extern "C"{

int
_open (const char * path, int flags, ...){
    return filesys.open(path);
}

int
_write (int file, const void * ptr, size_t len)
{
    filesys.write(file, ptr, len);
    return len;
}

// _ssize_t
// _write_r (struct _reent *ptr,
//      int fd,
//      const void *buf,
//      size_t cnt);

_READ_WRITE_RETURN_TYPE _read (int __fd, void *__buf, size_t __nbyte){return 0;}

// _ssize_t
// _read_r (struct _reent *ptr,
//      int fd,
//      void *buf,
//      size_t cnt){return 0;}

int     _close (int fd){return 0;}

// int _close_r(struct _reent *ptr,
//      int fd){return 0;}

int	_fstat (int __fd, struct stat *__sbuf ){return 0;}

// int
// _fstat_r (struct _reent *ptr,int fd,struct stat * pstat){return 0;}

extern const uint32_t _Min_Heap_Size;
extern const uint32_t _end;

ptrdiff_t prog_brk_off = 0;

void *  _sbrk (ptrdiff_t __incr){
    void* old = (void*)((uint32_t)&_end + prog_brk_off);
    if(prog_brk_off + __incr > (uint32_t)&_Min_Heap_Size)
        return (void*)-1;
    prog_brk_off += __incr;
    return old;
}

// void *
// _sbrk_r (struct _reent *ptr,
//      ptrdiff_t incr){return NULL;}

_off_t   _lseek (int __fildes, _off_t __offset, int __whence){return 0;}

// _off_t
// _lseek_r (struct _reent *ptr,
//      int fd,
//      _off_t pos,
//      int whence){return 0;}

int
_kill (int pid,
     int sig){return 0;}

// int
// _kill_r (struct _reent *ptr,
//      int pid,
//      int sig){return 0;}

int
_getpid (void) {return 0;}

// int
// _getpid_r (struct _reent *ptr){return 0;};

// int
// _isatty_r (struct _reent *ptr,int fd){return 0;}

int	_isatty (int fd){return 0;}

void    _exit       (int sig){
    while(1){}
}
}

