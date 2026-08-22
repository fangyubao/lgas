#include "stm32f1xx.h"

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

/* Minimal Newlib system-call layer for a bare-metal target. */
int _close(int file)
{
    (void)file;
    errno = EBADF;
    return -1;
}

int _fstat(int file, struct stat *st)
{
    (void)file;
    if (st != NULL)
    {
        st->st_mode = S_IFCHR;
        return 0;
    }
    errno = EINVAL;
    return -1;
}

int _getpid(void)
{
    return 1;
}

int _isatty(int file)
{
    (void)file;
    return 1;
}

int _kill(int pid, int sig)
{
    (void)pid;
    (void)sig;
    errno = EINVAL;
    return -1;
}

off_t _lseek(int file, off_t offset, int whence)
{
    (void)file;
    (void)offset;
    (void)whence;
    errno = ESPIPE;
    return (off_t)-1;
}

ssize_t _read(int file, void *ptr, size_t len)
{
    (void)file;
    (void)ptr;
    (void)len;
    return 0;
}

ssize_t _write(int file, const void *ptr, size_t len)
{
    (void)file;
    (void)ptr;
    return (ssize_t)len;
}

void _exit(int status)
{
    (void)status;
    __disable_irq();
    for (;;)
    {
    }
}

void *_sbrk(ptrdiff_t increment)
{
    extern char end;
    extern char _estack;
    static char *heap_end;
    char *previous;

    if (heap_end == NULL)
    {
        heap_end = &end;
    }

    previous = heap_end;
    if ((increment > 0) && ((heap_end + increment) > &_estack))
    {
        errno = ENOMEM;
        return (void *)-1;
    }
    if ((increment < 0) && ((heap_end + increment) < &end))
    {
        errno = EINVAL;
        return (void *)-1;
    }

    heap_end += increment;
    return previous;
}
