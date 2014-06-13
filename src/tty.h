/* tty.h */

#ifndef _tty_h
#define _tty_h

#ifdef __cplusplus
extern "C" {
#endif


void ttyInit(void);

int ttyRead(void);

int ttyWrite(char c);


#ifdef __cplusplus
}
#endif

#endif /* _tty_h */

