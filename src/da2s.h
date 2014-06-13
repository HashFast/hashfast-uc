/* da2s.h */

#ifndef _da2s_h
#define _da2s_h

#ifdef __cplusplus
extern "C" {
#endif


extern char da2sEnabled;


void da2sEnable(char enable, uint32_t baud);

void da2sTask(void);


#ifdef __cplusplus
}
#endif

#endif /* _da2s_h */

