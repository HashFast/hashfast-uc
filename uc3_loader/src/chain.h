/* chain.h */

#ifndef _chain_h
#define _chain_h

#ifdef __cplusplus
extern "C" {
#endif


extern int chainMaster;

extern int chainState;


void chainInit(void);

void chainTask(void);


#ifdef __cplusplus
}
#endif

#endif /* _chain_h */

