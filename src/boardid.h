/* boardid.h */

#ifndef _boardid_h
#define _boardid_h

#ifdef __cplusplus
extern "C" {
#endif


typedef enum {
    unknownBID,
    rev0_1_11_12_15BID,
    habaneroBID,
    iraBID
} boardidT;


extern boardidT boardid;


void boardidInit(void);


#ifdef __cplusplus
}
#endif

#endif /* _boardid_h */

