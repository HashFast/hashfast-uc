/* hf_loader_p.h */

/*
    Copyright (c) 2013, 2014 HashFast Technologies LLC
*/

#ifndef _hf_loader_p_h
#define _hf_loader_p_h

#ifdef __cplusplus
extern "C" {
#endif


#define HF_LOADER_BLOCK_MAGIC    0x68666d31

#define HF_LOADER_SUFFIX_MAGIC   0x68664d31


typedef struct {
    uint32_t magic;
    uint32_t addr;
    uint32_t length;
} hfLoaderBlockHeaderT;

typedef struct {
    uint32_t magic;
    uint32_t length;
    uint32_t entry;
    uint32_t crc;
} hfLoaderAppSuffixT;


#ifdef __cplusplus
}
#endif

#endif /* _hf_loader_p_h */


