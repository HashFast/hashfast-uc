/* crc.h */

#ifndef _crc_h
#define _crc_h

#ifdef __cplusplus
extern "C" {
#endif


#define CRC_INITIAL      0xffffffff


uint32_t crcAccumulate(uint32_t crc, uint8_t *data, unsigned int len);


#ifdef __cplusplus
}
#endif

#endif /* _crc_h */

