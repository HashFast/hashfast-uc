/* twi.h */

#ifndef _twi_h
#define _twi_h

#ifdef __cplusplus
extern "C" {
#endif


#define TWI_SUCCESS              0
#define TWI_INVALID_ARG          1
#define TWI_BUSY                 2
#define TWI_NACK                 3
#define TWI_ARB_LOST             4
/* timeout cannot be returned by this module; this define is for use by a
   layer above this one. */
#define TWI_TIMEOUT              5


typedef struct {
    int master;
    int address;
    int freq;
    void *slaveRxBuffer;
    unsigned int slaveRxBufferSize;
    void (*callback)(unsigned int);
} twiConfigT;


void twiInit(void);

void twiReset(void);

void twiConfig(const twiConfigT *config);

int twiStatus(void);

int twiMasterWriteRead(uint8_t addr,
                       const void *txBuffer, unsigned int txLength,
                       void *rxBuffer, unsigned int rxLength);

int twiSlaveSetTx(const void *txBuffer, unsigned int txLength);

#ifdef __cplusplus
}
#endif

#endif /* _twi_h */

