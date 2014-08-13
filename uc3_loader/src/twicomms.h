/* twicomms.h */

#ifndef _twicomms_h
#define _twicomms_h


#ifndef bool
#define bool int
#endif
#include "twi_handler.h"  /* get command defs from main app */


#ifdef __cplusplus
extern "C" {
#endif


#define TWICOMMS_MAX_DATA_PAYLOAD             64


#define TWICOMMS_MASTER_ADDR               TWI_MASTER_ADDRESS /* irrelevant */

#define TWICOMMS_SLAVE_STARTADDR           TWI_SLAVE_STARTADDR


#define TWICOMMS_CMD_POWERUP               TWICMD_POWERUP
#define TWICOMMS_CMD_POWERDOWN             TWICMD_POWERDOWN
#define TWICOMMS_CMD_ADDRESS               TWICMD_ADDRESS
#define TWICOMMS_CMD_ADDRESSING_COMPLETE   TWICMD_ADDRESSING_COMPLETE
#define TWICOMMS_CMD_VERSION               TWICMD_VERSION
#define TWICOMMS_CMD_REBOOT                TWICMD_REBOOT
#define TWICOMMS_CMD_SIZE                  TWICMD_SIZE

#define TWICOMMS_CMD_LOADER_START            102
#define TWICOMMS_CMD_LOADER_DATA             103
#define TWICOMMS_CMD_LOADER_STATUS           104



void twicommsInit(void);

void twicommsEnable(int enable);

void twicommsRestartAddr(int rebootSlaves);

uint32_t twicommsSlaveVersion(int slave);

int twicommsSlaveCRC(int slave, uint32_t *crc);

void twicommsSlaveSerialNumber(int slave, uint8_t *sn);

uint32_t twicommsSlaveSize(int slave);

uint32_t twicommsSlaveCmdSize(int slave);

int twicommsNumSlaves(uint8_t *slaves);

void twicommsRebootRequest(int loader, int slave);

void twicommsUploadBegin(int slave);

void twicommsUpload(int slave, const void *data, unsigned int length);

void twicommsUploadEnd(int slave);

int twicommsUploadStatus(int slave);

void twicommsTask(void);



#ifdef __cplusplus
}
#endif

#endif /* _twicomms_h */

