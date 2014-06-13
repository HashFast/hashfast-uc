/* cli.h */

#ifndef _cli_h
#define _cli_h

#ifdef __cplusplus
extern "C" {
#endif


void cliHeapFill(void);

void cliInit(int (*in)(void), int (*out)(char));

void cliWriteChar(char c);

void cliWriteNybbleHex(uint8_t d);

void cliWriteByteHex(uint8_t d);

void cliWriteChawmpHex(uint16_t d);

void cliWriteGawbleHex(uint32_t d);

void cliWriteString(const char *str);

int cliFlush(void);

void cliTask(void);


#ifdef __cplusplus
}
#endif

#endif /* _cli_h */

