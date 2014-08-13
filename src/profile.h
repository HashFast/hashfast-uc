/** @file profile.h
 * @brief uC profiler
 *
 * @copyright
 * Copyright (c) 2014, HashFast Technologies LLC
 * All rights reserved.
 *
 * @page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   1.  Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *   2.  Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *   3.  Neither the name of HashFast Technologies LLC nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL HASHFAST TECHNOLOGIES LLC BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _profile_h
#define _profile_h

#ifdef __cplusplus
extern "C" {
#endif

#define PROFILE_CHANNELS                64

/* some fixed channels */
#define PROFILE_CHANNEL_TC_ISR           0
#define PROFILE_CHANNEL_GPIO_ISR         1
#define PROFILE_CHANNEL_SPIDMA_ISR       2
#define PROFILE_CHANNEL_TWI_ISR          3
#define PROFILE_CHANNEL_UART_ISR         4
#define PROFILE_CHANNEL_UARTTXDMA_ISR    5
#define PROFILE_CHANNEL_UARTRXDMA_ISR    6
#define PROFILE_CHANNEL_USB_ISR          7
#define PROFILE_CHANNEL_MAINLOOP         8

#define PROFILE_FLAGS_ISR             0x01

#ifdef FEATURE_PROFILE

void profileInit(void);
void profileReset(void);
void profileConfigure(unsigned int channel, uint8_t flags);
void profileEnter(unsigned int channel);
void profileExit(unsigned int channel);
int profileCLI(int first, int parmCount, uint32_t *parms);

#else /* FEATURE_PROFILE */

#define profileInit()
#define profileReset()
#define profileConfigure(channel, flags)
#define profileEnter(channel)
#define profileExit(channel)

#endif /* FEATURE_PROFILE */

#ifdef __cplusplus
}
#endif

#endif /* _profile_h */
