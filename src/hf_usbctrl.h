/** @file hf_usbctrl.h
 * @brief USB control channel defines
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

#ifndef _hf_usbctrl_h
#define _hf_usbctrl_h

#ifdef __cplusplus
extern "C" {
#endif


#define HF_USBCTRL_REBOOT               0x60
#define HF_USBCTRL_VERSION              0x61
#define HF_USBCTRL_CONFIG               0x62
#define HF_USBCTRL_STATUS               0x63
/* 0x64 - 0x66 reserved (used by loader) */
#define HF_USBCTRL_SERIAL               0x67

#define HF_USBCTRL_NAME                 0x70
#define HF_USBCTRL_FAN                  0x71
#define HF_USBCTRL_POWER                0x72
#define HF_USBCTRL_FAN_PARMS            0x73
#define HF_USBCTRL_ASIC_PARMS           0x74
#define HF_USBCTRL_VOLTAGE              0x75

#define HF_USBCTRL_ASIC_CTRL            0x90
#define HF_USBCTRL_ASIC_CTRL_VALUE_RESET        0x8000
#define HF_USBCTRL_ASIC_CTRL_VALUE_PLL_BYPASS   0x4000

#define HF_USBCTRL_MODE                 0x91

/* core control */
#define HF_USBCTRL_CORE_OVERVIEW        0xa0
#define HF_USBCTRL_CORE_ENABLE          0xa1
#define HF_USBCTRL_CORE_DISABLE         0xa2
#define HF_USBCTRL_CORE_CLEAR           0xa3
#define HF_USBCTRL_CORE_STATUS          0xa4
#define HF_USBCTRL_CORE_DIE_STATUS      0xa5
#define HF_USBCTRL_CORE_ASIC_STATUS     0xa6

/* debug control */
#define HF_USBCTRL_DEBUG_BUFFER         0xd0
#define HF_USBCTRL_DEBUG_STREAM         0xd1
#define HF_USBCTRL_DEBUG_CLI            0xd2

#ifdef __cplusplus
}
#endif

#endif /* _hf_usbctrl_h */
