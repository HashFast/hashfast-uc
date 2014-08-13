/** @file ir3566b.h
 * @brief IR3566B register locations and bitmasks
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

#ifndef _ir3566b_h
#define _ir3566b_h

/* IR modules are defaulted into mobile mode */
#define IRA3566B_MOBILE_MODE

/* 0x12 I2C configuration */
#define IR3566B_REG_I2C                        0x12
#define IR3566B_REG_I2C_ENABLE                     0x80
#define IR3566B_REG_I2C_ADDR_POS                      0
#define IR3566B_REG_I2C_ADDR_MASK                  0x7f
/* 0x17 L1 boot voltage */
#define IR3566B_REG_L1_VBOOT                   0x17
/* 0x27 PMBus configuration */
#define IR3566B_REG_PMBUS                      0x27
#define IR3566B_REG_PMBUS_ADDR_MASK                0xf0
#define IR3566B_REG_PMBUS_ADDR_POS                    4
#define IR3566B_REG_PMBUS_DELAY_MODE               0x08
#define IR3566B_REG_PMBUS_DELAY_TIME_MASK          0x07
#define IR3566B_REG_PMBUS_DELAY_TIME_POS              0
/* 0x5D sets various faults and options */
#define IR3566B_REG_5D                         0x5d
#define IR3566B_REG_5D_VAUX_ENABLE                 0x40
/* 0x6A L1 manual voltage */
#define IR3566B_REG_L1_MANUAL_VID              0x6a
/* 0x98 input supply voltage */
#define IR3566B_REG_VIN_SUPPLY                 0x98
/* 0x9A LI output voltage (1/128V resolution) */
#define IR3566B_REG_L1_VOUT                    0x9a
/* 0x9C LI output current (2A resolution) */
#define IR3566B_REG_L1_IOUT                    0x9c
/* 0x9E temperature 1 value (1C resolution) */
#define IR3566B_REG_TEMP1                      0x9e
/* 0x9F temperature 2 value (1C resolution) */
#define IR3566B_REG_TEMP2                      0x9f
/* 0xA1 L1 fail codes (non-stick) */
#define IR3566B_REG_L1_FAIL_NONSTICK           0xa1
/* 0xA3 L1 fail codes (sticky) */
#define IR3566B_REG_L1_FAIL                    0xa3
#define IR3566B_REG_L1_FAIL_OVER_TEMP              0x80
#define IR3566B_REG_L1_FAIL_OVER_CURRENT           0x40
#define IR3566B_REG_L1_FAIL_VCPU_HIGH              0x20
#define IR3566B_REG_L1_FAIL_VCPU_LOW               0x10
#define IR3566B_REG_L1_FAIL_V12_LOW                0x08
#define IR3566B_REG_L1_FAIL_V3_LOW                 0x04
#define IR3566B_REG_L1_FAIL_PHASE_FAULT            0x02
#define IR3566B_REG_L1_FAIL_SLOW_OVER_CURRENT      0x01
/* 0xA6 L1 min/max current fault indicator */
#define IR3566B_REG_L1_CURRENT_FAULT           0xa6
#define IR3566B_REG_L1_CURRENT_FAULT_MIN           0x40
#define IR3566B_REG_L1_CURRENT_FAULT_MAX           0x80
/* 0xB8 which phase has a current fault */
#define IR3566B_REG_PHASE_FAULT                0xb8
#define IR3566B_REG_PHASE_FAULT_PHASE1             0x00
#define IR3566B_REG_PHASE_FAULT_PHASE2             0x01
#define IR3566B_REG_PHASE_FAULT_PHASE3             0x02
#define IR3566B_REG_PHASE_FAULT_PHASE4             0x03
#define IR3566B_REG_PHASE_FAULT_PHASE5             0x04
#define IR3566B_REG_PHASE_FAULT_PHASE6             0x05
#define IR3566B_REG_PHASE_FAULT_PHASE7             0x06
/* 0xCB L1 input current (1/8A resolution) */
#define IR3566B_REG_L1_IIN                     0xcb
/* 0xE2 clear fail codes */
#define IR3566B_REG_CLEAR_FAIL                 0xe2
#define IR3566B_REG_CLEAR_FAIL_L1_STICKY           0x02
/* 0xF4 clear phase fault */
#define IR3566B_REG_CLEAR_PHASE_FAULT          0xf4
#define IR3566B_REG_CLEAR_PHASE_FAULT_BIT          0x20

#endif /* _ir3566b_h */
