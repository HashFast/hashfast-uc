/** @file twi_handler.h
 * @brief TWI interfaces
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

/* TODO
 * Change this to 400000 for the "real" modules, after waveform integrity
 * confirmation.
 */
#define TWI_SPEED                   200000

#define TWICMD_POWERUP              1
#define TWICMD_POWERDOWN            2
#define TWICMD_ADDRESS              3
#define TWICMD_POWER_STATUS         4
#define TWICMD_ADDRESSING_COMPLETE  5
#define TWICMD_FPGA_ASIC_CTL        6
#define TWICMD_VERSION              7
#define TWICMD_REBOOT               8
#define TWICMD_BOARD_TEMPERATURES   9
#define TWICMD_TACHS               10
#define TWICMD_SERIAL_NUMBER       11
#define TWICMD_FAN_SET             12
#define TWICMD_DIE_SETTINGS        13
#define TWICMD_BAD_CORE_BITMAP     14
#define TWICMD_MIXED_BAUD          15
#define TWICMD_STARTUP             16
#define TWICMD_VOLTAGE_SET         17
#define TWICMD_SIZE                18

#define TWI_MASTER_ADDRESS          8
#define TWI_BROADCAST               0
#define TWI_SLAVE_STARTADDR         16

#define TWI_IR3566B_STARTADDR         0x08
#define TWI_IR3566B_PMBUS_STARTADDR   0x70

/* Worst case is for TWICMD_BAD_CORE_BITMAP read */
#define TWI_BUFSIZE                 (G1_CORES/8+4)

#define TWI_BUS_UC                  0
#define TWI_BUS_FPGA                1
#define TWI_BUS_COUNT               (TWI_BUS_FPGA + 1)

#define TWI_BUS_IR3566B             TWI_BUS_UC

/**
 * TWI Request Structure
 */
typedef struct twiRequestS {
    uint8_t addr;
    const uint8_t *tx;
    unsigned int txLength;
    uint8_t *rx;
    unsigned int rxLength;
    int pending;
    int result;
    struct twiRequestS *next;
} twiRequestT;

void twi_setup(void);
bool ir3566b_programmer(void);
void twi_master_setup(void);
void twi_slave_setup(uint8_t);
void twi_handler(void);
bool twi_broadcast(uint8_t, uint8_t);
bool twi_get_slave_data(uint8_t, uint8_t, uint8_t *, int);
bool twi_sync_rw(uint8_t bus, uint8_t dev, const uint8_t *tx_buffer, unsigned int tx_length, uint8_t *rx_buffer, unsigned int rx_length);
void twiQueueRequest(uint8_t bus, twiRequestT *req);
