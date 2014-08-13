/** @file spi_handler.h
 * @brief SPI interfaces
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

/**
 * SPI Request
 */
typedef struct spiRequestS {
    int which;
    int countA;
    int countB;
    const void *txA;
    const void *txB;
    void *rxA;
    void *rxB;
    volatile bool pending;
    struct spiRequestS *next;
} spiRequestT;

void spi_master_setup_pins(void);
void spi_master_setup(void);
void spi_handler(void);
void dac_write(bool, uint16_t);
status_code_t fpga_spi_write_packet(volatile avr32_spi_t *spi, const uint8_t *data, size_t len);
bool fpga_programmer(void);
status_code_t hf_spi_write_read(uint8_t, uint8_t *, int, uint8_t *, int);
status_code_t hf_spi_read_block(uint8_t, uint8_t, uint8_t *, size_t);
status_code_t hf_fpga_write_csr(uint8_t, uint16_t);
void spiQueue(spiRequestT *req);
void spiFpgaTwiReset(void);
int spiFpgaTwiStatus(void);
int spiFpgaTwiMasterWriteRead(uint8_t addr, const void *txBuffer, unsigned int txLength, void *rxBuffer, unsigned int rxLength);
void spiFpgaTwiMasterHandler(void);

extern uint16_t dac_value;

/* FPGA addressing */
#define FA_READ                      0x80

#define FA_MAGIC                        0
#define FA_VERSION                      1
#define FA_SWITCHES                     2
#define FA_CONFIG                       3
#define FA_ASIC_CONTROL                 4
#define FA_REG_STATUS                   5
#define FA_REG_ENABLE                   5
#define FA_VADC_CONFIG                  6
#define FA_IADC_CONFIG                  7
#define FA_OC_REFERENCE                 8

#define FA_PER_PHASE_CURRENT            10
#define FA_TACHO_0                      12
#define FA_TACHO_1                      13
#define FA_TACHO_2                      14
#define FA_TACHO_3                      15
#define FA_ROUTE_UC_SIN                 16
#define FA_ROUTE_UP_SIN                 17
#define FA_ROUTE_DOWN_SIN               18
#define FA_ROUTE_DIE0_SIN               19
#define FA_ROUTE_DIE1_SIN               20
#define FA_ROUTE_DIE2_SIN               21
#define FA_ROUTE_DIE3_SIN               22

#define FA_STICKY_PGOOD                 30
#define FA_ADC_CONTROL                  31
#define FA_I2C_BASE                     32
#define FA_I2C_PRER_LO                  (FA_I2C_BASE + 0)
#define FA_I2C_PRER_HI                  (FA_I2C_BASE + 1)
#define FA_I2C_CTR                      (FA_I2C_BASE + 2)
#define FA_I2C_CTR_EN                        0x80
#define FA_I2C_CTR_IEN                       0x40
#define FA_I2C_TXR                      (FA_I2C_BASE + 3)
#define FA_I2C_RXR                      (FA_I2C_BASE + 3)
#define FA_I2C_CR                       (FA_I2C_BASE + 4)
#define FA_I2C_CR_STA                        0x80
#define FA_I2C_CR_STO                        0x40
#define FA_I2C_CR_RD                         0x20
#define FA_I2C_CR_WR                         0x10
#define FA_I2C_CR_ACK                        0x08
#define FA_I2C_CR_IACK                       0x01
#define FA_I2C_SR                       (FA_I2C_BASE + 4)
#define FA_I2C_SR_RXACK                      0x80
#define FA_I2C_SR_BUSY                       0x40
#define FA_I2C_SR_AL                         0x20
#define FA_I2C_SR_TIP                        0x02
#define FA_I2C_SR_IF                         0x01

#define FA_POWER_SUPPLY_DATA            64

/* Common FPGA numbers */
#define F_MAGIC                         42
#define F_BIG_ENDIAN                    0x2
#define F_ASIC_UNRESET                  0x10    //!< Reset is active low
#define F_ASIC_PLL_BYPASS               0x8
#define F_ASIC_BAUD_MASK                0x7
#define F_FORCE_BAUD                    0x80    //!< Internal, not a supported FPGA bit

/* Serial routing definitions for the FPGA */
#define ROUTE_FROM_INACTIVE             0
#define ROUTE_FROM_UC_SOUT              1
#define ROUTE_FROM_UP_SOUT              2
#define ROUTE_FROM_DOWN_SOUT            3
#define ROUTE_FROM_DIE0_SOUT            4
#define ROUTE_FROM_DIE1_SOUT            5
#define ROUTE_FROM_DIE2_SOUT            6
#define ROUTE_FROM_DIE3_SOUT            7

/**
 * Power Supply Status
 *
 * Structure of power supply status that can be read from the FPGA.
 * You can read this in bits 'n pieces, but the FIFO that contains it will only
 * be flushed if you read the last word - vadc_status - so we simply set up a
 * DMA controller and periodically read the lot.
 */
struct fpga_ps_status {
    uint16_t phase_current[16];                 //!< 4 phases across 4 regulators
    uint16_t regulator_voltage[4];
    uint16_t regulator_current[4];              //!< = Sum of the phase currents
    uint16_t phase_status[4];                   //!< [P3|P2|P1|P0]
    uint16_t vadc_status;                       //!< [P3|P2|P1|P0]
};

#define PHASE_AE       0x1                      //!< Address Error in I2C communication
#define PHASE_DE       0x2                      //!< Data Error in I2C communication
#define PHASE_OVER     0x4                      //!< Over current
#define PHASE_LIMIT    0x8                      //!< Current limit occurred
