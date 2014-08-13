/** @file spi_handler.c
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

#include "main.h"
#include "twi.h"
#include <spi_master.h>

#define FPGA_POLLING_INTERVAL   250         //!< How often (msec) to poll p/s data out of the FPGA

static bool fpga_programmed;                //!< Has the FPGA been programmed this powerup

static volatile spiRequestT *reqNext;
static volatile spiRequestT *reqLast;

/**
 * TWI to the FPGA
 */
static struct {
    struct {
        spiRequestT req;
        uint8_t r;
        uint8_t d;
    } req[2];
    enum {
        resetFS = 0,
        resetBFS,
        resetWaitFS,
        idleFS,
        sendingFS,
        sendWaitFS,
        receivingFS,
        receiveWaitFS,
        receiveDataFS,
        stopFS,
        stopWaitFS
    } state;
    uint8_t phases;
    int result;
    uint8_t addr;
    const uint8_t *tx;
    unsigned int txRemaining;
    uint8_t *rx;
    unsigned int rxRemaining;
} fpgatwi;

/**
 * Setup pins on master for SPI
 */
void spi_master_setup_pins(void) {
    gpio_enable_module_pin(SPI_MOSI, AVR32_SPI_MOSI_0_0_FUNCTION);
    gpio_enable_module_pin(SPI_SCLK, AVR32_SPI_SCK_0_0_FUNCTION);
    gpio_enable_module_pin(SPI_MISO, AVR32_SPI_MISO_0_0_FUNCTION);

    gpio_enable_module_pin(FPGA_SPI_EN_0, AVR32_SPI_NPCS_0_0_FUNCTION);
    gpio_enable_module_pin(FPGA_SPI_EN_1, AVR32_SPI_NPCS_1_0_FUNCTION);
    gpio_enable_module_pin(SPI_EN_2, AVR32_SPI_NPCS_2_0_FUNCTION);

    gpio_configure_pin(CDONE, GPIO_DIR_INPUT);
    gpio_configure_pin(CRESET, GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);

    gpio_configure_pin(FPGA_INIT_BAR, GPIO_DIR_INPUT);
}

static struct spi_device spi_conf = {.id = 0, };

/**
 * PDCA rx channel options
 */
static pdca_channel_options_t pdca_spi_rx = {
    .addr = NULL,
    .size = 0,
    .r_addr = NULL,
    .r_size = 0,
    .pid = AVR32_PDCA_PID_SPI_RX,
    .transfer_size = PDCA_TRANSFER_SIZE_BYTE
};

/**
 * PDCA tx channel options
 * Dummy write channel
 */
static pdca_channel_options_t pdca_spi_tx = {
    .addr = NULL,
    .size = 0,
    .r_addr = NULL,
    .r_size = 0,
    .pid = AVR32_PDCA_PID_SPI_TX,
    .transfer_size = PDCA_TRANSFER_SIZE_BYTE
};

/**
 * Initiaite transfer
 */
static void initiate_xfer(void) {

    AVR32_SPI.mr |= AVR32_SPI_MR_PCS_MASK;
    /* select device */
    if (AVR32_SPI.mr & AVR32_SPI_MR_PCSDEC_MASK) {
        AVR32_SPI.mr &= ~AVR32_SPI_MR_PCS_MASK | ((uint32_t) reqNext->which << AVR32_SPI_MR_PCS_OFFSET);
    } else {
        AVR32_SPI.mr &= ~(1 << (reqNext->which + AVR32_SPI_MR_PCS_OFFSET));
    }

    /* flush left over rx data */
    AVR32_SPI.rdr;

    pdca_spi_rx.addr = reqNext->rxA;
    pdca_spi_rx.size = reqNext->countA;
    pdca_spi_rx.r_addr = reqNext->rxB;
    pdca_spi_rx.r_size = reqNext->countB;
    pdca_init_channel(DMA_SPI_RX_CHANNEL, &pdca_spi_rx);
    pdca_disable_interrupt_reload_counter_zero(DMA_SPI_RX_CHANNEL);
    pdca_enable_interrupt_transfer_complete(DMA_SPI_RX_CHANNEL);
    pdca_enable(DMA_SPI_RX_CHANNEL);

    pdca_spi_tx.addr = (void *) reqNext->txA;
    pdca_spi_tx.size = reqNext->countA;
    pdca_spi_tx.r_addr = (void *) reqNext->txB;
    pdca_spi_tx.r_size = reqNext->countB;
    pdca_init_channel(DMA_SPI_TX_CHANNEL, &pdca_spi_tx);
    pdca_disable_interrupt_reload_counter_zero(DMA_SPI_TX_CHANNEL);
    pdca_disable_interrupt_transfer_complete(DMA_SPI_TX_CHANNEL);
    pdca_enable(DMA_SPI_TX_CHANNEL);
}

/**
 * SPI PDCA interrupt routine
 */
__attribute__((__interrupt__)) static void hf_spi_pdca_int_handler(void) {
    profileEnter(PROFILE_CHANNEL_SPIDMA_ISR);

    Disable_global_interrupt();

    pdca_disable_interrupt_transfer_complete(DMA_SPI_RX_CHANNEL);
    pdca_disable(DMA_SPI_RX_CHANNEL);
    pdca_disable_interrupt_transfer_complete(DMA_SPI_TX_CHANNEL);
    pdca_disable(DMA_SPI_TX_CHANNEL);
    /* deselect slave */
    AVR32_SPI.mr |= AVR32_SPI_MR_PCS_MASK;
    AVR32_SPI.cr = AVR32_SPI_CR_LASTXFER_MASK;
    reqNext->pending = false;
    if (reqLast == reqNext) {
        reqLast = NULL;
    }
    reqNext = reqNext->next;
    if (reqNext) {
        initiate_xfer();
    }

    Enable_global_interrupt();

    profileExit(PROFILE_CHANNEL_SPIDMA_ISR);
}

/**
 * SPI initialization - normal case
 */
void spi_master_setup(void) {
    spi_disable(&AVR32_SPI);
    spi_master_init(&AVR32_SPI);

    /* FPGA programming */
    spi_conf.id = 0;
    spi_master_setup_device(&AVR32_SPI, &spi_conf, SPI_MODE_0, 1000000, 0);

    /* FPGA register I/O */
    spi_conf.id = 1;
    spi_master_setup_device(&AVR32_SPI, &spi_conf, SPI_MODE_0, 1000000, 0);

    /* DAC */
    spi_conf.id = 2;
    spi_master_setup_device(&AVR32_SPI, &spi_conf, SPI_MODE_0, 1000000, 0);

    INTC_register_interrupt(&hf_spi_pdca_int_handler, AVR32_PDCA_IRQ_3, AVR32_INTC_INT1);

    spi_enable(&AVR32_SPI);
}

/**
 * Polled write 16 bit values to an FPGA CSR register
 * @param addr
 * @param val
 * @return
 */
status_code_t hf_fpga_write_csr(uint8_t addr, uint16_t val) {
    uint8_t buf[3];
    status_code_t sts;

    while (reqNext);

    spi_conf.id = 1;
    spi_select_device(&AVR32_SPI, &spi_conf);

    buf[0] = addr;
    buf[1] = (uint8_t) val;
    buf[2] = (uint8_t) (val >> 8);
    sts = spi_write_packet(&AVR32_SPI, buf, 3);

    spi_deselect_device(&AVR32_SPI, &spi_conf);

    return sts;
}

/**
 * Polled block read, emits a start address then reads the requested number of
 * bytes. Hacked from spi_read_packet.
 * @param id
 * @param address
 * @param data
 * @param len
 * @return
 */
status_code_t hf_spi_read_block(uint8_t id, uint8_t address, uint8_t *data, size_t len) {
    unsigned int timeout = SPI_TIMEOUT;
    uint8_t val;
    size_t i = 0;

    while (reqNext)
        ;
    spi_conf.id = id;
    spi_select_device(&AVR32_SPI, &spi_conf);

    /* ensure its a read */
    address |= FA_READ;

    /* emit the address byte */
    timeout = SPI_TIMEOUT;
    while (!spi_is_tx_ready(&AVR32_SPI)) {
        if (!timeout--) {
            return ERR_TIMEOUT;
        }
    }

    spi_write_single(&AVR32_SPI, address);

    while (len) {
        timeout = SPI_TIMEOUT;
        while (!spi_is_tx_ready(&AVR32_SPI)) {
            if (!timeout--) {
                return ERR_TIMEOUT;
            }
        }
        spi_write_single(&AVR32_SPI, CONFIG_SPI_MASTER_DUMMY);
        timeout = SPI_TIMEOUT;
        while (!spi_is_rx_ready(&AVR32_SPI)) {
            if (!timeout--) {
                return ERR_TIMEOUT;
            }
        }
        spi_read_single(&AVR32_SPI, &val);
        data[i] = val;
        i++;
        len--;
    }

    spi_deselect_device(&AVR32_SPI, &spi_conf);
    return STATUS_OK;
}

/**
 * Polled I/O function that can do a write, and then an optional read
 * @param id
 * @param write_data
 * @param write_len
 * @param read_data
 * @param read_len
 * @return 0 == STATUS_OK, otherwise a -ve number
 */
status_code_t hf_spi_write_read(uint8_t id, uint8_t *write_data, int write_len, uint8_t *read_data, int read_len) {
    status_code_t sts;

    while (reqNext)
        ;
    spi_conf.id = id;

    spi_select_device(&AVR32_SPI, &spi_conf);
    sts = spi_write_packet(&AVR32_SPI, write_data, write_len);
    if (sts == STATUS_OK && read_len) {
        sts = spi_read_packet(&AVR32_SPI, read_data, read_len);
    }
    spi_deselect_device(&AVR32_SPI, &spi_conf);

    return (sts);
}

#include "fpga_program.h"

/*
 * Reading Power Supply status from the FPGA
 */

static struct fpga_ps_status ps_status;
static uint8_t tachs[4];
static uint8_t power_good;

/**
 * Initialize the TWI structure for a given buffer
 * @param slot
 */
static void initTWISlot(int slot) {
    fpgatwi.req[slot].req.which = 1;
    fpgatwi.req[slot].req.txA = &fpgatwi.req[slot].r;
    fpgatwi.req[slot].req.rxA = &fpgatwi.req[slot].r;
    fpgatwi.req[slot].req.countA = 1;
    fpgatwi.req[slot].req.txB = &fpgatwi.req[slot].d;
    fpgatwi.req[slot].req.rxB = &fpgatwi.req[slot].d;
    fpgatwi.req[slot].req.countB = 1;
}

/**
 * Access a TWI register
 * @param slot
 * @param r
 * @param d
 */
static void accessTWIReg(int slot, uint8_t r, uint8_t d) {
    fpgatwi.req[slot].r = r;
    fpgatwi.req[slot].d = d;
    spiQueue(&fpgatwi.req[slot].req);
}

#define FPGA_TWI_SPEED          400000
#define FPGA_TWI_CLK_DIV      ((8000000 + 5 * FPGA_TWI_SPEED - 1) / (5 * FPGA_TWI_SPEED) - 1)

/**
 * Periodic task for master
 */
void spiFpgaTwiMasterHandler(void) {
    uint8_t c;

    switch (fpgatwi.state) {
    case resetFS:
        if (fpga_programmed && !fpgatwi.req[0].req.pending && !fpgatwi.req[1].req.pending) {
            initTWISlot(0);
            initTWISlot(1);
            accessTWIReg(0, FA_I2C_CTR, 0);
            accessTWIReg(1, FA_I2C_PRER_LO, FPGA_TWI_CLK_DIV & 0xff);
            fpgatwi.state = resetBFS;
        }
        break;
    case resetBFS:
        if (!fpgatwi.req[1].req.pending) {
            accessTWIReg(0, FA_I2C_PRER_HI, FPGA_TWI_CLK_DIV >> 8);
            accessTWIReg(1, FA_I2C_CTR, FA_I2C_CTR_EN);
            fpgatwi.state = resetWaitFS;
        }
        break;
    case resetWaitFS:
        if (!fpgatwi.req[1].req.pending)
            fpgatwi.state = idleFS;
        break;
    case idleFS:
        break;
    case sendingFS:
        if (!fpgatwi.req[1].req.pending) {
            accessTWIReg(0, FA_I2C_SR | FA_READ, 0);
            fpgatwi.state = sendWaitFS;
        }
        break;
    case sendWaitFS:
        if (!fpgatwi.req[0].req.pending) {
            if (fpgatwi.req[0].d & FA_I2C_SR_TIP)
                accessTWIReg(0, FA_I2C_SR | FA_READ, 0);
            else {
                if (fpgatwi.req[0].d & FA_I2C_SR_AL) {
                    fpgatwi.result = TWI_ARB_LOST;
                    fpgatwi.state = idleFS;
                } else if (fpgatwi.req[0].d & FA_I2C_SR_RXACK) {
                    fpgatwi.result = TWI_NACK;
                    accessTWIReg(0, FA_I2C_CR, FA_I2C_CR_STO);
                    fpgatwi.state = stopFS;
                } else {
                    if (fpgatwi.txRemaining) {
                        accessTWIReg(0, FA_I2C_TXR, *fpgatwi.tx++);
                        c = FA_I2C_CR_WR;
                        if (--fpgatwi.txRemaining == 0 && fpgatwi.rxRemaining == 0)
                            c |= FA_I2C_CR_STO;
                        accessTWIReg(1, FA_I2C_CR, c);
                        fpgatwi.state = sendingFS;
                    } else if (fpgatwi.rxRemaining) {
                        if (fpgatwi.phases == 2) {
                            accessTWIReg(0, FA_I2C_TXR, (fpgatwi.addr << 1) | 0x01);
                            accessTWIReg(1, FA_I2C_CR,
                            FA_I2C_CR_STA | FA_I2C_CR_WR);
                            fpgatwi.phases = 1;
                            fpgatwi.state = sendingFS;
                        } else {
                            c = FA_I2C_CR_RD;
                            if (fpgatwi.rxRemaining == 1)
                                c |= FA_I2C_CR_ACK | FA_I2C_CR_STO;
                            accessTWIReg(0, FA_I2C_CR, c);
                            fpgatwi.state = receivingFS;
                        }
                    } else {
                        fpgatwi.result = TWI_SUCCESS;
                        fpgatwi.state = idleFS;
                    }
                }
            }
        }
        break;
    case receivingFS:
        if (!fpgatwi.req[0].req.pending) {
            accessTWIReg(0, FA_I2C_SR | FA_READ, 0);
            fpgatwi.state = receiveWaitFS;
        }
        break;
    case receiveWaitFS:
        if (!fpgatwi.req[0].req.pending) {
            if (fpgatwi.req[0].d & FA_I2C_SR_TIP)
                accessTWIReg(0, FA_I2C_SR | FA_READ, 0);
            else {
                accessTWIReg(0, FA_I2C_RXR | FA_READ, 0);
                fpgatwi.state = receiveDataFS;
            }
        }
        break;
    case receiveDataFS:
        if (!fpgatwi.req[0].req.pending) {
            *fpgatwi.rx++ = fpgatwi.req[0].d;
            fpgatwi.rxRemaining--;
            if (fpgatwi.rxRemaining == 0) {
                fpgatwi.result = TWI_SUCCESS;
                fpgatwi.state = idleFS;
            } else {
                c = FA_I2C_CR_RD;
                if (fpgatwi.rxRemaining == 1)
                    c |= FA_I2C_CR_ACK | FA_I2C_CR_STO;
                accessTWIReg(0, FA_I2C_CR, c);
                fpgatwi.state = receivingFS;
            }
        }
        break;
    case stopFS:
        if (!fpgatwi.req[0].req.pending) {
            accessTWIReg(0, FA_I2C_SR | FA_READ, 0);
            fpgatwi.state = stopWaitFS;
        }
        break;
    case stopWaitFS:
        if (!fpgatwi.req[0].req.pending) {
            if (fpgatwi.req[0].d & FA_I2C_SR_TIP)
                accessTWIReg(0, FA_I2C_SR | FA_READ, 0);
            else
                fpgatwi.state = idleFS;
        }
        break;
    }
}

/**
 * Periodic task
 */
void spi_handler(void) {
    static enum {
        idleSS,
        waitForTachsSS,
        waitForPowerGoodStatusSS,
        waitForPSDataSS
    } state;
    static uint16_t lastPoll;
    static spiRequestT req;
    static uint8_t r;
    int i;

    switch (state) {
    case idleSS:
        if (elapsed_since(lastPoll) >= FPGA_POLLING_INTERVAL && ucinfo.board_initialized && ucinfo.powered_up) {
            lastPoll = msec_ticker;
            r = FA_TACHO_0 | FA_READ;
            req.which = 1;
            req.txA = &r;
            req.rxA = &r;
            req.countA = 1;
            req.txB = &tachs;
            req.rxB = &tachs;
            req.countB = sizeof(tachs);
            spiQueue(&req);
            state = waitForTachsSS;
        }
        break;
    case waitForTachsSS:
        if (!req.pending) {
            for (i = 0; i < 4; i++)
                gwq_update_tach(i, (uint16_t) tachs[i] * 30);
            r = FA_STICKY_PGOOD | FA_READ;
            req.which = 1;
            req.txA = &r;
            req.rxA = &r;
            req.countA = 1;
            req.txB = &power_good;
            req.rxB = &power_good;
            req.countB = sizeof(power_good);
            spiQueue(&req);
            state = waitForPowerGoodStatusSS;
        }
        break;
    case waitForPowerGoodStatusSS:
        if (!req.pending) {

#if 0
            /*
             * Power supply data, besides being unavailable due to ADCs not
             * being stuffed on modules, also does not appear to be implemented
             * in FPGA code. So don't bother asking for it until after it can be
             * tested.
             */
            r = FA_POWER_SUPPLY_DATA | FA_READ;
            req.which = 1;
            req.txA = &r;
            req.rxA = &r;
            req.countA = 1;
            req.txB = &ps_status;
            req.rxB = &ps_status;
            req.countB = sizeof(ps_status);
            spiQueue(&req);
            state = waitForPSDataSS;
#else /* 0 */
            state = idleSS;
#endif /* 0 */
        }
        break;
    case waitForPSDataSS:
        if (!req.pending) {
            gwq_update_phase_currents(ps_status.phase_current);
            gwq_update_regulator_voltage(ps_status.regulator_voltage);
            state = idleSS;
        }
        break;
    }
}


uint16_t dac_value = 0;

/**
 * Polled write to the DAC
 *
 *      DAC     Core Voltage
 *      ---     ------------
 *      0       1.0 Volts
 *      1024    0.8V
 *      2048    0.6V
 *      4095   -0.24V
 *
 * Values beyond 2048 are hardly useful.
 *
 * @param enable
 * @param data range is 0 - 0xfff
 */
void dac_write(bool enable, uint16_t data) {
    uint8_t dac_data[2];

    dac_value = data;

    dac_data[0] = (data >> 8) & 0xf;
    dac_data[0] |= (enable) ? 0x30 : 0x20;
    dac_data[1] = (uint8_t) data;

    while (reqNext)
        ;

    spi_conf.id = 2;
    spi_select_device(&AVR32_SPI, &spi_conf);

    spi_write_packet(&AVR32_SPI, dac_data, sizeof(dac_data));

    spi_deselect_device(&AVR32_SPI, &spi_conf);
}

/**
 * Delay for loops
 * @param loops
 */
static inline void delay(volatile int loops) {
    for (; loops > 0; loops--);
}

/**
 * FPGA programmer. This is only ever called once at system startup, to program
 * the FPGA. It uses polled I/O, and ties the system up for a while. The FPGA is
 * a OTP thing, but there seems no need to program the thing when the loading of
 * it takes less than a second.
 * @return
 */
bool fpga_programmer(void) {
    int pol;
    uint8_t pad[7] = {0};

    while (reqNext)
        ;
    spi_conf.id = 0;

    gpio_set_pin_low(CRESET);
    spi_select_device(&AVR32_SPI, &spi_conf);

    if (gpio_pin_is_high(FPGA_INIT_BAR)) {
        /*
         * Device has no image in the NVCM and is likely to be in SPI master
         * mode probing for an external SPI ROM. Must wait here for that to
         * time out, cannot find a specification for that period.
         */
        delay_msec(200);
    }
    /*
     * Waggle the SPI clock a bit. This seems necessary to wake the slave up.
     * Do not know if this is required when the device has an NVCM image or just
     * when breaking it out of SPI master mode.
     */
    spi_write_packet(&AVR32_SPI, pad, 1);

    gpio_set_pin_high(CRESET);

    /* delay at lease 300us */
    delay(10000);

    /* send image */
    spi_write_packet(&AVR32_SPI, fpga_program, sizeof(fpga_program));

    /* send the trailing padding... */
    spi_write_packet(&AVR32_SPI, pad, sizeof(pad));

    spi_deselect_device(&AVR32_SPI, &spi_conf);

    for (pol = 0; pol < 10; pol++) {
        if (gpio_pin_is_high(CDONE)) {
            fpga_programmed = true;
            return (true);
        }
        delay(100);
    }
    return (false);
}

/**
 * Queue SPI request
 * @param req
 */
void spiQueue(spiRequestT *req) {
    irqflags_t irq;

    req->pending = true;
    req->next = NULL;

    irq = cpu_irq_save();
    if (reqLast) {
        reqLast->next = req;
        reqLast = req;
    } else {
        reqNext = req;
        reqLast = req;
        initiate_xfer();
    }
    cpu_irq_restore(irq);
}

/**
 * Set FPGA in reset
 */
void spiFpgaTwiReset(void) {
    fpgatwi.state = resetFS;
}

/**
 * Get FPGA state
 * @return
 */
int spiFpgaTwiStatus(void) {
    int status;

    if (fpgatwi.state == idleFS) {
        status = fpgatwi.result;
    } else {
        status = TWI_BUSY;
    }
    return status;
}

/**
 * Master write and read result
 * @param addr
 * @param txBuffer
 * @param txLength
 * @param rxBuffer
 * @param rxLength
 * @return
 */
int spiFpgaTwiMasterWriteRead(uint8_t addr, const void *txBuffer, unsigned int txLength, void *rxBuffer, unsigned int rxLength) {
    int result;

    result = TWI_SUCCESS;
    if (fpgatwi.state != idleFS)
        result = TWI_BUSY;

    if (result == TWI_SUCCESS) {
        fpgatwi.addr = addr;
        fpgatwi.tx = txBuffer;
        fpgatwi.txRemaining = txLength;
        fpgatwi.rx = rxBuffer;
        fpgatwi.rxRemaining = rxLength;
        fpgatwi.phases = (txLength && rxLength) ? 2 : 1;
        fpgatwi.state = sendingFS;
        accessTWIReg(0, FA_I2C_TXR, (addr << 1) | (txLength ? 0x00 : 0x01));
        accessTWIReg(1, FA_I2C_CR, FA_I2C_CR_STA | FA_I2C_CR_WR);
    }

    return result;
}
