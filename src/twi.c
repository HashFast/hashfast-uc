/* twi.c */

/*
    Copyright (c) 2013, 2014 HashFast Technologies LLC
*/

#include <stdint.h>
#include <avr32/io.h>

#include <intc.h>
#include <sysclk.h>

#include "main.h"
#include "twi.h"


static struct {
    int modeMaster;
    volatile enum {masterIdleTS, masterReadingTS, masterWritingTS,
                   slaveTS} state;
    int result;
    struct {
        unsigned int count;
        union {
            const uint8_t *tx;
            uint8_t *rx;
        } ptr;
    } master;
    struct {
        uint8_t *rxBuffer;
        unsigned int rxBufferSize;
        unsigned int rxBufferIndex;
        const uint8_t *txBuffer;
        unsigned int txBufferLength;
        unsigned int txBufferIndex;
        void (*callback)(unsigned int);
    } slave;
} twi = {
    0, masterIdleTS, TWI_SUCCESS, {0, {0}}, {0, 0, 0, 0, 0, 0, 0}
};


__attribute__((__interrupt__)) static void twiInterrupt(void) {
    uint32_t sr, mask;

    profileEnter(PROFILE_CHANNEL_TWI_ISR);
    sr = AVR32_TWI.sr;
    mask = AVR32_TWI.imr;
    switch (twi.state) {
    case masterIdleTS:
        break;
    case masterReadingTS:
        if ((sr & mask) & AVR32_TWI_SR_ARBLST_MASK) {
            AVR32_TWI.idr = AVR32_TWI_IDR_RXRDY_MASK |
                            AVR32_TWI_IDR_TXCOMP_MASK |
                            AVR32_TWI_IDR_NACK_MASK |
                            AVR32_TWI_IDR_ARBLST_MASK;
            twi.result = TWI_ARB_LOST;
            twi.state = masterIdleTS;
        } else if ((sr & mask) & AVR32_TWI_SR_NACK_MASK) {
            AVR32_TWI.idr = AVR32_TWI_IDR_RXRDY_MASK |
                            AVR32_TWI_IDR_TXCOMP_MASK |
                            AVR32_TWI_IDR_NACK_MASK |
                            AVR32_TWI_IDR_ARBLST_MASK;
            twi.result = TWI_NACK;
            twi.state = masterIdleTS;
        } else if (((sr & mask) & AVR32_TWI_SR_RXRDY_MASK) &&
                   twi.master.count) {
            *twi.master.ptr.rx++ = (uint8_t) AVR32_TWI.rhr;
            twi.master.count--;
            if (twi.master.count == 1)
                AVR32_TWI.cr = AVR32_TWI_CR_STOP_MASK;
            if (twi.master.count == 0) {
                AVR32_TWI.idr = AVR32_TWI_IDR_RXRDY_MASK;
                AVR32_TWI.ier = AVR32_TWI_IER_TXCOMP_MASK;
            }
        } else if (((sr & mask) & AVR32_TWI_SR_TXCOMP_MASK) &&
                   twi.master.count == 0) {
            AVR32_TWI.idr = AVR32_TWI_IDR_TXCOMP_MASK |
                            AVR32_TWI_IDR_NACK_MASK |
                            AVR32_TWI_IDR_ARBLST_MASK;
            twi.result = TWI_SUCCESS;
            twi.state = masterIdleTS;
        }
        break;
    case masterWritingTS:
        if ((sr & mask) & AVR32_TWI_SR_ARBLST_MASK) {
            AVR32_TWI.idr = AVR32_TWI_IDR_TXRDY_MASK |
                            AVR32_TWI_IDR_TXCOMP_MASK |
                            AVR32_TWI_IDR_NACK_MASK |
                            AVR32_TWI_IDR_ARBLST_MASK;
            twi.result = TWI_ARB_LOST;
            twi.state = masterIdleTS;
        } else if ((sr & mask) & AVR32_TWI_SR_NACK_MASK) {
            AVR32_TWI.idr = AVR32_TWI_IDR_TXRDY_MASK |
                            AVR32_TWI_IDR_TXCOMP_MASK |
                            AVR32_TWI_IDR_NACK_MASK |
                            AVR32_TWI_IDR_ARBLST_MASK;
            twi.result = TWI_NACK;
            twi.state = masterIdleTS;
        } else if (((sr & mask) & AVR32_TWI_SR_TXRDY_MASK) &&
                   twi.master.count) {
            AVR32_TWI.thr = (uint32_t) *twi.master.ptr.tx++;
            twi.master.count--;
        } else if (((sr & mask) & AVR32_TWI_SR_TXRDY_MASK) &&
                   twi.master.count == 0) {
            AVR32_TWI.cr = AVR32_TWI_CR_STOP_MASK;
            AVR32_TWI.idr = AVR32_TWI_IDR_TXRDY_MASK;
            AVR32_TWI.ier = AVR32_TWI_IER_TXCOMP_MASK;
        } else if (((sr & mask) & AVR32_TWI_SR_TXCOMP_MASK) &&
                   twi.master.count == 0) {
            AVR32_TWI.idr = AVR32_TWI_IDR_TXCOMP_MASK |
                            AVR32_TWI_IDR_NACK_MASK |
                            AVR32_TWI_IDR_ARBLST_MASK;
            twi.result = TWI_SUCCESS;
            twi.state = masterIdleTS;
        }
        break;
    case slaveTS:
        /* It appears to me that Atmel's TWI slave has a serious design
           flaw.  Consider the following sequence:

           Scenario 1:

             The master is doing a write transaction

             Following the last byte of the transaction, but before the stop
             bit, the processor reads that last byte from the RHR.

             The processor disables interrupts for a while.

             The transaction finishes, so EOSACC goes true and SLVACC
             goes false.

             The master starts another write transaction so SLVACC goes true.

             The first byte is sent, so RXRDY goes true.

             The processor reenables interrupts.

             The processor sees SLVACC, RXRDY, and EOSACC all true.

           Scenario 2:

             The master is doing a write transaction

             Prior to the last byte of the transaction arriving, the
             processor disables interrupts.

             The last byte of the transaction finishes so RXRDY goes true.

             The stop bit is sent so EOSACC goes true and SLVACC goes false.

             The master starts another transaction so SLVACC goes true.

             The processor reenables interrupts.

             The processor sees SLVACC, RXRDY, and EOSACC all true.

           There does not appear to be any way for the processor to
           differentiate between those two scenarios.  It cannot determine
           whether the byte in the RHR is the last of the first transaction
           or the first byte of the second transaction.
        */
        if (sr & AVR32_TWI_SR_RXRDY_MASK) {
            if (twi.slave.rxBufferIndex < twi.slave.rxBufferSize)
                twi.slave.rxBuffer[twi.slave.rxBufferIndex++] = AVR32_TWI.rhr;
            else
                AVR32_TWI.rhr; /* toss */
        }
        if (sr & AVR32_TWI_SR_EOSACC_MASK) {
            if (twi.slave.rxBufferIndex) {
                if (twi.slave.callback)
                    twi.slave.callback(twi.slave.rxBufferIndex);
                twi.slave.rxBufferIndex = 0;
            }
            AVR32_TWI.ier = AVR32_TWI_IER_SVACC_MASK;
            AVR32_TWI.idr = AVR32_TWI_IER_TXRDY_MASK;
            mask &= ~AVR32_TWI_IER_TXRDY_MASK;
        }
        if ((sr & mask) & AVR32_TWI_SR_SVACC_MASK) {
            AVR32_TWI.idr = AVR32_TWI_IDR_SVACC_MASK;
            if (sr & AVR32_TWI_SR_SVREAD_MASK) {
                twi.slave.txBufferIndex = 0;
                /* master is reading */
                AVR32_TWI.ier = AVR32_TWI_IER_TXRDY_MASK;
                mask |= AVR32_TWI_IMR_TXRDY_MASK;
            }
        }
        if ((sr & mask) & AVR32_TWI_SR_TXRDY_MASK) {
            if (sr & AVR32_TWI_SR_NACK_MASK) {
                /* done writing */
#if 0
                /* Atmel's ASF driver does this, but it may not be correct.
                   Figure 19-24 of doc32059.pdf (the uc3b* manual), at the
                   trailing edge of NACK, says "Read RHR" which would seem
                   to mean that reading the RHR clears the NACK bit, and
                   that is what the ASF driver does.  That is dangerous.  If
                   we had interrupts locked out for a while, another TWI
                   transaction could have started and the master could have
                   already sent us a byte, which this read would lose.  The
                   description of the NACK bit in the SR section says "clear
                   on read", meaning the act of reading SR clears it.
                   Fortunately that description, "clear on read", appears to
                   be the accurate one as that's a far more sane design.
                */
                AVR32_TWI.rhr;
#endif
                AVR32_TWI.idr = AVR32_TWI_SR_TXRDY_MASK;
            } else {
                if (twi.slave.txBufferIndex < twi.slave.txBufferLength)
                    AVR32_TWI.thr =
                        twi.slave.txBuffer[twi.slave.txBufferIndex++];
                else
                    AVR32_TWI.thr = 42; /* dummy fill */
            }
        }
        break;
    default:
        /* cannot get here */
        AVR32_TWI.idr = ~ (uint32_t) 0;
        break;
    }
    profileExit(PROFILE_CHANNEL_TWI_ISR);
}

void twiInit(void) {

    twiReset();
    INTC_register_interrupt(&twiInterrupt, AVR32_TWI_IRQ, AVR32_INTC_INT0);
}

void twiReset(void) {

    AVR32_TWI.idr = ~ (uint32_t) 0;
    AVR32_TWI.cr = AVR32_TWI_CR_SWRST_MASK;
    AVR32_TWI.rhr;
    AVR32_TWI.sr;
    AVR32_TWI.cr = AVR32_TWI_CR_MSDIS_MASK | AVR32_TWI_CR_SVDIS_MASK;
    twi.modeMaster = -1;
}

void twiConfig(const twiConfigT *config) {
    uint32_t divisor;
    uint8_t prescale;

    if (config->master != twi.modeMaster)
        twiReset();
    if (config->freq) {
        divisor = (sysclk_get_pba_hz() + 2 * config->freq - 1) /
                  (2 * config->freq);
        if (divisor > 4)
            divisor -= 4;
        else
            divisor = 0;
        prescale = 0;
        while (divisor > 0xff && prescale <= 7) {
            divisor = (divisor + 1) >> 1;
            prescale++;
        }
        if (divisor <= 0xff && prescale <= 7)
            AVR32_TWI.cwgr = (divisor << AVR32_TWI_CWGR_CLDIV_OFFSET) |
                             (divisor << AVR32_TWI_CWGR_CHDIV_OFFSET) |
                             ((uint32_t) prescale <<
                              AVR32_TWI_CWGR_CKDIV_OFFSET);
    }

    AVR32_TWI.smr = ((uint32_t) config->address << AVR32_TWI_SMR_SADR_OFFSET);

    if (!config->master) {
        twi.slave.rxBuffer = config->slaveRxBuffer;
        twi.slave.rxBufferSize = config->slaveRxBufferSize;
        twi.slave.txBufferLength = 0;
        twi.slave.callback = config->callback;
        twi.result = TWI_SUCCESS;
    }

    if (config->master != twi.modeMaster) {
        if (config->master) {
            twi.state = masterIdleTS;
            AVR32_TWI.cr = AVR32_TWI_CR_SVDIS_MASK | AVR32_TWI_CR_MSEN_MASK;
        } else {
            twi.state = slaveTS;
            AVR32_TWI.cr = AVR32_TWI_CR_SVEN_MASK | AVR32_TWI_CR_MSDIS_MASK;
            AVR32_TWI.ier = AVR32_TWI_IER_SVACC_MASK |
                            AVR32_TWI_IER_EOSACC_MASK |
                            AVR32_TWI_IER_RXRDY_MASK;
        }
        twi.modeMaster = config->master;
    }
}

int twiStatus(void) {

    if (twi.state != masterIdleTS)
        return TWI_BUSY;

    return twi.result;
}

int twiMasterWriteRead(uint8_t addr,
                       const void *txBuffer, unsigned int txLength,
                       void *rxBuffer, unsigned int rxLength) {
    uint32_t iadr;
    uint32_t mmr;
    const uint8_t *tx;

    if ((txLength == 0 && rxLength == 0) || !twi.modeMaster)
        return TWI_INVALID_ARG;

    /* Atmel, like a surprising number of others, created a damaged
       I2C module.  I don't know why it seems to be so difficult to
       get right.  I've done one in Verilog before and didn't have
       any trouble coming up with a clean design that doesn't impose
       arbitrary restrictions on the types of transactions it can
       perform.  The Atmel one cannot do a write, repeated start,
       and read, unless the write is no more than three bytes. */
    if (rxLength && txLength > 3)
        return TWI_INVALID_ARG;

    /* this function will not be called by any isrs and there's no
       preemption so no locking is required here */
    if (twi.state != masterIdleTS)
        return TWI_BUSY;
    twi.state = rxLength ? masterReadingTS : masterWritingTS;

/*dbgPrintf("twi: tx %d rx %d\n", (int) txLength, (int) rxLength);*/
    mmr = addr << AVR32_TWI_MMR_DADR_OFFSET;
    if (rxLength) {
        mmr |= AVR32_TWI_MMR_MREAD_MASK;
        switch (txLength) {
        case 1:
            mmr |= AVR32_TWI_MMR_IADRSZ_ONE_BYTE <<
                   AVR32_TWI_MMR_IADRSZ_OFFSET;
            break;
        case 2:
            mmr |= AVR32_TWI_MMR_IADRSZ_TWO_BYTES <<
                   AVR32_TWI_MMR_IADRSZ_OFFSET;
            break;
        case 3:
            mmr |= AVR32_TWI_MMR_IADRSZ_THREE_BYTES <<
                   AVR32_TWI_MMR_IADRSZ_OFFSET;
            break;
        }
        iadr = 0;
        tx = txBuffer;
        while (txLength--) {
            iadr <<= 8;
            iadr |= *tx++;
        }
        AVR32_TWI.iadr = iadr;
        twi.master.ptr.rx = rxBuffer;
        twi.master.count = rxLength;
    } else {
        twi.master.ptr.tx = txBuffer;
        twi.master.count = txLength;
    }
    AVR32_TWI.mmr = mmr;
    if (rxLength) {
        if (rxLength == 1)
            AVR32_TWI.cr = AVR32_TWI_CR_START_MASK | AVR32_TWI_CR_STOP_MASK;
        else
            AVR32_TWI.cr = AVR32_TWI_CR_START_MASK;
        AVR32_TWI.ier = AVR32_TWI_IER_RXRDY_MASK |
                        AVR32_TWI_IER_ARBLST_MASK |
                        AVR32_TWI_IER_NACK_MASK;
    } else
        AVR32_TWI.ier = AVR32_TWI_IER_TXRDY_MASK |
                        AVR32_TWI_IER_ARBLST_MASK |
                        AVR32_TWI_IER_NACK_MASK;

    return TWI_SUCCESS;
}

int twiSlaveSetTx(const void *txBuffer, unsigned int txLength) {

    if ((txLength && !txBuffer) || twi.modeMaster)
        return TWI_INVALID_ARG;

    twi.slave.txBuffer = txBuffer;
    twi.slave.txBufferLength = txLength;

    return TWI_SUCCESS;
}

