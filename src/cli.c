/** @file cli.c
 * @brief Command line interface over USB control channel.
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

#include <string.h>
#include <stdint.h>
#include <avr32/io.h>

#include "main.h"
#include "boardid.h"
#include "mpu.h"
#include "cli.h"

#ifdef FEATURE_DEBUG_CLI

/* only define this if heap is not used */
#define HEAP_CHECK

/* linker defined symbols */
extern uint32_t __heap_start__[];
extern uint32_t __heap_end__[];
extern uint32_t _stack[];
extern uint32_t _estack[];

static int help(int first, int parmCount, uint32_t *parms);
static int info(int first, int parmCount, uint32_t *parms);
static int irf(int first, int parmCount, uint32_t *parms);
static int dumpBytes(int first, int parmCount, uint32_t *parms);
static int dumpChawmps(int first, int parmCount, uint32_t *parms);
static int dumpGawbles(int first, int parmCount, uint32_t *parms);
static int writeFPGA(int first, int parmCount, uint32_t *parms);
static int dumpFPGA(int first, int parmCount, uint32_t *parms);
static int gpio(int first, int parmCount, uint32_t *parms);
static int writeBytes(int first, int parmCount, uint32_t *parms);
static int writeChawmps(int first, int parmCount, uint32_t *parms);
static int writeGawbles(int first, int parmCount, uint32_t *parms);
static int dumpTemperatures(int first, int parmCount, uint32_t *parms);
static int fan(int first, int parmCount, uint32_t *parms);
static int stackCheck(int first, int parmCount, uint32_t *parms);
static int mpuEnable(int first, int parmCount, uint32_t *parms);
static int i2c(int first, int parmCount, uint32_t *parms);
static int voltOffset(int first, int parmCount, uint32_t *parms);
#ifdef HEAP_CHECK
static int heapInit(int first, int parmCount, uint32_t *parms);
static int heapCheck(int first, int parmCount, uint32_t *parms);
#endif

static struct {
    int (*in)(void);
    int (*out)(char);
    enum {
        resetMS,
        promptMS,
        inputMS,
        parseMS,
        commandMS,
        commandContinueMS
    } state;
    struct {
        char rx[72];
        int length;
        int parmCount;
        uint32_t parms[16 + 1];
    } input;
    struct {
        char tx[96];
        int length;
        int sent;
    } output;
    int command;
} cli;

static const struct {
    const char *cmd;
    int (*func)(int, int, uint32_t *);
} commands[] = {
    {"?", help},
    {"db", dumpBytes},
    {"dc", dumpChawmps},
    {"dg", dumpGawbles},
    {"fan", fan},
    {"fpgaread", dumpFPGA},
    {"fpgawrite", writeFPGA},
    {"gpio", gpio},
    {"gwqstats", gwq_cli_stats},
#ifdef HEAP_CHECK
    {"heapcheck", heapCheck},
    {"heapinit", heapInit},
#endif
    {"i2c", i2c},
    {"info", info},
    {"irf", irf},
    {"help", help},
    {"mpudump", mpuDump},
    {"mpuenable", mpuEnable},
#ifdef FEATURE_PROFILE
    {"profile", profileCLI},
#endif /* FEATURE_PROFILE */
    {"stackcheck", stackCheck},
    {"temperature", dumpTemperatures},
    {"uartstats", uart_cli_stats},
    {"usbstats", usb_cli_stats},
    {"vo", voltOffset},
    {"wb", writeBytes},
    {"wc", writeChawmps},
    {"wg", writeGawbles},
};

static const char *helpStrings[] = {
    "All parameters are hexadecimal.\n",
    "db <addr> <count>            dump <count> bytes from <addr>\n",
    "dc <addr> <count>            dump <count> chawmps from <addr>\n",
    "dg <addr> <count>            dump <count> gawbles from <addr>\n",
    "fan                          dump fan data\n",
    "fan <module> <fan> <value>   set fan speed\n",
    "fpgaread <addr>              read fpga register <addr>\n",
    "fpgawrite <addr> <data>      write <data> to fpga register <addr>\n",
    "gpio <bank> <bit>            read gpio pin\n",
    "gwqstats                     dump gwq stats\n",
#ifdef HEAP_CHECK
    "heapcheck                    check if heap has been written since heapinit\n",
    "heapinit                     fill heap with fixed pattern\n",
#endif
    "i2c <addr> <data...> <read>  write <data> to dev <addr>, read <read> bytes\n",
    "info                         dump misc config info\n",
    "irf                          program ir regulator\n",
    "mpudump                      dump mpu exception info\n",
    "mpuenable                    enable mpu\n",
#ifdef FEATURE_PROFILE
    "profile <reset>              show profile stats, optionally reset stats\n",
#endif /* FEATURE_PROFILE */
    "stackcheck                   check how much of stack has been used\n",
    "temperature                  dump temperature data\n",
    "uartstats                    dump uart stats\n",
    "usbstats                     dump usb stats\n",
    "vo <0 to 15> [die]           IR only Voltage Offset, Die 0-3 optional\n",
    "wb <addr> <data>             write byte <data> to <addr>\n",
    "wc <addr> <data>             write chawmp <data> to <addr>\n",
    "wg <addr> <data>             write gawble <data> to <addr>\n",
};

static const char parameterError[] = "parameter error\n";

static const char alignmentError[] = "alignment error\n";

/**
 * Display help text
 * @param first
 * @param paramCount
 * @param params
 * @return success
 */
static int help(int first, int parmCount, uint32_t *parms) {
    static int i;

    if (first)
        i = 0;

    cliWriteString(helpStrings[i++]);

    return (i >= sizeof(helpStrings) / sizeof(helpStrings[0])) ? 1 : 0;
}

/**
 * Displays system information
 * @param first
 * @param paramCount
 * @param params
 * @return success
 */
static int info(int first, int parmCount, uint32_t *parms) {
    static int chunk, module;
    serial_number_t serial;
    uint32_t u;
    int i;
    int done;

    if (first) {
        chunk = 0;
        module = 0;
    }

    done = 0;
    switch (chunk++) {
    case 0:
        cliWriteString("chain config ");
        cliWriteByteHex(ucinfo.chain_configuration);
        cliWriteString(" master ");
        cliWriteByteHex(ucinfo.master);
        cliWriteString(" num slaves ");
        cliWriteByteHex(ucinfo.num_slaves);
        cliWriteChar('\n');
        break;
    case 1:
        cliWriteString("cores ");
        cliWriteChawmpHex(ucinfo.total_cores);
        cliWriteString(" good cores ");
        cliWriteChawmpHex(ucinfo.total_good_cores);
        cliWriteChar('\n');
        break;
    case 2:
        if (module <= ucinfo.num_slaves) {
            cliWriteString("module ");
            cliWriteNybbleHex(module);
            cliWriteString(" type ");
            cliWriteByteHex(module_type(module));
            cliWriteString(" version ");
            cliWriteGawbleHex(fw_version(module));
            if (fw_crc(module, &u)) {
                cliWriteString(" crc ");
                cliWriteGawbleHex(u);
            }
            cliWriteChar('\n');
        } else
            done = 1;
        break;
    case 3:
        module_serial(module, &serial);
        if (serial.magic == U_MAGIC && serial.start_barrier[0] == 'H' && serial.start_barrier[1] == 'F' && serial.start_barrier[2] == ':' && serial.start_barrier[3] == ':' && serial.stop_barrier[0] == ':' && serial.stop_barrier[1] == ':' && serial.stop_barrier[2] == 'F' && serial.stop_barrier[3] == 'H') {
            cliWriteString("  serial number ");
            for (i = 0; i < sizeof(serial.unique_id); i++) {
                cliWriteByteHex(serial.unique_id[i]);
                cliWriteChar(' ');
            }
            cliWriteChar('\n');
        } else {
            cliWriteString("  no serial number\n");
        }
        module++;
        chunk -= 2;
        break;
    default:
        done = 1;
        break;
    }

    return done;
}

/**
 * Program IR parts
 * @param first
 * @param paramCount
 * @param params
 * @return success
 */
static int irf(int first, int parmCount, uint32_t *parms) {
    if (!ir3566b_programmer())
        cliWriteString("program failed\n");
    return 1;
}

/**
 * Dump bytes
 * @param first
 * @param paramCount
 * @param params
 * @return bytesToGo
 */
static int dumpBytes(int first, int parmCount, uint32_t *parms) {
    static uint32_t nextAddr;
    static uint32_t lastCount;
    static uint32_t bytesToGo;
    volatile uint8_t *ptr;
    unsigned int i;

    if (first) {
        if (parmCount) {
            nextAddr = parms[0];
            if (parmCount > 1)
                lastCount = parms[1];
        }
        if (lastCount == 0)
            lastCount = 1;
        bytesToGo = lastCount;
    }

    cliWriteGawbleHex(nextAddr);
    cliWriteString(": ");
    ptr = (volatile uint8_t *) nextAddr;
    for (i = 0; i < (nextAddr & 0x0f); i++) {
        cliWriteString("   ");
        if (i == 7)
            cliWriteChar(' ');
    }
    for (; i < 16 && bytesToGo; i++) {
        cliWriteByteHex(*ptr++);
        cliWriteChar(' ');
        if (i == 7)
            cliWriteChar(' ');
        bytesToGo--;
    }
    cliWriteChar('\n');
    nextAddr = (uint32_t) ptr;

    return bytesToGo ? 0 : 1;
}

/**
 * Dump double bytes (16-bit words)
 * @param first
 * @param paramCount
 * @param params
 * @return chawmpsToGo
 */
static int dumpChawmps(int first, int parmCount, uint32_t *parms) {
    static uint32_t nextAddr;
    static uint32_t lastCount;
    static uint32_t chawmpsToGo;
    volatile uint16_t *ptr;
    unsigned int i;

    if (first && parmCount && (parms[0] & 1)) {
        cliWriteString(alignmentError);
        return 1;
    }

    if (first) {
        if (parmCount) {
            nextAddr = parms[0];
            if (parmCount > 1)
                lastCount = parms[1];
        }
        if (lastCount == 0)
            lastCount = 1;
        chawmpsToGo = lastCount;
    }

    cliWriteGawbleHex(nextAddr);
    cliWriteString(": ");
    ptr = (volatile uint16_t *) nextAddr;
    for (i = 0; i < (nextAddr & 0x0f); i += 2) {
        cliWriteString("     ");
        if (i == 6)
            cliWriteChar(' ');
    }
    for (; i < 16 && chawmpsToGo; i += 2) {
        cliWriteChawmpHex(*ptr++);
        cliWriteChar(' ');
        if (i == 6)
            cliWriteChar(' ');
        chawmpsToGo--;
    }
    cliWriteChar('\n');
    nextAddr = (uint32_t) ptr;

    return chawmpsToGo ? 0 : 1;
}

/**
 * Dump 32-bit words
 * @param first
 * @param paramCount
 * @param params
 * @return gawblesToGo
 */
static int dumpGawbles(int first, int parmCount, uint32_t *parms) {
    static uint32_t nextAddr;
    static uint32_t lastCount;
    static uint32_t gawblesToGo;
    volatile uint32_t *ptr;
    unsigned int i;

    if (first && parmCount && (parms[0] & 3)) {
        cliWriteString(alignmentError);
        return 1;
    }

    if (first) {
        if (parmCount) {
            nextAddr = parms[0];
            if (parmCount > 1)
                lastCount = parms[1];
        }
        if (lastCount == 0)
            lastCount = 1;
        gawblesToGo = lastCount;
    }

    cliWriteGawbleHex(nextAddr);
    cliWriteString(": ");
    ptr = (volatile uint32_t *) nextAddr;
    for (i = 0; i < (nextAddr & 0x0f); i += 4) {
        cliWriteString("         ");
        if (i == 4)
            cliWriteChar(' ');
    }
    for (; i < 16 && gawblesToGo; i += 4) {
        cliWriteGawbleHex(*ptr++);
        cliWriteChar(' ');
        if (i == 4)
            cliWriteChar(' ');
        gawblesToGo--;
    }
    cliWriteChar('\n');
    nextAddr = (uint32_t) ptr;

    return gawblesToGo ? 0 : 1;
}

/**
 * Dump registers in the FPGA
 * @param first
 * @param paramCount
 * @param params
 * @return success
 */
static int dumpFPGA(int first, int parmCount, uint32_t *parms) {
    static uint8_t addr = 0;
    uint8_t d;

    if (parmCount >= 1)
        addr = (uint8_t) parms[0];
    else
        addr++;
    hf_spi_read_block(1, addr, &d, 1);
    cliWriteByteHex(d);
    cliWriteChar('\n');

    return 1;
}

/**
 * Write register in the FPGA
 * @param first
 * @param paramCount
 * @param params
 * @return success
 */
static int writeFPGA(int first, int parmCount, uint32_t *parms) {
    if (parmCount != 2)
        cliWriteString(parameterError);
    else
        fpga_reg_write(parms[0], parms[1]);
    return 1;
}

/**
 * Turn on/off GPIO pins
 * @param first
 * @param paramCount
 * @param params
 * @return success
 */
static int gpio(int first, int parmCount, uint32_t *parms) {
    uint32_t which;

    which = parms[0];
    if (which >= 10)
        which -= 10;
    which *= 32;
    which += parms[1];
    if (parmCount == 2) {
        cliWriteNybbleHex(gpio_get_pin_value(which) ? 1 : 0);
        cliWriteChar('\n');
    } else if (parmCount == 3) {
        switch (parms[2]) {
        case 0:
            gpio_configure_pin(which, GPIO_DIR_OUTPUT | GPIO_INIT_LOW);
            break;
        case 1:
            gpio_configure_pin(which, GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);
            break;
        default:
            gpio_configure_pin(which, GPIO_DIR_INPUT);
            cliWriteString("pin configured as input\n");
            break;
        }
    } else
        cliWriteString(parameterError);

    return 1;
}

/**
 * Control fans
 * @param first
 * @param paramCount
 * @param params
 * @return success
 */
static int fan(int first, int parmCount, uint32_t *parms) {
    static unsigned int module;
    int i;
    int done;

    if (parmCount == 3) {
        fan_set(parms[0], parms[1], parms[2]);
        done = 1;
    } else if (parmCount == 0) {
        if (first)
            module = 0;

        cliWriteString("module ");
        cliWriteNybbleHex((uint8_t) module);
        cliWriteString(" tachometers: ");
        for (i = 0; i < 4; i++) {
            cliWriteChawmpHex(gwq_get_tach(module * 4 + i));
            cliWriteChar(' ');
        }
        cliWriteChar('\n');
        module++;

        done = (module > ucinfo.num_slaves) ? 1 : 0;
    } else {
        cliWriteString(parameterError);
        done = 1;
    }

    return done;
}

/**
 * Display board temperatures
 * @param first
 * @param paramCount
 * @param params
 * @return success
 */
static int dumpTemperatures(int first, int parmCount, uint32_t *parms) {
    static unsigned int module;
    static int die;
    int i;

    if (first) {
        module = 0;
        die = 0;
    }
    if (!die) {
        cliWriteString("module ");
        cliWriteNybbleHex((uint8_t) module);
        cliWriteString(" regulator temps: ");
        for (i = 0; i < 4; i++) {
            cliWriteChawmpHex(gwq_get_board_temperature(module * 4 + i));
            cliWriteChar(' ');
        }
        cliWriteChar('\n');
        if (++module > ucinfo.num_slaves) {
            module = 0;
            die = 1;
        }
    } else {
        cliWriteString("module ");
        cliWriteNybbleHex((uint8_t) module);
        cliWriteString(" die temps: ");
        for (i = 0; i < 4; i++) {
            cliWriteChawmpHex(gwq_get_die_temperature(module * 4 + i));
            cliWriteChar(' ');
        }
        cliWriteChar('\n');
        module++;
    }

    return (module > ucinfo.num_slaves) ? 1 : 0;
}

/**
 * Write a byte
 * @param first
 * @param paramCount
 * @param params
 * @return success
 */
static int writeBytes(int first, int parmCount, uint32_t *parms) {
    volatile uint8_t *ptr;
    unsigned int i;

    if (parmCount < 2)
        cliWriteString(parameterError);
    else {
        ptr = (volatile uint8_t *) parms[0];
        for (i = 1; i < parmCount; i++)
            *ptr++ = (uint8_t) parms[i];
    }

    return 1;
}

/**
 * Write a double byte (16-bit word)
 * @param first
 * @param paramCount
 * @param params
 * @return success
 */
static int writeChawmps(int first, int parmCount, uint32_t *parms) {
    volatile uint16_t *ptr;
    unsigned int i;

    if (parmCount < 2)
        cliWriteString(parameterError);
    else if (parms[0] & 1)
        cliWriteString(alignmentError);
    else {
        ptr = (volatile uint16_t *) parms[0];
        for (i = 1; i < parmCount; i++)
            *ptr++ = (uint16_t) parms[i];
    }

    return 1;
}

/**
 * Write a 32-bit word
 * @param first
 * @param paramCount
 * @param params
 * @return success
 */
static int writeGawbles(int first, int parmCount, uint32_t *parms) {
    volatile uint32_t *ptr;
    unsigned int i;

    if (parmCount < 2)
        cliWriteString(parameterError);
    else if (parms[0] & 3)
        cliWriteString(alignmentError);
    else {
        ptr = (volatile uint32_t *) parms[0];
        for (i = 1; i < parmCount; i++)
            *ptr++ = (uint32_t) parms[i];
    }

    return 1;
}

/**
 * Show stack size and unused
 * @param first
 * @param paramCount
 * @param params
 * @return success
 */
static int stackCheck(int first, int parmCount, uint32_t *parms) {
    uint32_t i;
    uint32_t *ptr;

    for (i = 0, ptr = _stack; ptr < _estack && *ptr == 0xdeadbeef; i++, ptr++)
        ;

    cliWriteString("stack size ");
    cliWriteGawbleHex((_estack - _stack) * 4);
    cliWriteString(", unused ");
    cliWriteGawbleHex(i * 4);
    cliWriteChar('\n');

    return 1;
}

/**
 * Fill the heap with 0xDEADBEEF
 * @param first
 * @param paramCount
 * @param params
 */
void cliHeapFill(void) {
    uint32_t *ptr;

    ptr = __heap_start__;
    while (ptr < __heap_end__)
        *ptr++ = 0xdeadbeef;
    spurious1 = 0xdeadbeef;
}

#ifdef HEAP_CHECK

/**
 * Initialize the heap by filling it with 0xDEADBEEF
 * @param first
 * @param paramCount
 * @param params
 * @return success
 */
static int heapInit(int first, int parmCount, uint32_t *parms) {
    cliHeapFill();
    return 1;
}

/**
 * Check the heap for 0xDEADBEEF
 * @param first
 * @param paramCount
 * @param params
 * @return success
 */
static int heapCheck(int first, int parmCount, uint32_t *parms) {
    static int chunk;
    uint32_t *ptr;
    int done;

    if (first)
        chunk = 0;

    done = 0;

    switch (chunk++) {
    case 0:
        cliWriteString("spurious (");
        cliWriteGawbleHex((uint32_t) &spurious1);
        if (spurious1 == 0xdeadbeef)
            cliWriteString(") untouched\n");
        else {
            cliWriteString(") modified: ");
            cliWriteGawbleHex(spurious1);
            cliWriteChar('\n');
        }
        break;
    case 1:
        for (ptr = __heap_start__; ptr < __heap_end__ && *ptr == 0xdeadbeef;
                ptr++)
            ;
        cliWriteString("heap (");
        cliWriteGawbleHex((uint32_t) __heap_start__);
        cliWriteString(" - ");
        cliWriteGawbleHex((uint32_t) __heap_end__ - 1);
        if (ptr == __heap_end__)
            cliWriteString(") untouched\n");
        else {
            cliWriteString(") modified starting at ");
            cliWriteGawbleHex((uint32_t) ptr);
            cliWriteChar('\n');
        }
        /* fall through */
        /* no break */
    default:
        done = 1;
    }

    return done;
}
#endif /* HEAP_CHECK */

/**
 * Enable the MPU
 * @param first
 * @param paramCount
 * @param params
 * @return success
 */
static int mpuEnable(int first, int parmCount, uint32_t *parms) {

#ifdef HEAP_CHECK
    mpuSetup(1);
#else /* HEAP_CHECK */
    mpuSetup(0);
#endif /* HEAP_CHECK */

    return 1;
}

/**
 * Send data over I2C
 * @param first
 * @param paramCount
 * @param params
 * @return success
 */
static int i2c(int first, int parmCount, uint32_t *parms) {
    static uint8_t rxData[64];
    static int chunk;
    uint8_t txData[16];
    unsigned int i;
    int done;

    done = 0;
    if (parmCount < 2 || parmCount > sizeof(txData) + 2 || parms[parmCount - 1] > sizeof(rxData)) {
        cliWriteString(parameterError);
        done = 1;
    } else if (first) {
        for (i = 0; i < parmCount - 2; i++)
            txData[i] = (uint8_t) parms[i + 1];
        if (twi_sync_rw(parms[0] >> 8, parms[0] & 0xff, txData, parmCount - 2, rxData, parms[parmCount - 1]))
            chunk = 0;
        else {
            cliWriteString("i2c op failed\n");
            done = 1;
        }
    } else {
        for (i = chunk * 16; i < (chunk + 1) * 16 && i < parms[parmCount - 1];
                i++) {
            cliWriteByteHex(rxData[i]);
            cliWriteChar(' ');
            if ((i & 0x0f) == 0x07)
                cliWriteChar(' ');
        }
        cliWriteChar('\n');
        chunk++;
        if (i >= parms[parmCount - 1])
            done = 1;
    }

    return done;
}

/**
 * Set a voltage offset on IR parts
 * @param first
 * @param paramCount
 * @param params
 * @return success
 */
static int voltOffset(int first, int parmCount, uint32_t *parms) {
    static uint8_t rxData[2];
    uint8_t txData[2] = {0x26};
    unsigned int i;

    //reg 26 -7-8 high nyb

    if (parmCount >= 1 && parmCount <= 2) {
        if (parms[0] >= 65) {
            cliWriteString("offset out of range! [0-15]\n");
        } else {
            txData[1] = (uint8_t) parms[0] * 16;
            if (parmCount == 1) {
                for (i = 0; i <= 3; i++) {
                    if (!twi_sync_rw(TWI_BUS_IR3566B, i + TWI_IR3566B_STARTADDR, txData, 2, NULL, 0)) {
                        cliWriteString("i2c op failed\n");
                    }
                }
            } else {
                if (parms[1] >= 4) {
                    cliWriteString("Die number out of range! [0-3]\n");
                } else {
                    if (!twi_sync_rw(TWI_BUS_IR3566B, parms[1] + TWI_IR3566B_STARTADDR, txData, 2, NULL, 0)) {
                        cliWriteString("i2c op failed\n");
                    }
                }
            }
        }
    }

    // FIXME: is this necessary, or can we try to read the values a few times on fail?
    delay_msec(10);

    // FIXME: is this the read code, or is 0x26 still correct here?
    // txData[0] = 0x17;

    for (i = 0; i <= 3; i++) {
        if (!twi_sync_rw(TWI_BUS_IR3566B, i + TWI_IR3566B_STARTADDR, txData, 1, rxData, 1)) {
            cliWriteString("i2c op failed\n");
        } else {
            cliWriteString("Offset ");
            cliWriteNybbleHex(i);
            cliWriteString(": ");
            cliWriteNybbleHex(rxData[0] / 16);
            cliWriteString("\n");
        }
    }

    return 1;
}

/**
 * Write character to CLI
 * @param c
 */
void cliWriteChar(char c) {
    if (c == '\n')
        cliWriteChar('\r');
    if (cli.output.length < sizeof(cli.output.tx)) {
        cli.output.tx[cli.output.length] = c;
        cli.output.length++;
    }
}

/**
 * Write half-byte (4-bit word)
 * @param d
 */
void cliWriteNybbleHex(uint8_t d) {
    d &= 0x0f;
    if (d >= 10)
        d += 'a' - 10;
    else
        d += '0';
    cliWriteChar((char) d);
}

/**
 * Write byte
 * @param d
 */
void cliWriteByteHex(uint8_t d) {
    cliWriteNybbleHex(d >> 4);
    cliWriteNybbleHex(d & 0x0f);
}

/**
 * Write double byte (16-bit word)
 * @param d
 */
void cliWriteChawmpHex(uint16_t d) {
    cliWriteByteHex(d >> 8);
    cliWriteByteHex(d & 0xff);
}

/**
 * Write 32-bit word
 * @param d
 */
void cliWriteGawbleHex(uint32_t d) {
    cliWriteChawmpHex(d >> 16);
    cliWriteChawmpHex(d & 0xffff);
}

/**
 * Write string
 * @param str
 */
void cliWriteString(const char *str) {
    while (*str)
        cliWriteChar(*str++);
}

/**
 * Initialize stack check
 */
static void stackCheckInit(void) {
    uint32_t *ptr;
    irqflags_t irq;

    ptr = _stack;
    irq = cpu_irq_save();
    while (ptr + 16 < (uint32_t *) &ptr)
        *ptr++ = 0xdeadbeef;
    cpu_irq_restore(irq);
}

/**
 * Initialize CLI
 * @param in
 * @param out
 */
void cliInit(int (*in)(void), int (*out)(char)) {
    memset(&cli, 0, sizeof(cli));
    cli.state = resetMS;
    cli.in = in;
    cli.out = out;
    stackCheckInit();
}

/**
 * Flush CLI
 * @return success
 */
int cliFlush(void) {
    while (cli.output.sent < cli.output.length) {
        if (cli.out(cli.output.tx[cli.output.sent]) >= 0)
            cli.output.sent++;
        else
            break;
    }
    if (cli.output.sent >= cli.output.length) {
        cli.output.sent = 0;
        cli.output.length = 0;
    }

    return (cli.output.length == 0) ? 1 : 0;
}

/**
 * Periodic CLI task
 */
void cliTask(void) {
    int c;
    enum {
        beforeCmdPS,
        inCmdPS,
        beforeParmPS,
        inParmPS,
        errorPS
    } parseState;
    int cmdStart, cmdLength;
    int i;

    cliFlush();
    if (cliFlush()) {
        switch (cli.state) {
        case resetMS:
            cliWriteString("Shall we play a game?\n");
            cli.state = promptMS;
            break;
        case promptMS:
            cliWriteString("* ");
            cli.state = inputMS;
            /* no break */
        case inputMS:
            c = cli.in();
            if (c >= 0) {
                switch (c) {
                case '\r':
                case '\n':
                    cliWriteChar('\n');
                    cli.state = parseMS;
                    break;
                case '\b':
                case '\177':
                    if (cli.input.length) {
                        cliWriteString("\b \b");
                        cli.input.length--;
                    }
                    break;
                case '\t':
                    c = ' ';
                    /* fall through */
                default:
                    if (c >= ' ') {
                        if (cli.input.length < sizeof(cli.input.rx))
                            cli.input.rx[cli.input.length++] = (char) c;
                        else
                            c = '\a';
                        cliWriteChar(c);
                    }
                    break;
                }
            }
            break;
        case parseMS:
            parseState = beforeCmdPS;
            cli.input.parmCount = 0;
            cmdStart = -1;
            cmdLength = 0;
            i = 0;
            while (i < cli.input.length && parseState != errorPS) {
                switch (parseState) {
                case beforeCmdPS:
                    if (cli.input.rx[i] == ' ')
                        i++;
                    else {
                        parseState = inCmdPS;
                        cmdStart = i;
                    }
                    break;
                case inCmdPS:
                    if (cli.input.rx[i] != ' ') {
                        i++;
                        cmdLength++;
                    } else
                        parseState = beforeParmPS;
                    break;
                case beforeParmPS:
                    if (cli.input.rx[i] == ' ')
                        i++;
                    else if (cli.input.parmCount < sizeof(cli.input.parms) / sizeof(cli.input.parms[0])) {
                        cli.input.parms[cli.input.parmCount] = 0;
                        cli.input.parmCount++;
                        parseState = inParmPS;
                    } else
                        parseState = errorPS;
                    break;
                case inParmPS:
                    if (cli.input.rx[i] == ' ')
                        parseState = beforeParmPS;
                    else if (cli.input.rx[i] >= '0' && cli.input.rx[i] <= '9') {
                        cli.input.parms[cli.input.parmCount - 1] <<= 4;
                        cli.input.parms[cli.input.parmCount - 1] |= cli.input.rx[i++] - '0';
                    } else if (cli.input.rx[i] >= 'a' && cli.input.rx[i] <= 'f') {
                        cli.input.parms[cli.input.parmCount - 1] <<= 4;
                        cli.input.parms[cli.input.parmCount - 1] |= cli.input.rx[i++] - 'a' + 10;
                    } else if (cli.input.rx[i] >= 'A' && cli.input.rx[i] <= 'F') {
                        cli.input.parms[cli.input.parmCount - 1] <<= 4;
                        cli.input.parms[cli.input.parmCount - 1] |= cli.input.rx[i++] - 'A' + 10;
                    } else
                        parseState = errorPS;
                    break;
                case errorPS:
                    break;
                }
            }
            if (cmdStart < 0) {
                cli.input.length = 0;
                cli.state = promptMS;
            } else if (parseState == errorPS) {
                cliWriteString("parse error\n");
                cli.input.length = 0;
                cli.state = promptMS;
            } else {
                cli.command = -1;
                for (i = 0; i < sizeof(commands) / sizeof(commands[0]); i++) {
                    if (cmdLength == strlen(commands[i].cmd) && memcmp(&cli.input.rx[cmdStart], commands[i].cmd, cmdLength) == 0) {
                        cli.command = i;
                        break;
                    }
                }
                if (cli.command < 0) {
                    cliWriteString("unknown command (type ? for help)\n");
                    cli.input.length = 0;
                    cli.state = promptMS;
                } else
                    cli.state = commandMS;
            }
            break;
        case commandMS:
            /* fall through */
        case commandContinueMS:
            if (commands[cli.command].func((cli.state == commandMS) ? 1 : 0, cli.input.parmCount, cli.input.parms) == 0)
                cli.state = commandContinueMS;
            else if (cli.state != resetMS) {
                cli.input.length = 0;
                cli.state = promptMS;
            }
            break;
        }
    }
}

#endif /* FEATURE_DEBUG_CLI */
