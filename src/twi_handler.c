//
// TWI interfaces
//

#include "main.h"
#include "hf_loader_p.h"
#include "boardid.h"
#include "ir3566b.h"
#include "twi.h"


#define TWI_TIME_MAX                       100

#define TWI_MODULE_POLLING_INTERVAL        250

#define TWI_IR_POLLING_INTERVAL            250


#ifndef MAX
#define MAX(x,y)   ((x) > (y) ? (x) : (y))
#endif


void twi_setup(void) {

    gpio_enable_module_pin(TWI_SCL, AVR32_TWI_SCL_0_0_FUNCTION);
    gpio_enable_module_pin(TWI_DATA, AVR32_TWI_SDA_0_0_FUNCTION);

    twiInit();
}

////////////////////////////////////////////////////////////////////////////////
// Master operations
////////////////////////////////////////////////////////////////////////////////
void twi_master_setup(void) {
    twiConfigT config;

    memset(&config, 0, sizeof(config));
    config.master = 1;
    config.address = TWI_MASTER_ADDRESS;
    config.freq = TWI_SPEED;
    twiConfig(&config);
}

#include "ir3566b_program.h"
bool ir3566b_programmer() {
    bool result;
    unsigned int i;
    unsigned int quadrant;
    uint8_t cmd[2];

    // Send image
    result = true;
    for (quadrant = 0; quadrant < 4 && result; quadrant++) {
        for (i = 0; i < sizeof(ir3566b_program) && result; i += 3) {
            /* if die voltage is stored in the user page in mV, replace
               the default value. */
            if (ir3566b_program[i] == IR3566B_REG_L1_VBOOT &&
                all_die_settings[quadrant].voltage >= 250 &&
                all_die_settings[quadrant].voltage <= 1500) {
                cmd[0] = IR3566B_REG_L1_VBOOT;
                cmd[1] = (all_die_settings[quadrant].voltage - 245) / 5;
                result = twi_sync_rw(TWI_BUS_IR3566B,
                                     TWI_IR3566B_STARTADDR + quadrant,
                                     cmd, 2, NULL, 0);
            } else if (ir3566b_program[i] == IR3566B_REG_5D) {
                cmd[0] = ir3566b_program[i + 0];
                cmd[1] = ir3566b_program[i + 1];
                if (quadrant)
                    cmd[1] &= ~IR3566B_REG_5D_VAUX_ENABLE;
                result = twi_sync_rw(TWI_BUS_IR3566B,
                                     TWI_IR3566B_STARTADDR + quadrant,
                                     cmd, 2, NULL, 0);
            } else
                result = twi_sync_rw(TWI_BUS_IR3566B,
                                     TWI_IR3566B_STARTADDR + quadrant,
                                     &ir3566b_program[i], 2,
                                     NULL, 0);
        }
    }

    return result;
}

////////////////////////////////////////////////////////////////////////////////
// Slave operations
////////////////////////////////////////////////////////////////////////////////

#define TS_ADDR     0
#define TS_DATA     1

static bool ts_rx_valid = false;
static bool my_twi_address_set = false;
static uint8_t my_twi_address;

static uint8_t slave_rx_addr;
static uint8_t slave_rx_nb;
static struct {
    uint8_t b[TWI_BUFSIZE];
    op_settings_t opSettings;
} slave_rx_data;
static uint8_t rxBuffer[TWI_BUFSIZE];

static void twiCallback(unsigned int length) {
    static union {
        uint8_t b[32];
        serial_number_t serial;
        op_settings_t opSettings;
        uint16_t badCores[G1_CORES / 4];
    } txBuffer;
    hfLoaderAppSuffixT *suffix;
    unsigned int responseLength;
    uint16_t v;
    int i;

    responseLength = 0;
    if (length) {
        if (ts_rx_valid)
            ; // XXX overrun, shouldn't happen
        switch (rxBuffer[0]) {
        case TWICMD_POWER_STATUS:
            txBuffer.b[0] = input_power_good;
            txBuffer.b[1] = 0;
            for (i = 0; i < 4; i++) {
                txBuffer.b[i * 4 + 2] = moduleStatus[0].inputMillivolts[i] >>
                                        8;
                txBuffer.b[i * 4 + 3] = moduleStatus[0].inputMillivolts[i] &
                                        0xff;
                txBuffer.b[i * 4 + 4] = moduleStatus[0].outputMillivolts[i] >>
                                        8;
                txBuffer.b[i * 4 + 5] = moduleStatus[0].outputMillivolts[i] &
                                        0xff;
            }
            responseLength = 18;
            break;
        case TWICMD_VERSION:
            txBuffer.b[0] = 0x00;
            txBuffer.b[1] = 0x00;
            txBuffer.b[2] = (FIRMWARE_VERSION >> 8) & 0xff;
            txBuffer.b[3] = FIRMWARE_VERSION & 0xff;
            suffix = (hfLoaderAppSuffixT *) (AVR32_FLASH +
                                             AVR32_FLASH_SIZE -
                                             sizeof(hfLoaderAppSuffixT));
            if (suffix->magic == HF_LOADER_SUFFIX_MAGIC)
                txBuffer.b[4] = 1;
            else
                txBuffer.b[4] = 0;
            txBuffer.b[5] = (suffix->crc >> 24) & 0xff;
            txBuffer.b[6] = (suffix->crc >> 16) & 0xff;
            txBuffer.b[7] = (suffix->crc >> 8) & 0xff;
            txBuffer.b[8] = (suffix->crc >> 0) & 0xff;
            txBuffer.b[9] = (uint8_t) boardid;
            responseLength = 10;
            break;
        case TWICMD_BOARD_TEMPERATURES:
            for (i = 0; i < 4; i++) {
                v = gwq_get_board_temperature(i);
                txBuffer.b[i * 2 + 0] = v >> 8;
                txBuffer.b[i * 2 + 1] = v & 0xff;
            }
            responseLength = 4 * 2;
            break;
        case TWICMD_TACHS:
            for (i = 0; i < 4; i++) {
                v = gwq_get_tach(i);
                txBuffer.b[i * 2 + 0] = v >> 8;
                txBuffer.b[i * 2 + 1] = v & 0xff;
            }
            responseLength = 4 * 2;
            break;
        case TWICMD_SERIAL_NUMBER:
            hf_nvram_get_serial(&txBuffer.serial);
            responseLength = sizeof(serial_number_t);
            break;
        case TWICMD_DIE_SETTINGS:
            if (length == 1) {
                hf_nvram_read_die_settings(&txBuffer.opSettings);
                responseLength = sizeof(op_settings_t);
            }
            break;
        case TWICMD_BAD_CORE_BITMAP:
            if (length == 1) {
                hf_nvram_read_bad_core_bitmap(0, txBuffer.badCores);
                responseLength = G1_CORES/8;
            }
            break;
        }
        slave_rx_addr = rxBuffer[0];
        slave_rx_nb = length - 1;
        if (slave_rx_nb)
            memcpy(slave_rx_data.b, &rxBuffer[1], slave_rx_nb);
        ts_rx_valid = true;
    }
    twiSlaveSetTx(txBuffer.b, responseLength);
}

void twi_slave_setup(uint8_t addr) {
    twiConfigT config;

    memset(&config, 0, sizeof(config));
    config.master = 0;
    config.address = addr;
    config.freq = TWI_SPEED; // not used
    config.slaveRxBuffer = rxBuffer;
    config.slaveRxBufferSize = sizeof(rxBuffer);
    config.callback = twiCallback;
    twiConfig(&config);
}


static twiRequestT *masterRequestHead[TWI_BUS_COUNT];


static void masterHandler(void) {
    static twiRequestT req;
    static enum {idleTS = 0,
                 queryRegulatorTemperature1TS, waitForRegulatorTemperature1TS,
                 queryRegulatorTemperature2TS, waitForRegulatorTemperature2TS,
                 queryInputVoltageTS, waitForInputVoltageTS,
                 queryOutputVoltageTS, waitForOutputVoltageTS,
                 queryBoardTempsTS, waitForBoardTempsTS,
                 queryTachsTS, waitForTachsTS,
                 queryPowerStatusTS, waitForPowerStatusTS} state;
    static enum {resetBS = 0, idleBS,
                 retrySubmitBS, busyBS} busState[TWI_BUS_COUNT];
    static uint16_t lastModulePoll;
    static uint16_t lastIRPoll;
    static uint16_t watchdog[TWI_BUS_COUNT];
    static uint8_t txBuffer[16];
    static uint8_t rxBuffer[32];
    static uint8_t slave;
    static uint8_t ir;
    static int16_t temperature;
    int status;
    uint16_t v;
    uint8_t i;

    spiFpgaTwiMasterHandler();

    for (i = 0; i < TWI_BUS_COUNT; i++) {
        if (masterRequestHead[i]) {
            switch (busState[i]) {
            case resetBS:
                switch (i) {
                case TWI_BUS_UC:
                    twiReset();
                    twi_master_setup();
                    break;
                case TWI_BUS_FPGA:
                    spiFpgaTwiReset();
                    break;
                }
                busState[i] = idleBS;
                break;
            case idleBS:
                /* fall through */
            case retrySubmitBS:
                switch (i) {
                case TWI_BUS_UC:
                    status = twiMasterWriteRead(
                                 masterRequestHead[TWI_BUS_UC]->addr,
                                 masterRequestHead[TWI_BUS_UC]->tx,
                                 masterRequestHead[TWI_BUS_UC]->txLength,
                                 masterRequestHead[TWI_BUS_UC]->rx,
                                 masterRequestHead[TWI_BUS_UC]->rxLength);
                    break;
                case TWI_BUS_FPGA:
                    status = spiFpgaTwiMasterWriteRead(
                                 masterRequestHead[TWI_BUS_FPGA]->addr,
                                 masterRequestHead[TWI_BUS_FPGA]->tx,
                                 masterRequestHead[TWI_BUS_FPGA]->txLength,
                                 masterRequestHead[TWI_BUS_FPGA]->rx,
                                 masterRequestHead[TWI_BUS_FPGA]->rxLength);
                    break;
                }
                if (status == TWI_SUCCESS) {
                    busState[i] = busyBS;
                    watchdog[i] = msec_ticker;
                } else {
                    if (busState[i] == idleBS) {
                        watchdog[i] = msec_ticker;
                        busState[i] = retrySubmitBS;
                    } else if (elapsed_since(watchdog[i]) > TWI_TIME_MAX) {
                        masterRequestHead[i]->result = status;
                        masterRequestHead[i]->pending = 0;
                        masterRequestHead[i] = masterRequestHead[i]->next;
                        busState[i] = resetBS;
                    }
                }
                break;
            case busyBS:
                switch (i) {
                case TWI_BUS_UC:
                    status = twiStatus();
                    break;
                case TWI_BUS_FPGA:
                    status = spiFpgaTwiStatus();
                    break;
                }
                if (status != TWI_BUSY) {
                    masterRequestHead[i]->result = status;
                    masterRequestHead[i]->pending = 0;
                    masterRequestHead[i] = masterRequestHead[i]->next;
                    busState[i] = idleBS;
                } else if (elapsed_since(watchdog[i]) > TWI_TIME_MAX) {
                    masterRequestHead[i]->result = TWI_TIMEOUT;
                    masterRequestHead[i]->pending = 0;
                    masterRequestHead[i] = masterRequestHead[i]->next;
                    busState[i] = resetBS;
                }
                break;
            }
        }
    }

    switch (state) {
    case idleTS:
        if (boardid == iraBID &&
            elapsed_since(lastIRPoll) >= TWI_IR_POLLING_INTERVAL) {
            lastIRPoll = msec_ticker;
            ir = 0;
            state = queryRegulatorTemperature1TS;
        } else if (ucinfo.master && ucinfo.num_slaves &&
                   elapsed_since(lastModulePoll) >=
                   TWI_MODULE_POLLING_INTERVAL) {
            lastModulePoll = msec_ticker;
            slave = 0;
            state = queryBoardTempsTS;
        }
        break;
    case queryRegulatorTemperature1TS:
        txBuffer[0] = IR3566B_REG_TEMP1;
        req.addr = TWI_IR3566B_STARTADDR + ir;
        req.tx = txBuffer;
        req.txLength = 1;
        req.rx = rxBuffer;
        req.rxLength = 1;
        twiQueueRequest(TWI_BUS_IR3566B, &req);
        state = waitForRegulatorTemperature1TS;
        break;
    case waitForRegulatorTemperature1TS:
        if (!req.pending) {
            if (req.result == TWI_SUCCESS)
                temperature = rxBuffer[0];
            else
                temperature = -1;
            state = queryRegulatorTemperature2TS;
        }
        break;
    case queryRegulatorTemperature2TS:
        txBuffer[0] = IR3566B_REG_TEMP2;
        req.addr = TWI_IR3566B_STARTADDR + ir;
        req.tx = txBuffer;
        req.txLength = 1;
        req.rx = rxBuffer;
        req.rxLength = 1;
        twiQueueRequest(TWI_BUS_IR3566B, &req);
        state = waitForRegulatorTemperature2TS;
        break;
    case waitForRegulatorTemperature2TS:
        if (!req.pending) {
            if (req.result == TWI_SUCCESS) {
                if ((int16_t) rxBuffer[0] > temperature)
                    temperature = rxBuffer[0];
            }
            if (temperature >= 0)
                gwq_update_board_temperature(ir, 0x1000 |
                                                 (uint16_t) temperature);
            state = queryInputVoltageTS;
        }
        break;
    case queryInputVoltageTS:
        txBuffer[0] = IR3566B_REG_VIN_SUPPLY;
        req.addr = TWI_IR3566B_STARTADDR + ir;
        req.tx = txBuffer;
        req.txLength = 1;
        req.rx = rxBuffer;
        req.rxLength = 1;
        twiQueueRequest(TWI_BUS_IR3566B, &req);
        state = waitForInputVoltageTS;
        break;
    case waitForInputVoltageTS:
        if (!req.pending) {
            if (req.result == TWI_SUCCESS)
                moduleStatus[0].inputMillivolts[ir] = (uint16_t)
                                                      ((uint32_t) rxBuffer[0] *
                                                       1000 / 8);
            state = queryOutputVoltageTS;
        }
        break;
    case queryOutputVoltageTS:
        txBuffer[0] = IR3566B_REG_L1_VOUT;
        req.addr = TWI_IR3566B_STARTADDR + ir;
        req.tx = txBuffer;
        req.txLength = 1;
        req.rx = rxBuffer;
        req.rxLength = 1;
        twiQueueRequest(TWI_BUS_IR3566B, &req);
        state = waitForOutputVoltageTS;
        break;
    case waitForOutputVoltageTS:
        if (!req.pending) {
            if (req.result == TWI_SUCCESS)
                moduleStatus[0].outputMillivolts[ir] = (uint16_t)
                                                       ((uint32_t) rxBuffer[0] *
                                                        1000 / 128);
            if (++ir < 4)
                state = queryRegulatorTemperature1TS;
            else
                state = idleTS;
        }
        break;
    case queryBoardTempsTS:
        txBuffer[0] = TWICMD_BOARD_TEMPERATURES;
        req.addr = TWI_SLAVE_STARTADDR + slave;
        req.tx = txBuffer;
        req.txLength = 1;
        req.rx = rxBuffer;
        req.rxLength = 8;
        twiQueueRequest(TWI_BUS_UC, &req);
        state = waitForBoardTempsTS;
        break;
    case waitForBoardTempsTS:
        if (!req.pending) {
            if (req.result == TWI_SUCCESS) {
                for (i = 0; i < 4; i++) {
                    v = ((uint16_t) rxBuffer[i * 2 + 0] << 8) |
                        rxBuffer[i * 2 + 1];
                    gwq_update_board_temperature((slave + 1) * 4 + i, v);
                }
            }
            state = queryTachsTS;
        }
        break;
    case queryTachsTS:
        txBuffer[0] = TWICMD_TACHS;
        req.addr = TWI_SLAVE_STARTADDR + slave;
        req.tx = txBuffer;
        req.txLength = 1;
        req.rx = rxBuffer;
        req.rxLength = 8;
        twiQueueRequest(TWI_BUS_UC, &req);
        state = waitForTachsTS;
        break;
    case waitForTachsTS:
        if (!req.pending) {
            if (req.result == TWI_SUCCESS) {
                for (i = 0; i < 4; i++) {
                    v = ((uint16_t) rxBuffer[i * 2 + 0] << 8) |
                        rxBuffer[i * 2 + 1];
                    gwq_update_tach((slave + 1) * 4 + i, v);
                }
            }
            state = queryPowerStatusTS;
        }
        break;
    case queryPowerStatusTS:
        txBuffer[0] = TWICMD_POWER_STATUS;
        req.addr = TWI_SLAVE_STARTADDR + slave;
        req.tx = txBuffer;
        req.txLength = 1;
        req.rx = rxBuffer;
        req.rxLength = 18;
        twiQueueRequest(TWI_BUS_UC, &req);
        state = waitForPowerStatusTS;
        break;
    case waitForPowerStatusTS:
        if (!req.pending) {
            if (req.result == TWI_SUCCESS) {
                for (i = 0; i < 4; i++) {
                    v = ((uint16_t) rxBuffer[i * 4 + 2] << 8) |
                        rxBuffer[i * 4 + 3];
                    moduleStatus[slave + 1].inputMillivolts[i] = v;
                    v = ((uint16_t) rxBuffer[i * 4 + 4] << 8) |
                        rxBuffer[i * 4 + 5];
                    moduleStatus[slave + 1].outputMillivolts[i] = v;
                }
            }
            if (++slave < ucinfo.num_slaves)
                state = queryBoardTempsTS;
            else
                state = idleTS;
        }
        break;
    }
}

//
// Main handler
//

static int tmp1;
static int addresses[10];
static int su_pin[10];
static int sd_pin[10];

void twi_handler(void) {
    struct ucinfo_t *info = &ucinfo;

    if (ts_rx_valid == true) {
        // I must be a slave, I've received something
        switch (slave_rx_addr) {
        case TWICMD_POWERUP:
            // Turn on a the power supply. If I'm getting this command, I'm a slave and
            // I must have standby power on, so there must be a power supply on me.
            // Separating this and TWICMD_STARTUP gets power to ALL slaves, even those
            // without power supplies, for when the TWICMD_STARTUP happens.
            info->addressing_complete = false;
            gpio_set_pin_high(SPARE_DOWN);
            gpio_set_pin_high(HAVE_USB);
            power_supply_on();
            break;

        case TWICMD_STARTUP:
            // restart addressing cycle
            info->addressing_complete = false;
            gpio_set_pin_high(SPARE_DOWN);
            gpio_set_pin_high(HAVE_USB);
            usb_powerup_request();
            break;

        case TWICMD_POWERDOWN:
            usb_powerdown_request();
            break;

        case TWICMD_ADDRESS:
            if (tmp1 < sizeof(addresses) / sizeof(addresses[0])) {
                addresses[tmp1] = slave_rx_data.b[0];
                su_pin[tmp1] = gpio_pin_is_high(SPARE_UP);
                sd_pin[tmp1] = gpio_pin_is_high(SPARE_DOWN);
                tmp1++;
            }

            if (gpio_pin_is_low(SPARE_UP) && gpio_pin_is_high(SPARE_DOWN)) {
                // This is MY address! Grab it.
                my_twi_address = slave_rx_data.b[0];
                my_twi_address_set = true;
                delay_msec(1);
                twi_slave_setup(my_twi_address);
                delay_msec(1);

                if (info->chain_configuration == CC_OPEN_UP)
                    gpio_set_pin_low(HAVE_USB);             // Send notification back up chain to master, last address done
                else
                    gpio_set_pin_low(SPARE_DOWN);           // Propagate down so next module gets next address
            }
            break;

        case TWICMD_ADDRESSING_COMPLETE:
            info->addressing_complete = true;
            gpio_set_pin_high(SPARE_DOWN);
            gpio_set_pin_high(HAVE_USB);
            break;

        case TWICMD_FPGA_ASIC_CTL:
            fpga_reg_write(FA_ASIC_CONTROL,
                           (slave_rx_data.b[0] & F_FORCE_BAUD) ?
                           (slave_rx_data.b[0] & ~F_FORCE_BAUD) :
                           ((slave_rx_data.b[0] & ~F_ASIC_BAUD_MASK) |
                            ucinfo.asic_baud_rate_code));
            break;

        case TWICMD_REBOOT:
            if (slave_rx_nb && slave_rx_data.b[0]) {
                // tell Atmel DFU loader not to start app ever again
                // (harmless with custom loader)
                flashc_erase_gp_fuse_bit(31, true);
                flashc_write_gp_fuse_bit(31, true);
                // tell custom bootloader not to start app on this boot
                // (harmless with Atmel DFU loader)
                AVR32_PM.gplp[1] = 0x73746179;
            }
            self_reset();                               // Never returns
            break;

        case TWICMD_FAN_SET:
            set_fan_speed(slave_rx_data.b[0], slave_rx_data.b[1]);
            break;

        case TWICMD_DIE_SETTINGS:
            if (slave_rx_nb)
                hf_nvram_write_die_settings(0, &slave_rx_data.opSettings);
            break;

        case TWICMD_BAD_CORE_BITMAP:
            if (slave_rx_nb)
                hf_nvram_write_bad_core_bitmap(0, (((uint16_t)slave_rx_data.b[0]) << 8) | slave_rx_data.b[1]);
            break;

        case TWICMD_MIXED_BAUD:
            set_mixed_slave_baudrate();
            break;

        case TWICMD_VOLTAGE_SET:
            if (slave_rx_nb >= 3)
                module_voltage_set(0, slave_rx_data.b[0],
                                   ((uint16_t) slave_rx_data.b[1] << 8) |
                                   slave_rx_data.b[2]);
            break;

        default:
            break;
        }

        ts_rx_valid = false;
    }

    if (info->master == false) {
        if (info->addressing_complete == false) {
            // Propagate HAVE_USB back up chain to master during address cycle
            if (info->chain_configuration == CC_MIDDLE) {
                if (gpio_pin_is_high(USB_DOWN))
                    gpio_set_pin_high(HAVE_USB);
                else
                    gpio_set_pin_low(HAVE_USB);
            }
        }
    }

    masterHandler();
}

void twiQueueRequest(uint8_t bus, twiRequestT *req) {
    twiRequestT *walk;

    if (bus < TWI_BUS_COUNT) {
        req->pending = 1;
        req->next = NULL;
        walk = masterRequestHead[bus];
        while (walk && walk->next)
            walk = walk->next;
        if (walk)
            walk->next = req;
        else
            masterRequestHead[bus] = req;
    } else {
        req->result = TWI_INVALID_ARG;
        req->pending = 0;
    }
}

bool twi_sync_rw(uint8_t bus, uint8_t dev,
                 const uint8_t *tx_buffer, unsigned int tx_length,
                 uint8_t *rx_buffer, unsigned int rx_length) {
    twiRequestT req;
    int retries;

    retries = 0;
    do {
        req.addr = dev;
        req.tx = tx_buffer;
        req.txLength = tx_length;
        req.rx = rx_buffer;
        req.rxLength = rx_length;
        twiQueueRequest(bus, &req);
        while (req.pending)
            masterHandler();
        if (dev == TWI_BROADCAST && req.result == TWI_NACK)
            req.result = TWI_SUCCESS;
    } while (req.result != TWI_SUCCESS && retries++ < 3);

    return (req.result == TWI_SUCCESS) ? true : false;
}

//
// Send a 1 byte command with a 1 byte value to the broadcast address
//

bool twi_broadcast(uint8_t cmd, uint8_t value) {
    uint8_t txBuffer[2];

    txBuffer[0] = cmd;
    txBuffer[1] = value;

    return twi_sync_rw(TWI_BUS_UC, TWI_BROADCAST, txBuffer, 2, NULL, 0);
}

//
// Polled means to get slave data, used at startup time
//

bool twi_get_slave_data(uint8_t slave_address, uint8_t addr, uint8_t *buf,
                        int len) {
    uint8_t txBuffer[1];

    txBuffer[0] = addr;

    return twi_sync_rw(TWI_BUS_UC, slave_address, txBuffer, 1, buf, len);
}

