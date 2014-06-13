
#include <stdio.h>
#include <stdarg.h>
#include <asf.h>
#include "conf_usb.h"
#include "adc.h"
#include "ui.h"
#include "uart.h"
#include "da2s.h"
#include "pm.h"
#include "boardid.h"
#include "cli.h"

#include "conf_usb.h"
#include "hf_loader_p.h"

volatile bool main_b_cdc_enable = false;
static void usb_clk_gen_start(void);

#define NOTIFY_HOST_CLOCK 512000
uint32_t notify_host_clock = 0;

uint8_t led_mode = LED_IDLE;
uint16_t activity_led_counter = 0;
uint16_t power_led_counter = 0;
uint16_t led_counter = 0;

__attribute__((__section__(".spurious1"))) uint32_t spurious1;

int main(void)
    {
    struct ucinfo_t *info = &ucinfo;

    wdt_disable();

    Disable_global_interrupt();
    INTC_init_interrupts();

    boardidInit();

    sleepmgr_init();
    sysclk_init();

    // do this before anything that might make use of the heap
#ifdef FEATURE_DEBUG_CLI
    cliHeapFill();
#endif

    module_init();

    usb_clk_gen_start();

    ui_init();
    ui_powerdown();

    wdt_init();

    hf_nvram_init_die_settings();

    uart_open();
    asic_init();
    gwq_init();

    hf_nvram_check();

    CONFIGURE_SCOPE_PINS;

    Enable_global_interrupt();

    hf_nvram_set_usb_serial();
    udc_start();                            // Start USB stack to authorize VBus monitoring

    fans_init();
    spi_master_setup_pins();
    spi_master_setup();
    twi_setup();
    adcInit();

    POWER_LED_ON
    ACTIVITY_LED_OFF

#ifdef FEATURE_DEBUG_CLI
    cliInit(&usbctrlDebugMonitorRead, &usbctrlDebugMonitorWrite);
#endif
    profileInit();

    while (true)
        {
        profileEnter(PROFILE_CHANNEL_MAINLOOP);
        led_handler();
        notify_host_clock++;
#ifdef FEATURE_NOTIFY_HOST_DEBUG
        if (notify_host_clock >= NOTIFY_HOST_CLOCK) {
            notify_host_clock = 0;
            notify_host("THIS IS NOT PRODUCTION FIRMWARE.  FOR DEBUGGING USE ONLY.");
        }
#endif // FEATURE_NOTIFY_HOST_DEBUG

        //sleepmgr_enter_sleep();
        if (info->master)
            {
            if (!da2sEnabled)
                {
                CheckUsbIncoming();
                CheckUartIncoming();
                CheckUartOutgoing();
                chain_handler();
                }

            check_watchdog();
            }
        else if (info->slave_autonomous)
            {
            CheckUartOutgoing();
            chain_handler();
            }

        module_handler();
        thermal_control();
        spi_handler();
        twi_handler();
        adcTask();
        usbctrlTask();
        da2sTask();

#ifdef FEATURE_DEBUG_CLI
        cliTask();
#endif // FEATURE_DEBUG_CLI

#ifdef FEATURE_HW_WATCHDOG
        wdt_reenable();
#endif // FEATURE_HW_WATCHDOG
        profileExit(PROFILE_CHANNEL_MAINLOOP);
        }
    }

// activity_led_counter turns on LED when non-zero
// power_led_counter turns *off* LED when non-zero
// led_counter is used to control timing
void led_handler(void) {
        if (led_mode == LED_STATIC) {
            // don't automatically adjust.  manual control elsewhere in code.
            return;
        }
        led_counter--;

        if (activity_led_counter) {
            activity_led_counter--;
            ACTIVITY_LED_ON
        } else {
            ACTIVITY_LED_OFF
        }
        if (power_led_counter) {
            power_led_counter--;
            POWER_LED_OFF
        } else {
            POWER_LED_ON
        }

        switch (led_mode) {
            case LED_IDLE:
            case LED_AUTO:
                // LEDs have been setup elsewhere, run via the counter
                break;
            case LED_PLAID:
                // They've gone to Plaid...
                if (led_counter == 0) {
                    led_counter = 0xffff / 16;
                    power_led_counter = 0xffff / 32;
                    activity_led_counter = 0xffff / 64;
                }
                break;
            case LED_ERROR:
                // Toggle blinkenlights ping-pong
                if (led_counter == 0) {
                    led_counter = 0xffff;
                    power_led_counter = 0xffff / 2;
                    activity_led_counter = 0xffff / 2;
                }
                break;
            case LED_HASH_REALLYSLOW:
                // Very Slowly flash activity light
                if (led_counter == 0) {
                    led_counter = 0xffff;
                    power_led_counter = 0;
                    activity_led_counter = 0xffff / 2;
                }
                break;
            case LED_HASH_SLOW:
                // Slowly flash activity light
                if (led_counter == 0) {
                    led_counter = 0xffff / 2;
                    power_led_counter = 0;
                    activity_led_counter = 0xffff / 4;
                }
                break;
            case LED_HASH_FAST:
                // Quickly flash activity light
                if (led_counter == 0) {
                    led_counter = 0xffff / 16;
                    power_led_counter = 0;
                    activity_led_counter = 0xffff / 32;
                }
                break;
            default:
                // Both LED on solid because there's a logic error
                power_led_counter = 0;
                activity_led_counter = 2;
        }
}


void main_suspend_action(void)
{
	ui_powerdown();
}

void main_resume_action(void)
{
	ui_wakeup();
}

void main_sof_action(void)
{
	if (!main_b_cdc_enable)
		return;
	ui_process(udd_get_frame_number());
}

bool main_cdc_enable(uint8_t port)
{
	main_b_cdc_enable = true;
	// Open communication
	//uart_open(port);
	return true;
}

void main_cdc_disable(uint8_t port)
{
	main_b_cdc_enable = false;
	// Close communication
	//uart_close(port);
}

void main_cdc_set_dtr(uint8_t port, bool b_enable)
{
	if (b_enable) {
		// Host terminal has open COM
		ui_com_open(port);
	}else{
		// Host terminal has close COM
		ui_com_close(port);
	}
}


static void usb_clk_gen_start(void)
    {
    volatile avr32_pm_t *pm = &AVR32_PM;

    // Set PLL1 @ 48 MHz from Osc0.
    pm_pll_setup(pm, 1,   // pll.
                     5,   // mul.
                     1,   // div.
                     0,   // osc.
                     16); // lockcount.

    // Enable PLL1.
    pm_pll_enable(pm, 1);

    // Wait for PLL1 locked.
    pm_wait_for_pll1_locked(pm);

    // Setup USB GCLK.
    pm_gc_setup(pm, AVR32_PM_GCLK_USBB,  // gc.
                        1,              // osc_or_pll: use Osc (if 0) or PLL (if 1).
                        1,              // pll_osc: select Osc0/PLL0 or Osc1/PLL1.
                        0,              // diven.
                        0);             // div.

    // Enable USB GCLK.
    pm_gc_enable(pm, AVR32_PM_GCLK_USBB);

    // Enable PWM clock
    pm_enable_module(pm, AVR32_PWM_CLK_PBA);
    pm_enable_module(pm, AVR32_TWI_CLK_PBA);
    pm_enable_module(pm, AVR32_SPI_CLK_PBA);
    pm_enable_module(pm, AVR32_USART1_CLK_PBA);
    }

const char *iface_ver = NULL;
const char *firmware_ver = NULL;
const char *when_compiled = NULL;

static uint8_t op_version_fail(struct hf_header *h, enum ver_fail vf);
static uint8_t load_version_data(struct hf_header *h, const char *ver_string);

void convert_to_hex(unsigned char *data, char *s, int len)
{
    int i;
    static const char hexchars[16] = {'0', '1', '2', '3', '4', '5', '6', '7',
                                      '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};

    for(i=0; i < len; i++)
        if(i%2 == 0)
            *(s + i) = hexchars[(*(data + i/2) & 0xf0) >> 4];
        else
            *(s + i) = hexchars[*(data + i/2) & 0x0f];
}

/* Puts bytes of val into data suitable for display (aka "big-endian").
 * (Architecture independent.) */
static void display_order(unsigned char *data, uint32_t val)
{
    *(data + 0) = (val & 0xff000000) >> 24;
    *(data + 1) = (val & 0x00ff0000) >> 16;
    *(data + 2) = (val & 0x0000ff00) >> 8;
    *(data + 3) =  val & 0x000000ff;
}

// OP_VERSION: Report different kinds of version information.
// chip_address: board number.  (Primary board is 0.)
// core_address: type of version information requested.
// hdata: 0 (required)
// data_length: 0 (required)
//
// If the version string is not defined (i.e. NULL), a packet
// with the same chip_address and core_address is returned
// without following data.  This is considered "undefined".
//
// If the version string is defined, it is returned in the data
// area.  The first byte is the number of characters that follow.
// The string is not NUL terminated.  The length byte allows
// representation of any length of string.  The data area
// is required by the format to be a multiple of four bytes.
// Unused bytes in the data area are set to NUL.
//
// Problems are reported by returning a packet with core_address
// of version_fail and no data area.  The type of failure is
// reported in the hdata field.
uint8_t op_version(struct hf_header *h)
{
    const static int maxdata = (int) TX_BUFFER_SIZE - sizeof(struct hf_header) -
                               sizeof(struct uart_sendinfo);

    if(h->preamble != 0xaa ||
       h->operation_code != OP_VERSION ||
       h->hdata != 0 ||
       h->data_length != 0)
        return op_version_fail(h, bad_header);

    if(4*h->data_length > maxdata)
        return op_version_fail(h, bad_header);

    // Only the primary board (0) is supported at this time.
    if(h->chip_address != 0)
        return op_version_fail(h, not_primary_board);

    if(h->core_address == interface_version) {
        /* Interface version. */
        if(iface_ver != NULL)
            return load_version_data(h, iface_ver);
        else
            h->data_length = 0;
    }
    else if(h->core_address == firmware_version) {
        /* Firmware version. */
        if(firmware_ver != NULL)
            return load_version_data(h, firmware_ver);
        else
            h->data_length = 0;
    }
    else if(h->core_address == time_compiled) {
        /* Date and time compiled. */
        if(when_compiled != NULL)
            return load_version_data(h, when_compiled);
        else
            h->data_length = 0;
    }
    else if(h->core_address == firmware_crc32) {
        char crc_string[9];
        hfLoaderAppSuffixT *ast_ptr =
	  (hfLoaderAppSuffixT *) (AVR32_FLASH + AVR32_FLASH_SIZE -
				    sizeof(hfLoaderAppSuffixT));
        unsigned char crcbytes[4];

	if(ast_ptr->magic != HF_LOADER_SUFFIX_MAGIC)
            return op_version_fail(h, app_magic_bad);

        display_order((unsigned char *) &crcbytes, ast_ptr->crc);
        convert_to_hex(crcbytes, crc_string, 8);
        crc_string[8] = '\0';

        return load_version_data(h, crc_string);
    }
    else {
        return op_version_fail(h, unknown_ver_type);
    }

    h->crc8 = hf_crc8((uint8_t *)h);

    return(sizeof(*h) + h->data_length * 4);
}

static uint8_t load_version_data(struct hf_header *h, const char *ver_string)
{
    unsigned char *data = (unsigned char *) h + sizeof(struct hf_header);
    const static int maxdata = (int) TX_BUFFER_SIZE - sizeof(struct hf_header) -
                               sizeof(struct uart_sendinfo);
    size_t slen = strlen((char *) ver_string);
    uint8_t datalen = slen + 1;

    if(datalen > maxdata)
        return op_version_fail(h, ver_string_too_large);

    if(datalen % 4 == 0)
        h->data_length = datalen / 4;
    else
        h->data_length = datalen / 4 + 1;

    /* Wipe the data area. */
    memset(data, 0, maxdata);
    /* Wipe the first part of the transmit buffer, before the hf_header. */
    memset(h - sizeof(struct uart_sendinfo), 0, sizeof(struct uart_sendinfo));

    data[0] = slen; /* Length byte */
    memcpy(data+1, ver_string, slen);

    h->crc8 = hf_crc8((uint8_t *)h);

    return(sizeof(*h) + h->data_length * 4);
}

static uint8_t op_version_fail(struct hf_header *h, enum ver_fail vf)
{
    unsigned char *data = (unsigned char *) h + sizeof(struct hf_header);
    const static int maxdata = (int) TX_BUFFER_SIZE - sizeof(struct hf_header) -
                               sizeof(struct uart_sendinfo);

    h->core_address = version_fail;
    h->data_length = 0;
    h->hdata = cpu_to_le16(vf);
    h->crc8 = hf_crc8((uint8_t *)h);

    /* Wipe the data area. */
    memset(data, 0, maxdata);
    /* Wipe the first part of the transmit buffer, before the hf_header. */
    memset(h - sizeof(struct uart_sendinfo), 0, sizeof(struct uart_sendinfo));

    return(sizeof(*h) + h->data_length * 4);
}

uint8_t op_ping(struct hf_header *h)
{
    unsigned char *data = (unsigned char *) h + sizeof(struct hf_header);
    const static int maxdata = (int) TX_BUFFER_SIZE - sizeof(struct hf_header) -
                               sizeof(struct uart_sendinfo);

    if(h->preamble != 0xaa ||
       h->chip_address != 0 || /* Only primary board (0) currently supported. */
       h->core_address != 0 ||
       h->hdata != 0)
      /* Fix: Should return some kind of failure indicator. */
      return 0;

    if(h->data_length*4 > maxdata)
      /* Fix: Should return some kind of failure indicator. */
      return 0;

    /* Wipe the extra data area. */
    memset(data + 4*h->data_length, 0, maxdata - 4*h->data_length);
    /* Wipe the first part of the transmit buffer, before the hf_header. */
    memset(h - sizeof(struct uart_sendinfo), 0, sizeof(struct uart_sendinfo));

    return(sizeof(*h) + h->data_length * 4);
}
