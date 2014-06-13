//
// Everything to do with the USER flash page
//

#include "main.h"

#define __UPAGE __attribute__((__section__(".userpage")))

// Ordering and content cannot change between releases, unless a revision
// update procedure is added in here that converts old -> new
__UPAGE static serial_number_t  serial_data;                        // Serial number of module
__UPAGE static op_limits_t      operational_limits;                 // Limits that cannot be exceeded
__UPAGE static uint16_t         bad_core_bitmap[G1_CORES/16];       // Cores mapped out at manufacturing time, 1==bad
__UPAGE static op_history_t     history;                            // Operational history information
__UPAGE static fan_settings_t   fan_settings;
__UPAGE static op_settings_t    die_settings;                       // Settings for each die
__UPAGE static uint8_t          ds_expansion[12] __attribute__ ((unused)); // Room for die settings expansion, not included
                                                                    // in structure to minimize communication sizes
__UPAGE static char             my_name[HF_NAME_SIZE];              // This system's name


uint8_t usb_serial_number[USB_DEVICE_GET_SERIAL_NAME_LENGTH];

// Combined die_settings from all chained modules
// Constructed from master and slave USER page data at startup
die_settings_t all_die_settings[MAX_DIE] = { {0,0} };               // Indexed by physical die #
uint8_t all_ref_clocks[MAX_SLAVES + 1] = {0};
uint8_t module_ref_clocks[MAX_DIE] = {0};                           // Indexed by virtual die #
static uint8_t die_virtual_to_physical[MAX_DIE];

// Check to see if the nvram is programmed, if not assert factory mode
void hf_nvram_check(void)
    {
    struct ucinfo_t *info = &ucinfo;
    serial_number_t *sn = &serial_data;

#ifdef FEATURE_FORCE_FACTORY_MODE
    info->factory_mode = true;
    return;
#endif // FEATURE_FORCE_FACTORY_MODE

    flashc_issue_command(AVR32_FLASHC_FCMD_CMD_QPRUP, -1);
    if ((info->factory_mode = flashc_is_page_erased()) == true)
        return;

    if (sn->magic == U_MAGIC)
        info->factory_mode = false;
    else
        info->factory_mode = true;
    }

// Read last four bytes of serial number, return as uint32 per protocol spec
uint32_t hf_nvram_get_short_serial(void) {
    if (serial_data.magic == U_MAGIC &&
          serial_data.start_barrier[0] == 'H' &&
          serial_data.start_barrier[1] == 'F' &&
          serial_data.start_barrier[2] == ':' &&
          serial_data.start_barrier[3] == ':' &&
          serial_data.stop_barrier[0] == ':' &&
          serial_data.stop_barrier[1] == ':' &&
          serial_data.stop_barrier[2] == 'F' &&
          serial_data.stop_barrier[3] == 'H') {
        uint32_t serial_number = (( serial_data.unique_id[12] << 24) |
                                 ( serial_data.unique_id[13] << 16) |
                                 ( serial_data.unique_id[14] << 8) |
                                 ( serial_data.unique_id[15] ));

        return serial_number;
    } else {
        return 0;
    }
}

// Set or Read serial number information
uint8_t hf_nvram_op_serial(struct hf_header *h)
    {
    unsigned char *data = (unsigned char *) h + sizeof(struct hf_header);
    const static int maxdata = (int) TX_BUFFER_SIZE - sizeof(struct hf_header) -
                               sizeof(struct uart_sendinfo);
    serial_number_t *s = (serial_number_t *) data;

    /* chip_address should refer to the board, but we only support
     * the primary board at the moment. */
    if(h->chip_address != 0)
        /* Fix: Should return some kind of failure indicator. */
        return(0);

    if(h->data_length*4 > maxdata)
        /* Fix: Should return some kind of failure indicator. */
        return(0);

    if (le16_to_cpu(h->hdata) == U_MAGIC)           // Extra magic to verify initialized user page
        {
        if (h->core_address > 0)
            {
            if(h->data_length*4 != sizeof(serial_number_t))
                /* Fix: Should return some kind of failure indicator. */
                return(0);

            if(s->magic == U_MAGIC &&
               s->start_barrier[0] == 'H' &&
               s->start_barrier[1] == 'F' &&
               s->start_barrier[2] == ':' &&
               s->start_barrier[3] == ':' &&
               s->stop_barrier[0] == ':' &&
               s->stop_barrier[1] == ':' &&
               s->stop_barrier[2] == 'F' &&
               s->stop_barrier[3] == 'H')
                {
                // Write serial number
                flashc_memcpy((void *)&serial_data, (void *)s, sizeof(*s), true);
                // Return original header as acknowledgement.
                return(sizeof(*h)+sizeof(*s));
                }
            else
                {
                /* Fix: Should return some kind of failure indicator. */
                return(0);
                }
            }
        else
            {
            if(sizeof(serial_number_t) > maxdata)
                /* Fix: Should return some kind of failure indicator. */
                return(0);

            // Read serial number data
            memcpy(s, &serial_data, sizeof(*s));
            h->data_length = sizeof(*s)/4;
            h->crc8 = hf_crc8((uint8_t *)h);
            return(sizeof(*h)+sizeof(*s));
            }
        }
    else
        return(0);
    }

void hf_nvram_get_serial(serial_number_t *ptr)
    {

    memcpy(ptr, &serial_data, sizeof(serial_number_t));
    }

void hf_nvram_set_usb_serial(void)
    {

    if(serial_data.magic == U_MAGIC &&
       serial_data.start_barrier[0] == 'H' &&
       serial_data.start_barrier[1] == 'F' &&
       serial_data.start_barrier[2] == ':' &&
       serial_data.start_barrier[3] == ':' &&
       serial_data.stop_barrier[0] == ':' &&
       serial_data.stop_barrier[1] == ':' &&
       serial_data.stop_barrier[2] == 'F' &&
       serial_data.stop_barrier[3] == 'H')
        convert_to_hex(serial_data.unique_id, (char *) usb_serial_number,
                       sizeof(usb_serial_number));
    else
        memset(usb_serial_number, 'x', sizeof(usb_serial_number));

    }

// Write or read operational limits
uint8_t hf_nvram_op_limits(struct hf_header *h)
    {
    op_limits_t *o = (op_limits_t *)(h+1);

    if (le16_to_cpu(h->hdata) == U_MAGIC)           // Extra magic to verify initialized user page
        {
        if (h->core_address > 0)
            {
            // Write operational limits
            flashc_memcpy((void *)&operational_limits, (void *)o, sizeof(*o), true);
            return(0);
            }
        else
            {
            // Read operational limits
            memcpy(o, &operational_limits, sizeof(*o));
            h->data_length = sizeof(*o)/4;
            h->crc8 = hf_crc8((uint8_t *)h);
            return(sizeof(*h)+sizeof(*o));
            }
        }
    else
        return(0);
    }

// Read or clear operational history
uint8_t hf_nvram_op_history(struct hf_header *h)
    {
    op_limits_t *oh = (op_limits_t *)(h+1);

    if (le16_to_cpu(h->hdata) == U_MAGIC)           // Extra magic to verify initialized user page
        {
        if (h->core_address > 0)
            {
            flashc_memset8((void *)&history, 0, sizeof(history), true);
            return(0);
            }
        else
            {
            // Read operational history
            memcpy(oh, &history, sizeof(*oh));
            h->data_length = sizeof(*oh)/4;
            h->crc8 = hf_crc8((uint8_t *)h);
            return(sizeof(*h)+sizeof(*oh));
            }
        }
    else
        return(0);
    }


#ifdef FEATURE_INITIALIZE_NON_VALID_DIE_SETTINGS
static const op_settings_t rev1_default_die_settings = {
    REV1_DIE_SETTINGS_REVISION,
    DEFAULT_REF_CLOCK,
    U_MAGIC,
        {
          {DEFAULT_G1_HASHCLOCK,DS_DEFAULT_VOLTAGE},
          {DEFAULT_G1_HASHCLOCK,DS_DEFAULT_VOLTAGE},
          {DEFAULT_G1_HASHCLOCK,DS_DEFAULT_VOLTAGE},
          {DEFAULT_G1_HASHCLOCK,DS_DEFAULT_VOLTAGE}
        }
    };
#endif

// Initialize the die settings array, called once at startup by the master
void hf_nvram_init_die_settings()
    {
    int i, j;

#ifdef FEATURE_INITIALIZE_NON_VALID_DIE_SETTINGS
    if (!VALID_DIE_SETTINGS(die_settings))
        {
        hf_nvram_write_die_settings(0, &rev1_default_die_settings);
        hf_nvram_write_bad_core_bitmap(0, 0x8000);
        }
#endif

    // Copy my local user page die settings in, regardless of master or slave
    if (VALID_DIE_SETTINGS(die_settings))
        {
        all_ref_clocks[0] = die_settings.ref_frequency;
        for (i = 0, j = 0; i < 4; i++)
            {
            all_die_settings[i].voltage = die_settings.die[i].voltage;
            all_die_settings[i].frequency = die_settings.die[i].frequency;
            if (die_settings.die[i].voltage != DS_DISABLED)
                {
                die_virtual_to_physical[j] = i;
                module_ref_clocks[j++] = die_settings.ref_frequency;
                }
            }
        }
    else
        {
        all_ref_clocks[0] = DEFAULT_REF_CLOCK;
        for (i = 0; i < 4; i++)
            {
            die_virtual_to_physical[i] = i;
            module_ref_clocks[i] = DEFAULT_REF_CLOCK;
            all_die_settings[i].voltage = DS_DEFAULT_VOLTAGE;
            all_die_settings[i].frequency = DEFAULT_G1_HASHCLOCK;
            }
        }
    }

// Get slave die settings, called once after chain initialization
void hf_nvram_get_slave_die_settings()
    {
    struct ucinfo_t *info = &ucinfo;
    op_settings_t settings;
    int slave_offset;
    bool use_defaults;
    int i, j, k;

    for (i = 0, k = 0; i < 4; i++)
        {
        if (die_settings.die[i].voltage != DS_DISABLED)
            {
            die_virtual_to_physical[k] = i;
            module_ref_clocks[k++] = die_settings.ref_frequency;
            }
        }
    if (info->master && info->num_slaves)
        {
        // There are slaves, get their die settings and place in position
        for (slave_offset = 0; slave_offset < info->num_slaves; slave_offset++)
            {
            use_defaults = true;
            if ((twi_get_slave_data(TWI_SLAVE_STARTADDR+slave_offset, TWICMD_DIE_SETTINGS, (uint8_t *)&settings, sizeof(settings))) == true)
                {
#ifdef FEATURE_INITIALIZE_NON_VALID_DIE_SETTINGS
                if (!VALID_DIE_SETTINGS(settings))
                    {
                    uprintf(UD_STARTUP, "Forcing default die settings into an uninitialized device\n");
                    hf_nvram_write_die_settings(slave_offset+1, &rev1_default_die_settings);
                    hf_nvram_write_bad_core_bitmap(slave_offset+1, 0x8000);
                    }
#endif
                // See what we've got - may never have been set.
                if (VALID_DIE_SETTINGS(settings))
                    {
                    use_defaults = false;
                    if (settings.ref_frequency != die_settings.ref_frequency)
                        info->mixed_reference_clocks = true;
                    all_ref_clocks[slave_offset + 1] = settings.ref_frequency;
                    for (j = 0; j < 4; j++, i++)
                        {
                        all_die_settings[i].voltage = settings.die[j].voltage;
                        all_die_settings[i].frequency = settings.die[j].frequency;
                        if (all_die_settings[i].voltage != DS_DISABLED)
                            {
                            die_virtual_to_physical[k] = i;
                            module_ref_clocks[k++] = settings.ref_frequency;
                            }
                        }
                    }
                }

            if (use_defaults)
                {
                // TWI read failed, or chained to a board with old firmware, or die settings never written. Use defaults.
                all_ref_clocks[slave_offset + 1] = DEFAULT_REF_CLOCK;
                for (j = 0; j < 4; j++, i++)
                    {
                    all_die_settings[i].voltage = DS_DEFAULT_VOLTAGE;
                    all_die_settings[i].frequency = DEFAULT_G1_HASHCLOCK;
                    die_virtual_to_physical[k] = i;
                    module_ref_clocks[k++] = DEFAULT_REF_CLOCK;
                    }
                }
            }
        }
    }

// May be written by a characterization run (hcm utility)
void hf_nvram_read_die_settings(op_settings_t *ds)
    {
    memcpy((void *)ds, (void *)&die_settings, sizeof(die_settings));
    }
bool hf_nvram_die_settings_valid()
    {
    return(VALID_DIE_SETTINGS(die_settings));
    }
op_settings_t *hf_nvram_die_settings()
    {
    return(&die_settings);
    }
#ifdef FEATURE_INITIALIZE_NON_VALID_DIE_SETTINGS
uint8_t hf_nvram_physical_die(uint8_t virtual)
    {
    return(die_virtual_to_physical[virtual]);
    }
#endif
bool hf_nvram_write_die_settings(uint8_t module, const op_settings_t *s)
    {
    uint8_t txbuffer[1+sizeof(*s)];
    bool res = true;

    if (module == 0)
        flashc_memcpy((void *)&die_settings, (void *)s, sizeof(die_settings), true);
    else
        {
        txbuffer[0] = TWICMD_DIE_SETTINGS;
        uprintf(0xffff, "wds: m 0x%04x v %x %x %x %x\n", s->magic, s->die[0].voltage, s->die[1].voltage, s->die[2].voltage, s->die[3].voltage);
        memcpy(&txbuffer[1], (uint8_t *)s, sizeof(*s));
        res = twi_sync_rw(TWI_BUS_UC, TWI_SLAVE_STARTADDR+module-1, txbuffer, sizeof(txbuffer), NULL, 0);
        }
    return(res);
    }

// Note: synchronous TWI operations are OK because this is all done under hcm / diagnostic conditions
uint8_t hf_nvram_op_die_settings(struct hf_header *h)
    {
    op_settings_t *s = (op_settings_t *)(h+1);
    int i;

    if (le16_to_cpu(h->hdata) == U_MAGIC)
        {
        if (h->core_address > 0)
            {
            // A write operation
            s->magic = le16_to_cpu(s->magic);
            for (i = 0; i < 4; i++)
                {
                s->die[i].voltage = le16_to_cpu(s->die[i].voltage);
                s->die[i].frequency = le16_to_cpu(s->die[i].frequency);
                }
            // Write die settings
            hf_nvram_write_die_settings(h->chip_address, s);
            return(0);
            }
        else
            {
            // Read die settings
            if (h->chip_address == 0)
                memcpy(s, &die_settings, sizeof(*s));
            else
                {
                // Read these settings from the correct slave
                if ((twi_get_slave_data(TWI_SLAVE_STARTADDR+h->chip_address-1, TWICMD_DIE_SETTINGS, (uint8_t *)s, sizeof(*s))) == false)
                    s->revision = 0xff;             // Make sure it is seen as junk
                }
            uprintf(0xffff, "rds: m 0x%04x v %x %x %x %x\n", s->magic, s->die[0].voltage, s->die[1].voltage, s->die[2].voltage, s->die[3].voltage);
            s->magic = cpu_to_le16(s->magic);
            for (i = 0; i < 4; i++)
                {
                s->die[i].voltage = cpu_to_le16(s->die[i].voltage);
                s->die[i].frequency = cpu_to_le16(s->die[i].frequency);
                }
            h->data_length = sizeof(*s)/4;
            h->chip_address = ucinfo.num_slaves;    // So hcm can figure system size out, and make subsequent read requests for the slave settings
            h->crc8 = hf_crc8((uint8_t *)h);
            return(sizeof(*h)+sizeof(*s));
            }
        }
    else
        return(0);
    }

void hf_nvram_fan_settings_set(const fan_settings_t *fan)
    {
        flashc_memcpy(&fan_settings, fan, sizeof(fan_settings), true);
    }

uint8_t hf_nvram_fan_settings(struct hf_header *h)
    {
    fan_settings_t *f = (fan_settings_t *)(h+1);

    if (le16_to_cpu(h->hdata) == U_MAGIC)
        {
        if (h->core_address > 0)
            {
            // Write fan settings
            flashc_memcpy(&fan_settings, f, sizeof(fan_settings), true);
            return(0);
            }
        else
            {
            // Read fan settings
            memcpy(f, &fan_settings, sizeof(*f));
            h->data_length = sizeof(*f)/4;
            h->crc8 = hf_crc8((uint8_t *)h);
            return(sizeof(*h)+sizeof(*f));
            }
        }
    else
        return(0);
    }

fan_settings_t *hf_nvram_get_fan_settings(void)
    {
    return &fan_settings;
    }

//
// Bad core bitmap interfaces
//
void hf_nvram_write_bad_core_bitmap(uint8_t module, uint16_t hdata)
    {
    uint16_t bad[G1_CORES/16];
    uint16_t core;

    if (module)
        {
        if (module <= ucinfo.num_slaves)
            {
            uint8_t txbuffer[3];

            txbuffer[0] = TWICMD_BAD_CORE_BITMAP;
            txbuffer[1] = hdata >> 8;
            txbuffer[2] = hdata & 0xff;
            twi_sync_rw(TWI_BUS_UC, TWI_SLAVE_STARTADDR+module-1, txbuffer, sizeof(txbuffer), NULL, 0);
            }
        return;
        }

    if (hdata & 0x8000)
        {
        // Initialize bad map, to all cores enabled
        memset(bad, 0xff, sizeof(bad));
        flashc_memcpy(&bad_core_bitmap, bad, sizeof(bad), true);
        }
    if (hdata & 0x4000)
        {
        // Set a core as bad
        core = hdata & 0x1ff;
        if (core < sizeof(bad)*8)
            {
            memcpy(bad, bad_core_bitmap, sizeof(bad));
            bad[core>>4] &= ~(1<<(core&0xf));
            flashc_memcpy(&bad_core_bitmap, bad, sizeof(bad), true);
            }
        }
    }
void hf_nvram_read_bad_core_bitmap(uint8_t module, uint16_t *b)
    {
    if (module == 0)
        memcpy((uint16_t *)b, &bad_core_bitmap[0], sizeof(bad_core_bitmap));        // The local module
    else
        {
        if ((twi_get_slave_data(TWI_SLAVE_STARTADDR+module-1, TWICMD_BAD_CORE_BITMAP, (uint8_t *)b, sizeof(bad_core_bitmap))) == false)
            memset(b, 0xff, sizeof(bad_core_bitmap));
        }
    }

// Host interface
// h->chip_address == die address
// h->core_address == 1 for write, 0 for read, per other Factory operations
// h->hdata == write data:
//      b15 = If set, initialize bad core block map to 0
//      b14 = If set, Permanently set a core as bad
//      b6:0 = Core to set bad, 0 - 95
//
uint8_t hf_nvram_op_bad_core(struct hf_header *h)
    {
    uint16_t hdata = le16_to_cpu(h->hdata);
    uint16_t *p = (uint16_t *)(h+1);
    uint8_t module = h->chip_address >> 2;
    int i;

    if (h->core_address > 0)
        {
        hdata += (h->chip_address & 0x3) * 96;  // Add in die offset to make total core offset within a module
        hf_nvram_write_bad_core_bitmap(module, hdata);
        return(0);
        }
    else
        {
        hf_nvram_read_bad_core_bitmap(module, p);

        // Fix up byte ordering for host
        for (i = 0; i < G1_CORES/16; i++, p++)
            *p = cpu_to_le16(*p);

        h->data_length = sizeof(bad_core_bitmap)/4;
        h->crc8 = hf_crc8((uint8_t *)h);

        return(sizeof(*h)+sizeof(bad_core_bitmap));
        }
    }

uint16_t *hf_nvram_bad_core_bitmap(void)
    {
    return(&bad_core_bitmap[0]);
    }
bool hf_nvram_bad_core_bitmap_valid(void)
    {
    return (VALID_DIE_SETTINGS(die_settings));                  // Bitmap is carried along with die settings
    }

//
// System name read/write
//
uint8_t hf_nvram_op_name(struct hf_header *h)
    {
    char *n = (char *)(h+1);

    if (h->core_address > 0)
        {
        flashc_memcpy(&my_name, n, sizeof(my_name), true);
        return(0);
        }
    else
        {
        memcpy(n, &my_name, sizeof(my_name));
        h->data_length = sizeof(my_name)/4;
        h->crc8 = hf_crc8((uint8_t *)h);
        return(sizeof(*h)+sizeof(my_name));
        }
    }

void hf_nvram_name_set(char *name)
    {

    if (name)
        flashc_memcpy(&my_name, name, sizeof(my_name), true);
    }

char *hf_nvram_name(void)
    {

    return my_name;
    }

