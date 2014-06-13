////////////////////////////////////////////////////////////////////////////////
//
// An emulator for G-1 based products.
//
// Requires the USART to be looped back (just use a jumper on an A3BU board
//
// 1. Examines incoming packets (that we would have just sent), and transforms
//    them to make it look like we're talking to a real device.
//
// 2. Generates OP_STATUS packets, age-ing jobs appropriately
//
// 3. Generates phoney nonces at around one per second. Requires a CGMiner
//    driver patch to ignore these nonces, which are always 0x4242ddcc where
//    dd=die # and cc=core #.
//
////////////////////////////////////////////////////////////////////////////////

#include "main.h"
#include "hf_util.h"
#include "hf_protocol.h"


#ifdef INCLUDE_ASIC_EMULATOR

extern bool enable_software_emulator;

typedef struct em_core {
  uint8_t active:7;
  uint8_t pending:1;
  } __attribute__((packed,aligned(1))) em_core_t;

// Emulation sizing
#define EM_DIE              4
#define EM_CORES            96

#define EM_TICK             64              // 64 msec granularity
#define EM_CORE_TICKS       55              // Core job time


#define EM_TOTAL_CORES  (EM_DIE*EM_CORES)
static em_core_t core_timer[EM_TOTAL_CORES];

static uint16_t active_sequence[EM_TOTAL_CORES];
static uint16_t pending_sequence[EM_TOTAL_CORES];

static uint8_t group_id[EM_TOTAL_CORES];
static uint8_t group_ntime_offset[EM_TOTAL_CORES];

static void asic_emulator_nonce(uint16_t, uint32_t);
static void asic_emulator_op_status(void);

//
// Translate an incoming (serial link) packet
//
// Modifies the packet in-place to turn it into something else
//
// Returns:
//  0 = Toss away the packet
//  1 = Send packet through to user/host as if it had been received

static struct hf_config_data config_data;

static uint16_t last_hash_sequence_number;

#if 0
static void dump_groups(void);
static void dump_groups()
    {
    struct ucinfo_t *info = &ucinfo;
    static uint8_t once = 1;
    uint16_t i;

    if (once)
        {
        once = 0;

        uprintf(1, "ntime_roll_total %d per_core %d", info->ntime_roll_total, info->ntime_roll_per_core);
        uprintf(1, "cores_per_group %d, total_cores %d, groups %d", info->cores_per_group, info->total_cores, info->groups);
        uprintf(1, "cores_per_group_cycle %d, groups_per_group_cycle %d group_core_offset %d", 
                  info->cores_per_group_cycle, info->groups_per_group_cycle, info->group_core_offset);
        uprintf(0, (char *)NULL);

        for (i = 0; i < 97; i++)
            {
            uprintf(1, "%3d: 0x%02x %2d", i, group_id[i], group_ntime_offset[i]);
            }
        uprintf(0, (char *)NULL);
        }
    }
#endif

static struct flags {
    uint8_t enabled:1;
    uint8_t addressed:1;
    } ef;

uint8_t asic_emulator_translate_packet(struct hf_header *h)
    {
    struct ucinfo_t *info = &ucinfo;
    struct hf_hash_serial *hash;
    struct hf_group_data *g;
    static bool first_address_packet = 1;
    em_core_t *em;
    uint16_t *p;
    uint32_t *q;
    uint16_t idx;
    uint8_t i;

    uint8_t return_this_packet;

    if (h->chip_address == HF_BROADCAST_ADDRESS)
        return_this_packet = 1;
    else
        return_this_packet = 0;

    switch (h->operation_code)
        {
        case OP_RESET:
            ef.enabled = 0;
            ef.addressed = 0;
            memset(&config_data, 0, sizeof(config_data));
            em = core_timer;
            if (h->chip_address == HF_BROADCAST_ADDRESS)
                {
                for (idx = 0; idx < info->total_cores; idx++, em++)
                    {
                    em->active = 0;
                    em->pending = 0;
                    group_id[idx] = 0;
                    group_ntime_offset[idx] = 0;
                    }
                }
            else
                {
                em += (uint16_t)h->chip_address;
                for (idx = 0; idx < info->core_count; idx++, em++)
                    {
                    em->active = 0;
                    em->pending = 0;
                    group_id[idx] = 0;
                    group_ntime_offset[idx] = 0;
                    }
                }
            break;

        case OP_ADDRESS:
            ef.addressed = 1;
            if (first_address_packet)
                {
                first_address_packet = 0;
                if (h->chip_address > 0)
                    {
                    // Must be connected to something real, like the FPGA Emulator.
                    // In that case, turn the software emulation off
                    enable_software_emulator = false;
                    return_this_packet = 1;
                    }
                }

            if (!h->chip_address)
                {
                // We're the chain receiving this OP_ADDRESS. Fake it out.
                h->chip_address = EM_DIE;                   // info->die_count;
                h->core_address = EM_CORES;                 // info->core_count;
                h->hdata = cpu_to_le16((125<<8)|1);         // G-1
                h->crc8 = hf_crc8((uint8_t *)h);
                ef.enabled = 1;
                return_this_packet = 1;
                }
            break;

        case OP_HASH:
            last_hash_sequence_number = le16_to_cpu(h->hdata);
            hash = (struct hf_hash_serial *)(h+1);

            if (hash->option & 0x2)
                {
                // A group hash. Fill in the tables.
                if (h->chip_address == HF_BROADCAST_ADDRESS)
                    {
                    for (idx = 0, em = core_timer; idx < info->total_cores; idx++, em++)
                        {
                        if (hash->group == group_id[idx])
                            {
                            if (em->active == 0)
                                {
                                em->active = EM_CORE_TICKS;
                                active_sequence[idx] = le16_to_cpu(h->hdata);
                                }
                            else
                                {
                                em->pending = 1;
                                pending_sequence[idx] = le16_to_cpu(h->hdata);
                                }
                            }
                        }
                    }
                else
                    {
                    idx = (uint16_t)h->chip_address * info->core_count;
                    em = core_timer;
                    em += idx;
                    for (i = 0; i < info->core_count; i++, idx++, em++)
                        if (group_id[idx] == hash->group)
                            {
                            if (em->active == 0)
                                {
                                em->active = EM_CORE_TICKS;
                                active_sequence[idx] = le16_to_cpu(h->hdata);
                                }
                            else
                                {
                                em->pending = 1;
                                pending_sequence[idx] = le16_to_cpu(h->hdata);
                                }
                            }
                    }
                }
            else
                {
                // No ntime rolling
                idx = (uint16_t)h->chip_address * info->core_count + h->core_address;
                em = core_timer;
                em += idx;

                if (em->active == 0)
                    {
                    if (active_sequence[idx] != (uint16_t)~0)
                        {
                        for(;;);
                        }
                    em->active = EM_CORE_TICKS;
                    active_sequence[idx] = le16_to_cpu(h->hdata);
                    }
                else
                    {
                    if (em->pending == 1 || pending_sequence[idx] != (uint16_t)~0)
                        {
                        for(;;);
                        }
                    em->pending = 1;
                    pending_sequence[idx] = le16_to_cpu(h->hdata);
                    }

                // See if this is the test case
                if (le32_to_cpu(hash->bits) == 0xd7690d1a)
                    {
                    int bad;

                    // It is, send back a successful reply, force the core back to be quiet
                    if (!(info->usb_init_header))
                        {
                        bad++;
                        }
                    asic_emulator_nonce(idx, (uint32_t)0x71b9235a);
                    em->active = 0;
                    active_sequence[idx] = (uint16_t)~0;
                    }
                }
            break;

        case OP_PLL_CONFIG:
            break;

        case OP_GPIO:
            break;

        case OP_STATUS:
            return_this_packet = 1;
            break;

        case OP_NONCE:
            return_this_packet = 1;
            break;

        case OP_CONFIG:
            memcpy(&config_data, (struct hf_config_data *)(h+1), sizeof(config_data));
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
            // Swap fields to make compatible with big endian structure
            p = (uint16_t *)&config_data;
            *p = cpu_to_le16(*p);
            q = (uint32_t *)&config_data;
            q += 2;
            *q = cpu_to_le32(*q);
            q++;
            p = (uint16_t *)q;
            *p = cpu_to_le16(*p);
            p++;
            *p = cpu_to_le16(*p);
#endif
            break;

        case OP_GROUP:
            g = (struct hf_group_data *)(h+1);
            idx = (uint16_t)h->chip_address * (uint16_t)info->core_count + (uint16_t)h->core_address;
            if (h->hdata == 1)
                {
                group_id[idx] = (uint8_t)h->hdata;
                group_ntime_offset[idx] = (uint8_t)g->ntime_offset;     // Only allowed 8 bits of ntime
                }
            else
                {
                group_id[idx] = 0;
                group_ntime_offset[idx] = 0;
                }
            // No support for nonce_msoffset
            break;

        // XXX TODO: Implement group aborts
        case OP_ABORT:
            em = core_timer;
            if (h->chip_address == HF_BROADCAST_ADDRESS && h->core_address == HF_BROADCAST_ADDRESS)
                {
                for (idx = 0; idx < info->total_cores; idx++, em++)
                    {
                    if (le16_to_cpu(h->hdata) & 0x4)
                        {
                        if (group_id[idx] == (le16_to_cpu(h->hdata) >> 8))
                            {
                            if (le16_to_cpu(h->hdata) & 0x1)
                                em->active = 0;
                            if (le16_to_cpu(h->hdata) & 0x2)
                                em->pending = 0;
                            }
                        }
                    else
                        {
                        if (le16_to_cpu(h->hdata) & 0x1)
                            {
                            active_sequence[idx] = (uint16_t)~0;
                            em->active = 0;
                            }
                        if (le16_to_cpu(h->hdata) & 0x2)
                            {
                            pending_sequence[idx] = (uint16_t)~0;
                            em->pending = 0;
                            }
                        }
                    }
                }
            else if (h->core_address == HF_BROADCAST_ADDRESS)
                {
                em += (uint16_t)h->chip_address * (uint16_t)info->core_count;
                for (idx = 0; idx < info->core_count; idx++, em++)
                    {
                    if (le16_to_cpu(h->hdata) & 0x4)
                        {
                        if (group_id[idx] == (le16_to_cpu(h->hdata) >> 8))
                            {
                            if (le16_to_cpu(h->hdata) & 0x1)
                                em->active = 0;
                            if (le16_to_cpu(h->hdata) & 0x2)
                                em->pending = 0;
                            }
                        }
                    else
                        {
                        if (le16_to_cpu(h->hdata) & 0x1)
                            {
                            active_sequence[idx] = (uint16_t)~0;
                            em->active = 0;
                            }
                        if (le16_to_cpu(h->hdata) & 0x2)
                            {
                            pending_sequence[idx] = (uint16_t)~0;
                            em->pending = 0;
                            }
                        }
                    }
                }
            else
                {
                em += (uint16_t)h->chip_address * (uint16_t)info->core_count + h->core_address;
                if (le16_to_cpu(h->hdata) & 0x4)
                    {
                    if (group_id[idx] == (le16_to_cpu(h->hdata) >> 8))
                        {
                        if (le16_to_cpu(h->hdata) & 0x1)
                            em->active = 0;
                        if (le16_to_cpu(h->hdata) & 0x2)
                            em->pending = 0;
                        }
                    }
                else
                    {
                    if (le16_to_cpu(h->hdata) & 0x1)
                        {
                        active_sequence[idx] = (uint16_t)~0;
                        em->active = 0;
                        }
                    if (le16_to_cpu(h->hdata) & 0x2)
                        {
                        pending_sequence[idx] = (uint16_t)~0;
                        em->pending = 0;
                        }
                    }
                }
            break;
			
	case OP_CLOCKGATE:
	    break;

        case OP_LOOPBACK_UART:
            return_this_packet = 1;
            break;

        default:
            break;
        }

    return(return_this_packet);
    }


void asic_emulator_loop()
    {
    struct ucinfo_t *info = &ucinfo;
    static uint16_t last_time;
    static uint16_t last_sec;
    static uint16_t last_msec_status;

    uint16_t i;

    em_core_t *em;

    uint16_t idx;
    uint16_t msec;
    uint16_t msec_status;
    bool all_cores_idle = false;

    msec = elapsed_since(last_time);
    msec_status = elapsed_since(last_msec_status);

    if (info->connected)
        ; //LEDs_TurnOnLEDs(LEDS_LED2);
    else
        ; //LEDs_TurnOffLEDs(LEDS_LED2);

    // Generate phoney nonces sometimes
    if (info->connected && ef.enabled && sec_ticker != last_sec)
        {
        idx = rand() % EM_TOTAL_CORES;
        em = core_timer;
        em += idx;
        if (em->active)
            {
            // Found an active core, generate a phoney nonce
            asic_emulator_nonce(idx, (uint32_t)0);
            }
        last_sec = sec_ticker;
        }

    // Iterate through all the cores every 250 msec
    if (msec >= EM_TICK)
        {
        for (i = 0, em = core_timer, all_cores_idle = true; i < (uint16_t)EM_TOTAL_CORES; i++, em++)
            {
            if (em->active)
                {
                em->active--;
                if (!em->active)
                    {
                    if (em->pending)
                        {
                        // Pending job goes active.
                        em->active = EM_CORE_TICKS;
                        active_sequence[i] = pending_sequence[i];
                        em->pending = 0;
                        pending_sequence[i] = ~0;
                        all_cores_idle = false;
                        }
                    else
                        active_sequence[i] = ~0;
                    }
                else
                    all_cores_idle = false;
                }
            }
        last_time = msec_ticker;
        }

    // Generate an OP_STATUS every half second
    if (info->connected && !info->usb_init_header &&
               (config_data.enable_periodic_status && msec_status >= 500 /* config_data.status_period */)
            || (config_data.send_status_on_core_idle && all_cores_idle)
          )
        {
        asic_emulator_op_status();
        last_msec_status = msec_ticker;
        }


    }

int bad = 0;
void check_spurious(void)
    {
#if 0
    if (*(uint32_t *) 0x181C == 0)
        {
        bad++;
        *(uint32_t *) 0x181C = 0xffffffff;         // Mark it again
        }
#endif
    }

//
// Generate a nonce. We then force it to be transmitted, which in turn causes us to receive it,
// which is the easiest way to get it passed back to the host!!!
//

static void asic_emulator_nonce(uint16_t idx, uint32_t value)
    {
    struct hf_header *h;
    struct hf_candidate_nonce *n;
    uint16_t *p;

    if (!(h = (struct hf_header *)asic_get_tx_buffer()))
        return;

    h->preamble = HF_PREAMBLE;
    h->operation_code = OP_NONCE;
    h->chip_address = (idx / EM_CORES);           // Address of the die generating the nonce
    h->core_address = 0;
    h->hdata = 0;
    h->data_length = sizeof(*n)/4;
    h->crc8 = hf_crc8((uint8_t *)h);

    n = (struct hf_candidate_nonce *)(h+1);
    if (value == (uint32_t)0)
        {
        // Fake
        n->nonce = cpu_to_le32(0x42420000 | ((uint32_t)h->chip_address << 8) | (idx % EM_CORES));
        n->ntime = cpu_to_le16(group_ntime_offset[idx]);
        }
    else
        {
        n->nonce = cpu_to_le32(value);
        n->ntime = 0;
        }
    n->sequence = cpu_to_le16(active_sequence[idx]);

    hf_crc32((uint8_t *)n, sizeof(*n), 1);

    asic_queue_transmit();
    }

static void asic_emulator_op_status()
    {
    struct ucinfo_t *info = &ucinfo;
    struct hf_header *h;
    em_core_t *em;
    uint16_t *m;
    uint16_t i, j, k;
    uint8_t len;
    uint16_t bitmap;

    len = 2 + (info->core_count + 15) / 16;

    for (i = (uint16_t)0; i < info->die_count; i++)
        {
        if (!(h = (struct hf_header *)asic_get_tx_buffer()))
            return;

        h->preamble = HF_PREAMBLE;
        h->operation_code = OP_STATUS;
        h->chip_address = i;
        h->core_address = 0;
        h->hdata = cpu_to_le16(last_hash_sequence_number);
        h->data_length = len;
        h->crc8 = hf_crc8((uint8_t *)h);

        em = core_timer;
        em += (i * info->core_count);

        m = (uint16_t *)(h+1);

        // Monitor data
        *m++ = cpu_to_le16((uint16_t)2159);              // 65 degrees C
        *m++ = cpu_to_le16((uint16_t)((149<<8)|150));
        *m++ = cpu_to_le16((uint16_t)((151<<8)|152));
        *m++ = cpu_to_le16((uint16_t)((153<<8)|154));

        for (j = (uint16_t)0; j < info->core_count; j += (uint16_t)8)
            {
            bitmap = (uint16_t)0;
            for (k = (uint16_t)0; k < (uint16_t)8; k++)
                {
                if ((j+k) < info->core_count)
                    {
                    if (em->active)
                        bitmap |= ((uint16_t)1 << (2*k));
                    if (em->pending)
                        bitmap |= ((uint16_t)1 << (2*k+1));
                    em++;
                    }
                }
            *m++ = cpu_to_le16(bitmap);
            }

        m = (uint16_t *)(h+1);
        hf_crc32((uint8_t *)m, len*4, 1);
        asic_queue_transmit();
        }
    }

void asic_emulator_init(void)
    {
    em_core_t *em;
    uint16_t i;

    for (i = 0, em = core_timer; i < (uint16_t)EM_TOTAL_CORES; i++, em++)
        {
        em->active = 0;
        em->pending = 0;
        active_sequence[i] = ~0;
        pending_sequence[i] = ~0;
        }
    for (i = 0; i < sizeof(group_ntime_offset); i++)
        group_ntime_offset[i] = 0;
    last_hash_sequence_number = 0;
#if 0
    *(uint32_t *) 0x181C = 0xffffffff;         // XXX
#endif
    }

#endif

