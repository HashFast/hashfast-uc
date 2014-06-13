//
// Global Work Queue protocol
//
// Manage all the cores locally, so the host doesn't have to.
// Use ntime rolling to bring about work multiplication
// See the relevant section(s) of the protocol user's guide.
//

//#define  DEBUG_RESUMPTION_AFTER_WORK_RESTART

#include "main.h"
#include "cli.h"

static void gwq_update_sequence_tail(uint8_t);
static uint8_t gwq_monitor(struct hf_header *, bool);

DEBUG_RESUMPTION_AFTER_WORK_RESTART_DEFINES;

// Bit set for valid, configured groups                                 // Typical spaces
static uint16_t group_valid_bitmap[MAX_GROUPS/16];                      //    6 bytes

// Bit set for active, pending status each group
static uint16_t group_pending[MAX_GROUPS/16];                           //    6 bytes
static uint16_t group_active[MAX_GROUPS/16];                            //    6 bytes

// So 24 to 1 causes the effective core count to be 96/24 = 4 per die
// 6 modules would be 4*4*6 = 96, so a sequence space of >192 = 256 (2^8) will work OK
//
// A max work queue of (256-192) = 64 entries. Each OP_HASH is 56 bytes, so that's 3584 bytes of buffer space.
// But that's a lot to go stale and require flushing. We halve this to 32 entries, 1792 bytes

// Sequence space indexed work
// bit7 = work active
// b6:0 = count (max split count 63)
static uint8_t split_count[MAX_GROUP_SEQUENCE_SPACE];                   //  256 bytes

// For each core, active and passive sequence #
struct uc_core_t {
    uint16_t active;     // Sequence # of active job
    uint16_t pending;    // Sequence # of pending job
    };

static struct uc_core_t core_sequence[MAX_CORES];                       // 2304 bytes   (OP_STATUS lookups)

static uint16_t sequence_bitmap[MAX_SEQUENCE_SPACE/16];

// The odd bad core means #cores per group can vary
static uint8_t  group_actual_cores[MAX_GROUPS];                         //   48 bytes

// Bitmaps for whether pending/active are valid
static uint16_t core_pending[MAX_CORES/16];                             //   72 bytes
static uint16_t core_active[MAX_CORES/16];                              //   72 bytes

// Fogbugz 6138
static uint8_t sequence_to_core_index[MAX_SEQUENCE_SPACE];              // 8192 bytes

// Accumulated hash count for passing back to host periodically
static uint64_t hash_count;

//
// Per device info
//
struct ucinfo_t ucinfo __attribute__((aligned(2))) = {0};

uint16_t group_sequence_head = 0;
uint16_t group_sequence_tail = 0;
static uint16_t last_group_sequence_tail = 0;

//
// If a core ever gets stuck (due to excessive over-clocking), the tail pointer will get stuck.
// In this case, the watchdog will fire, and the system will be re-started.
//
static void gwq_update_sequence_tail(uint8_t group)
    {
    uint16_t i;

    i = group_sequence_tail;

    if (group)
        {
        while (ucinfo.inflight)
            {
            if ((++i) >= ucinfo.num_sequence)
                i = (uint16_t)0;

            if (split_count[i] & 0x80)
                return;                         // This work is still out there

            ucinfo.inflight--;                   // This work must have finished
            group_sequence_tail = i;
            }
        }
    else
        {
        // No groups, all single cores
        while (ucinfo.inflight)
            {
            if ((++i) >= ucinfo.num_sequence)
                i = (uint16_t)0;

            if (sequence_bitmap[i>>4] & ((uint16_t)1<<(i&0xf)))
                return;                             // This job is still out there

            ucinfo.inflight--;
            if (i >= ucinfo.num_sequence)
                uprintf(UD_SEQUENCE, "ERROR: gwq_ust: i %d about to be tail %d\n", i, group_sequence_tail);
            group_sequence_tail = i;
            }
        }
    }

////////////////////////////////////////////////////////////////////////////////
//
// Input: Either an OP_STATUS, an OP_NONCE or an OP_STATISTICS
//
////////////////////////////////////////////////////////////////////////////////

static uint16_t work_restart_start_time;
static uint16_t last_tail_update_call;
static uint16_t last_hash_count_check;
static uint32_t avg_hash_rate;
static uint16_t last_sequence;
static uint16_t blabber_chip_address;
static uint16_t blabber_core_address;
static uint16_t blabber_start;
static uint32_t blabber_count;
static bool     work_completion_flag;

uint8_t gwq_process_input(struct hf_header *h)
    {
    uint16_t core_index;                            // Core index across all die
    uint16_t bitmap_mask;
    uint8_t  bitmap_idx;
    uint8_t  num_nonces;
    uint8_t  group_index;
    uint8_t  group_valid;                           // True if core being considered is a member of a group
    uint16_t status_sequence;                       // The sequence number from an OP_STATUS
    uint8_t  pass_up_to_host;
    bool     completed_something;
    uint8_t  i;

    struct ucinfo_t *info = &ucinfo;
    struct uc_core_t *core;
    struct hf_candidate_nonce *n;
    bool    force = false;

    uint16_t *p;
    uint16_t active_bitmask;
    uint16_t pending_bitmask;
    uint16_t sd;
    uint64_t hash_increment;

    pass_up_to_host = 0;

    switch (h->operation_code)
        {
        case OP_STATUS:
            // A status message. Examine the activity map.

            status_watchdog_clock = STATUS_WATCHDOG_RECHARGE;

            status_sequence = le16_to_cpu(h->hdata);
            if (info->ntime_roll_total > 1)
                status_sequence &= info->group_mask;
            p = (uint16_t *) (h + 1);
            p += 4;                                                         // Now points to start of bitmap
#ifdef WORK_RESTART_DEBUG
            gwq_work_restart_check(h->chip_address, p);
#endif // WORK_RESTART_DEBUG
            *p = le16_to_cpu(*p);

            // Core status is contained in the remainder of the data packet
            core_index = CORE_INDEX(h->chip_address,0);                     // Index into core_sequence
            core = &core_sequence[0] + core_index;
            completed_something = false;

            // Figure the number of hashes for any completed jobs for this die.
            hash_increment = (dynamic_nonce_range[h->chip_address]) ? (uint64_t)dynamic_nonce_range[h->chip_address] : info->hash_loops;
            if (info->work_restart_in_progress == true)
                hash_increment >>= 1;                                       // Very rough approximation, assume average half done

            // TODO: investigate bit space for a "current overload"
            if (h->core_address & 0x80)
                {
                // Wow, a thermal overload has occurred on this die. We're toast and we have to clear out every job
                // on this die because it has reset itself. This is a hash clock reset only - the framer etc. will
                // still be active, dynamic baud rates will stay the same etc. We don't bother with any of this, the
                // host gets notification (through OP_GWQ_STATUS) and resets us.
                }
            DEBUG_RESUMPTION_AFTER_WORK_RESTART_STATUS_DUMP;

            // Check each core's pending/busy status on this die, and adjust jobs as they complete.
            // Be very careful with the following code. Introducing a bug that causes the loss of core
            // status temporal coherency can lead to mayhem.
            group_index = 0;                                                // XXX Need something to determine group index for starting core on this die.
            active_bitmask = (1<<0);
#ifdef DEBUG_RESUMPTION_AFTER_WORK_RESTART
            if (resumption_blackout == false)
#endif
            for (i = 0; i < info->core_count; i++, core++, core_index++)
                {
                group_valid = (info->ntime_roll_total > 1) && GROUP_VALID(group_index);
                bitmap_idx = core_index>>4;                                 // Index into the core_* bitmasks
                bitmap_mask = (1 << (i & 0xf));                             // Bitmask for the core_* bitmasks
                pending_bitmask = active_bitmask << 1;
                //current_status = ((*p) >> ((core_index & 0x7) << 1)) & 0x3; // b1 = pending active, b0 = core active

                if (core_pending[bitmap_idx] & bitmap_mask)
                    {
                    if ((sd = HF_SEQUENCE_DISTANCE(status_sequence,core->pending)) < info->num_sequence/2)
                        {
                        // Small number or zero - this status is "up to date" relative to this core's pending job
                        //if ((current_status & 0x2) == 0)
                        if (!(*p & pending_bitmask))
                            {
                            // Pending isn't "busy", so the active job must have finished!
                            DEBUG_RESUMPTION_AFTER_WORK_RESTART_FREEZE_IF_FILL_STILL_ACTIVE;
                            if (CORE_ACTIVE(core_index))
                                {
                                // Finished. Release the corresponding job.
                                if (group_valid)
                                    {
                                    split_count[core->active]--;
                                    if (split_count[core->active] & 0x7f)
                                        {
                                        }
                                    else
                                        {
                                        split_count[core->active] = 0;
                                        if (group_valid)
                                            GROUP_TRANSFER_PENDING_TO_ACTIVE;
                                        }
                                    }
                                else
                                    {
                                    sequence_bitmap[core->active>>4] &= ~((uint16_t)1 << (core->active&0xf));     // Mark this sequence as done
                                    }
                                completed_something = true;
                                info->active_jobs--;
                                hash_count += hash_increment;
                                core->active = core->pending;                                           // Move pending sequence # to active
                                core_pending[bitmap_idx] = (core_pending[bitmap_idx] & (~bitmap_mask)); // Clear core pending busy bit
                                }
                            // Check to see if pending job finished as well....
                            //if ((core_active[bitmap_idx] & bitmap_mask) && ((current_status & 0x1) == 0))
                            if ((core_active[bitmap_idx] & bitmap_mask) && !(*p & active_bitmask))
                                {
                                DEBUG_RESUMPTION_AFTER_WORK_RESTART_FREEZE_IF_FILL_STILL_ACTIVE;
                                // Finished. Release the corresponding job.
                                if (group_valid)
                                    {
                                    split_count[core->active]--;
                                    if (split_count[core->active] & 0x7f)
                                        {
                                        }
                                    else
                                        {
                                        split_count[core->active] = 0;
                                        if (group_valid)
                                            GROUP_TRANSFER_PENDING_TO_ACTIVE;
                                        }
                                    }
                                else
                                    {
                                    sequence_bitmap[core->active>>4] &= ~((uint16_t)1 << (core->active&0xf));     // Mark this sequence as done
                                    }
                                completed_something = true;
                                info->active_jobs--;
                                hash_count += hash_increment;
                                core_active[bitmap_idx] = core_active[bitmap_idx] & (~bitmap_mask);
                                }
                            }
                        }
                    }
                else if (core_active[bitmap_idx] & bitmap_mask)                 // No pending job. Check active only
                    {
                    if ((sd = HF_SEQUENCE_DISTANCE(status_sequence,core->active)) < info->num_sequence/2)
                        {
                        // Small number or zero - this status is "up to date" relative to this core's active job
                        //if ((current_status & 0x1) == 0)
                        if (!(*p & active_bitmask))
                            {
                            DEBUG_RESUMPTION_AFTER_WORK_RESTART_FREEZE_IF_FILL_STILL_ACTIVE;
                            // Finished. Release the corresponding job.
                            if (group_valid)
                                {
                                split_count[core->active]--;
                                if (split_count[core->active] & 0x7f)
                                    {
                                    }
                                else
                                    {
                                    split_count[core->active] = 0;
                                    if (group_valid)
                                        group_active[group_index>>4] &= ~(1 << (group_index&0xf));
                                    }
                                }
                            else
                                {
                                sequence_bitmap[core->active>>4] &= ~((uint16_t)1 << (core->active&0xf));     // Mark this sequence as done
                                }
                            completed_something = true;
                            info->active_jobs--;
                            hash_count += hash_increment;
                            core_active[bitmap_idx] = core_active[bitmap_idx] & (~bitmap_mask);
                            }
                        }
                    }

                // Increment to next 16 bits of buffer every 8 cores
                if ((i & 0x7) == 7)
                    {
                    ++p;
                    *p = le16_to_cpu(*p);
                    active_bitmask = (1<<0);
                    }
                else
                    {
                    active_bitmask <<= 2;
                    }

                if (++group_index >= info->group_core_offset)
                    group_index = 0;
                }

            if (completed_something)
                {
                work_completion_flag = true;
                last_tail_update_call = msec_ticker;
                gwq_update_sequence_tail(group_valid);
                }

            if (info->work_restart_in_progress == true)
                {
#ifdef FEATURE_COWARDLY_WORK_RESTART
                bool got_pending;

                if (info->restart_phase == RESTART_PHASE_1)
                    {
                    for (i = 0, got_pending = false; i < DIMENSION(core_pending); i++)
                        if (core_pending[i])
                            {
                            got_pending = true;
                            break;
                            }
                    if (!got_pending)
                        {
                        info->restart_phase = RESTART_PHASE_2;
                        }
                    }
#endif
                }
            pass_up_to_host = gwq_monitor(h, force);                // Deal with the monitoring data
            break;

        case OP_NONCE:
            pass_up_to_host = 1;
            // One or more nonces have been returned.
            n = (struct hf_candidate_nonce *)(h+1);
            num_nonces = (h->data_length<<2)/sizeof(struct hf_candidate_nonce);
            if (info->ntime_roll_total > 1)
                {
                // The ntime to use will be in the MS bits of the sequence number!
                for (i = 0; i < num_nonces; i++, n++)
                    {
                    status_sequence = le16_to_cpu(n->sequence);
                    n->ntime = cpu_to_le16(status_sequence >> info->group_shift);
                    n->sequence = cpu_to_le16(status_sequence & info->group_mask);
                    }
                }
            if (num_nonces == 1)
                {
                // Fogbugz 6138
                h->core_address = sequence_to_core_index[le16_to_cpu(n->sequence) & (sizeof(sequence_to_core_index)-1)];
                h->crc8 = hf_crc8((uint8_t *)h);
                if (blabber_count == 0)
                    {
                    blabber_core_address = h->core_address;
                    blabber_chip_address = h->chip_address;
                    }
                }

            // Do some nonce sanity checks, and drop them if they look bad
            if (h->chip_address >= info->die_count)
                {
                // XXX This should be impossible, but we've seen an address of 255 on OP_NONCE's when
                // hardware problems have occurred
                pass_up_to_host = 0;
                }
            else
                {
                for (i = 0; i < num_nonces; i++, n++)
                    {
                    if (le32_to_cpu(n->nonce) == 0xffffff6f
                        || le32_to_cpu(n->nonce) == 0xffffffd7
                        || le32_to_cpu(n->nonce) == 0x00000001
                        || le32_to_cpu(n->nonce) == 0x80000001)
                        {
                        //++stats.bad_nonce_value;
                        pass_up_to_host = 0;                // Drop the invalid nonce(s)
                        }
                    if (le16_to_cpu(n->sequence) & (~(uint16_t)(info->num_sequence-1)))
                        {
                        //++stats.bad_nonce_sequence;
                        pass_up_to_host = 0;                // Drop the nonce(s)
                        }
                    if (le16_to_cpu(n->ntime) & HF_NONCE_SEARCH)
                        {
                        // AP - Fogbugz report to follow
                        pass_up_to_host = 0;                // Drop the nonce(s)
                        }
                    if (le16_to_cpu(n->sequence) == last_sequence)
                        {
                        //++stats.repeated_nonce_sequence;
                        if (++blabber_count >= 3 && elapsed_since(blabber_start) < 50 && !system_powering_down())
                            {
#ifdef FEATURE_CORE_BLABBER_USB_SQUELCH
                            pass_up_to_host = 0;                // Drop the nonce(s)
#endif // FEATURE_CORE_BLABBER_USB_SQUELCH
                            blabber_start = msec_ticker;        // Keep suppressing

                            if (blabber_count == 3)
                                { // do once
                                if (blabber_chip_address != h->chip_address || blabber_core_address != h->core_address)
                                    {
                                    notify_host("Blabber core recorded (%d / %d) does not match current core (%d / %d).", blabber_chip_address, blabber_core_address, h->chip_address, h->core_address);
                                    }
                                else
                                    {
                                    notify_host("Hash Core Error: Squelching die %d core %d", h->chip_address, h->core_address);
                                    }

#ifdef FEATURE_CORE_BLABBER_ABORT
                                // Send an abort
                                struct hf_header *ha = (struct hf_header *)asic_get_tx_buffer();
                                if (ha)
                                {
                                    make_abort(ha, h->chip_address, h->core_address, 3);
                                    asic_queue_transmit();
                                    uprintf(-1, "Squelched Blabber: die %d core %d\n", h->chip_address, h->core_address);
                                } else {
                                    notify_host("Error: Could not get asic tx buffer.");
                                }
#endif // FEATURE_CORE_BLABBER_ABORT

#ifdef FEATURE_CORE_BLABBER_HARDMAP
#ifndef FEATURE_INITIALIZE_NON_VALID_DIE_SETTINGS
  #error You have to enable FEATURE_INITIALIZE_NON_VALID_DIE_SETTINGS if you use FEATURE_CORE_BLABBER_HARDMAP
#endif // warning message

                                // Map out the core in the user page
                                uprintf(UD_SEQUENCE, "Hash Core Error: Mapping out die %d core %d\n", h->chip_address, h->core_address);
                                hf_nvram_write_bad_core_bitmap(h->chip_address>>2, 0x4000 | ((h->chip_address & 0x3) * 96) + h->core_address);
#ifdef FEATURE_CORE_BLABBER_MAP_REBOOT
                                // shutdown.  USB will re-enumerate, then host will hotplug the device, with the core mapped out
                                usb_powerdown_request();
#endif // FEATURE_CORE_BLABBER_MAP_REBOOT
#endif // FEATURE_CORE_BLABBER_HARDMAP

#ifdef FEATURE_CORE_BLABBER_SOFTMAP
                                if (ucinfo.shed_supported)
                                    {
                                    // Mark it bad in the ephemeral good cores map
                                    uint16_t core_idx = CORE_INDEX(h->chip_address, h->core_address);
                                    // Test is to protect shed_amount from growing, we can wind up here multiple times in the same blabber event
                                    if (core_map_set_disabled(core_idx, false))
                                        {
                                        uprintf(UD_SEQUENCE, "Hash Core Marked Bad: Mapping out die %d core %d index %d\n", h->chip_address, h->core_address, core_idx);
                                        }
                                    }
#endif // FEATURE_CORE_BLABBER_SOFTMAP

                                }
                            if (blabber_count % 100 == 0)
                                {
                                notify_host("Hash Core Error: Still emitting hashes: die %d core %d, count: %d", h->chip_address, h->core_address, blabber_count);
                                }
                            }
                        }
                    // Keep counting blabber until time expires, not until new sequence seen
                    else if (blabber_count > 3 && elapsed_since(blabber_start) < 50 && !system_powering_down())
                        {
                        // Skip over valid nonces that are interspersed when we are in blabber mode
                        }
                    else // new sequence starting after blabber stopped
                        {
                        blabber_count = 0;
                        blabber_start = msec_ticker;
                        last_sequence = le16_to_cpu(n->sequence);
                        }
                    }
                }
            break;

        case OP_STATISTICS:
            pass_up_to_host = 1;
            break;

        case OP_CONFIG:
            //if (info->work_restart_in_progress == true || info->resend_op_config == true)
                pass_up_to_host = 0;
            break;

        case OP_ABORT:
        case OP_PLL_CONFIG:
            pass_up_to_host = 0;
            break;

        default:
            // Nothing to do with me
            pass_up_to_host = 1;
            break;
        }

    // Inhibit passing any packets to host during OP_USB_INIT and OP_USB_SHUTDOWN operations
    if (info->usb_init_header)
        pass_up_to_host = 0;

    return(pass_up_to_host);
    }

//
// Extract monitoring information
//
// If it is an EVEN die, we combine it with the next die's stored data, and send
// them back to the host as an OP_DIE_STATUS
//
// If it is die #1, then we turn it into an OP_GWQ_STATUS. This is just using the
// ASIC's natural OP_STATUS timing to generate OP_GWQ_STATUS's, rather than having
// to bother with a timer locally.
//

static struct hf_g1_die_data monitor_data[MAX_DIE];

void gwq_update_board_temperature(uint8_t i, uint16_t temperature)
    {
    monitor_data[i].temperature = cpu_to_le16(temperature);
    }

uint16_t gwq_get_board_temperature(uint8_t i)
    {
    return le16_to_cpu(monitor_data[i].temperature);
    }

void gwq_update_tach(uint8_t i, uint16_t tach)
    {
    monitor_data[i].tacho = cpu_to_le16(tach);
    }

uint16_t gwq_get_tach(uint8_t i)
    {
    return le16_to_cpu(monitor_data[i].tacho);
    }

uint16_t gwq_get_die_temperature(uint8_t i)
    {
    return monitor_data[i].die.die_temperature;
    }

void gwq_update_phase_currents(uint16_t *phase_current)
    {
    int die, j;

    for (die = 0; die < 4; die++)
        for (j = 0; j < 4; j++)
            {
            monitor_data[die].phase_currents[j] = *phase_current++;
            }
    }

void gwq_update_regulator_voltage(uint16_t *voltage)
    {
    int die;

    for (die = 0; die < 4; die++)
        monitor_data[die].voltage = *voltage++;
    }

static uint8_t gwq_monitor(struct hf_header *h, bool force_gwq_status)
    {
    struct ucinfo_t *info = &ucinfo;
    struct hf_g1_monitor *m = (struct hf_g1_monitor *)(h+1);
    struct hf_gwq_data *g;
    struct hf_g1_die_data *d;
    static uint8_t flip = 0;
    uint8_t addr = h->chip_address;

    monitor_data[addr].die.die_temperature = le16_to_cpu(m->die_temperature);
    memcpy(&monitor_data[addr].die.core_voltage[0], m->core_voltage, sizeof(m->core_voltage));
    // XXX Something else has to fill the other fields in

    if (info->die_count == 1)
        {
        // If there is only one die, the following addressing stuff breaks, so we alternate
        // between OP_DIE_STATUS and OP_GWQ_STATUS frame generation.
        flip = ~flip;
        }
    else
        flip = 1;

    if (h->core_address & 0x80)
        force_gwq_status = true;

    if (!force_gwq_status && flip && (addr & 0x1) == 0)
        {
        // For even die, we turn it into TWO OP_DIE_STATUS
        h->operation_code = OP_DIE_STATUS;
        h->hdata = 0;
        h->data_length = (sizeof(struct hf_g1_die_data)/4);

        d = (struct hf_g1_die_data *)(h+1);
        if (info->device_type == HFD_VC709)
            {
            // Convert VC-709 temperature to G-1 representation, and stick in dummy core voltages
            float vc709_die_temperature;

            vc709_die_temperature = (*(uint16_t *)d * 503.975) / 65536.0 - 273.15;
            d->die.die_temperature = cpu_to_le16((uint16_t)((vc709_die_temperature + 61.5) * (4096.0/240.0)));
            d->die.core_voltage[0] = (uint8_t)(0.80*256.0/1.2);
            d->die.core_voltage[1] = (uint8_t)(0.81*256.0/1.2);
            d->die.core_voltage[2] = (uint8_t)(0.80*256.0/1.2);
            d->die.core_voltage[3] = (uint8_t)(0.81*256.0/1.2);
            d->die.core_voltage[4] = (uint8_t)(0.80*256.0/1.2);
            d->die.core_voltage[5] = (uint8_t)(0.81*256.0/1.2);
            }
        else
            {
            memcpy((uint8_t *)(d), &monitor_data[addr], sizeof(*d));
            d->die.die_temperature = cpu_to_le16(d->die.die_temperature);
            }
        if (info->die_count >= (addr+2))
            {
            h->data_length += (sizeof(struct hf_g1_die_data)/4);
            d++;
            memcpy((uint8_t *)(d), &monitor_data[addr+1], sizeof(*d));
            d->die.die_temperature = cpu_to_le16(d->die.die_temperature);
            }
        h->crc8 = hf_crc8((uint8_t *)h);

        if (info->work_restart_in_progress)
            return(0);

        return 1;
        }
    else if (force_gwq_status || ((info->die_count == 1 && !flip) || addr == 1))
        {
        // If it is die #1, we turn it into an OP_GWQ_STATUS
        h->operation_code = OP_GWQ_STATUS;
        h->chip_address = HF_GWQ_ADDRESS;
        h->hdata = 0;
        h->data_length = sizeof(struct hf_gwq_data)/4;
        h->crc8 = hf_crc8((uint8_t *)h);

        g = (struct hf_gwq_data *)(h+1);
        g->hash_count = cpu_to_le64(hash_count);
        // FIXME: gigahash or gibahash?  Giga for now.
        // last_hash_count_check is in msec
        // target is 100 GH/sec per die
        uint32_t ms_elapsed = elapsed_since(last_hash_count_check);
        if (!ms_elapsed) ms_elapsed = 1;  // don't want to DIV0
        const uint64_t gigahash = 1000UL * 1000 * 1000;
        uint64_t hash_rate = ((hash_count * 1000) / ms_elapsed) / gigahash;

        if (hash_rate > 500 * info->die_count || hash_rate < 50) hash_rate = 50;  // toss out extremes to dampen
        if (!avg_hash_rate) avg_hash_rate = hash_rate;
        avg_hash_rate += ((int64_t)hash_rate - avg_hash_rate) * 0.1;

        // TODO: info->die_count
        if (avg_hash_rate > 250 * info->die_count) {
            led_mode = LED_PLAID;
        } else if (avg_hash_rate > 75 * info->die_count) {
            led_mode = LED_HASH_FAST;
        } else if (avg_hash_rate > 50 * info->die_count) {
            led_mode = LED_HASH_SLOW;
        } else if (avg_hash_rate > 25 * info->die_count) {
            led_mode = LED_HASH_REALLYSLOW;
        } else {
            led_mode = LED_IDLE;
        }
        last_hash_count_check = msec_ticker;

        hash_count = (uint64_t)0;

        if (info->work_restart_in_progress)
            return(0);                                              // Hold off OP_GWQ_STATUS to host while work restart in progress

        g->sequence_head = cpu_to_le16(group_sequence_head);
        g->sequence_tail = cpu_to_le16(last_group_sequence_tail);   // XXX Need to make sure head doesn't pass last tail
        g->shed_count = cpu_to_le16(ucinfo.shed_amount);            // Non zero if cores turned off on the fly
        g->spare = 0;
        last_group_sequence_tail = group_sequence_tail;
        return 1;
        }

    return 0;
    }

////////////////////////////////////////////////////////////////////////////////
//
// When a pending slot becomes available, the OP_GWQ_STATUS message going back to the host informs
// the host that work is needed. The host queues an OP_HASH to chip address 254, and it winds up
// here. We modify the OP_HASH to tie it to an empty group, make it into a multicast, and let it
// be sent on to the ASICs.
//
// Or for G-1, we fake out the group functionality by sending this hash, modified, to each
// core in the group. The ntime is carried in the MS bits of the sequence number, so we
// can fiddle any nonces that come back and get the correct ntime into them.
//
//
////////////////////////////////////////////////////////////////////////////////

bool gwq_process_hash_group(struct hf_header *h)
    {
    uint16_t    i;
    uint16_t    j, k;
    uint16_t    mask = 0;
    uint16_t    group_valid, group_busy;
    uint16_t    base_core;                          // The base core of the group we're dealing with
    uint8_t     pending;
    uint8_t     group_index;

    struct uc_core_t *core;
    struct ucinfo_t *info = &ucinfo;

#ifndef G1_5_OR_LATER
    struct uart_sendinfo *s;
#endif

    // Find a free (and enabled) group. If something in group_active[] is clear, the core is
    // idle so we choose that one.  Otherwise, we try and find a group with an empty pending slot.
    for (pending = 0; pending <= 1; pending++)
        {
        for (j = 0, group_index = 255; j < DIMENSION(group_active) && (j<<4) < info->groups; j++)
            {
            group_busy = (pending) ? group_pending[j] : group_active[j];
            group_valid = group_valid_bitmap[j];

            if (group_valid & (~group_busy))
                {
                // Something is available in this word. Find it.
                for (i = 0, mask = (uint16_t)1; i < 16 && ((j<<4)+i) < info->groups; i++, mask <<= 1)
                    {
                    if (group_valid & ~group_busy & mask)
                        {
                        group_index = (j<<4)+i;
                        goto group_found;
                        }
                    }
                }
            }
        }
group_found:
    if (group_index > info->groups-1)
        {
        // This is a bug, resulting in the "work" disappearing. Log it.
        ucinfo.work_overrun++;
        return(false);
        }
    group_sequence_head = le16_to_cpu(h->hdata);

    h->data_length = 15;

#ifdef G1_5_OR_LATER
    // Send the work to this group
    h->chip_address = HF_BROADCAST_ADDRESS;
    h->core_address = HF_BROADCAST_ADDRESS;
    h->crc8 = hf_crc8((uint8_t *)h);

    // Fix up the OP_HASH
    hash = (struct hf_hash_serial *) (h+1);
    hash->option = 2;                               // Bit 1 == GROUP JOB
    hash->group = GROUP_ID(group_index);
    memset(hash->spare3, 0, sizeof(hash->spare3));
    // Done! The OP_HASH is ready to send (happens on return)
#endif
    // Update local status information. First, figure out the base core
    for (k = group_index, base_core = 0; k > info->groups_per_group_cycle; k -= info->groups_per_group_cycle, base_core += info->cores_per_group_cycle)
        ;
    base_core += k;

    core = &core_sequence[0] + base_core;
    split_count[(uint8_t)(le16_to_cpu(h->hdata))] = 0x80 | group_actual_cores[group_index];    // Bad cores mean this could be < info->cores_per_group

#ifndef G1_5_OR_LATER
    s = (struct uart_sendinfo *)h;
    s--;
    s->code = US_REPEAT;
    s->ntime = 0;
    s->core_index = base_core;
    s->ntime_limit = info->cores_per_group - 1;             // XXX Need to account for bad cores
#endif

    // Now set the pending or active bit, and remember sequence numbers
    if (pending)
        {
        group_pending[j] |= mask;
        for (i = 0, k = base_core; i < info->cores_per_group; i++, k += info->group_core_offset, core += info->group_core_offset)
            {
            core->pending = (uint8_t)(le16_to_cpu(h->hdata));
            core_pending[k>>4] |= ((uint16_t)1 << ((k) & 0xf));
            }
        }
    else
        {
        group_active[j] |= mask;
        for (i = 0, k = base_core; i < info->cores_per_group; i++, k += info->group_core_offset, core += info->group_core_offset)
            {
            core->active = (uint8_t)le16_to_cpu(h->hdata);
            core_active[k>>4] |= ((uint16_t)1 << ((k) & 0xf));
            }
        }
    ucinfo.inflight++;                                                              // Count the work

    return(true);
    }

////////////////////////////////////////////////////////////////////////////////
//
// Emulate group behavior in G-1
//
////////////////////////////////////////////////////////////////////////////////

void gwq_process_hash_repeat(struct uart_sendinfo *s)
    {
    struct hf_header *h = (struct hf_header *)(s+1);
    struct hf_hash_serial *hash = (struct hf_hash_serial *)(h+1);
    struct ucinfo_t *info = &ucinfo;

    hash->timestamp += s->ntime;                            // Already Big Endian

    h->chip_address = s->core_index / info->core_count;
    h->core_address = s->core_index - (uint16_t)((uint16_t)h->chip_address * (uint16_t)info->core_count);

    h->hdata = cpu_to_le16((le16_to_cpu(h->hdata)&info->group_mask) | ((uint16_t)s->ntime << info->group_shift));     // Store ntime offset in sequence
    h->crc8 = hf_crc8((uint8_t *)h);

    sequence_to_core_index[le16_to_cpu(h->hdata) & (sizeof(sequence_to_core_index)-1)] = h->core_address;

    s->core_index += info->group_core_offset;               // Adjust for next time
    if (s->ntime++ >= s->ntime_limit)
        s->code = 0;                                        // No more repeats after this one
    }

////////////////////////////////////////////////////////////////////////////////
//
// Process OP_HASH when no roll is to occur.
//
// When a pending slot becomes available, the OP_GWQ_STATUS message going back to the host informs
// the host that work is needed. The host queues an OP_HASH to chip address 254, and it winds up
// here. We modify the OP_HASH to tie it to an available core, and let it be sent on to the ASICs.
//
////////////////////////////////////////////////////////////////////////////////

// Optimization var's - resume search from last point each pass if no completions have
// occurred. We hit this most of the time, greatly reducing search times.
static uint8_t last_pending = 0;
static uint16_t last_j = 0;

bool gwq_process_hash_nogroup(struct hf_header *h)
    {
    uint16_t    i;
    uint16_t    j;
    uint16_t    mask;
    uint16_t    core_valid, core_busy;
    uint16_t    core_index;
    uint16_t    sequence = le16_to_cpu(h->hdata);
    uint8_t     pending;

    struct uc_core_t *core;
    struct ucinfo_t *info = &ucinfo;
    struct hf_hash_serial *hs;

#ifdef FEATURE_ACTIVITY_LED_IS_HASH
    ACTIVITY_LED_ON
    activity_led_counter += 100;
    led_mode = LED_AUTO;
#endif // FEATURE_ACTIVITY_LED_IS_HASH

    if (sequence >= MAX_SEQUENCE_SPACE)
        {
        ucinfo.bad_sequence++;
        return(false);
        }

    // Find a free (and enabled) core. If something in core_active[] is clear, the core is
    // idle so we choose that one. Otherwise, we try and find a core with an empty pending slot.

    if (work_completion_flag == true)
        {
        // One or more job completions happened since we were last in here.
        // Commence the search from the beginning again.
        last_pending = 0;
        last_j = 0;
        work_completion_flag = false;
        }

    // XXX Change the search direction to be from the top down. So that when loading cores from
    // XXX idle, such as when a work_restart occurs, we will send the OP_HASH's out to the farthest
    // XXX die first, and work back from there. That way, we're not flooding the entire chain of die
    // XXX with long packets, possibly flooding out nonces in the process.
    for (pending = last_pending; pending <= 1; pending++)
        {
        for (j = last_j, core_index = ~0; j < DIMENSION(core_active) && (j<<4) < info->total_cores; j++)
            {
            core_busy = (pending) ? core_pending[j] : core_active[j];
            core_valid = core_good[j];

            if (core_valid & (~core_busy))
                {
                // Something is available in this word. Find it.
                for (i = 0, mask = (uint16_t)1; i < 16 && ((j<<4)+i) < info->total_cores; i++, mask <<= 1)
                    {
                    if (core_valid & ~core_busy & mask)
                        {
                        core_index = (j<<4)+i;
                        goto core_found;
                        }
                    }
                }
            }
        }

core_found:
    if (core_index > info->total_cores-1)
        {
        // This is a bug, resulting in the "work" disappearing. Log it.
        ucinfo.work_overrun++;
        last_pending = 0;
        last_j = 0;
        uprintf(-1, "work_overrun %d: tick tick ... we'll blow up on the next work restart", ucinfo.work_overrun);
        return(false);
        }

#if 0
    // XXX Trouble interacting with work restarts - disabled for now
    // Resume next search at same point
    last_j = j;
    last_pending = pending;
#endif

    DEBUG_RESUMPTION_AFTER_WORK_RESTART_ADJUST_FILL_COUNTERS;
    group_sequence_head = sequence;

    // Send the work to the selected core
    h->chip_address = core_index / info->core_count;
    h->core_address = core_index - (h->chip_address * info->core_count);
    h->data_length = 15;
    h->crc8 = hf_crc8((uint8_t *)h);

    // Set dynamic nonce range to allow wide differences in die hash clock rates
    hs = (struct hf_hash_serial *)(h+1);
    hs->nonce_loops = cpu_to_le32(dynamic_nonce_range[h->chip_address]);

    // Done! The OP_HASH is ready to send (happens on return)
    // Update local status information.
    // Set the pending or active bit, and remember sequence numbers
    core = &core_sequence[0] + core_index;
    if (pending)
        {
        core_pending[j] |= mask;
        core->pending = sequence;
        }
    else
        {
        core_active[j] |= mask;
        core->active = sequence;
        }
    sequence_bitmap[sequence>>4] |= (1 << (sequence&0xf));

    sequence_to_core_index[sequence] = h->core_address;

    ucinfo.active_jobs++;
    ucinfo.inflight++;                                                          // Count the work
    return(true);
    }


////////////////////////////////////////////////////////////////////////////////
// Utility functions
////////////////////////////////////////////////////////////////////////////////

void gwq_set_group_valid(uint8_t group)
    {
    group_valid_bitmap[group>>4] |= (1 << (group & 0xf));
    }

////////////////////////////////////////////////////////////////////////////////
// Initiate a restart of everything.
////////////////////////////////////////////////////////////////////////////////
uint8_t gwq_work_restart(struct hf_header *h)
    {
    struct hf_header *h1;
    struct ucinfo_t *info = &ucinfo;
    uint32_t *b;

    // Change OP_CONFIG to do trigger on status change, with very small delay.
    // Inhibit status going back to host
    // Send OP_ABORT's to wind down
    // When everything is clear, change OP_CONFIG back to normal
    // Re-enable status going back to host
    // Host will send a fresh load of work down to us

    // OR:
    // Inhibit nonces and status
    // Move tail pointer to head, i.e. artificially finish all work
    // Arm special "abort ahead of hash" condition with group map
    // Leave existing work running. All future OP_HASH's are for the new work cycle.
    // For each new OP_HASH received (that will become the first active job in the new work cycle):
    //     - Do the normal group assignment
    //     - Prepend each OP_HASH per group with an OP_ABORT <everything>
    // Re-enable status messages once all engines have been OP_ABORT'ed
    // Old work gets aborted, new work gets shuffled in and started with very little gap

    // OR:
    // Forget all this fancy crap, and just abort EVERYTHING

#ifdef FEATURE_DISABLE_ABORTS
    return(0);
    #error In order to use FEATURE_DISABLE_ABORTS, you have to change cgminer to NOT send the next work round immediately
#else
    uprintf(UD_WORK_RESTART, "work restart: 0 ms head %4d tail %4d\n", group_sequence_head, group_sequence_tail);
    ucinfo.work_restart_start_time = msec_ticker;

    // Save potential hash_clock frequency change request
    info->hash_clock_change = le16_to_cpu(h->hdata);            // Non zero if any change request is present

    // Save potential clock change bitmap. Only allowing up to 32 die here, plenty...
    if (h->data_length > 0)
        {
        b = (uint32_t *)(h+1);
        info->hash_clock_change_bitmap = le32_to_cpu(*b);
        }
    else
        info->hash_clock_change_bitmap = ~0;                    // Enable change to every die

    //notify_host("gwq_work_restart: hdata 0x%04x bitmap 0x%08x", info->hash_clock_change, info->hash_clock_change_bitmap);
    if (group_sequence_head == group_sequence_tail)
        {
        if (info->hash_clock_change)
            {
            info->work_restart_in_progress = true;              // Fake a work restart so clock change gets made
            info->restart_phase = RESTART_PHASE_3;
            }
        return(0);                                              // No work to abort
        }

    // Change configuration to send status on core idle (few msec batch time)
    // Non-zero periodic status is to account for status frame getting lost due to transmission error,
    // after all cores have gone inactive, which would otherwise be a deadlock.
    h1 = (struct hf_header *)((uint8_t *)h + make_config_frame(h, HF_BROADCAST_ADDRESS, NULL, 5, 50, 0));

    h1->preamble = HF_PREAMBLE;
    h1->operation_code = OP_ABORT;
    h1->chip_address = HF_BROADCAST_ADDRESS;
    h1->core_address = HF_BROADCAST_ADDRESS;

#ifdef FEATURE_VIOLENT_WORK_RESTART
    // Abort all pending and active jobs, everywhere. abruptly
    h1->hdata = cpu_to_le16(0x3);
    info->restart_phase = RESTART_PHASE_3;
#endif

#ifdef FEATURE_COWARDLY_WORK_RESTART
    // For G-1 based modules that:
    //  - Have power supplies that are not agile enough to cope with the sudden load collapse
    // Abort all "active" jobs, which will leave pending jobs only
    h1->hdata = cpu_to_le16(0x1);                                    // Abort active jobs only
    info->restart_phase = RESTART_PHASE_1;
#endif

    h1->data_length = 0;
    h1->crc8 = hf_crc8((uint8_t *)h1);
    info->work_restart_in_progress = true;
    work_restart_start_time = msec_ticker;

    return (sizeof(*h1) + sizeof(struct hf_config_data) + sizeof(*h));  // CRC length gets added when it is computed
#endif // FEATURE_DISABLE_ABORTS
    }

//
// This function is in here because it is highly common with gwq_work_restart() and the associated state machines
//
uint8_t ums_clock_change(struct hf_header *h)
    {
    uint32_t *b;

#ifndef FEATURE_DISABLE_ABORTS
    ucinfo.work_restart_start_time = msec_ticker;

    // Save hash_clock frequency change request
    ucinfo.hash_clock_change = le16_to_cpu(h->hdata);            // Non zero if any change request is present

    // Save potential clock change bitmap. Only allowing up to 32 die here, plenty...
    if (h->data_length > 0)
        {
        b = (uint32_t *)(h+1);
        ucinfo.hash_clock_change_bitmap = le32_to_cpu(*b);
        }
    else
        ucinfo.hash_clock_change_bitmap = ~0;                    // Enable change to every die

    if (ucinfo.hash_clock_change)
        {
        ucinfo.work_restart_in_progress = true;                  // Fake a work restart so clock change gets made
        ucinfo.restart_phase = RESTART_PHASE_4;
        }
#endif // FEATURE_DISABLE_ABORTS
    return(0);
    }

void gwq_work_restart_complete(void)
    {
    DEBUG_RESUMPTION_AFTER_WORK_RESTART_COMPLETE_TASKS;
    }

////////////////////////////////////////////////////////////////////////////////
// Send GWQ statistics to host
////////////////////////////////////////////////////////////////////////////////

void send_gwq_stats()
    {
    struct hf_header stuff, *h = &stuff;

    memset(h, 0, sizeof(*h));
    h->preamble = HF_PREAMBLE;
    h->operation_code = OP_USB_GWQSTATS;
    h->hdata = cpu_to_le16(((((uint16_t)group_sequence_tail)<<8)|group_sequence_head));
    h->data_length = 9;
    h->crc8 = hf_crc8((uint8_t *)h);
#if 0
    CDC_Device_SendData(&VirtualSerial1_CDC_Interface, h, sizeof(*h));          // 8
    CDC_Device_SendData(&VirtualSerial1_CDC_Interface, &core_good, 2);
    CDC_Device_SendData(&VirtualSerial1_CDC_Interface, &group_valid_bitmap, 2);
    CDC_Device_SendData(&VirtualSerial1_CDC_Interface, &group_pending, 2);
    CDC_Device_SendData(&VirtualSerial1_CDC_Interface, &group_active, 2);
    CDC_Device_SendData(&VirtualSerial1_CDC_Interface, &core_pending, 2);
    CDC_Device_SendData(&VirtualSerial1_CDC_Interface, &core_active, 2);
    CDC_Device_SendData(&VirtualSerial1_CDC_Interface, &split_count[1], 8);
    CDC_Device_SendData(&VirtualSerial1_CDC_Interface, &core_sequence[0], 16);     // 36
    CDC_Device_Flush(&VirtualSerial1_CDC_Interface);
#endif
    }

#ifdef FEATURE_DEBUG_CLI
int gwq_cli_stats(int first, int parm_count, uint32_t *parms)
    {
    static int chunk;
    int done;
    uint16_t active;
    uint16_t pending;
    uint16_t b;
    int i;

    if (first)
        chunk = 0;

    done = 0;
    switch (chunk++)
        {
        case 0:
            cliWriteString("inflight ");
            cliWriteChawmpHex(ucinfo.inflight);
            cliWriteString(" activejobs ");
            cliWriteChawmpHex(ucinfo.active_jobs);
            cliWriteChar('\n');
            break;
        case 1:
            active = 0;
            pending = 0;
            for (i = 0; i < sizeof(core_active) /sizeof(core_active[0]); i++)
                {
                b = core_active[i];
                while (b)
                    {
                    if (b & 1)
                        active++;
                    b >>= 1;
                    }
                b = core_pending[i];
                while (b)
                    {
                    if (b & 1)
                        pending++;
                    b >>= 1;
                    }
                }
            cliWriteString("cores active ");
            cliWriteChawmpHex(active);
            cliWriteString(" cores pending ");
            cliWriteChawmpHex(pending);
            cliWriteChar('\n');
        case 2:
            cliWriteString("group sequence head ");
            cliWriteChawmpHex(group_sequence_head);
            cliWriteString(" tail ");
            cliWriteChawmpHex(group_sequence_tail);
            cliWriteChar('\n');
            /* fall through */
        default:
            done = 1;
            break;
        }

    return done;
    }
#endif // FEATURE_DEBUG_CLI

////////////////////////////////////////////////////////////////////////////////
//
// Initialize everything
//
////////////////////////////////////////////////////////////////////////////////

void gwq_init(void)
    {
    struct ucinfo_t *info = &ucinfo;
    uint8_t i;

    for (i = 0; i < DIMENSION(group_actual_cores); i++)
        {
        group_actual_cores[i] = info->cores_per_group;            // XXX Diagnostics could reduce this amount (bad cores)
        }
    gwq_reinit();
    }

void gwq_init_tables(void)
    {
    int i;
    uint16_t safe_sequence;

    for (i = 0; i < DIMENSION(group_valid_bitmap); i++)
        {
        group_valid_bitmap[i] = (uint16_t) 0x0;
        group_active[i] = (uint16_t) 0x0;
        group_pending[i] = (uint16_t) 0x0;
        }
    for (i = 0; i < DIMENSION(core_active); i++)
        {
        core_active[i] = (uint16_t)0;
        core_pending[i] = (uint16_t)0;
        }

    // Sequence numbers get located so that the status sequence comparison doesn't lead to false
    // positives after a WORK_RESTART
    safe_sequence = (group_sequence_head + ucinfo.num_sequence/2 + 1) % ucinfo.num_sequence;
    for (i = 0; i < DIMENSION(core_sequence); i++)
        {
        core_sequence[i].active = safe_sequence;
        core_sequence[i].pending = safe_sequence;
        }

    for (i = 0; i < DIMENSION(split_count); i++)
        split_count[i] = 0;

    work_completion_flag = true;
    last_sequence = ~0;
    last_hash_count_check = msec_ticker;
    avg_hash_rate = 0;
    memset(sequence_bitmap, 0, sizeof(sequence_bitmap));
    ucinfo.inflight = 0;
    hash_count = (uint64_t)0;
    }

void gwq_reinit(void)
    {
    uprintf(UD_SEQUENCE, "gwq_reinit: head %d tail %d\n", group_sequence_head, group_sequence_tail);

    group_sequence_head = 0;
    group_sequence_tail = 0;
    last_group_sequence_tail = 0;

    gwq_init_tables();
    }
