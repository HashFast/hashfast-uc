//
// Header file for the global resource queue handler
//

// How group ID's work
#define GROUP_ID(a)                 (0x80 | (a))

// A G-1 module has 4 * 96 = 384 cores
// 6 modules in a chassis would be 2,308 cores, requiring a sequence space of >4,608 - say 8192 (2^13).
// Dealing with cores on an individual basis becomes very memory and compute intensive. So, we
// use multicast groups and ntime rolling to reduce the core count.
//
// Possible ntime roll amounts that go into 96:
//
//  Roll time (seconds)     # active work items
//          48                       2
//          32                       3
//          24                       4              <==
//          16                       6
//          12                       8
//           8                      12
//           6                      16
//
// Options for how work is spread:
//      - Consecutive cores on one die
//      - Staggered cores on one die
//      - Either mechanism across multiple die (Easier on power)
//
// Example:
//      - Sierra (3 ASICs = 12 die)
//      - Roll across 24 seconds, 2 core per die
//      - 48 total work items active at any time, plus 48 pending
//      - Sequence space min 96, make it 256 (8 bits)
//
// How this is coded right now (so works the same for BJ or Sierra):
//      - Roll across 24 seconds, 6 core per die
//      - 16 groups (BJ) or 48 groups (Sierra)
//      - Sequence space 256
//

#define DEFAULT_NTIME_ROLL              1
#define DEFAULT_NTIME_ROLL_PER_CORE     1
#define MAX_GROUP_SEQUENCE_SPACE        256
#define MAX_SEQUENCE_SPACE              8192

// How many multicast groups, maximum, there will be. Must be a multiple of 16.
#define MAX_DIE                         20
#define MAX_GROUPS                      96
#define MAX_CORES                       (MAX_GROUPS*MAX_DIE)        // Sierra

// Work restart phases
#define RESTART_PHASE_1                 1
#define RESTART_PHASE_2                 2
#define RESTART_PHASE_3                 3
#define RESTART_PHASE_4                 4
#define RESTART_PHASE_5                 5

#define CORE_INDEX(die,core)    (uint16_t)((uint16_t)(die)*(uint16_t)info->core_count+(uint16_t)(core))

// Check group_valid_bitmap[] according to index
#define GROUP_VALID(a)              (group_valid_bitmap[(a)>>4]&((uint16_t)1 << ((a) & 0xf)) ? 1 : 0)

#define CORE_ACTIVE(x)  (core_active [((x)>>4)] & ((uint16_t)1 << ((x) & 0xf)))
#define CORE_PENDING(x) (core_pending[((x)>>4)] & ((uint16_t)1 << ((x) & 0xf)))
#define CORE_ENABLED(x) (core_enabled[((x)>>4)] & ((uint16_t)1 << ((x) & 0xf)))

#define GROUP_TRANSFER_PENDING_TO_ACTIVE    group_pending[group_index>>4] &= ~(1 << (group_index&0xf))

typedef struct ucinfo_t {
    uint8_t device_type;
    uint8_t die_count;
    uint8_t core_count;
    uint8_t ref_frequency;

    uint16_t num_sequence;                  // Sequence number range, must be a power of 2
    uint16_t core_frequency;                // ASIC core frequency to use in MHz
    uint16_t thermal_trip_limit;            // If non zero, thermal trip limit in ASICs set to this value
    uint8_t  asic_watchdog;                 // If non zero, watchdog in ASICs set to this value
    uint8_t  status_period_10ms;            // Status period in 10 msec intervals
    uint8_t  status_batch_delay_10ms;       // Status batch delay in 10 msec intervals
    uint8_t  physical_die_count;            // How many die are physically present

    uint8_t  gwq_enabled;                   // Set if Global Work Queue protocol is to be used
    uint8_t  ntime_roll_total;              // Total ntime roll amount
    uint8_t  ntime_roll_per_core;           // How much of the total is rolled per-core
    uint8_t  cores_per_group;               // How many cores (max, if all good) per group
    uint16_t total_cores;                   // Total # of cores = die_count * core_count
    uint16_t total_good_cores;              // Total # of useable cores = total cores - bad cores
    bool     shed_supported;                // Does the miner honor the shed count
    uint16_t groups;                        // How many groups there are, total, across all die/cores
    uint16_t cores_per_group_cycle;         // How many cores in a cycle of groups, before the cycle is repeated
    uint8_t  groups_per_group_cycle;        // Pre-calculated for convenience
    uint8_t  group_core_offset;             // Linear offset between cores in the same group
#ifndef G1_5_OR_LATER
    uint16_t group_mask;                    // A mask applied to a sequence number, to limit to the group modulus
    uint8_t  group_shift;                   // Shift # to get ntime offset
    uint8_t  spare1;
#endif
    struct hf_usb_init_header *usb_init_header;

    uint16_t inflight;                      // How many OP_HASH jobs are "out there", according to sequence numbers
    uint16_t active_jobs;                   // How many active jobs are "out there", according to running calcs
                                            // Used to find software bugs...
    uint64_t hash_loops;
    uint16_t no_matching_work;
    uint8_t  work_overrun;
    uint8_t  tacho_enable;

    uint8_t  connected;                     // True if connected to a USB host
    int8_t   fault_code;
    uint8_t  fault_extra;
    uint8_t  switches;                      // DIP switches on the board, if present

    uint8_t  bad_sequence;                  // Host sent a bad sequence number (exceeded MAX_SEQUENCE)
    uint8_t  asic_baud_rate_code;           // XXX Needs to come from init, and have a default
    uint8_t  die_enable[4];

    bool     master;                        // True if this module is the "master"
    bool     no_slaves;                     // True if there are no slave modules
    bool     work_restart_requested;        // True if UsbIncoming hash received a work restart
    bool     work_restart_in_progress;      // True if a work restart is in progress
    uint16_t work_restart_start_time;       // Timer for entire work restart process
    bool     resend_op_config;              // Resend OP_CONFIG data
#ifdef FEATURE_COWARDLY_WORK_RESTART
    uint8_t  restart_phase;                 // State variable for restart steps
#endif
    int      num_slaves;                    // Number of slave modules present
    int      chain_configuration;           // See CC_ definitions in module_handler.h
    bool     board_initialized;             // Goes true once all board power-up diags are done
    bool     powered_up;                    // Goes true once we're powered up
    bool     addressing_complete;           // Master has finished giving all slaves addresses
    uint32_t hash_clock_change_bitmap;      // Bitmap of die to change (allow adjusting some but not others)
    bool     mixed_reference_clocks;        // One or more slaves have different reference clocks than the master
    uint32_t dynamic_baud_rate;             // If 0, default static baud rates used in a uniform system

    bool     factory_mode;                  // Factory test mode. No serial number assigned yet
    bool     fake_standalone_test;          // A fake standalone test is running, in factory mode
    bool     inhibit_watchdogs;             // Used only during characterization (diagnostic) runs
    bool     slave_autonomous;              // Used only during slaves setting baud rate in mixed configurations

    uint16_t hash_clock_change;             // Work restart includes a clock change request if this is non zero
    uint16_t shed_amount;                   // Work shed (only happens due to blabber right now)
    uint16_t die_temp_low_control_limit_x10;
    uint16_t die_temp_high_control_limit_x10;
    uint16_t target_die_temperature_x10;
    uint16_t highest_die_temperature_x10;
    uint16_t target_board_temperature_x10;
    uint16_t highest_board_temperature_x10;
    } ucinfo_t __attribute__((aligned(4)));

extern struct ucinfo_t ucinfo;
extern uint16_t group_sequence_head;
extern uint16_t group_sequence_tail;

void gwq_init(void);
void gwq_reinit(void);
void gwq_init_tables(void);
uint8_t gwq_process_input(struct hf_header *);
void gwq_set_group_valid(uint8_t); 
uint16_t gwq_get_sequence(uint16_t);
bool gwq_process_hash_group(struct hf_header *);
bool gwq_process_hash_nogroup(struct hf_header *);
uint8_t gwq_work_restart(struct hf_header *);
void gwq_work_restart_complete(void);
uint8_t ums_clock_change(struct hf_header *);
void send_gwq_stats(void);
int gwq_cli_stats(int, int, uint32_t *);
void gwq_process_hash_repeat(struct uart_sendinfo *);
void gwq_update_board_temperature(uint8_t, uint16_t);
uint16_t gwq_get_board_temperature(uint8_t);
void gwq_update_tach(uint8_t, uint16_t);
uint16_t gwq_get_tach(uint8_t);
uint16_t gwq_get_die_temperature(uint8_t);
void gwq_update_phase_currents(uint16_t *);
void gwq_update_regulator_voltage(uint16_t *);


//
// To find the multiple root causes of Fogbugz 8364, I hacked various things into the
// code to monitor the events after a work restart. The debug code was useful, so I've
// macro-ized it here so as not to clutter up the source. It is normally compiled out
//
#ifdef DEBUG_RESUMPTION_AFTER_WORK_RESTART

#define DEBUG_RESUMPTION_AFTER_WORK_RESTART_STATUS_DUMP \
                do  { \
                    static uint16_t first_last[12], first_sequence, last_sequence; \
                    static bool got_first = false; \
                    static bool got_last = false; \
                    if (h->chip_address == 0) \
                        { \
                        got_first = true; \
                        first_sequence = status_sequence; \
                        for (i = 0; i < 96/16; i++) \
                            first_last[i] = (i == 0) ? *p : le16_to_cpu((*(p+i))); \ \
                        } \
                    else if (h->chip_address && h->chip_address == ucinfo.die_count - 1) \
                        { \
                        got_last = true; \
                        last_sequence = status_sequence; \
                        for (i = 0; i < 96/16; i++) \
                            first_last[i+6] = (i == 0) ? *p : le16_to_cpu((*(p+i))); \
                        } \
                    if (got_first && (got_last || ucinfo.die_count == 1)) \
                        { \
                        notify_host("OP_STS [F %4d L %4d H %4d T %4d]: %04x %04x %04x %04x %04x %04x / %04x %04x %04x %04x %04x %04x\n", \
                            first_sequence, last_sequence, group_sequence_head, group_sequence_tail, \
                            first_last[0], first_last[1], first_last[2], first_last[3], first_last[4], first_last[5], \
                            first_last[6], first_last[7], first_last[8], first_last[9], first_last[10], first_last[11]); \
                        got_first = got_last = false; \
                        } \
                    if (resumption_blackout == true) \
                        { \
                        if ((uint16_t)elapsed_since(resumption_start) > (uint16_t)10000) \
                            resumption_blackout = false; \
                        notify_host("OP_STS RES %d die %d seq %4d sts [%04x %04x %04x %04x %04x %04x]", \
                            elapsed_since(resumption_start), h->chip_address, status_sequence, *p, \
                            (uint16_t)le16_to_cpu((*(p+1))), (uint16_t)le16_to_cpu((*(p+2))), \
                            (uint16_t)le16_to_cpu((*(p+3))), (uint16_t)le16_to_cpu((*(p+4))), (uint16_t)le16_to_cpu((*(p+5)))); \
                        } \
                    } while (0)

// To be useful, you need to use AtmelStudio for the following definition because the
// following trap simply puts the uC into an infinite loop, you're meant to set breakpoints at
// those places in order to halt the system and determine the cause(s). Allowing the system
// to run on is not an option, the evidence will be destroyed almost instantly.
#define DEBUG_RESUMPTION_AFTER_WORK_RESTART_FREEZE_IF_FILL_STILL_ACTIVE \
                if (fill_pending_after_work_restart || fill_active_after_work_restart) \
                    while (1) 

#define DEBUG_RESUMPTION_AFTER_WORK_RESTART_DEFINES \
    static int fill_active_after_work_restart; \
    static int fill_pending_after_work_restart; \
    static bool resumption_blackout; \
    static uint16_t resumption_start

#define DEBUG_RESUMPTION_AFTER_WORK_RESTART_ADJUST_FILL_COUNTERS \
    if (pending) \
        { \
        if (fill_pending_after_work_restart) \
            fill_pending_after_work_restart--; \
        } \
    else 
        if (fill_active_after_work_restart) \
            fill_active_after_work_restart--

#define DEBUG_RESUMPTION_AFTER_WORK_RESTART_COMPLETE_TASKS \
    fill_active_after_work_restart = ucinfo.total_good_cores; \
    fill_pending_after_work_restart = ucinfo.total_good_cores; \
    resumption_blackout = true; \
    resumption_start = msec_ticker

#else

#define DEBUG_RESUMPTION_AFTER_WORK_RESTART_STATUS_DUMP
#define DEBUG_RESUMPTION_AFTER_WORK_RESTART_FREEZE_IF_FILL_STILL_ACTIVE
#define DEBUG_RESUMPTION_AFTER_WORK_RESTART_DEFINES
#define DEBUG_RESUMPTION_AFTER_WORK_RESTART_ADJUST_FILL_COUNTERS
#define DEBUG_RESUMPTION_AFTER_WORK_RESTART_COMPLETE_TASKS

#endif

