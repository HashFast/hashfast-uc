//
// Secret operation codes and structures, that are only used for Factory test
// and internal diagnostics.
//

#define OP_SERIAL       50                          // Serial number read/write
#define OP_LIMITS       51                          // Operational limits read/write
#define OP_HISTORY      52                          // Read operational history data
#define OP_CHARACTERIZE 53                          // Characterize one or more die
#define OP_CHAR_RESULT  54                          // Characterization result
#define OP_SETTINGS     55                          // Read or write settings
#define OP_FAN_SETTINGS 56
#define OP_POWER        57
#define OP_BAD_CORE     58                          // Set or clear bad core status

#define DIAGNOSTIC_POWER_ON     0x1
#define DIAGNOSTIC_POWER_OFF    0x2

// USER page magic number
#define U_MAGIC     0x42aa

// OP_CHARACTERIZE usb data.
struct hf_characterize {
        uint16_t dac_low, dac_high;             // DAC settings

        uint8_t  v_low, v_high;                 // Core voltage settings, in mV*10, 81 = 0.81 volts
        uint8_t  dac_incr, v_incr;              // Only one allowed - either DAC or voltage

        uint16_t f_low, f_high;                 // Frequency range, inclusive (MHz)

        uint8_t  f_incr;                        // Frequency increment
        uint8_t  die_low, die_high, die_incr;   // Which die to step through, inclusive

        uint8_t  temp_low, temp_high, temp_incr;// Die temperature setpoint and steppings
        uint8_t  thermal_override;              // Thermal overload limit if not zero

        uint16_t flags;
        uint8_t  other[2];
        } __attribute__((packed,aligned(4)));

// Flags in OP_CHARACTERIZE
#define F_TEST_WITH_ALL_DIE_CORES_HASHING       0x1
#define F_TEST_WITH_ALL_ASIC_CORES_HASHING      0x2
#define F_RETURN_CORE_MAPS                      0x4
#define F_RETURN_PLL_PARAMETERS                 0x8
#define F_DONT_DO_PLL_TWEAKUP                   0x10
#define F_HASH_STANDALONE                       0x20
#define F_FORCE_PLL_R                           0x40            // R in other[0]
#define F_FORCE_PLL_RANGE                       0x80            // Range in other[1]
#define F_PLL_TABLE_SWEEP                       0x100           // Sweep frequency across PLL table between set ranges


// OP_CHARACTERIZE results.
// As many of these as are implied by the test range will be returned as the characterization run progresses
struct hf_characterize_result {
        uint16_t dac_setting;                   // Either from dac_nnn or as a result of setting v_nnn
        uint16_t frequency;                     // MHz

        uint16_t die_temperature;               // As an ADC count, convert with usual macro
        uint8_t  core_voltage;                  // As set
        uint8_t  die_index;

        uint8_t  error_code;
        uint8_t  measured_voltage;              // As measured
        uint8_t  spare2;
        uint8_t  spare3;

        uint8_t  good_cores_low_speed;          // How many cores were good in the low frequency isolated test (this die)
        uint8_t  good_core_count;               // How many cores were good in this test case (this die)
        uint8_t  good_half_cores;               // How many cores were "half" good for this test case (this die)
        uint8_t  flags;

        uint32_t other_data;                    // Optional requested data
        } __attribute__((packed,aligned(4)));   // 20 bytes (+ core_map if present)

// Flags in OP_CHAR_RESULT
#define F_FAILURE                   0x1
#define F_LAST_THIS_DIE             0x2
#define F_LAST                      0x4
#define F_CORE_MAP_TO_FOLLOW        0x8
#define F_PLL_DATA_RETURNED         0x10


// Pre-configured operating settings, held in nvram (user page - see hf_nvram.c)
// These may be determined and set up with a manufacturing or installation "characterization" run
// Note for chained modules:
//     Master uC will need to get frequency for each die from the
//     slave modules, since the master is what does the PLL configuration, and
//     people could swap modules out to repair systems, so the settings need to
//     stay as part of the module itself.
//
typedef struct {
  uint16_t frequency;               // In Mhz
  uint16_t voltage;                 // In mV. 0 = die not enabled. (Rev-1 kludge right now - If b15 set = dac_setting)
  } die_settings_t;

typedef struct {
  uint8_t revision;                 // Revision of settings to follow
  uint8_t ref_frequency;            // Reference clock frequency
  uint16_t magic;                   // Extra validation
  die_settings_t die[4];            //
  } op_settings_t;                  // 20 bytes total

#define DIE_SETTINGS_REVISION           1
#define VALID_DIE_SETTINGS(x)           ((x).revision != 0 && (x).revision != 255 && (x).revision <= DIE_SETTINGS_REVISION && (x).magic == U_MAGIC)

#define REV1_DIE_SETTINGS_REVISION      1               // For forcing setting into field Rev-0 and Rev-1 boards

#define DEFAULT_LINEAR_DAC_SETTING      1150
#define DEFAULT_G1_HASHCLOCK            550

#define DS_DACSETTING                   0x8000
#define DS_DEFAULT_VOLTAGE              (DS_DACSETTING | DEFAULT_LINEAR_DAC_SETTING)
#define DS_DISABLED                     0x0
#define DS_NEVER_SET                    0xffff

