

void asic_init(void);
void chain_init(uint8_t);
void chain_handler(void);
void display_chain_status(void);
void chain_usb_init(struct hf_usb_init_header *);
void shutdown_request(void);
int make_config_frame(struct hf_header *, uint8_t, struct hf_config_data *, uint16_t, uint16_t, uint8_t);
uint8_t send_core_map(struct hf_header *h);
void make_abort(struct hf_header *, uint8_t, uint8_t, uint16_t);
void set_mixed_slave_baudrate(void);
#ifdef FEATURE_COWARDLY_WORK_RESTART
bool gwq_work_restart_process(struct hf_header *);
void gwq_work_restart_check(uint8_t, uint16_t *);
#endif

typedef struct {
    uint8_t F, R, Q, range;
    } pll_divisors_t;

typedef struct {
    uint32_t freq;
    uint8_t F, R, Q, range;
    } pll_entry_t;

extern const pll_entry_t pll_table[];
extern const uint8_t pll_table_entries;

extern uint32_t last_pll_parameters;
extern bool     dont_do_pll_tweakup;
extern int8_t   hcm_force_pll_r;
extern int8_t   hcm_force_pll_range;

extern uint32_t dynamic_nonce_range[MAX_DIE];
