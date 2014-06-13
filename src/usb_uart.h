//
// Defines for usb_uart.c
//


// USB send buffers
#define RX_BUFFERS                  32
#define RX_BUFFER_SIZE              72

extern volatile bool main_b_cdc_enable;

//
// Send info - used to emulate the broken ntime rolling feature by sending the
// same hash job with adjusted ntime offsets to multiple targets
struct uart_sendinfo {
uint8_t code;                       // 0 = Do nothing, single send
uint8_t ntime;
uint8_t ntime_limit;
uint8_t spare1;
uint16_t core_index;
uint16_t spare2;
} __attribute__((packed,aligned(4)));

// Values for "code", 0 means no special treatment
#define US_REPEAT                   1
#define US_MULTIPLE                 2


// UART transmit buffers, which are also USB receive buffers
#define TX_BUFFERS                  64
//#define TX_BUFFER_SIZE              (sizeof(struct hf_header) + sizeof(struct hf_hash_serial) + 4 + sizeof(struct uart_sendinfo))
// AP: That's 80 bytes per frame. I've seen a potential for an overrun so I'm sizing
//     this up to 128 bytes per tx buffer for now.
#define TX_BUFFER_SIZE              128


void asic_enable_local(void);
void asic_enable_host(void);
//bool inline asic_host_enabled(void);
struct hf_header *asic_get_tx_buffer(void);
void asic_queue_transmit(void);
uint8_t asic_get_receive_count(void);
uint8_t asic_get_transmit_count(void);
struct hf_header *asic_get_receive(void);
void asic_pop_receive(void);
void display_usb_uart_status(void);
int uart_cli_stats(int, int, uint32_t *);
int usb_cli_stats(int, int, uint32_t *);
void usb_vbus_event(bool);
void usb_bus_reset(void);
void uart_set_baudrate(unsigned long);
void uart_set_default_baudrate(void);
