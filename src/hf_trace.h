//
// Header file for tracing
//

#ifdef INCLUDE_TRACING

typedef struct tracerecord_t {
    uint16_t time;                  // In usec relative
    uint8_t event;
    uint8_t data;
    } TRACERECORD_T;

#define TRACE_RECORDS 256

// Definitions of trace events

enum trace_events_t {
    TR_NULL = 0,
    TR_USB_INCOMING_PREAMBLE,
    TR_USB_INCOMING_HEADER_CRC,
    TR_USB_INCOMING_DISPATCH_SHORT,
    TR_USB_INCOMING_DISPATCH_LONG,
    TR_UART_OUTGOING_BEGIN,
    TR_UART_TX_DMA_START,
    TR_UART_TX_DMA_INTERRUPT,
    TR_UART_RX_INTERRUPT_PREAMBLE,
    TR_UART_RX_DMA_START,
    TR_UART_RX_DMA_HEADER_DONE,
    TR_UART_RX_DMA_DATA_DONE,
    TR_UART_INCOMING_START,
    TR_UART_INCOMING_USB_SEND_DONE,
    TR_USB_INC_TX_BUFFERS,
    TR_UART_INCOMING_SEQUENCE,
    TR_UART_INCOMING_SIZE,
    TR_USB_INCOMING_CRC
    };

void hf_trace(uint8_t, uint8_t);
void hf_trace_init(void);

extern struct tracerecord_t trace_records[TRACE_RECORDS];
extern struct tracerecord_t *t_head;

#define TRACE_HEAD_OFFSET ((uint16_t)(t_head - &trace_records[0])/sizeof(struct tracerecord_t))

#else

#define hf_trace(a,b)
#define hf_trace_init()

#endif

#define SPIN_0          AVR32_PIN_PB10
#define SPIN_1          AVR32_PIN_PB11
#define SPIN_2          AVR32_PIN_PA29
#define SPIN_3          AVR32_PIN_PA30

void a3bu_trace_init(void);

#if 0
#define SCOPE_0_ON   gpio_set_pin_high(SPIN_0);
#define SCOPE_0_OFF  gpio_set_pin_low(SPIN_0);
#define SCOPE_1_ON   gpio_set_pin_high(SPIN_1);
#define SCOPE_1_OFF  gpio_set_pin_low(SPIN_1);
#define SCOPE_2_ON   gpio_set_pin_high(SPIN_2);
#define SCOPE_2_OFF  gpio_set_pin_low(SPIN_2);
#define SCOPE_3_ON   gpio_set_pin_high(SPIN_3);
#define SCOPE_3_OFF  gpio_set_pin_low(SPIN_3);

#define CONFIGURE_SCOPE_PINS \
    gpio_enable_gpio_pin(SPIN_0); \
    gpio_enable_gpio_pin(SPIN_1); \
    gpio_enable_gpio_pin(SPIN_2); \
    gpio_enable_gpio_pin(SPIN_3); \
    gpio_configure_pin(SPIN_0, GPIO_DIR_OUTPUT); \
    gpio_configure_pin(SPIN_1, GPIO_DIR_OUTPUT); \
    gpio_configure_pin(SPIN_2, GPIO_DIR_OUTPUT); \
    gpio_configure_pin(SPIN_3, GPIO_DIR_OUTPUT)
#else
#define SCOPE_0_ON   gpio_set_pin_high(LED_POWER)
#define SCOPE_0_OFF  gpio_set_pin_low(LED_POWER)
#define SCOPE_1_ON   gpio_set_pin_high(LED_ACTIVITY)
#define SCOPE_1_OFF  gpio_set_pin_low(LED_ACTIVITY)
#define SCOPE_2_ON
#define SCOPE_2_OFF
#define SCOPE_3_ON
#define SCOPE_3_OFF

#define CONFIGURE_SCOPE_PINS



#endif
