

extern const uint8_t const crc8_table[256];
extern const uint32_t const crc32_table[256];
extern uint8_t hf_crc8(uint8_t *);
extern uint32_t hf_crc32(uint8_t *, int, int);
