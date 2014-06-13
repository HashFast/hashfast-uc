//
// Emulator
//

void asic_emulator_init(void);
void asic_emulator_loop(void);
uint8_t asic_emulator_translate_packet(struct hf_header *);
void check_spurious(void);
