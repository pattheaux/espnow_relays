
void rxrelay_loop();
void rxrelay_setRelay(uint8_t relay, uint8_t value);
void rxrelay_setup();

extern uint8_t rxrelay_ir_send;
extern uint8_t rxrelay_ir_command;
extern void rxrelay_set_relaypins(uint8_t *pins);
extern void rxrelay_set_buttons(uint8_t *buttonpins);
extern void rxrelay_set_usebuttons(uint8_t use);

