
// IR Stuff
#include <Arduino.h>

#if !defined(ARDUINO_ESP32C3_DEV) // This is due to a bug in RISC-V compiler, which requires unused function sections :-(.
#define DISABLE_CODE_FOR_RECEIVER // Disables static receiver code like receive timer ISR handler and static IRReceiver and irparams data. Saves 450 bytes program memory and 269 bytes RAM if receiving functions are not required.
#endif
//#define SEND_PWM_BY_TIMER         // Disable carrier PWM generation in software and use (restricted) hardware PWM.
//#define USE_NO_SEND_PWM           // Use no carrier PWM, just simulate an active low receiver signal. Overrides SEND_PWM_BY_TIMER definition

void ir_setup();

uint8_t ir_send_command = 0;
uint8_t sCommand = 0x0D; // room leds on

/*
 * This include defines the actual pin number for pins like IR_RECEIVE_PIN, IR_SEND_PIN for many different boards and architectures
 */
#include "PinDefinitionsAndMore.h"
#include <IRremote.hpp> // include the library

#include "rxrelay.h"

static uint8_t relay_pins[] = { 16, 17, 18, 19 };

void setup() {
  int i;
  // Initialize Serial Monitor
  Serial.begin(115200);
  Serial.println("starting");

  rxrelay_set_usebuttons(0);
  rxrelay_set_relaypins(relay_pins);
  rxrelay_setup();
  ir_setup();
}

void loop() {
  int i;

  rxrelay_loop();
  ir_loop();
}


void ir_setup() {
    //pinMode(LED_BUILTIN, OUTPUT);

    //Serial.begin(115200);

    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));
    Serial.print(F("Send IR signals at pin "));
    Serial.println(IR_SEND_PIN);

    /*
     * The IR library setup. That's all!
     */
    IrSender.begin(); // Start with IR_SEND_PIN -which is defined in PinDefinitionsAndMore.h- as send pin and enable feedback LED at default feedback LED pin
    disableLEDFeedback(); // Disable feedback LED at default feedback LED pin
}

/*
 * Set up the data to be sent.
 * For most protocols, the data is build up with a constant 8 (or 16 byte) address
 * and a variable 8 bit command.
 * There are exceptions like Sony and Denon, which have 5 bit address.
 */

uint8_t sRepeats = 4;

void ir_loop() {
    if(rxrelay_ir_send <= 0) {
      return;
    }

    /*
     * Print current send values
     */
    Serial.println();
    Serial.print(F("Send now: address=0x00, command=0x"));
    Serial.print(rxrelay_ir_command, HEX);
    Serial.print(F(", repeats="));
    Serial.print(sRepeats);
    Serial.println();

    Serial.println(F("Send standard NEC with 8 bit address"));
    Serial.flush();

    // Receiver output for the first loop must be: Protocol=NEC Address=0x102 Command=0x34 Raw-Data=0xCB340102 (32 bits)
    IrSender.sendNEC(0x00, rxrelay_ir_command, sRepeats);

    rxrelay_ir_send = 0;
}
