/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp8266-nodemcu-arduino-ide/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#ifdef ESP32
#include <esp_now.h>
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#include <espnow.h>

static const uint8_t D0   = 16;
static const uint8_t D1   = 5;
static const uint8_t D2   = 4;
static const uint8_t D3   = 0;
static const uint8_t D4   = 2;
static const uint8_t D5   = 14;
static const uint8_t D6   = 12;
static const uint8_t D7   = 13;
static const uint8_t D8   = 15;
static const uint8_t D9   = 3;
static const uint8_t D10  = 1;

static const uint8_t BUTTON  = D1;
#endif

#ifdef ESP32
static const uint8_t BUTTONMAIN = 16;
static const uint8_t BUTTON1 = 17;
static const uint8_t BUTTON2 = 18;
static const uint8_t BUTTON3 = 19;
static const uint8_t BUTTON4 = 14;
static const uint8_t BUTTONLED = 4;

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

#else
static const uint8_t BUTTONMAIN = D6;
static const uint8_t BUTTON1 = D1;
static const uint8_t BUTTON2 = D2;
static const uint8_t BUTTON3 = D3;
static const uint8_t BUTTON4 = D4;
static const uint8_t BUTTONLED = D6;
#endif

uint8_t mac0[] = {0xc4, 0xd8, 0xd5, 0x13, 0x3d, 0xfc}; // orig box 1 4 outlets
uint8_t mac1[] = {0x4c, 0xeb, 0xd6, 0x1f, 0x50, 0x5c}; // 2 outlets, 1 contact (vpin), 1 IR (room leds)
uint8_t mac2[] = {0xbc, 0xdd, 0xc2, 0x79, 0x92, 0x2d}; // orig box 2 3 outlets, 1 contact (crystal castles)
uint8_t mac3[] = {0x78, 0xee, 0x4c, 0x01, 0x5a, 0xc8}; // ESP32 box. qbert, starwars, starwars leds
uint8_t mac4[] = {0x78, 0xee, 0x4c, 0x01, 0x50, 0xec}; // Blacklight

uint8_t macx[] = {0xc4, 0xd8, 0xd5, 0x0e, 0xbf, 0xc1}; // dead

//mac2 bc:dd:c2:79:92:2d

#define NMACS 4
uint8_t *macs[] = {
  mac0,
  mac1,
  mac2,
  mac3,
  mac4,
  NULL
};

#define RELAY 0
#define IRLED 1

typedef struct step {
  uint8_t module;
  uint8_t relay;
  uint8_t value;
  long    delay;
  uint8_t mode; // 0=relay, 1=IR
} step_t;

#define RESET_STEPS   { 2, 3, 0, 10, RELAY }, { 1, 2, 0, 10, RELAY },  { 3, 3, 0, 10, RELAY }

step_t steps[] = {
  //{ 1, 3, 0x0D, 0 }, { 1, 3, 0x4D, 0 }, // Room LEDs on, colors //{ 1, 3, 0x00, 0 }, // strobe

  RESET_STEPS,

  { 4, 0, 1, 10, RELAY }, // blacklight on

  { 2, 0, 1, 10, RELAY }, // EARLY 1up A
  { 0, 0, 1, 10, RELAY }, // Pacman 1

  { 1, 2, 250, 10, RELAY }, // Vpin early

  { 0, 1, 1, 0,  RELAY }, // Ghost1
  { 2, 1, 1, 10, RELAY }, // EARLY 1up B
  { 0, 2, 1, 0,  RELAY }, // disco1

  { 0, 3, 1, 0,  RELAY }, // Wild Life

  { 1, 0, 1, 0, RELAY }, // BOP  //{ 1, 2, 250, 0, RELAY }, // BOP, Vpin

  { 3, 0, 1, 0, RELAY }, // Disco2
  { 3, 1, 1, 0, RELAY }, // Ghost2

  { 3, 2, 1, 0, RELAY }, // qbert
  { 2, 2, 1, 1, RELAY }, // Crystal Castles

  { 3, 3, 0x40, 0, IRLED}, // star wars leds toggle

  { 1, 3, 0x0D, 0, IRLED }, { 1, 3, 0x00, 0, IRLED },
  { 1, 3, 0x04, 3000, IRLED }, // Blue

  RESET_STEPS,

  { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF } // Done
};

step_t off_steps[] = {
  //{ 1, 3, 0x4D, 0}, { 1, 3, 0x1F, 0}, // Room LEDs colors, off
  //{ 0xFF, 0xFF, 0xFF, 0xFF }, // Done

  RESET_STEPS,

  { 4, 0, 0, 10, RELAY }, // blacklight off

  { 1, 3, 0x4D, 10, IRLED}, // Room LEDs colors
  { 2, 3, 250, 10, RELAY }, // Crystal Castles button

  { 0, 0, 0, 0, RELAY }, { 0, 1, 0, 0, RELAY }, { 0, 2, 0, 0, RELAY }, // PacMan and disco1
  { 0, 3, 0, 0, RELAY }, // Wild Life

  { 1, 0, 0, 0, RELAY }, { 1, 2, 250, 0, RELAY }, // BOP, Vpin

  { 2, 0, 0, 0, RELAY }, { 2, 1, 0, 0, RELAY }, // 1ups

  { 1, 3, 0x1F, 1, IRLED}, // Room LEDS off

  { 3, 0, 0, 0, RELAY }, // Disco2
  { 3, 1, 0, 0, RELAY }, // Ghost2
  { 3, 3, 250, 1, RELAY }, // qbert button

  //{ 1, 3, 0x1F, 0, IRLED}, // Room LEDS off

  { 3, 3, 0x40, 0, IRLED}, // star wars leds toggle

  { 2, 2, 0, 2000, RELAY }, // Crystal Castles off
  { 3, 2, 0, 2000, RELAY }, // qbert off

  RESET_STEPS,

  { 0xFF, 0xFF, 0xFF, 0xFF } // Done
};

step_t reset_steps[] = {
  { 2, 3, 0, 10, RELAY }, // Crystal Castles button
  { 1, 2, 0, 10, RELAY },  // Vpin button
  { 3, 3, 0, 10, RELAY },  // qbert button

  { 0xFF, 0xFF, 0xFF, 0xFF } // Done
};

/*
step_t b1_steps[] = {
  { 3, 3, 0x40, 0, IRLED }, // star wars leds toggle

  { 3, 0, 1, 0, RELAY }, // Disco2
  { 3, 1, 1, 0, RELAY }, // Ghost2
  { 3, 2, 1, 0, RELAY }, // qbert

  { 0xFF, 0xFF, 0xFF, 0xFF } // Done
};
step_t b1_steps_off[] = {
  { 3, 0, 0, 0, RELAY }, // Disco2
  { 3, 1, 0, 0, RELAY }, // Ghost2
  { 3, 3, 250, 1, RELAY }, // qbert button
  { 3, 2, 0, 5000, RELAY }, // qbert off
  { 3, 3, 0x40, 0, IRLED }, // star wars leds toggle

  { 0xFF, 0xFF, 0xFF, 0xFF } // Done
};
*/


step_t b1_steps[] = {
  { 4, 0, 1, 10 }, // blacklight on
  { 0xFF, 0xFF, 0xFF, 0xFF } // Done
};
step_t b1_steps_off[] = {
  { 4, 0, 0, 10 }, // blacklight on
  { 0xFF, 0xFF, 0xFF, 0xFF } // Done
};


// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  int module;
  int relay;
  int value;
  int data[32];
} struct_message;

// Create a struct_message called myData
struct_message myData;

typedef struct button {
  uint8_t button;
  uint8_t bstate;
  unsigned long lastTime;
  uint8_t value;
  uint8_t on;
} button_t;

#define NBUTTONS 4
int relay_value = 0;
button_t buttons[NBUTTONS] = { { BUTTON1, 0, 0, 0, 0 }, { BUTTON2, 0, 0, 0, 0 }, { BUTTON3, 0, 0, 0, 0 }, { BUTTONMAIN, 0, 0, 0, 1 } };

uint8_t startup = 1;

typedef struct mmode_t {
  uint8_t mode;
  step_t *curstep;
  long curstep_next;

  uint8_t submode;
  uint8_t flash;
  unsigned long last; 
  unsigned long next;
  unsigned long nextFlash;
} mmode_t;

unsigned long durations[] = { 1000, 1000, 1000, 1000 };
mmode_t mmode = { 0, NULL, 0, 0, 0, 0, 0, 0 };

//unsigned long lastTime = 0;
unsigned long timerDelay = 1000;  // send readings timer

void send(int module, int relay, int value, int mode);

// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0){
    Serial.println("Delivery success");
  } else {
    Serial.print("Delivery FAIL ");
    for(int i=0;i<6;i++) {
      Serial.print(mac_addr[i],HEX);
      Serial.print(":");
    }
    Serial.println();
  }
}

// Callback when data is sent
void OnDataSent32(const uint8_t *mac_addr, esp_now_send_status_t sendStatus) {
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0){
    Serial.println("Delivery success");
  } else {
    Serial.print("Delivery FAIL ");
    for(int i=0;i<6;i++) {
      Serial.print(mac_addr[i],HEX);
      Serial.print(":");
    }
    Serial.println();
  }
}
 
void setup() {
  int i;
  // Init Serial Monitor
  Serial.begin(115200);
 
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BUTTONLED, OUTPUT);

  for(i=0;i<NBUTTONS;i++) {
    if(buttons[i].on) {
#ifdef ESP32
      pinMode(buttons[i].button, INPUT_PULLDOWN);
#else
      pinMode(buttons[i].button, INPUT);
#endif
    } else {
      pinMode(buttons[i].button, INPUT_PULLUP);
    }
    buttons[i].bstate = 0;
    buttons[i].lastTime = 0;
    buttons[i].value = 0;
  }

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  memset((uint8_t *) &myData, 0, sizeof(myData));

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
#ifdef ESP32
  Serial.println("This is an ESP32");
  esp_now_register_send_cb(OnDataSent32);

  for(i=0;macs[i]!=NULL;i++) {
    esp_now_peer_info_t peerInfo;
    // Register peer
    memcpy(peerInfo.peer_addr, macs[i], 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
  
    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add peer");
      return;
    }
  }
#else
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_add_peer(macs[0], ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
#endif

}
 
void flash(int on, int setNext) {
  digitalWrite(BUTTONLED, on?LOW:HIGH);
  if(!setNext) {
    mmode.nextFlash = 0;
    mmode.flash = 1;
    if(!on) analogWrite(BUTTONLED, 220); // dim
    return;
  }

  mmode.flash = !mmode.flash;
  mmode.nextFlash = (mmode.flash?100:200) + millis();
}

// OLD
void mode1Next() {
  Serial.print("MODE1 step ");
  Serial.println(mmode.submode);
  //digitalWrite(D5, (mmode.submode%2==0)?LOW:HIGH);
  send((int)(mmode.submode/4),mmode.submode%4,1,0);
  mmode.next = millis() + 800;
  mmode.submode++;
  if(mmode.submode >= 8) {
    Serial.println("MODE1 finished");
    mmode.mode = 0;
    flash(1, 0);
    //digitalWrite(D5, HIGH);
  }
}
// OLD
void mode2Next() {
  Serial.print("MODE2 step ");
  Serial.println(mmode.submode);
  send((int)(mmode.submode/4),mmode.submode%4,0,0);
  mmode.next = millis() + 500;
  mmode.submode++;
  if(mmode.submode >= 8) {
    Serial.println("MODE2 finished");
    mmode.mode = 0;
    flash(0, 0);
  }
}

void modeNext() {
    Serial.print("NEXT mode=");
    Serial.println(mmode.mode);
    switch(mmode.mode) {
      case 1: mode1Next(); return;
      case 2: mode2Next(); return;
    }
}

void queueStep(step_t *step) {
  if(step->module == 0xFF) {
    Serial.println("STEP finished");
    mmode.curstep = NULL;
    mmode.curstep_next = 0L;

    Serial.println("MODE finished");
    flash(mmode.mode==1?1:0, 0);
    mmode.mode = 0;

    return;
  }

  mmode.curstep = step;
  long delay = step->delay;
  if(delay <= 0) delay = 1000;
  Serial.print("STEP next in");
  Serial.println(delay);
  mmode.curstep_next = millis() + delay;
}

void blink() {
  Serial.println("blink");
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(1000);                      // wait for a second
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  delay(1000);                      // wait for a second
}

void loop() {
  int i;

  //blink();
  //return;

  if(startup) {
    startup = 0;
    digitalWrite(BUTTONLED, HIGH);
    Serial.println("sender start");
#ifdef ESP32
    analogWriteResolution(BUTTONLED,8);
#else
    analogWriteRange(255);
#endif
    flash(0,0);
  }

  if(mmode.nextFlash > 0 && mmode.nextFlash <= millis()) {
    flash(mmode.flash, 1);
  }

  if((mmode.curstep != NULL) && (mmode.curstep_next <= millis())) {
    Serial.println("STEP send");
    send(mmode.curstep->module,mmode.curstep->relay,mmode.curstep->value,mmode.curstep->mode);
    queueStep(mmode.curstep+1);
    //flash(1,0); FIXME!!
  }

  if(mmode.curstep != NULL) {
    // Ignore buttons if we're stepping
    return;
  }

  //if(mmode.mode > 0 && mmode.next <= millis()) {
  //  modeNext();
  //}

  for(i=0;i<NBUTTONS;i++) {
    loop_button(buttons+i,i);
  }
}

void loop_button(button_t *btn, int i) {

  uint8_t mode = btn->bstate;
  uint8_t bstate = digitalRead(btn->button);
  if(btn->on) {
    bstate = !bstate;
  }

  if ((mode == 1) && ((millis() - btn->lastTime) > timerDelay) && (bstate)) {
    Serial.print("disarm ");
    Serial.println(i);
    btn->bstate = 0;
    digitalWrite(LED_BUILTIN, HIGH);
    return;
  }

  if((mode == 0) && (!bstate)) {
    btn->bstate = 1;
    btn->lastTime = millis();
    digitalWrite(LED_BUILTIN, LOW);
    btn->value = !btn->value;
    if(btn->button == BUTTONMAIN) {
      memset(&mmode,0,sizeof(mmode_t));
      mmode.mode = btn->value?1:2;
      mmode.next = millis();

      Serial.println("BUTTONMAIN");
      queueStep(mmode.mode==1?steps:off_steps);

      flash(1, 1);
    } else if(btn->button == BUTTON1) {
      memset(&mmode,0,sizeof(mmode_t));
      mmode.mode = btn->value?1:2;
      mmode.next = millis();

      Serial.println("BUTTON1");
      queueStep(mmode.mode==1?b1_steps:b1_steps_off);

      flash(1, 1);
    }

    Serial.print("click ");
    Serial.print(i);
    Serial.print("=");
    Serial.println(btn->value);
  }
}

void send(int module, int relay, int value, int mode) {
    // Set values to send
    myData.module = module;
    myData.relay = relay;
    myData.value = value;
    myData.data[0] = mode;

    // Send message via ESP-NOW
    Serial.print("send M:");
    Serial.print(myData.module);
    Serial.print(" R:");
    Serial.print(myData.relay);
    Serial.print("=");
    Serial.print(myData.value);
    Serial.println();

    esp_now_send(macs[myData.module], (uint8_t *) &myData, sizeof(myData));
    return;
}