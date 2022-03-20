// rf69 demo tx rx.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messageing client
// with the RH_RF69 class. RH_RF69 class does not provide for addressing or
// reliability, so you should only use RH_RF69  if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example rf69_server.
// Demonstrates the use of AES encryption, setting the frequency and modem 
// configuration

#include <SPI.h>
#include <RH_RF69.h>

/************ Radio Setup ***************/

#define __AVR_ATmega328P__

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 433.0


#define RFM69_INT     3  // 
#define RFM69_CS      4  //
#define RFM69_RST     2  // "A"
#define LED           13



/* Teensy 3.x w/wing
#define RFM69_RST     9   // "A"
#define RFM69_CS      10   // "B"
#define RFM69_IRQ     4    // "C"
#define RFM69_IRQN    digitalPinToInterrupt(RFM69_IRQ )
*/
 
/* WICED Feather w/wing 
#define RFM69_RST     PA4     // "A"
#define RFM69_CS      PB4     // "B"
#define RFM69_IRQ     PA15    // "C"
#define RFM69_IRQN    RFM69_IRQ
*/

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

int16_t packetnum = 0;  // packet counter, we increment per xmission

void setup() 
{
  Serial.begin(115200);
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  pinMode(LED, OUTPUT);     
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  //Serial.println("Feather RFM69 RX Test!");

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  if (!rf69.init()) {
    //Serial.println("RFM69 radio init failed");
    while (1);
  }
  //Serial.println("RFM69 radio init OK!");
  
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    //Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
  
  pinMode(LED, OUTPUT);

  //Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
}


void loop() {

  delay(1000);

  // Total size = 20 bytes
  struct radiopacket {
    unsigned long t; // time is in milliseconds - unsigned long = 4 bytes
    float x; // float = 4 bytes
    float y; // float = 4 bytes
    float z; // float = 4 bytes
    float temp; // float = 4 bytes
  };

  if (rf69.available()) {
    // Should be a message for us now   
    uint8_t buf[20];
    uint8_t len = 20;
    if (rf69.recv(buf, &len)) {
      if (!len) return;
      //buf[len] = 0; // why is this?
      radiopacket (&rp) = (radiopacket (&))(*(buf));
      //Serial.print("Received [");
      //Serial.print(len);
      //Serial.print("]: ");
      //Serial.print("     t: "); Serial.println(rp.t);
      //Serial.print(",    x: "); Serial.println(rp.x);
      //Serial.print(",    y: "); Serial.println(rp.y);
      //Serial.print(",    z: "); Serial.println(rp.z);
      //Serial.print(",    temp: "); Serial.println(rp.temp);
      //Serial.print(" RSSI: ");
      //Serial.println(rf69.lastRssi(), DEC);
      Serial.println(rp.t);
      Serial.println(rp.x);
      Serial.println(rp.y);
      Serial.println(rp.z);
      Serial.println(rp.temp);
    } else {
      //Serial.println("Receive failed");
    }
  }
}


void Blink(byte PIN, byte DELAY_MS, byte loops) {
  for (byte i=0; i<loops; i++)  {
    digitalWrite(PIN,HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN,LOW);
    delay(DELAY_MS);
  }
}