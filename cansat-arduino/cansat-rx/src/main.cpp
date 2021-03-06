#include <SPI.h>
#include <RH_RF69.h>

/************ Radio Setup ***************/

#define __AVR_ATmega328P__

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 434.0


#define RFM69_INT     3  // 
#define RFM69_CS      4  //
#define RFM69_RST     2  // "A"
#define LED           13

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

int16_t packetnum = 0;  // packet counter, we increment per xmission

void setup() 
{
  Serial.begin(115200);
  while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

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

  delay(500);

  // Total size = 32 bytes
  struct radiopacket {
    unsigned long t; // time is in milliseconds

    float dp;

    float temp;
    float pres;
    float altit;

    float aX;
    float aY;
    float aZ;
  };

  if (rf69.available()) {
    // Should be a message for us now   
    uint8_t buf[32];
    uint8_t len = 32;
    if (rf69.recv(buf, &len)) {
      if (!len) return;
      //buf[len] = 0; // why is this?
      radiopacket (&rp) = (radiopacket (&))(*(buf));
      Serial.println(rp.t);

      Serial.println(rp.dp);

      Serial.println(rp.temp);
      Serial.println(rp.pres);
      Serial.println(rp.altit);

      Serial.println(rp.aX);
      Serial.println(rp.aY);
      Serial.println(rp.aZ);
    } 
  }
}

