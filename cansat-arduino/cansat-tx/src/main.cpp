#include <SPI.h>
#include <RH_RF69.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Arduino.h>
#include <Adafruit_BMP280.h>

////////////////////////
// ADXL345 accelerometer
////////////////////////

// Assign a unique ID to this sensor at the same time
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

///////////////////
// Manometer sensor
///////////////////

int DPPin = A0;

/////////////////////
// BMP sensor
/////////////////////

Adafruit_BMP280 bmp; // I2C Interface

//////////////////////////
// Radio Transceiver setup
//////////////////////////

#define ADAFRUIT_FEATHER_M0
#define RF69_FREQ     433.0
#define RFM69_CS      6
#define RFM69_INT     9
#define RFM69_RST     10
#define LED           13

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

int16_t packetnum = 0;  // packet counter, we increment per xmission

////////////////////////
//      SETUP
////////////////////////

void setup() 
{
  // Initialise serial port to PC for debugging purposes
  Serial.begin(115200);
  while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer
  Serial.println("1. Starting");

  ////////////////////////
  // ADXL345 accelerometer
  ////////////////////////

  // Initialise the sensor
  if(!accel.begin())
  {
    Serial.println("2. ADXL345: Not detected!");
  } else {
    Serial.println("2. ADXL345: detected!");
  }

  // Set the range to whatever is appropriate for your project
  accel.setRange(ADXL345_RANGE_16_G);
  // accel.setRange(ADXL345_RANGE_8_G);
  // accel.setRange(ADXL345_RANGE_4_G);
  // accel.setRange(ADXL345_RANGE_2_G);

  ////////////////////
  // BMP sensor
  ////////////////////

  // Initialise the sensor
  if(!bmp.begin())
  {
    Serial.println("3. BMP280: Not detected!");
  } else {
    Serial.println("3. BMP280: detected!");
  }

  // Default settings from datasheet
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  ////////////////////
  // RFM69 transceiver
  ////////////////////

  pinMode(LED, OUTPUT);     
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  Serial.println("4. Feather RFM69 TX Test!");
  Serial.println();

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  if (!rf69.init()) {
    Serial.println("4. RFM69 radio init failed");
    while (1);
  }
  Serial.println("4. RFM69 radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("4. setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
  
  pinMode(LED, OUTPUT);

  Serial.print("4. RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
}

////////////////////////
//       LOOP
////////////////////////

void loop() {

  ////////////////////
  // RFM69 transceiver
  ////////////////////

  // Total size = 32 bytes
  // 1x unsigned long 4 bytes = 4 bytes
  // 7x float 4 bytes = 28 bytes
  struct radiopacket {
    unsigned long t; // time is in milliseconds
    float x;
    float y;
    float z;
    float dp;
    float temp;
    float pres;
    float altit;
  } rp;

  // Time of reading
  rp.t = millis(); // could not use event.timestamp as it is always 0 for this sensor

  // ADXL345 accelerometer
  sensors_event_t event; 
  accel.getEvent(&event);
  rp.x = event.acceleration.x;
  rp.y = event.acceleration.y;
  rp.z = event.acceleration.z;

  // Manometer
  float vout = analogRead(DPPin);  // read the input pin
  rp.dp = 1000.00*((vout/1023.00) -0.04)/0.09 ; // in Pa;

  // BMP280
  rp.temp = bmp.readTemperature();
  rp.pres = bmp.readPressure()/100; //displaying the Pressure in hPa, you can change the unit
  rp.altit = bmp.readAltitude(1019.66); //The "1019.66" is the pressure(hPa) at sea level in day in your region
                                        //If you don't know it, modify it until you get your current altitude

  Serial.println("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
  Serial.print("t="); Serial.print(rp.t); Serial.print("\n");
  Serial.print("x="); Serial.print(rp.x); Serial.print("\n");
  Serial.print("y="); Serial.print(rp.y); Serial.print("\n");
  Serial.print("z="); Serial.print(rp.z); Serial.print("\n");
  Serial.print("temperature= "); Serial.print(rp.temp); Serial.print("C"); Serial.print("\n");
  Serial.print("pressure= "); Serial.print(rp.pres); Serial.print("\n");
  Serial.print("altitude= "); Serial.print(rp.altit); Serial.print("\n");
  Serial.print("diff pressure="); Serial.print(rp.dp); Serial.print("\n");
  Serial.print("Sending "); Serial.print(sizeof(rp)); Serial.println(" bytes");

  rf69.send((uint8_t *)&rp, sizeof(rp));
  rf69.waitPacketSent();

  delay(1000);  // Wait 1 second between transmits
}