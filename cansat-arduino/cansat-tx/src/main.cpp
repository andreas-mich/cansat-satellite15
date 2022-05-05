#include <SPI.h>
#include <RH_RF69.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Arduino.h>
#include <Adafruit_BMP280.h>
#include <SD.h>

//********************************************  GENERAL ********************************************//  

#define READ_DELAY   1000

//********************************************  ACCELEROMETER  ********************************************//  

// Assign a unique ID to this sensor at the same time
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

//********************************************  BUZZER  ********************************************//  

const int buzzer = 11; //buzzer to arduino pin 11

//********************************************  BMP SENSOR  ********************************************//  

Adafruit_BMP280 bmp; // I2C Interface

//********************************************  RF SETUP  ********************************************//  

#define ADAFRUIT_FEATHER_M0
#define RF69_FREQ     434.0
#define RFM69_CS      6
#define RFM69_INT     9
#define RFM69_RST     10
#define LED           13

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);
int16_t packetnum = 0;  // packet counter, we increment per xmission



void setup() 
{
  // Initialise serial port to PC for debugging purposes
  Serial.begin(115200);
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer
  Serial.println("1. Starting");

//********************************************  BMP SENSOR  ********************************************//  

  // Initialise the sensor
  if(!bmp.begin())
  {
    Serial.println("2. BMP280: Not detected!");
  } else {
    Serial.println("2. BMP280: detected!");
  }

  // Default settings from datasheet
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     // Operating Mode
                  Adafruit_BMP280::SAMPLING_X2,     // Temp. oversampling
                  Adafruit_BMP280::SAMPLING_X16,    // Pressure oversampling
                  Adafruit_BMP280::FILTER_X16,      // Filtering
                  Adafruit_BMP280::STANDBY_MS_500); // Standby time

//********************************************  BUZZER  ********************************************//  

  pinMode(buzzer, OUTPUT); // Set buzzer - pin 9 as an output
  
//********************************************  GYROSCOPE  ********************************************//  

  // Initialise the sensor
  if(!accel.begin())
  {
    Serial.println("3. ADXL345: Not detected!");
  } else {
    Serial.println("3. ADXL345: detected!");
  }
  /* Set the range to whatever is appropriate for your project */
  accel.setRange(ADXL345_RANGE_16_G);

//********************************************  RFM69 TRANSCEIVER  ********************************************//  

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
  //Serial.println("4. RFM69 radio init OK!");
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



void loop() {

//********************************************  RFM69 TRANSCEIVER  ********************************************//  

  // Total size = 32 bytes
  // 1x unsigned long 4 bytes = 4 bytes
  // 7x float 4 bytes = 28 bytes
  struct radiopacket {
    unsigned long time; // time is in milliseconds

    float dp;

    float temp;
    float pres;
    float altit;

    float aX;
    float aY;
    float aZ;
  } rp;

  // TIME OF READING
  rp.time = millis(); // could not use event.timestamp as it is always 0 for this sensor

  //MANOMETER
  int sensorValue = analogRead(A0); // read the input on analog pin 0:
  rp.dp = sensorValue;

  // BMP280
  rp.temp = bmp.readTemperature() - 2;
  rp.pres = bmp.readPressure()/100; //displaying the Pressure in hPa, you can change the unit
  rp.altit = bmp.readAltitude(1019.66) + 4; //The "1019.66" is the pressure(hPa) at sea level in day in your region
                                            //If you don't know it, modify it until you get your current altitude


  // GYROSCOPE
  sensors_event_t event; 
  accel.getEvent(&event);

  rp.aX = event.acceleration.x;
  rp.aY = event.acceleration.y;
  rp.aZ = event.acceleration.z;
  

  // BUZZER
  tone(buzzer, 1500); // Send 1KHz sound signal...
  delay(1000);        // ...for 1 sec
  noTone(buzzer);     // Stop sound...
  delay(100);        // ...for 1sec
  
  int timeSec = rp.time / 1000;
  //Serial.print("Time: "); 
  Serial.println(timeSec); //Serial.println(" seconds");

  //Serial.print("Differential pressure: "); 
  Serial.println(rp.dp); //Serial.println(" pascal");

  //Serial.print("Temperature1: "); 
  Serial.println(rp.temp); //Serial.println(" C");
  //Serial.print("Pressure: "); 
  Serial.println(rp.pres); //Serial.println(" hpascal");
  //Serial.print("Altitude: "); 
  Serial.println(rp.altit); //Serial.println(" meters");

  //Serial.print("Accelerometer X: "); 
  Serial.println(rp.aX); //Serial.println(" m/s^2");
  //Serial.print("Accelerometer Y: "); 
  Serial.println(rp.aY); //Serial.println(" m/s^2");
  //Serial.print("Accelerometer Z: "); 
  Serial.println(rp.aZ); //Serial.println(" m/s^2");

  Serial.print("Sending "); Serial.print(sizeof(rp)); Serial.println(" bytes");

  rf69.send((uint8_t *)&rp, sizeof(rp));
  Serial.println("waitPacketSent");
  rf69.waitPacketSent();
  Serial.println("Packet sent");

  delay(READ_DELAY);  // Wait between sensor readings and transmits
}