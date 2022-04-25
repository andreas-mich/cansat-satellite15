#include <SPI.h>
#include <RH_RF69.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Arduino.h>
#include <Adafruit_BMP280.h>


//********************************************  GYROSCOPE  ********************************************//  

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.

int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
int16_t temperature; // variables for temperature data

char tmp_str[7]; // temporary variable used in convert function

char* convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

//********************************************  BUZZER  ********************************************//  

const int buzzer = 11; //buzzer to arduino pin 11

//********************************************  BMP SENSOR  ********************************************//  

Adafruit_BMP280 bmp; // I2C Interface

//********************************************  RF SETUP  ********************************************//  

#define ADAFRUIT_FEATHER_M0
#define RF69_FREQ     433.0
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
  while (!Serial) { delay(1000); } // wait until serial console is open, remove if not tethered to computer
  Serial.println("1. Starting");

//********************************************  BMP SENSOR  ********************************************//  

  // Initialise the sensor
  if(!bmp.begin())
  {
    Serial.println("3. BMP280: Not detected!");
  } else {
    Serial.println("3. BMP280: detected!");
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

  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

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



void loop() {

//********************************************  RFM69 TRANSCEIVER  ********************************************//  

  // Total size = 32 bytes
  // 1x unsigned long 4 bytes = 4 bytes
  // 7x float 4 bytes = 28 bytes
  struct radiopacket {
    unsigned long t; // time is in milliseconds

    float dp;

    float temp;
    float pres;
    float altit;

    float aX;
    float aY;
    float aZ;
    float tmp;
    float gX;
    float gY;
    float gZ;
  } rp;

  // TIME OF READING
  rp.t = millis(); // could not use event.timestamp as it is always 0 for this sensor

  //MANOMETER
  int sensorValue = analogRead(A0); // read the input on analog pin 0:
  rp.dp = sensorValue;

  // BMP280
  rp.temp = bmp.readTemperature() - 2;
  rp.pres = bmp.readPressure()/100; //displaying the Pressure in hPa, you can change the unit
  rp.altit = bmp.readAltitude(1019.66) + 4; //The "1019.66" is the pressure(hPa) at sea level in day in your region
                                            //If you don't know it, modify it until you get your current altitude


  // GYROSCOPE
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers
  
  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  accelerometer_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelerometer_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accelerometer_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  gyro_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  gyro_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  gyro_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
  
  rp.aX = accelerometer_x;
  rp.aY = accelerometer_y;
  rp.aZ = accelerometer_z;
  rp.tmp = temperature/340.00+36.53;
  rp.gX = gyro_x;
  rp.gY = gyro_y;
  rp.gZ = gyro_z;

  // BUZZER
  tone(buzzer, 1000); // Send 1KHz sound signal...
  delay(1000);        // ...for 1 sec
  noTone(buzzer);     // Stop sound...
  delay(1000);        // ...for 1sec
  

  //PRINT DATA SENT
  Serial.println("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
  
  int timeNew = rp.t / 1000;
  Serial.print("Time: "); Serial.print(timeNew); Serial.println(" seconds");

  Serial.print("Differential pressure: "); Serial.print(rp.dp); Serial.println(" pascal");
  Serial.print("Velocity from pitot: "); Serial.print(sqrt(2 * rp.dp / 1.225)); Serial.println("m/s");

  Serial.print("Temperature1: "); Serial.print(rp.temp); Serial.println(" C");
  Serial.print("Pressure: "); Serial.print(rp.pres); Serial.println(" hpascal");
  Serial.print("Altitude: "); Serial.print(rp.altit); Serial.println(" meters");

  Serial.print("Accelerometer X: "); Serial.print(rp.aX); Serial.println(" m/s^2");
  Serial.print("Accelerometer Y: "); Serial.print(rp.aY); Serial.println(" m/s^2");
  Serial.print("Accelerometer Z: "); Serial.print(rp.aZ); Serial.println(" m/s^2");
  Serial.print("Temperature 2: "); Serial.print(rp.tmp); Serial.println(" C");
  Serial.print("Gyroscope X: "); Serial.print(rp.gX); Serial.println(" m");
  Serial.print("Gyroscope Y: "); Serial.print(rp.gY); Serial.println(" m");
  Serial.print("Gyroscope Z: "); Serial.print(rp.gZ); Serial.println(" m");

  Serial.print("Sending "); Serial.print(sizeof(rp)); Serial.println(" bytes");

  rf69.send((uint8_t *)&rp, sizeof(rp));
  rf69.waitPacketSent();

  delay(989);  // Wait 1 second between transmits
}