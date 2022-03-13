To remember:

- keep all source code under src/ folder (e.g. copy all code into main.cpp)
- keep all libraries under lib/ folder (e.g. RadioHead)
  example to clone Adafruit RadioHead repo: git clone https://github.com/adafruit/RadioHead.git
- should a constant be detected based on the board? This did not work, so we had to define it manually:
  #define ADAFRUIT_FEATHER_M0
- in platformio.io define the serial port and its speed
  for the tx:
  upload_port = COM4
  monitor_speed = 115200
  for the rx:
  upload_port = COM3
  monitor_speed = 115200