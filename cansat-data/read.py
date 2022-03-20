from serial import Serial

ser = Serial('COM3',115200)
ser.close()
ser.open()

print("Connected to: " + ser.portstr)

while True:
  t = int(ser.readline().decode("utf-8"))
  print("Time: ")
  print(t)
  x = float(ser.readline().decode("utf-8"))
  print("X: ")
  print(x)
  y = float(ser.readline().decode("utf-8"))
  print("Y: ")
  print(y)
  z = float(ser.readline().decode("utf-8"))
  print("Z: ")
  print(z)
  temp = float(ser.readline().decode("utf-8"))
  print("Temp: ")
  print(temp)