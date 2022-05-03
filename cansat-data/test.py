import csv
import time
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from drawnow import *
import openpyxl
from math import sqrt
import serial
from serial import Serial

RH = float(0.7)

count = 0

print('Starting Variables Set Up')

plt.ion()

    

fieldnames = [
    "Time (s)",
    "GYRO Accel. X ",
    "GYRO Accel. Y ",
    "GYRO Accel. Z ",
    "GYRO Rot. X",
    "GYRO Rot. Y",
    "GYRO Rot. Z",
    "ACCEL. X",
    "ACCEL. X",
    "ACCEL. X",
    ]

with open('sensordata.csv', 'w') as csv_file:
    csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
    csv_writer.writeheader()
    print('CSV created')

ser = Serial('COM4',115200)
ser.close()
ser.open()
print("Connected to: " + ser.portstr)


while True:
    while (ser.inWaiting()==0):
        pass
    count+=1
    t = int(ser.readline().decode("utf-8"))
    gyroaccx = float(ser.readline().decode("utf-8"))
    gyroaccy = float(ser.readline().decode("utf-8"))
    gyroaccz = float(ser.readline().decode("utf-8"))
    gyrogyrox = float(ser.readline().decode("utf-8"))
    gyrogyroy = float(ser.readline().decode("utf-8"))
    gyrogyroz = float(ser.readline().decode("utf-8"))

    accx = float(ser.readline().decode("utf-8"))
    accy = float(ser.readline().decode("utf-8"))
    accz = float(ser.readline().decode("utf-8"))

    print(t)
    print(gyroaccx)
    print(gyroaccy)
    print(gyroaccz)
    print(gyroaccx)
    print(gyrogyrox)
    print(gyrogyroy)
    print(gyrogyroz)
    print(accx)
    print(accy)
    print(accz)
    


    with open('sensordata.csv', 'a') as csv_file:
        csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        info = {
                 "Time (s)": t,
                "GYRO Accel. X ": gyroaccx,
                "GYRO Accel. Y ": gyroaccy,
                "GYRO Accel. Z ": gyroaccz,
                "GYRO Rot. X": gyrogyrox,
                "GYRO Rot. Y": gyrogyroy,
                "GYRO Rot. Z": gyrogyroz,
                "ACCEL. X": accx,
                "ACCEL. X": accy,
                "ACCEL. X": accz,
            }
        csv_writer.writerow(info)
            
###########################################################################
            
    finaldata= pd.read_csv('sensordata.csv')
    finaldata.to_excel('finaldata.xlsx',sheet_name='SensorData',index=False)
    print('data saved to excel')
