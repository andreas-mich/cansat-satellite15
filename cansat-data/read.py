#This program serves 3 purposes:
    #1. A serial data reader (prints serial data from arduino)
    #2. Saves serial data to a csv file
    #3. Plots the Accelerometers X value and Temperature readings with Time
    #all in one live changing plot

import csv
import serial
from serial import Serial
import time
import random
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from drawnow import drawnow 

Timevar =[]
Xvar =[]
Tempvar =[]
plt.ion()
count=0

def makeFig():
    #plt.ylim(...)
    plt.title('Live Sensor Data')
    plt.grid(True)
    plt.ylabel('Acceleration-X-Value')
    plt.xlabel('Time')
    plt.plot(Timevar,Xvar,label='Acceleration(x)(m/s^2)')
    plt.legend(loc='upper left')
    #plt2=plt.twinx()
    #plt.ylim(...)
    #plt2.plot(Timevar,Tempvar,'r',label='Channel 2')
    #plt2.set_ylabel('Temperature(K)')
    #plt.ticklabel_format(useOffset=False)
    #plt2.legend(loc='upper right')
    

fieldnames = ["Time", "X", "Y", "Z", "Temperature","AtmosphericPressure","Altitude","DifferentialPressure(Pitot)"]

with open('sensordata.csv', 'w') as csv_file:
    csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
    csv_writer.writeheader()
    print('CSV created')

ser = Serial('COM3',115200)
ser.close()
ser.open()

while True:
    while (ser.inWaiting()==0):
        pass
    t = int(ser.readline().decode("utf-8"))
    print("Time:",t)
    x = float(ser.readline().decode("utf-8"))
    print("X:",x)
    y = float(ser.readline().decode("utf-8"))
    print("Y:",y)
    z = float(ser.readline().decode("utf-8"))
    print("Z:",z)
    temp = float(ser.readline().decode("utf-8"))
    print("Temp:",temp)
    press =  float(ser.readline().decode("utf-8"))
    print("AtmosphericPressure:",press)
    altit =  float(ser.readline().decode("utf-8"))
    print("Altitude:",altit)
    dp =  float(ser.readline().decode("utf-8"))
    print("Differential Pressure:",dp)

    with open('sensordata.csv', 'a') as csv_file:
        csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        info = {
                "Time": t,
                "X": x,
                "Y": y,
                "Z": z,
                "Temperature": temp,
                "AtmosphericPressure": press,
                "Altitude": altit,
                "DifferentialPressure(Pitot)": dp
            }
        csv_writer.writerow(info)
    Xvar.append(x)
    #Tempvar.append(temp)
    Timevar.append(t)
    drawnow(makeFig)
    plt.pause(.00001)
    count +=1
    if(count>10):
        Xvar.pop(0)
        Timevar.pop(0)
        Tempvar.pop(0)