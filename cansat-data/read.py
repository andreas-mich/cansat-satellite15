import csv #to save data to csv

#to save data to excel
import pandas as pd
import openpyxl

#for plotting
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from drawnow import *

#for arduino connection
import serial
from serial import Serial

import math

Timevar=[]
PitotVelocityvar=[]
Temperaturevar=[]
Pressurevar=[]

RH = 0.7
count=0

print('Starting Variables Set Up')

plt.ion()

def makeFig():

    plt.subplot(2,1,1)
    plt.plot(Timevar,PitotVelocityvar,"r",label='Velocity(Pitot)')
    plt.legend(loc='upper left')
    #plt.ylim(....)
    plt.title('Live Sensor Data')
    plt.grid(True)
    plt.ylabel('Velocity(Pitot)')
    plt.ticklabel_format(useOffset=False)


    plt.subplot(2,1,2)
    plt.plot(Timevar,Temperaturevar,'r',label='Temperature Plot')
    plt.legend(loc='upper left')
    #plt.ylim(...)
    plt.grid(True)
    plt.ylabel('Temperature')
    plt.xlabel('Time')
    plt.ticklabel_format(useOffset=False)
    plt2=plt.twinx()
    #plt.ylim(...)
    plt2.plot(Timevar,Pressurevar,label='Atmospheric Pressure')
    plt2.set_ylabel('A. Pressure')
    plt2.legend(loc='upper right')

print('Plot Settings Setup')

fieldnames = [
    "Time (s)",
    "Acceleration (m/s^2)",
    "Velocity(Pitot)(m/s)",
    "DifferentialPressure(Pitot)(Pa)",
    "Temperature (K)",
    "AtmosphericPressure (hPa)",
    "Altitude(m)",
    ]

with open('sensordata.csv', 'w') as csv_file:
    csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
    csv_writer.writeheader()
    print('CSV created')

ser = Serial('COM3',115200)
ser.close()
ser.open()
print("Connected to: " + ser.portstr)

while True:
    while (ser.inWaiting()==0):
        pass
    count+=1
    t = int(ser.readline().decode("utf-8"))
    print("Time:",t)
    accel = float(ser.readline().decode("utf-8"))
    print("Acceleration:",accel)
    x = float(ser.readline().decode("utf-8"))
    print("X:",x)
    y = float(ser.readline().decode("utf-8"))
    print("Y:",y)
    z = float(ser.readline().decode("utf-8"))
    print("Z:",z)
    dp =  float(ser.readline().decode("utf-8"))
    print("Differential Pressure:",dp)
    temp = float(ser.readline().decode("utf-8"))
    print("Temp:",temp)
    press =  float(ser.readline().decode("utf-8"))
    print("AtmosphericPressure:",press)
    altit =  float(ser.readline().decode("utf-8"))
    print("Altitude:",altit)

    press = 1020
    temp = 20.30

###########################################################################

    ρ = 1.2
    
    #Air density

    PitotVelocitySquared = pow(dp,2)/ρ

    PitotVelocity= math.sqrt(PitotVelocitySquared)
    #velocity from Pitot Tube    
###########################################################################   

    with open('sensordata.csv', 'a') as csv_file:
        csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        info = {
                "Time (s)": t,
                "Acceleration (m/s^2)": accel,
                "Velocity(Pitot)(m/s)": PitotVelocity,
                "DifferentialPressure(Pitot)(Pa)": dp,
                "Temperature (K)": temp,
                "AtmosphericPressure (hPa)": press,
                "Altitude(m)": altit,
            }
        csv_writer.writerow(info)

    PitotVelocityvar.append(PitotVelocity)
    Temperaturevar.append(temp)
    Pressurevar.append(press)
    Timevar.append(t)
    drawnow(makeFig)
    plt.pause(.00001)
    if(count>10):
        PitotVelocityvar.pop(0)
        Timevar.pop(0)
        Temperaturevar.pop(0)
        Pressurevar.pop(0)
    finaldata= pd.read_csv('sensordata.csv')
    finaldata.to_excel('finaldata.xlsx',sheet_name='SensorData',index=False)
    print('data saved to excel')
