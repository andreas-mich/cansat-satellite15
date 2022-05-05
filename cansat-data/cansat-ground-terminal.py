###############################################  Import Libraries  ###############################################

#for math
import math
from math import pi, sqrt, atan
import numpy as np

#for arduino connection
import serial
from serial import Serial

#to save data to csv
import csv 

#to save data to excel
import pandas as pd
import openpyxl

#for plotting
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from drawnow import *

###############################################  Initialise Global Variables ###############################################

print('Initialise Global Variables')

serial_port = 'COM8'
serial_rate = 115200

Timevar=[]
PitotTubeVelocityvar=[]
AltitudeVelocityvar=[]
Temperaturevar=[]
Pressurevar=[]
Altitudevar=[]

RH = 1

pitchF = 0
rollF = 0

###############################################  FUNCTION Make Figure  ###############################################
def makeFig():
    # 1st plot - 3D figure
    # define 3d figure
    x, y, z = np.indices((2, 4, 2))
    cuboid = ( x == 0) & (y == 0) & (z < 2)
    colors = np.empty(cuboid.shape, dtype=object)
    colors[cuboid] = 'red'
    # plot 3d figure
    ax = plt.subplot(1,4,1, projection='3d')
    ax.voxels(cuboid, facecolors=colors) #edgecolor='white'
    ax.axis("off")
    # change pitch and roll
    ax.view_init(azim=30, elev=pitch, roll=roll)

    # 2nd plot
    ax = plt.subplot(1,4,2)
    plt.plot(Timevar,PitotTubeVelocityvar,"r",label='Velocity(Pitot)')
    plt.legend(loc='upper left')
    #plt.ylim(...)
    plt.title('Secondary M.')
    plt.grid(True)
    #plt.ylabel('Velocity(Pitot)')
    plt.xlabel('Time')
    plt.ticklabel_format(useOffset=False)
    #plt3=plt.twinx()
    #plt.ylim(...)
    plt.plot(Timevar,AltitudeVelocityvar,label='V(BMP)')
    #plt3.set_ylabel('Velocity(BMP)')
    plt.legend(loc='best')

    plt.subplot(1,4,3)
    plt.plot(Timevar,Altitudevar,"r",label='Altitude')
    plt.legend(loc='upper left')
    #plt.ylim(....)
    plt.title('CanSat Altitude')
    plt.grid(True)
    #plt.ylabel('Altitude')
    plt.xlabel('Time')
    plt.ticklabel_format(useOffset=False)
    plt.suptitle("CANSAT")

    plt.subplot(1,4,4)
    plt.plot(Timevar,Temperaturevar,'r',label='Temperature')
    plt.legend(loc='upper left')
    #plt.ylim(...)
    plt.title('Primary M.')
    plt.grid(True)
    #plt.ylabel('Temperature')
    plt.xlabel('Time')
    plt.ticklabel_format(useOffset=False)
    plt2=plt.twinx()
    plt.ylim(800,1100)
    plt2.plot(Timevar,Pressurevar,label='Atmospheric Pressure')
    #plt2.set_ylabel('A. Pressure')
    plt2.legend(loc='best')

###############################################  Setup Plot  ###############################################

print('Setup Plot')

plt.ion()

fieldnames = [
    "Time (s)",
    "Time (ms)",
    "DifferentialPressure(Voltage)",
    "Temperature 1 (C)",
    "AtmosphericPressure (hPa)",
    "Altitude(m)",
    "Accel. X (m/s^2)",
    "Accel. Y (m/s^2)",
    "Accel. Z (m/s^2)",
    "PitotVelocity (m/s)",
    "AltitudeVelocity (m/s)",
    ]

###############################################  Open Serial Port  ###############################################

print('Open Serial Port')

with open('sensordata.csv', 'w') as csv_file:
    csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
    csv_writer.writeheader()
    print('CSV created')

ser = Serial(serial_port, serial_rate)
ser.close()
ser.open()
print("Connected to: " + ser.portstr)

###############################################  Read From Serial Port  ###############################################

count = 0

while True:
    while (ser.inWaiting()==0):
        pass
    count += 1

    ################################# Take Readings ################################# 
    t = int(ser.readline().decode("utf-8"))
    t_sec = round(t / 1000)
    print("Time:",t_sec)   
    dp =  float(ser.readline().decode("utf-8"))
    print("Differential Pressure Voltage:",dp)
    temp1 = float(ser.readline().decode("utf-8"))
    print("Temperature (Sensor):",temp1)
    press =  float(ser.readline().decode("utf-8"))
    print("AtmosphericPressure:",press)
    altit =  float(ser.readline().decode("utf-8"))
    print("Altitude:",altit)
    ax = float(ser.readline().decode("utf-8"))
    print("Accel. X:",ax)
    ay = float(ser.readline().decode("utf-8"))
    print("Accel. Y:",ay)
    az = float(ser.readline().decode("utf-8"))
    print("Accel. Z:",az)

    ################################# Compute Roll & Pitch ################################# 
    # Calculate Roll and Pitch (rotation around X-axis, rotation around Y-axis) from ax, ay, az
    roll = atan(ay / sqrt(ax ** 2 + az ** 2)) * 180 / pi
    pitch = atan(-1 * ax / sqrt(ay ** 2 + az ** 2)) * 180 / pi
    # Low-pass filter
    rollF = 0.94 * rollF + 0.06 * roll
    pitchF = 0.94 * pitchF + 0.06 * pitch

    ################################# Compute Differential Altitude ################################# 
    if count == 1:
        h1 = altit
        t1 = t_sec
        AltitudeVelocity = 0
    elif count > 1:
        h2 = altit
        t2 = t_sec
        AltitudeVelocity = (h2-h1)/(t2-t1)
        h1 = h2
        t1 = t2

    ################################# Compute Pitot ################################# 
    Rd = 287.058 #Specific gas constant for dry air
    Rv = 461.495 #Specific gas constant for water vapor
    A=math.pow(10,7.5*temp1 /(temp1 + 237.3))
    Psat = 6.1078 * A
    #Saturation Vapor Pressure (Vapor Pressure at 100% RH) 

    Pv = Psat * RH
    # Vapor Pressure

    Pd = press - Pv
    #Dry Air Pressure

    Tkelvin = temp1 + 273.15

    ro = ((Pd / (Rd * Tkelvin)) + (Pv / (Rv * Tkelvin)))*100
    #Air density
    
    #differentialPressure = (10000*(((dp-38.5)/1023)-0.5)/2)
    #print("Differential Pressure:",differentialPressure)
    #Differential Pressure Pitot Tube

    PitotVelocity = sqrt(abs(((dp-38.5)/1023)-0.5)*10000/ro)
    #print("Pitot Velocity:",PitotVelocity)
    #velocity from Pitot Tube
    
    ################################# Add to Figure Data and Draw Figure ################################# 
    PitotTubeVelocityvar.append(PitotVelocity)
    AltitudeVelocityvar.append(AltitudeVelocity)
    Temperaturevar.append(temp1)
    Pressurevar.append(press)
    Altitudevar.append(altit)
    Timevar.append(t_sec)

    drawnow(makeFig)
    plt.pause(.00001)

    ################################# CLear Figure if data points > 10 ################################# 
    if(count>10):
        PitotTubeVelocityvar.pop(0)
        Timevar.pop(0)
        Temperaturevar.pop(0)
        Altitudevar.pop(0)
        Pressurevar.pop(0)
        AltitudeVelocityvar.pop(0)

    ################################# Save Readings to CSV ################################# 
    with open('sensordata.csv', 'a') as csv_file:
        csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        info = {
                "Time (s)": t_sec,
                "Time (ms)": t,
                "DifferentialPressure(Voltage)": dp,
                "Temperature 1 (C)": temp1,
                "AtmosphericPressure (hPa)": press,
                "Altitude(m)": altit,
                "Accel. X (m/s^2)": ax,
                "Accel. Y (m/s^2)": ay,
                "Accel. Z (m/s^2)": az,
                "PitotVelocity (m/s)": PitotVelocity,
                "AltitudeVelocity (m/s)": AltitudeVelocity
                }
        csv_writer.writerow(info)
        print('data saved to csv')

    ################################# Save Readings to XLS ################################# 
    finaldata = pd.read_csv('sensordata.csv')
    finaldata.to_excel('finaldata.xlsx',sheet_name='SensorData',index=False)
    print('data saved to excel')