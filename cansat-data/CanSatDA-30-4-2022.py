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
PitotTubeVelocityvar=[]
AltitudeVelocityvar=[]
Temperaturevar=[]
Pressurevar=[]
Altitudevar=[]

RH = 1
count=0

print('Starting Variables Set Up')

plt.ion()

def makeFig():

    plt.subplot(1,3,1)
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


    plt.subplot(1,3,3)
    plt.plot(Timevar,Temperaturevar,'r',label='Temperature')
    plt.legend(loc='upper left')
    #plt.ylim(...)
    plt.title('Primary M.')
    plt.grid(True)
    #plt.ylabel('Temperature')
    plt.xlabel('Time')
    plt.ticklabel_format(useOffset=False)
    plt2=plt.twinx()
    #plt.ylim(...)
    plt2.plot(Timevar,Pressurevar,label='Atmospheric Pressure')
    #plt2.set_ylabel('A. Pressure')
    plt2.legend(loc='best')

    plt.subplot(1,3,2)
    plt.plot(Timevar,Altitudevar,"r",label='Altitude')
    plt.legend(loc='upper left')
    #plt.ylim(....)
    plt.title('CanSat Altitude')
    plt.grid(True)
    #plt.ylabel('Altitude')
    plt.xlabel('Time')
    plt.ticklabel_format(useOffset=False)

    plt.suptitle("CANSAT")

print('Plot Settings Setup')

fieldnames = [
    "Time (s)",
    "DifferentialPressure(Pitot)(Pa)",
    "Temperature 1 (C)",
    "AtmosphericPressure (hPa)",
    "Altitude(m)",
    "Accel. X (m/s^2)",
    "Accel. Y (m/s^2)",
    "Accel. Z (m/s^2)",
    "PitotVelocity (m/s)",
    "AltitudeVelocity (m/s)"
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
    print("Time:",t)
    
    dp =  float(ser.readline().decode("utf-8"))
    print("Differential Pressure:",dp)
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

    if count == 1:

        h1 = altit
        t1 = t
        AltitudeVelocity = 0

    elif count > 1:

        h2 = altit
        t2 = t
        AltitudeVelocity = (h2-h1)/(t2-t1)
        
        h1 = h2
        t1 = t2
        
    
    

###########################################################################

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

    ρ = ((Pd / (Rd * Tkelvin)) + (Pv / (Rv * Tkelvin)))*100
    #Air density
    
    #Air density

    PitotVelocity = math.sqrt((((dp-38.5)/1023)-0.5)*10000/ρ)
    #velocity from Pitot Tube
    
###########################################################################   

    with open('sensordata.csv', 'a') as csv_file:
        csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        info = {
                "Time (s)": t,
                "DifferentialPressure(Pitot)(Pa)": dp,
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

    PitotTubeVelocityvar.append(PitotVelocity)
    AltitudeVelocityvar.append(AltitudeVelocity)
    Temperaturevar.append(temp1)
    Pressurevar.append(press)
    Altitudevar.append(altit)
    Timevar.append(t)
    drawnow(makeFig)
    plt.pause(.00001)
    if(count>10):
        PitotTubeVelocityvar.pop(0)
        Timevar.pop(0)
        Temperaturevar.pop(0)
        Altitudevar.pop(0)
        Pressurevar.pop(0)
        AltitudeVelocityvar.pop(0)
    finaldata= pd.read_csv('sensordata.csv')
    finaldata.to_excel('finaldata.xlsx',sheet_name='SensorData',index=False)
    print('data saved to excel')

