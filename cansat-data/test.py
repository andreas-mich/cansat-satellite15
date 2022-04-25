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

Timevar =[]
AccelVvar =[]
PitotVvar =[]
Tempvar =[]
Pressvar =[]

count = 0

print('Starting Variables Set Up')

plt.ion()

def makeFig():

    plt.subplot(2,1,1)
    plt.plot(Timevar,AccelVvar,"r",label='AccelerometerVelocity Plot')
    plt.legend(loc='upper left')
    #plt.ylim(....)
    plt.title('Live Sensor Data')
    plt.grid(True)
    plt.ylabel('Accelerometer V')
    plt.ticklabel_format(useOffset=False)
    plt2=plt.twinx()
    #plt.ylim(...)
    plt2.plot(Timevar,PitotVvar,label='PitotVelocity Plot')
    plt2.set_ylabel('Pitot V')
    plt2.legend(loc='upper right')

    plt.subplot(2,1,2)
    plt.plot(Timevar,Tempvar,'r',label='Temperature Plot')
    plt.legend(loc='upper left')
    #plt.ylim(...)
    plt.grid(True)
    plt.ylabel('Temperature')
    plt.xlabel('Time')
    plt.ticklabel_format(useOffset=False)
    plt2=plt.twinx()
    #plt.ylim(...)
    plt2.plot(Timevar,Pressvar,label='Atmospheric Pressure')
    plt2.set_ylabel('A. Pressure')
    plt2.legend(loc='upper right')


print('Plot Settings Setup')
    

fieldnames = [
    "Time (s)",
    "Accel. X (m/s^2)",
    "Accel. Y (m/s^2)",
    "Accel. Z (m/s^2)",
    "Acceleration (m/s^2)",
    "Velocity(Accelerometer)(m/s)",
    "DifferentialPressure(Pitot)(Pa)",
    "Velocity(Pitot)(m/s)",
    "Temperature (K)",
    "AtmosphericPressure (hPa)",
    "Altitude(m)",
    "Velocity Error"
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

###########################################################################
        
    accelmid = (pow(x,2)+pow(y,2)+pow(z,2))
    accel = float(sqrt(accelmid))
    #acceleration from accelerometer (x,y,z)

    accelV = (accel*t)
    #velocity from accelerometer 
        
###########################################################################

    Rd = 287.058 #Specific gas constant for dry air
    Rv = 461.495 #Specific gas constant for water vapor

    A=pow(10,7.5*temp /(temp + 237.3))
    Psat = 6.1078 * A
    #Saturation Vapor Pressure (Vapor Pressure at 100% RH) 

    Pv = Psat * RH
    # Vapor Pressure

    Pd = press - Pv
    #Dry Air Pressure

    Tkelvin = temp + 273.15

    ρ = ((Pd / (Rd * Tkelvin)) + (Pv / (Rv * Tkelvin)))*100
    #Air density

    PitotVelocitySquared = pow(dp,2)/ρ

    PitotVelocity= sqrt(float(PitotVelocitySquared))
    #velocity from Pitot Tube

    Error=((AccelV-PitotVelocity)/AccelV)*100
    
###########################################################################        

    with open('sensordata.csv', 'a') as csv_file:
        csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        info = {
                 "Time (s)": t,
                "Accel. X (m/s^2)": x,
                "Accel. Y (m/s^2)": y,
                "Accel. Z (m/s^2)": z,
                "Acceleration (m/s^2)": accel,
                "Velocity(Accelerometer)(m/s)": accelV,
                "DifferentialPressure(Pitot)(Pa)": dp,
                "Velocity(Pitot)(m/s)": PitotVelocity,
                "Temperature (K)": temp,
                "AtmosphericPressure (hPa)": press,
                "Altitude(m)": altit,
                "Velocity Error": Error
            }
        csv_writer.writerow(info)
            
###########################################################################
            
    AccelVvar.append(accelV)
    PitotVvar.append(PitotVelocity)
    Tempvar.append(temp)
    Pressvar.append(press)
    Timevar.append(t)
    drawnow(makeFig)
    plt.pause(.00001)
    if(count>10):
        AccelVvar.pop(0)
        PitotVvar.pop(0)
        Timevar.pop(0)
        Tempvar.pop(0)
        Pressvar.pop(0)
    finaldata= pd.read_csv('sensordata.csv')
    finaldata.to_excel('finaldata.xlsx',sheet_name='SensorData',index=False)
    print('data saved to excel')
