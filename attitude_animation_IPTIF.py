import serial
import struct
import math
import numpy as np
from scipy import linalg as LA
from vpython import *
from scipy.signal import butter, lfilter

## Base code IMU1.1 in PSOC
## taken from imucode_IPTIF3_gyro.py in raspberry pi
## date 11/07/2023
## Implemented accel+mag based Euler angles and gyro based Euler angles
## IMU rotations and reflections taken care. Enter data is now with respect to PCB frame
# animation
scene.range = 5
toRad = 2*np.pi/360
toDeg = 1/toRad
scene.forward = vector(-1,-1,-1)
scene.width = 600
scene.height = 600

#vpython inertial(global) coordinates
global_xarrow = arrow(axis = vector(-1,0,0),length=3,width=0.2,height=0.2,shaftwidth=0.1,color=color.red)
global_yarrow = arrow(axis = vector(0,0,-1),length=3,width=0.2,height=0.2,shaftwidth=0.1,color=color.green)
global_zarrow = arrow(axis = vector(0,1,0),length=3,width=0.2,height=0.2,shaftwidth=0.1,color=color.blue)

# body design
base = box(length=8,width=2,height=0.2,opacity=0.3)  # length, height all are inches
pcb = box(length=1.7,width=2.5,height=0.2,opacity=0.3,color=color.green,pos=vector(-2,0.2,0))  # pos is the center of each body
rpi = box(length=1.7,width=2.5,height=0.2,opacity=0.3,color=color.green,pos=vector(2,0.2,0))   # pos means fixing center of that body
imu = box(length=0.5,width=0.7,height=0.2,color=color.red,pos=vector(-2.2,0.4,0.5))
body = compound([base,pcb,rpi,imu])  # make all parts into one body

# Body coordinates fixed to body
xarrow = arrow(axis = vector(-1,0,0),length=2,width=0.2,height=0.2,shaftwidth=0.1,color=color.red,opacity=0.5)
yarrow = arrow(axis = vector(0,0,-1),length=2,width=0.2,height=0.2,shaftwidth=0.1,color=color.green,opacity=0.5)
zarrow = arrow(axis = vector(0,1,0),length=2,width=0.2,height=0.2,shaftwidth=0.1,color=color.blue,opacity=0.5)


ser = serial.Serial('COM3',57600)
ser.flushInput()
ser.flushOutput()

delta_t = 1/80
while True:
    data_raw = ser.read(40)                        # in C, we sent 36 bytes(9*4) of data, so here we are reading 36 bytes
    X = (data_raw[0]*256 + data_raw[1])*65536 + (data_raw[2]*256 + data_raw[3])
    print(X)                               # example data:  b'\xe2m\x01\x00'
    data_line = struct.unpack('<iiiiiiiiii',data_raw)     # struct.unpack() converts bytes into python datatypes, '<' = little endian, 'i' = int, 'I' uint, 'H' is hexa.
    # print(int.from_bytes(data_raw, "big"))        # default i or I means, required 4 bytes, H requires 2 bytes.
    dummy_byte = data_line[0]
    #print(dummy_byte)
    if X == 0xFFFFFFFF:
        xAcc = float(data_line[1]) / 100000               # data_line is a tuple ( which is collection of items,
        yAcc = float(data_line[2]) / 100000              # tuple is one of the types of the arrays, it stores multiple items in a single variable )
        zAcc = float(data_line[3]) / 100000

        xMag = float(data_line[7]) * (100/100000)  # converted gauss to micro tesla units
        yMag = float(data_line[8]) * (100/100000)
        zMag = float(data_line[9]) * (100/100000)
        break
mag_m = np.array([-xMag, yMag, zMag])
mag0  = mag_m / LA.norm(mag_m)

acc_m = np.array([xAcc+0.255,yAcc+0.172,zAcc+0.43])
acc   = acc_m / LA.norm(acc_m)

## Convert Left hand coordinate frame to RH frame
acc[0]  = - acc[0]
mag0[0] = - mag0[0]

phi_di0   = math.atan2(acc[1], acc[2])
theta_di0 = math.atan2(-acc[0], math.sqrt(acc[1] * acc[1] + acc[2] * acc[2]))
psi_di0   = math.atan2(mag0[2] * math.sin(phi_di0) - mag0[1] * math.cos(phi_di0),
                        mag0[0] * math.cos(theta_di0) + mag0[1] * math.sin(theta_di0) * math.sin(phi_di0) + mag0[
                            2] * math.sin(theta_di0) * math.cos(phi_di0))

phi0   = phi_di0 * (180 / math.pi)
theta0 = theta_di0 * (180 / math.pi)
psi0   = psi_di0* (180 / math.pi)


while True:
    # data = ser.inWaiting()
    data_raw = ser.read(40)                        # in C, we sent 36 bytes(9*4) of data, so here we are reading 36 bytes
    X = (data_raw[0] * 256 + data_raw[1]) * 65536 + (data_raw[2] * 256 + data_raw[3])
    #print(data_raw)                               # example data:  b'\xe2m\x01\x00'
    data_line = struct.unpack('<iiiiiiiiii',data_raw)     # struct.unpack() converts bytes into python datatypes, '<' = little endian, 'i' = int, 'I' uint, 'H' is hexa.
   # print(int.from_bytes(data_raw, "big"))        # default i or I means, required 4 bytes, H requires 2 bytes.
    dummy_byte = data_line[0]
    if(X == 0xFFFFFFFF):
        xAcc = float(data_line[1]) / 100000               # data_line is a tuple ( which is collection of items,
        yAcc = float(data_line[2]) / 100000              # tuple is one of the types of the arrays, it stores multiple items in a single variable )
        zAcc = float(data_line[3]) / 100000
        xGyro = float(data_line[4]) *180/(np.pi *100000)  ## in degrees/sec
        yGyro = float(data_line[5]) *180/(np.pi *100000)
        zGyro = float(data_line[6]) *180/(np.pi *100000)
        xMag = float(data_line[7]) * (100/100000)
        yMag = float(data_line[8]) * (100/100000)
        zMag = float(data_line[9]) * (100/100000)

        gyro = np.array([xGyro, yGyro, zGyro])

        mag_m = np.array([-xMag, yMag, zMag])
        mag0 = mag_m / LA.norm(mag_m)

        acc_m = np.array([xAcc + 0.255, yAcc + 0.172, zAcc + 0.43])
        acc = acc_m / LA.norm(acc_m)

        ## Convert Left hand coordinate frame to RH frame
        acc[0] = - acc[0]
        mag0[0] = - mag0[0]
        gyro[0] = - gyro[0]

        phi_di = math.atan2(acc[1], acc[2])
        theta_di = math.atan2(-acc[0], math.sqrt(acc[1] * acc[1] + acc[2] * acc[2]))
        psi_di = math.atan2(mag0[2] * math.sin(phi_di) - mag0[1] * math.cos(phi_di),
                            mag0[0] * math.cos(theta_di) + mag0[1] * math.sin(theta_di) * math.sin(phi_di) + mag0[
                                2] * math.sin(theta_di) * math.cos(phi_di))

        phi2 = phi_di * (180 / math.pi)
        theta2 = theta_di * (180 / math.pi)
        psi2 = psi_di * (180 / math.pi)

        print(phi2, theta2, psi2)

        gyro = (gyro - np.array([2.2, -0.4, 0.6])) * (np.pi / 180)  # Gyro is already in degree/sec

        # Attitude from only gyroscope
        prev = np.array([phi_di0, theta_di0, psi_di0])
        T = np.array([[1, math.sin(phi_di0) * math.tan(theta_di0), math.cos(phi_di0) * math.tan(theta_di0)],
                      [0, math.cos(phi_di0), -math.sin(phi_di0)],
                      [0, math.sin(phi_di0) / math.cos(theta_di0), math.cos(phi_di0) / math.cos(theta_di0)]])
        rate = np.dot(T, gyro)
        attitude_gyro = (prev + rate * delta_t)
        phi3 = attitude_gyro[0] * (180 / np.pi)
        theta3 = attitude_gyro[1] * (180 / np.pi)
        psi3 = attitude_gyro[2] * (180 / np.pi)
        print(attitude_gyro * (180 / np.pi))
        phi_di0 = attitude_gyro[0]
        theta_di0 = attitude_gyro[1]
        psi_di0 = attitude_gyro[2]

        # animation , roll pitch yaw
        k = vector(cos(psi2*(math.pi/180)+math.pi) * cos(phi2*(math.pi/180)), sin(phi2*(math.pi/180)), sin(psi2*(math.pi/180)+math.pi) * cos(phi2*(math.pi/180)))
        y = vector(0, 1, 0)
        s = cross(k, y)
        v = cross(s, k)
        vrot = v*cos(theta2*(math.pi/180)) + cross(k,v)*sin(theta2*(math.pi/180))
        xarrow.axis = k
        yarrow.axis = s
        zarrow.axis = vrot
        body.axis = k
        body.up = vrot



