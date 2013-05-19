#!/usr/bin/env python

import math, serial, sys, time

def command(port, lin_vel, ang_vel):
    diffConversionFactor = 0.1 # Distance between the wheels in meters
    motorScaleFactor = 1000 # Commands are received in the Twist message in meters and the robot accepts them in mm. This is the scale factor.

    # Receive commanded velocity and angular velocity from message
    targetTransVel = lin_vel
    targetRotVel = ang_vel

    # Difference in speed between the wheels as this is a differential drive robot
    rotTerm = (math.pi/180.0) * targetRotVel / diffConversionFactor

    # Motor speeds in mm/s
    leftVel = int((targetTransVel - rotTerm) * motorScaleFactor)
    rightVel = int((targetTransVel + rotTerm) * motorScaleFactor)

    # Create string for commanding wheel speed and write it to the serial port.
    motorCommand = '[=<' + str(leftVel) + '><' + str(rightVel) + '>]' + '\n'
    port.write(motorCommand)

def closePort(port):
    leftVel = 0
    rightVel = 0
    motorCommand = '[=<' + str(leftVel) + '><' + str(rightVel) + '>]' + '\n'
    port.write(motorCommand)
    port.close()
    return 0

def createPort(port_address):
    port = serial.Serial()
    port.port = port_address
    port.baudrate = 19200
    port.timeout = 600
    port.bytesize = serial.EIGHTBITS
    port.stopbits = serial.STOPBITS_ONE
    port.parity = serial.PARITY_NONE
    port.open()
    print "Opened: " + port_address
    return port

def coupled_system(time):
    C1 = 1.0;
    omega1 = 1.0;
    phi1 = 1.0;
    C2 = 1.0;
    omega2 = 0.5;
    phi2 = 0.2;
    av1 = -C1*omega1*math.sin(omega1*time+phi1)/2 - C2*omega2*math.sin(omega2*time+phi2)/2
    av2 = -C1*omega1*math.sin(omega1*time+phi1)/2 + C2*omega2*math.sin(omega2*time+phi2)/2
    return av1,av2

def main():
    # argv = sys.argv
    port1 = createPort("/dev/rfcomm0")
    port2 = createPort("/dev/rfcomm1")
    start = time.clock()
    while(1):
        try:
            #print time.clock()
            av1,av2 = coupled_system(time.clock()-start)
            command(port1, 0, av1)
            command(port2, 0, av2)
            #time.sleep(0.1)
            
        except KeyboardInterrupt:
            print "Closing..."
            closePort(port1)
            closePort(port2)


if __name__ == "__main__":
    main()
