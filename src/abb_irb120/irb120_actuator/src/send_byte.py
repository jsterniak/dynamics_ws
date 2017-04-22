import serial
com = serial.Serial('/dev/ttyACM0',baudrate=9600)
com.write(0x01)
com.close()
