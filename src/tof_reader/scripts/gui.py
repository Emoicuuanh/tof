import serial
ser = serial.Serial('/dev/ttyS2',115200,timeout= 1)
ser.write(b'Test\n')
print(ser.read(100))
ser.close()