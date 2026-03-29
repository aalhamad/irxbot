import serial, time

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(2)
ser.write(b'Hello\n')

print('Received:', ser.read(100))
ser.close()
