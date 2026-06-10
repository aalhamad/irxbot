import serial, time
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(2)
ser.write(b'CMD 1000 1000\r\n')
time.sleep(0.5)
response = ser.read(100)
print('Received:', response)
ser.close()
