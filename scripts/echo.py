import serial, time

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=3)
time.sleep(5)  # Wait longer than boot delay
ser.write(b'CMD 3000 3000\n')
time.sleep(0.5)
response = ser.readline()
print('Response:', response)
ser.close()
