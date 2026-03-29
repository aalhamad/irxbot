import serial
import time

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=2)
time.sleep(4)

# Send character by character with small delay
cmd = b'CMD 3000 3000\n'
for byte in cmd:
    ser.write(bytes([byte]))
    time.sleep(0.001)

time.sleep(0.5)
response = ser.readline()
print('Response:', response)
ser.close()
