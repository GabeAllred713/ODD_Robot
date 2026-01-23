import serialExample
import time

device = serialExample.ExampleSerial("/dev/ttyACM0")

time.sleep(2) # Wait for arduino to boot up

device.motors(0, 0)
print(device.encoders())
print(device.encoders())
print(device.encoders())
device.motors(1, 1)
time.sleep(1)
print(device.encoders())
device.motors(-1, -1)
time.sleep(1)
print(device.encoders())
device.motors(0, 0)
device.close()