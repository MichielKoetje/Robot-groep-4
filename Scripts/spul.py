from pyax12.connection import Connection
import time
try:
  # Connect to the serial port
  serial_connection = Connection(port="/dev/serial0", baudrate=57600, rpi_gpio=True)
  dynamixel_id = 5
  serial_connection.set_cw_angle_limit(dynamixel_id, 0, degrees=False)
  serial_connection.set_ccw_angle_limit(dynamixel_id, 0, degrees=False)
  #serial_connection.set_ccw_angle_limit(dynamixel_id, 1023, degrees=False)
# Go to 0°
  for x in range(2):
    reverse = (x+1)%2
    a = 0x400*reverse
    b = 0x400*(1-reverse)
    serial_connection.set_speed(21, a+int(300))
    serial_connection.set_speed(5, b+int(400))
    time.sleep(1)
    serial_connection.set_speed(21, a+int(500))
    serial_connection.set_speed(5, b+int(600))
    time.sleep(2)
    serial_connection.set_speed(21, a+int(300))
    serial_connection.set_speed(5, b+int(400))
    time.sleep(1)
    serial_connection.set_speed(21, 0)
    serial_connection.set_speed(5, 0)
    time.sleep(1)    # Wait 1 second
except Exception as e:
    print(f"Error: {e}")
# Close the serial connection
serial_connection.close()


import smbus
import time
DEVICE_AS5600 = 0x36 # Default device I2C address
bus = smbus.SMBus(3)
def ReadRawAngle(): # Read angle (0-360 represented as 0-4096)
  read_bytes = bus.read_i2c_block_data(DEVICE_AS5600, 0x0C, 2)
  return (read_bytes[0]<<8) | read_bytes[1]


def ReadMagnitude(): # Read magnetism magnitude
  read_bytes = bus.read_i2c_block_data(DEVICE_AS5600, 0x1B, 2)
  return (read_bytes[0]<<8) | read_bytes[1]
while True:
#print(ReadMagnitude())
  print(((ReadRawAngle()*360/4096+5.5+180)%360)-180)
  time.sleep(0.1)
