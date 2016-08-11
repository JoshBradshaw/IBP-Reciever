import serial
import time

SERIAL_BAUDRATE = 115200


ser = serial.Serial()
if ser.isOpen():
	self.ser.close()

ser.baudrate = SERIAL_BAUDRATE
ser.timeout = 1 # 1 second just in case readline() ever hangs
ser.port = 'COM6'
ser.open()

start_time = time.time()

ii = 0

while ii < 20000:
	serial_line = ser.readline()
	ii += 1

print "--- %s seconds ---" % (time.time() - start_time) 