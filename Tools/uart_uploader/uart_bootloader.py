import time
import serial
import sys
import os
import string
import argparse
import zlib
import binascii

# Parsing the arguments
parser = argparse.ArgumentParser(description = 'SAMD Bootloader script')
parser.add_argument('-c','--port', help='COM port name',required=True)
parser.add_argument('-b','--baud', help='COM port Baudrate',required=True)
parser.add_argument('-i','--inputfile', help='Input file name (.bin file)',required=True)
args = parser.parse_args()

page_size = 64
row_size = 4*page_size

# Opening COM port
ser = serial.Serial(
	port = args.port,
	baudrate = args.baud,
	parity = serial.PARITY_NONE,
	stopbits = serial.STOPBITS_ONE,
	bytesize = serial.EIGHTBITS
)
print("Port %s opened" % ser.port)

# Check whether any boot device is available
length = ser.write('#'.encode())
while ser.inWaiting == 0:
	pass
read = ser.read(1)
if read != b's':
	print ("No response from device \n ({})".format(read))
	sys.exit()

# Read the application flash size
while ser.inWaiting == 0:
	pass
temp_app_size = ser.read(1)
app_size = ord(temp_app_size)
app_size = app_size * row_size
print ("Found boot media with %d Bytes application flash size!" %app_size)

# Read file size of input 
len = os.path.getsize(args.inputfile)

# Check whether file size fits the device memory
if len > app_size:
	print ("Input file size does not fit target memory! \n")
	sys.exit()

# Erase application flash region of the device
print ("Erasing flash...")
length = ser.write('e'.encode())
while ser.inWaiting == 0:
	pass
read = ser.read(1)
if read != b's':
	print ("No response from device \n")
	sys.exit()

print ("Erase Complete!")

# Open input file
out = 0
len = os.path.getsize(args.inputfile)
input_file = open(args.inputfile, 'rb')
block_len = 64
nb_blocks = len / block_len
rem_data = len % block_len

# Start programming flash
print ("Programming flash...")

# Program all pages except last one
out = ''
for x in range(0, int(nb_blocks)):
	print ("Programming page %d" % x)
	length = ser.write('p'.encode())
	while ser.inWaiting == 0:
		pass
	read = ser.read(1)
	if read != b's':
		print ("No response from device \n")
		sys.exit()
	for y in range(0, block_len):
		#print ("Data %d" % y)
		out = input_file.read(1)
		length = ser.write(out)
		#time.sleep(0.001)

	#print (".")
	while ser.inWaiting == 0:
		pass
	read = ser.read(1)
	if read != b's':
		print ("No response from device \n")
		sys.exit()

# Program last page
print ("Programming page %d" % nb_blocks)
length = ser.write('p'.encode())
while ser.inWaiting == 0:
	pass
read = ser.read(1)
if read != b's':
	print ("No response from device \n")
	sys.exit()
	
for x in range(0, rem_data):
	length = ser.write(input_file.read(1))
	#time.sleep(0.001)

for x in range(0, (page_size - rem_data)):
	length = ser.write(b"\xFF")
	#time.sleep(0.001)
     
while ser.inWaiting == 0:
	pass
read = ser.read(1)
if read != b's':
	print ("No response from device \n")
	sys.exit()

# Close file
input_file.close()
print ("\n")

# Open input file for verifying flash
input_file = open(args.inputfile, 'rb')
print ("Verifying flash...")

# Verify all pages except last one
for x in range(0, int(nb_blocks)):
	print ("Verifying page %d" % x)
	length = ser.write('v'.encode())
	while ser.inWaiting == 0:
		pass
	read = ser.read(1)
	if read != b's':
		print ("No response from device \n")
		sys.exit()
	for y in range(0, int(block_len)):
		while ser.inWaiting == 0:
			pass
		read = ser.read(1)
		out = input_file.read(1)
		if read != out:
			print("\nVerification Failed at address %d!", ((x*64)+y))
			sys.exit()
#	print (".")

# Verify last page
print ("Verifying page %d" % nb_blocks)
length = ser.write('v'.encode())
while ser.inWaiting == 0:
	pass
read = ser.read(1)
if read != b's':
	print ("No response from device \n")
	sys.exit()
	
for x in range(0, rem_data):
	while ser.inWaiting == 0:
		pass
	read = ser.read(1)
	out = input_file.read(1)
	if read != out:
		print("\nVerification Failed at address %d!", ((nb_blocks*64)+x))
		sys.exit()

for x in range(0, (page_size - rem_data)):
	while ser.inWaiting == 0:
		pass
	read = ser.read(1)

print ("\nVerification complete!")
input_file.close()

print("Reset target\n")
ser.write('r'.encode())

print("Done")
sys.exit()
