import socket
from PCA9685 import PCA9685
import serial




ser_baud = 115200
ser_port='/dev/ttyUSB0'
ser = serial.Serial(ser_port,ser_baud,timeout=0.005)
print(ser.name)

HOST = '0.0.0.0' # Server IP or Hostname
PORT = 12352 # Pick an open Port (1000+ recommended), must match the client sport
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print ('Socket created')

#managing error exception
try:
	s.bind((HOST, PORT))
except socket.error:
	print('Bind failed ')

s.listen(5)
print('Socket awaiting messages')


def ser_msg(msg):
	print('test1')
	print(msg)
	ser.write(bytes(msg,'utf-8'))
	print ('test')
	
	
ser_msg('l0,r0')

conn, addr = s.accept()
print('Connected')





	
# awaiting for message
while True:
	data = str(conn.recv(1024).decode('utf-8'))
	print('I sent a message back in response to: ' + data)
	reply = ''
	if data.startswith('ardu_msg'):
		try:
			msg=data.split(' ')[1]
			ser_msg(msg)
			
			reply = 'Sent to Arduino: ' + msg			
		except:
			reply = 'Ardu message corrupt: '+data

	#and so on and on until...
	elif data == 'quit':
		conn.send('Terminating')
		break
	else:
		reply = 'Unknown command'

	# Sending reply
	conn.send(bytes(reply, 'utf-8'))
	
	if ser.in_waiting:
		print(ser.read(100).decode('utf-8'))
conn.close() # Close connections
