import socket
from PCA9685 import PCA9685

pwm=PCA9685(0x40)

pwm.setPWMFreq(50)

#Set the Horizontal servo parameters
HPulse = 1050 #Sets the initial Pulse 50=> 80 grad rechts, 1050 => 0 1850=> 80 grad links
HStep = 0     #Sets the initial step length
pwm.setServoPulse(0,HPulse)

#Set the vertical servo parameters
VPulse = 1050  #Sets the initial Pulse 50 => 85 grad oben, 1050 gerade 1500 45 grad unten
VStep = 0      #Sets the initial step length
pwm.setServoPulse(1,VPulse)

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
conn, addr = s.accept()
print('Connected')

# awaiting for message
while True:
	data = conn.recv(1024)
	print('I sent a message back in response to: ' + data)
	reply = ''

	# process your message
	if data == 'Hello':
		reply = 'Hi, back!'
	elif data == 'This is important':
		reply = 'OK, I have done the important thing you have asked me!'
		
	elif data.startswith('servo'):
		try:
			h=int(data.split(' ')[1])
			v=int(data.split(' ')[2])
			pwm.setServoPulse(0,h)
			pwm.setServoPulse(1,v)
			reply = 'servo set to h='+str(h)+' v='+str(v)
			
		except:
			reply = 'servo message corrupt'






	#and so on and on until...
	elif data == 'quit':
		conn.send('Terminating')
		break
	else:
		reply = 'Unknown command'

	# Sending reply
	conn.send(reply)
conn.close() # Close connections
