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