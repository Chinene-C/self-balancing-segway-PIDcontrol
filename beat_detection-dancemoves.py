# encode dance routine as a set of ASCII characters and stored in a text file on your computer
# e.g. FFBB could mean move forward two steps on two beats, followed by backward two steps on the next two beats
# LRLRLRLR could mean step left than right and repeat again 
# You can then transfer the dance move text file to the SD card. 
# In your mPy program, you can open and read from this text
# file into an array of characters
# do the dance synchronised to the beat of the music


# program reads a line and the number of characters is the no. of beats
# assign a move to each letter, not a set of letters
# once beats completed, goes on to new line
# repeats the process once done so "while true" for infinite loop
#ascii - just limited characters
#does f.read(1) automatically move to next line or need to store values all in one line? 
# returns as a string

import os
import pyb
from pyb import Pin, Timer, ADC, DAC, LED
from array import array			# need this for memory allocation to buffers
from oled_938 import OLED_938	# Use OLED display driver
from audio import MICROPHONE

from neopixel import NeoPixel
import pyb
from pyb import Pin

import random

#  The following two lines are needed by micropython
#   ... must include if you use interrupt in your program
import micropython
micropython.alloc_emergency_exception_buf(100)

# motor stuff
# Define pins to control motor
A1 = Pin('X3', Pin.OUT_PP)		# Control direction of motor A
A2 = Pin('X4', Pin.OUT_PP)
PWMA = Pin('X1')				# Control speed of motor A
B1 = Pin('X7', Pin.OUT_PP)		# Control direction of motor B
B2 = Pin('X8', Pin.OUT_PP)
PWMB = Pin('X2')				# Control speed of motor B

# Configure timer 2 to produce 1KHz clock for PWM control
tim = Timer(2, freq = 1000)
motorA = tim.channel (1, Timer.PWM, pin = PWMA)
motorB = tim.channel (2, Timer.PWM, pin = PWMB)

# Define 5k Potentiometer
pot = pyb.ADC(Pin('X11'))

# I2C connected to Y9, Y10 (I2C bus 2) and Y11 is reset low active
i2c = pyb.I2C(2, pyb.I2C.MASTER)
devid = i2c.scan()				# find the I2C device number
oled = OLED_938(
    pinout={"sda": "Y10", "scl": "Y9", "res": "Y8"},
    height=64,
    external_vcc=False,
    i2c_devid=i2c.scan()[0],
)
oled.poweron()
oled.init_display()
oled.draw_text(0,0, 'Challenge 4')
oled.display()

# define ports for microphone, LEDs and trigger out (X5)

#b_LED = LED(4)		# flash for beats on blue LED
np = NeoPixel(Pin("Y12", Pin.OUT), 8)

# Create timer interrupt - one every 1/8000 sec or 125 usec
pyb.disable_irq()
sample_timer = pyb.Timer(7, freq=8000)	# set timer 7 for 8kHz

N = 160				# number of sample to calculate instant energy
mic = ADC(Pin('Y11'))
audio = MICROPHONE(sample_timer, mic, N)
pyb.enable_irq(True)

# Calculate energy over 50 epochs, each 20ms (i.e. 1 sec)
M = 50						# number of instantaneous energy epochs to sum
BEAT_THRESHOLD = 1.0		# threshold for c to indicate a beat
MIN_BEAT_PERIOD = 200	# no beat less than this

# initialise variables for main program loop
e_ptr = 0					# pointer to energy buffer
e_buf = array('L', 0 for i in range(M))	# reserve storage for energy buffer
sum_energy = 0				# total energy in last 50 epochs
oled.draw_text(0,20, 'Ready to GO')	# Useful to show what's happening?
oled.display()
pyb.delay(100)
tic = pyb.millis()			# mark time now in msec

def A_forward(value):
	A1.low()
	A2.high()
	motorA.pulse_width_percent(value)

def A_back(value):
	A2.low()
	A1.high()
	motorA.pulse_width_percent(value)
	
def A_stop():
	A1.high()
	A2.high()
	
def B_forward(value):
	B2.low()
	B1.high()
	motorB.pulse_width_percent(value)

def B_back(value):
	B1.low()
	B2.high()
	motorB.pulse_width_percent(value)
	
def B_stop():
	B1.high()
	B2.high()
	
# Initialise variables
speed = 0
A_speed = 0
A_count = 0
B_speed = 0
B_count = 0

#-------  Section to set up Interrupts ----------
def isr_motorA(dummy):	# motor A sensor ISR - just count transitions
	global A_count
	A_count += 1

def isr_motorB(dummy):	# motor B sensor ISR - just count transitions
	global B_count
	B_count += 1
		
def isr_speed_timer(dummy): 	# timer interrupt at 100msec intervals
	global A_count
	global A_speed
	global B_count
	global B_speed
	A_speed = A_count			# remember count value
	B_speed = B_count
	A_count = 0					# reset the count
	B_count = 0
	
# Create external interrupts for motorA Hall Effect Senor
# import micropython
# micropython.alloc_emergency_exception_buf(100)
from pyb import ExtInt

motorA_int = ExtInt ('Y4', ExtInt.IRQ_RISING, Pin.PULL_NONE,isr_motorA)
motorB_int = ExtInt ('Y6', ExtInt.IRQ_RISING, Pin.PULL_NONE,isr_motorB)

# Create timer interrupts at 100 msec intervals
speed_timer = pyb.Timer(4, freq=10)
speed_timer.callback(isr_speed_timer)

#-------  END of Interrupt Section  ----------

speed = int((pot.read()-2048)*200/4096)
#pyb.delay(100)
#path = "./dance_moves.txt"
dance_array = []

def make_array():
    with open("dance_moves.txt", 'r') as f:
        while chars is not ".":
            #put all dance moves in one line
            #use f.read() so a each element is one character
            chars = f.read(1);
            dance_array.append(chars)

def dance(i): 
    for i in range(i):
        if (dance_array[i] == "F"):
            A_forward(speed)
            B_forward(speed)
        elif (dance_array[i] == "B"):
            A_back(abs(speed))
            B_back(abs(speed))
        elif (dance_array[i] == "L"):
            A_forward(speed)
            B_speed(0)
        elif (dance_array[i] == "R"):
            A_forward(0)
            B_speed(speed)
        else:
            break

while True:				# Main program loop
	for i in range(len(dance_array)):
		if audio.buffer_is_filled():		# semaphore signal from ISR - set if buffer is full
			E = audio.inst_energy()			# fetch instantenous energy
			audio.reset_buffer()			# get ready for next epoch
			sum_energy = sum_energy - e_buf[e_ptr] + E
			e_buf[e_ptr] = E			# over-write earliest energy with most recent
			e_ptr = (e_ptr + 1) % M		# increment e_ptr with wraparound - 0 to M-1
			average_energy = sum_energy/M
			c = E/average_energy
			make_array()
			if (pyb.millis()-tic > MIN_BEAT_PERIOD):	# if longer than minimum period
					if (c>BEAT_THRESHOLD):		# look for a beat
						dance(i)
						tic = pyb.millis()		# reset tic
			buffer_full = False				# reset status flag


