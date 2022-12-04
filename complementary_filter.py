# Control the speed and direction by tilting the PyBench board
# one motor controlled with pitch angle and the other with roll angle
# display the speed of each motor (revolution per second) on the OLED display alongside the angles

# if angle over a certain amount, motor moves
# adjust duty cycle to change speed

import pyb
from pyb import LED, ADC, Pin, Timer
from oled_938 import OLED_938
from mpu6050 import MPU6050

# Create external interrupts for motorA Hall Effect Senor
import micropython
micropython.alloc_emergency_exception_buf(100)
from pyb import ExtInt

speed = 0
A_speed = 0
A_count = 0
B_speed = 0
B_count = 0

#-------  Section to set up Interrupts ----------
def isr_motorA(dummy):
    global A_count
    A_count += 1
    
def isr_motorB(dummy):
    global B_count
    B_count += 1
    
def isr_speed_timer(dummy):
    global A_count
    global A_speed
    global B_count
    global B_speed
    A_speed = A_count
    B_speed = B_count
    A_count = 0
    B_count = 0

motorA_int = ExtInt ('Y4', ExtInt.IRQ_RISING, Pin.PULL_NONE,isr_motorA)
motorB_int = ExtInt ('Y6', ExtInt.IRQ_RISING, Pin.PULL_NONE,isr_motorB)

# Create timer interrupts at 100 msec intervals
speed_timer = pyb.Timer(4, freq=10)
speed_timer.callback(isr_speed_timer)
#-------  END of Interrupt Section  ----------

# Define pins to control motor
A1 = Pin('X3', Pin.OUT_PP)
A2 = Pin('X4', Pin.OUT_PP)
PWMA = Pin('X1')
B1 = Pin('X7', Pin.OUT_PP)
B2 = Pin('X8', Pin.OUT_PP)
PWMB = Pin('X2')


# Configure timer 2 to produce 1KHz clock for PWM control
tim = Timer(2, freq = 1000)
motorA = tim.channel (1, Timer.PWM, pin = PWMA)
motorB = tim.channel (2, Timer.PWM, pin = PWMB)

# Define 5k Potentiometer
pot = pyb.ADC(Pin('X11'))

# I2C connected to Y9, Y10 (I2C bus 2) and Y11 is reset low active
i2c = pyb.I2C(2, pyb.I2C.MASTER)
devid = i2c.scan()

oled = OLED_938(
	pinout={"sda": "Y10", "scl": "Y9", "res": "Y8"},
	height=64, external_vcc=False, i2c_devid=i2c.scan()[0],
)
oled.poweron()
oled.init_display()
oled.draw_text(0,0, 'Challenge 2')
oled.display()


# motorA
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

# motorB
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

	
imu = MPU6050(1, False)


def pitch_estimate(pitch, dt, alpha):
    theta = imu.pitch()
    pitch_dot = imu.get_gy()
    pitch = alpha*(pitch + pitch_dot*dt) + (1-alpha)*theta
    return (pitch)

def roll_estimate(roll, dt, alpha):
    beta = imu.roll()
    roll_dot = imu.get_gx()
    roll = alpha*(roll + roll_dot*dt) + (1-alpha)*beta
    return (roll)

tic = pyb.millis()
alpha = 0.98
pitch = 0
roll = 0

while True:

    toc = pyb.millis() - tic

    filtered_pitch = pitch_estimate(pitch, toc, alpha)
    filtered_roll = roll_estimate(roll, toc, alpha)

    offset_pitch = filtered_pitch
    offset_roll = filtered_roll

    tic = pyb.millis()

    if (filtered_roll >= 0):		
        A_forward(filtered_roll)
    else:
        A_back(abs(filtered_roll))
        
    if (pitch >= 0):
        B_forward(speed)
    else:
        B_back(abs(speed))

	# Display new speed
    oled.draw_text(0,20,'Motor A:{:5.2f} rps'.format(A_speed/39))
    oled.draw_text(0,35,'Motor B:{:5.2f} rps'.format(B_speed/39))
    oled.draw_text(0,45,'Pitch:{:5.2f}'.format(offset_pitch))
    oled.draw_text(0,55,'Roll:{:5.2f}'.format(offset_roll))
    oled.display()
	#20, 35, 45
    pyb.delay(100)