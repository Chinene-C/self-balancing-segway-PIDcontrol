import pyb
import math
import gc
from pyb import LED, USB_VCP, DAC, Timer, ADC, Pin, UART, ExtInt
from array import array
from oled_938 import OLED_938
from mpu6050 import MPU6050
from motor import DRIVE
import time
import micropython


# Define pins to control motor
A1 = Pin('X3', Pin.OUT_PP)		# Control direction of motor A
A2 = Pin('X4', Pin.OUT_PP)
PWMA = Pin('X1')				# Control speed of motor A
B1 = Pin('X7', Pin.OUT_PP)		# Control direction of motor B
B2 = Pin('X8', Pin.OUT_PP)
PWMB = Pin('X2')				# Control speed of motor B

#motor sensors
A_sense = Pin('Y4', Pin.PULL_NONE)
B_sense = Pin('Y6', Pin.PULL_NONE)

imu = MPU6050(1, False) 

# I2C connected to Y9, Y10 (I2C bus 2) and Y11 is reset low active
oled = OLED_938(pinout={'sda': 'Y10', 'scl': 'Y9', 'res': 'Y8'}, height=64,
                   external_vcc=False, i2c_devid=60)
oled.poweron()
oled.init_display()
oled.draw_text(0,20, 'Press USR button')
oled.display()

tim = Timer(2, freq = 1000)
motorA = tim.channel (1, Timer.PWM, pin = PWMA)
motorB = tim.channel (2, Timer.PWM, pin = PWMB)

def A_forward(value):
    A1.high()
    A2.low()
    motorA.pulse_width_percent(value)

def A_backward(value):
    A2.high()
    A1.low()
    motorA.pulse_width_percent(value)

def A_stop():
    A1.low()
    A2.low()

def B_forward(value):
    B1.low()
    B2.high()
    motorB.pulse_width_percent(value)

def B_backward(value):
    B2.low()
    B1.high()
    motorB.pulse_width_percent(value)

def B_stop():
    B1.low()
    B2.low()


# def translate(value, leftMin, leftMax, rightMin, rightMax):
#     # Figure out how 'wide' each range is
#     leftSpan = leftMax - leftMin
#     rightSpan = rightMax - rightMin

#     # Convert the left range into a 0-1 range (float)
#     valueScaled = float(value - leftMin) / float(leftSpan)

#     # Convert the 0-1 range into a value in the right range.
#     return rightMin + (valueScaled * rightSpan)


A_state = 0
A_speed = 0
A_count = 0
B_state = 0
B_speed = 0
B_count = 0


def isr_motorA(dummy):
    global A_count
    A_count += 1

def isr_speed_timerA(dummy):
    global A_count
    global A_speed
    A_speed = A_count
    A_count = 0

def isr_motorB(dummy):
    global B_count
    B_count += 1

def isr_speed_timerB(dummy):
    global B_count
    global B_speed
    B_speed = B_count
    B_count = 0


motorA_int = ExtInt ('Y4', ExtInt.IRQ_RISING, Pin.PULL_NONE, isr_motorA)
motorB_int = ExtInt ('Y6', ExtInt.IRQ_RISING, Pin.PULL_NONE, isr_motorB)

micropython.alloc_emergency_exception_buf(100)

speed_timerA = pyb.Timer(4, freq=10)
speed_timerA.callback(isr_speed_timerA)
speed_timerB = pyb.Timer(5, freq=10)
speed_timerB.callback(isr_speed_timerB)


def pitch_estimate(pitch, dt, alpha):
    theta = imu.pitch()
    pitch_dot = imu.get_gy()
    pitch = alpha*(pitch + pitch_dot * dt) + (1 - alpha) * theta
    return (pitch, pitch_dot)

def read_imu(dt):
    global g_pitch
    alpha = 0.9    # larger = longer time constant
    pitch = int(imu.pitch())
    pitch_dot = imu.get_gy()
    roll = int(imu.roll())
    g_pitch = alpha*(g_pitch + imu.get_gy()*dt*0.001) + (1-alpha)*pitch
	# show graphics
    #oled.clear()
    #oled.draw_text(2,0, 'Pitch: {:5.2f}'.format(g_pitch))
    
    #oled.display()
    return (g_pitch, pitch_dot)



def driver(input):
    if input > 0:
        A_forward(input*0.9)
        B_forward(input)

    elif input < 0:
        A_backward(-input*0.9)
        B_backward(-input)




class PID:
    """
    Discrete PID control
    """

    def __init__(self, P=3., I=0.01, D=0.0):

        self.Kp=P
        self.Ki=I
        self.Kd=D

        self.I_value = 0
        self.P_value = 0
        self.D_value = 0

        self.I_max=75
        self.I_min=-75

        self.set_point=4.88

        self.prev_value = 0

        self.output = float(0) #convert an integer/string to a floating-point value

        self.last_update_time = pyb.millis()

    

    def update(self, input):

        #added -4 to account for error
        self.input = input

        if pyb.millis()-self.last_update_time > 0:

            """
            Calculate PID output value for given reference input and feedback
            """
            current_value = self.input
            self.error = self.set_point - current_value
            print('Pitch '+str(current_value))
            print('SP '+str(self.set_point))

            self.P_value = self.Kp * self.error
            self.D_value = self.Kd * (current_value-self.prev_value)

            self.prev_value = current_value

            lapsed_time = pyb.millis()-self.last_update_time
            lapsed_time/=1000. #convert to seconds
            self.last_update_time = pyb.millis()


            self.I_value += self.error * self.Ki

            if self.I_value > self.I_max:
                self.I_value = self.I_max
            elif self.I_value < self.I_min:
                self.I_value = self.I_min

            self.output = self.P_value + self.I_value + self.D_value

            if self.output<-100:
                self.output = -100
            if self.output>100:
                self.output = 100.0

            print("Setpoint: "+str(self.set_point))
            print("P: "+str(self.P_value))
            print("I: "+str(self.I_value))
            print("D: "+str(self.D_value))
            print("Output: "+str(self.output))

            # self.output = (self.output/100.0)

            self.last_update_time=pyb.millis()

            # print(type(self.output))
            driver(self.output)
        return self.output

print('Performing Milestone 1')
print('Waiting for button press')
trigger = pyb.Switch()

# scale = 2.0
# pot = pyb.ADC(Pin('X11'))

# while not trigger():
#     time.sleep(0.001)
#     K_p = pot.read() * scale / 4095
#     oled.draw_text(0, 30, 'Kp = {:5.2f}'.format(K_p))
#     oled.display()

# while trigger(): pass

# while not trigger():
#     time.sleep(0.001)
#     K_i = pot.read() * scale / 4095
#     oled.draw_text(0, 40, 'Ki = {:5.2f}'.format(K_i))
#     oled.display()

# while trigger(): pass

# while not trigger():
#     time.sleep(0.001)
#     K_d = pot.read() * scale / 4095
#     oled.draw_text(0, 50, 'Kd = {:5.2f}'.format(K_d))
#     oled.display()

# while trigger(): pass



while not trigger():
    time.sleep(0.001)

while trigger(): pass
print('Button Pressed - Running')

g_pitch = 0   


segwayDrive = PID(8.08, 4.65, 0.83)
tic = pyb.millis()

while True:
    toc = pyb.millis()
    time = toc-tic
    inputPitch = read_imu(time)
    time_int = time/1000
    segwayDrive.update(inputPitch[0])
    tic = pyb.millis()
    pyb.delay(80)