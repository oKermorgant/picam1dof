from RPi import GPIO
import time
from numpy import sin
from sys import argv, exit


GPIO.setmode(GPIO.BOARD)
GPIO.setup(12, GPIO.OUT, initial = GPIO.LOW)
F = 50
amp = 10
T = 2

def angle_to_cycle(a):
    a_min = -90
    a_max = 90
    c_min = 5.5 * F/50.
    c_max = 9.5 * F/50.
    return c_min + (a-a_min)*(c_max-c_min)/(a_max-a_min)

p = GPIO.PWM(12, F)
p.start(angle_to_cycle(0))

if '-s' in argv:
    p.stop()
    GPIO.cleanup()
    exit(0)

dt = 0.05

t = 0
while True:
    t += dt
    p.ChangeDutyCycle(angle_to_cycle(amp*sin(t/T)))
    time.sleep(dt)

