import pigpio
import time
from numpy import sin
import sys

PWM_PIN = 18
pwm = pigpio.pi()

amp = 80
T = 6
dt = 0.02

def interp(x, xm, xM, ym, yM):
    return ym + (x-xm)*(yM-ym)/(xM-xm)

def angle_to_cycle(a):
    a_min = -90
    a_max = 90
    c_min = 500
    c_max = 2500
    ret = interp(a, a_min, a_max, c_min, c_max)
    print('Angle {} -> PWM {}'.format(a, ret))
    return ret

def time_to_angle(t, T):
    if t < T/2:
        return interp(t, 0, T/2, -amp, amp)
    return interp(t, T/2, T, amp, -amp)


if '-s' in sys.argv:
    pwm.set_servo_pulsewidth(PWM_PIN, 0)
    time.sleep(2)
    sys.exit(0)


t = 0
while True:
    t += dt
    if t > T:
        t -= T
    pwm.set_servo_pulsewidth(PWM_PIN, angle_to_cycle(time_to_angle(t, T)))
    #pwm.set_servo_pulsewidth(PWM_PIN, angle_to_cycle(amp*sin(6.28*t/T)))
    time.sleep(dt)

