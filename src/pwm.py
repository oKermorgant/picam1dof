#!/usr/bin/python3
def interp(x, xm, xM, ym, yM):
    return ym + (x-xm)*(yM-ym)/(xM-xm)

import rclpy
from rclpy.node import Node
from picam1dof.msg import Cmd
import pigpio

class PWMNode(Node):
    
    # motor constant
    angle_min = -1.2
    angle_max = 1.2
    pwm_min = 500
    pwm_max = 2500
    pwm_pin = 18

    def __init__(self):
        super().__init__('pwm')
        self.cmd_sub = self.create_subscription(
            Cmd,
            'angle_cmd',
            self.cmd_callback,
            10)
        self.cmd_sub
        self.cmd = Cmd()
             
        self.dt = 0.02  
        self.angle = 0.
        self.timer = self.create_timer(self.dt, self.move)
        
        self.pwm = pigpio.pi()
        
    def cmd_callback(self, msg):
        self.cmd = msg
        
    def saturate(self):
        if self.angle < self.angle_min:
            self.angle = self.angle_min
        elif self.angle > self.angle_max:
            self.angle = self.angle_max            

    def move(self):
        # take in last cmd
        if self.cmd.mode == self.cmd.POSITION:
            print('position mode')
            self.angle = self.cmd.cmd
        else:
            print('velocity mode')
            self.angle += self.dt * self.cmd.cmd
            
        self.saturate()                
        self.pwm.set_servo_pulsewidth(self.pwm_pin, 
                                      interp(self.angle, self.angle_min, self.angle_max, self.pwm_min, self.pwm_max))
        
def main(args=None):
    rclpy.init(args=args)
    
    node = PWMNode()    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
