#!/usr/bin/python3
try:
    import pigpio
except:
    import sys
    sys.exit(0)

def interp(x, xm, xM, ym, yM):
    return ym + (x-xm)*(yM-ym)/(xM-xm)

import rclpy
from rclpy.node import Node
from picam1dof.msg import Cmd

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
        self.cmd_time = 0
        self.watchdog = self.create_timer(2, self.stop_delay)
        
        self.pwm = None
        
    def cmd_callback(self, msg):
        
        if self.pwm is None:
            self.pwm = pigpio.pi()
        self.cmd_time = self.time_sec()
        self.cmd = msg
        
    def saturate(self):
        if self.angle < self.angle_min:
            self.angle = self.angle_min
        elif self.angle > self.angle_max:
            self.angle = self.angle_max            
            
    def time_sec(self):
        return self.get_clock().now().seconds_nanoseconds()[0]

    def move(self): 
        
        if self.pwm is None:
            return
        
        # take in last cmd
        if self.cmd.mode == self.cmd.POSITION:
            self.angle = self.cmd.cmd
        else:
            self.angle += self.dt * self.cmd.cmd
            
        self.saturate()                
        self.pwm.set_servo_pulsewidth(self.pwm_pin, 
                                      interp(self.angle, self.angle_min, self.angle_max, self.pwm_min, self.pwm_max))
        
    def stop_delay(self):
        if self.pwm is not None and self.time_sec() - self.cmd_time > 2:
            self.pwm.set_servo_pulsewidth(self.pwm_pin,0)
            self.pwm.stop()
            self.pwm = None
        
def main(args=None):
    rclpy.init(args=args)
    
    node = PWMNode()    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
