from simple_launch import SimpleLauncher
from launch.substitutions import Command
from os import system

sl = SimpleLauncher()

sl.declare_arg('device', default_value=0, description='video device')
sl.declare_arg('width', default_value=640, description='image width')
sl.declare_arg('height', default_value=480, description='image height')

# on picam the camera is rotated
sl.declare_arg('rotation', default_value=270, description='image rotation')

def launch_setup():

    # rotate camera beforehand
    system(f'v4l2-ctl -c rotate={sl.arg("rotation")}')


    sl.node('image_tools', 'cam2image', parameters = sl.arg_map('device', 'width', 'height'), arguments=['--ros-args', '--log-level', 'warn'])

    # also, run the PWM
    sl.node('picam1dof', 'pwm.py')

    return sl.launch_description()


return sl.launch_description(opaque_function=launch_setup)
