from simple_launch import SimpleLauncher
from launch.substitutions import Command
from os import system

sl = SimpleLauncher()

sl.declare_arg('device', default_value=0, description='video device')
sl.declare_arg('width', default_value=640, description='image width')
sl.declare_arg('height', default_value=480, description='image height')

# on picam the camera is rotated
sl.declare_arg('rotate', default_value=270, description='image rotation')

def launch_setup():

    sl.node('v4l2_camera', 'v4l2_camera_node', parameters = sl.arg_map('device', 'width', 'height','rotate'))

    # also, run the PWM
    sl.node('picam1dof', 'pwm.py')

    return sl.launch_description()


generate_launch_description = sl.launch_description(opaque_function=launch_setup)
