from simple_launch import SimpleLauncher
from launch.substitutions import Command
from os import system

def generate_launch_description():
    sl = SimpleLauncher()
    
    sl.declare_arg('device', default_value=0, description='video device')
    sl.declare_arg('width', default_value=640, description='image width')
    sl.declare_arg('height', default_value=480, description='image height')
    
    # on picam the camera is rotated
    sl.declare_arg('rotation', default_value=270, description='image rotation')
    rotation_cmd = sl.name_join("'", Command(SimpleLauncher.name_join('echo $(v4l2-ctl -c rotate=', sl.arg('rotation'), ')')), "'")
    
    # add rotation_cmd as an argument so that it is called before the node    
    cam_args = sl.arg_map(('device', 'width', 'height'))
    cam_args.update({'dummy_cmd': rotation_cmd})
    sl.node('image_tools', 'cam2image', parameters = [cam_args], arguments=['--ros-args', '--log-level', 'warn'])
    
    # also, run the PWM
    sl.node('picam1dof', 'pwm.py')
       
    return sl.launch_description()
