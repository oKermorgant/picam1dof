from simple_launch import SimpleLauncher
from launch.substitutions import Command

def generate_launch_description():
    sl = SimpleLauncher()
    
    sl.declare_arg('device', default_value=00)
    sl.declare_arg('width', default_value=640)
    sl.declare_arg('height', default_value=480)
    
    # on picam the camera is rotated
    sl.declare_arg('rotation', default_value=270)
    rotation_cmd = sl.name_join("'", Command(SimpleLauncher.name_join('v4l2-ctl -c rotate=', sl.arg('rotation'))), "'")
    
    # add rotation_cmd as an argument so that it is called before the node
    cam_args = sl.arg_map(('device', 'width', 'height'))
    cam_args.update({'dummy_cmd': rotation_cmd})
    sl.node('image_tools', 'cam2image', parameters = [cam_args], arguments=['--ros-args', '--log-level', 'warn'])
    
    # also, run the PWM
       
    
        
    return sl.launch_description()
