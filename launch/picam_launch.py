from simple_launch import SimpleLauncher


def generate_launch_description():

    sl = SimpleLauncher()

    sl.declare_arg('width', default_value=800, description='image width')
    sl.declare_arg('height', default_value=600, description='image height')

    sl.node('camera_ros', 'camera_node',
            parameters = [sl.arg_map('width', 'height'),
                        {'camera_info_url': sl.find('picam1dof', 'picam.yaml')}],
            remappings={'image_raw': 'image', 'image_raw/compressed': 'image/compressed'})


    # also, run the PWM
    sl.node('picam1dof', 'pwm.py')

    return sl.launch_description()
