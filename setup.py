from setuptools import setup, find_packages

package_name = 'gestures_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['gestures_package', 
'gestures_package.*']),

    ##py_modules=[
    #    'gestures_package.gesture_controller',
    #    'gestures_package.gesture_tracker',
    ##],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', 
['launch/gesture_launch.launch.py']),
    ],
    install_requires=[
        'setuptools',
        'depthai',
        'mediapipe',
        'opencv-python',
        # Add other dependencies as needed
    ],
    zip_safe=True,
    maintainer='root',
    maintainer_email='djnighti@ucsd.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gesture_tracker = gestures_package.gesture_tracker:main',
            'gesture_controller = 
gestures_package.gesture_controller:main',
        ],
    },
)
