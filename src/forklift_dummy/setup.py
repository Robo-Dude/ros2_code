from setuptools import find_packages, setup

package_name = 'forklift_dummy'
submodules = "forklift_dummy/submodules"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros2',
    maintainer_email='ros2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = forklift_dummy.sample:main',
            'client = forklift_dummy.client:main',
            'controller = forklift_dummy.controller_m:main',
            'autocontroller = forklift_dummy.controller_p:main',
            'depth_paranoma = forklift_dummy.depth_cam:main',
            'cam_depth = forklift_dummy.cam_auto:main',
            'setpoint = forklift_dummy.auto_setpoint:main',
            'euler = forklift_dummy.euler_angle:main',
        ],
    },
)
