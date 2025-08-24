from setuptools import find_packages, setup

package_name = 'fire_tracks'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='han',
    maintainer_email='lhg9033@naver.com',
    description='cmd_vel ramp (0.5s) → 6-wheel velocities bridge',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmdvel_ramp_to_tracks = fire_tracks.cmdvel_ramp_to_tracks:main',
            'cmd_vel = fire_tracks.cmd_vel:main',
        ],
    },
)
