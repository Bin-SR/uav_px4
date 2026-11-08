from setuptools import setup

package_name = 'px4_offboard_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='si',
    maintainer_email='1072235132@qq.com',
    description='PX4 Offboard takeoff, hover and landing controller.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'offboard_controller = px4_offboard_controller.offboard_controller:main',
        ],
    },
)
