from setuptools import find_packages, setup

package_name = 'uav_rgbd_perception'

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
    maintainer='si',
    maintainer_email='1072235132@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'rgb_detector    = uav_rgbd_perception.rgb_detector_node:main',
            'depth_processor = uav_rgbd_perception.depth_processor_node:main',
        ],
    },
)
