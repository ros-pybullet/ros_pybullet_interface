from setuptools import setup

package_name = 'ros_pybullet_interface'

setup(
    name=package_name,
    version='3.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Christopher E. Mower',
    maintainer_email='christopher.mower@kcl.ac.uk',
    description='The ROS-PyBullet Interface implementation for ROS2.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rpbi_node = ros_pybullet_interface.rpbi_node:main',
            'dynamic_object_node = ros_pybullet_interface.dynamic_object_node:main',
            'visual_object_node = ros_pybullet_interface.visual_object_node:main',
        ],
    },
)
