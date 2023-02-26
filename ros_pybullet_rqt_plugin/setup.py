from setuptools import setup
from glob import glob

package_name = 'ros_pybullet_rqt_plugin'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', ['resource/PyBulletInterfaceControls_layout.ui']),
        ('share/' + package_name, ['plugin.xml']),
        ('share/' + package_name + '/resource/images', glob('resource/images/*.png')),
        ('share/' + package_name + '/resource/images', glob('resource/images/*.svg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jan Kaniuka',
    maintainer_email='jasiek491@gmail.com',
    description='ROS2-based RQT plugin for ROS-PyBullet Interface.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros_pybullet_rqt_plugin = ros_pybullet_rqt_plugin.main:main',
        ],
    },
)

