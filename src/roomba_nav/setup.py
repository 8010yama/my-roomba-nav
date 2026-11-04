from glob import glob
from setuptools import setup

package_name = 'roomba_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yamato Hiraiwa',
    maintainer_email='yamato.washizu8010@gmail.com',
    description='Self-made Roomba bringup and navigation',
    license='MIT',
)
