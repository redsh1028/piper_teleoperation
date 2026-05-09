from setuptools import find_packages, setup
import glob
import sys
import os
from glob import glob

package_name = 'piper'

python_version = f'{sys.version_info.major}.{sys.version_info.minor}'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Geownoo Cho',
    maintainer_email='gw.cho@wego-robotics.com',
    description='Package for Operating PiPER',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [      
            'piper_multi_ctrl = piper.piper_multi_ctrl_node:main',                
            'piper_single_ctrl = piper.piper_single_ctrl_node:main',          
        ],
    },    
)
