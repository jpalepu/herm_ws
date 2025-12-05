from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'herm_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install URDF files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jithin',
    maintainer_email='jithink385@gmail.com',
    description='HERM robot URDF + description package',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [],
    },
)
