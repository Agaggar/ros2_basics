import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'turtle_brick'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('urdf/*')),
        (os.path.join('share', package_name), glob('config/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ayush Gaggar',
    maintainer_email='ayushgaggar2027@u.northwestern.edu',
    description='Control a robot in rviz to cathc a brick',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['turtle_robot = turtle_brick.turtle_robot:main', 
        'arena = turtle_brick.arena:main',
        'catcher = turtle_brick.catcher:main'
        ],
    },
)
