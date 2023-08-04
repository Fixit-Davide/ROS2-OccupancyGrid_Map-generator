from setuptools import setup
import os

from glob import glob
from setuptools import setup

package_name = 'map_printer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Davide Graziato',
    maintainer_email='graziato.davide@outlook.it',
    description='print custom occupancy grid map',
    license='BSD',
    entry_points={
        'console_scripts': [
        'map_printer = map_printer.map_printer:main',
        ],
    },
)
