from setuptools import find_packages
from setuptools import setup

setup(
    name='motor_temperature_pkg',
    version='0.1.0',
    packages=find_packages(
        include=('motor_temperature_pkg', 'motor_temperature_pkg.*')),
)
