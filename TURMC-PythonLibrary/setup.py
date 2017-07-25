#!/usr/bin/env python2

from setuptools import setup, find_packages

setup(name="TURMC-PythonLibrary",
      version="1.0.14",
      description="A library of code used by Temple Robotics team",
      author="Brian Amin, Sam Wilson",
      url="www.github.com/templerobotics/main",
      packages=find_packages(),
      #install_requires=['numpy', 'matplotlib', 'pigpio', 'pyserial'],
      #provides=['Phidgets']
     )
