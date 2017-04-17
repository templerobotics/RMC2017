#!/usr/bin/env python2

from setuptools import setup, find_packages

setup(name="TURMC-PythonLibrary",
      version="1.0.5",
      description="A library of code used by Temple Robotic's RMC team",
      author="Brian Amin, Sam Wilson",
      url="www.github.com/TURMC1617/main",
      packages=find_packages(),
      install_requires=['numpy', 'matplotlib', 'scipy', 'pigpio']
     )
