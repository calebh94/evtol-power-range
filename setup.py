#!/usr/bin/env python

"""
Ref: https://github.com/argoai/argoverse-api/blob/master/setup.py
A setuptools based setup module.
See:
https://packaging.python.org/en/latest/distributing.html
https://github.com/pypa/sampleproject
"""

import platform
import sys
from codecs import open  # To use a consistent encoding
from os import path

# Always prefer setuptools over distutils
from setuptools import find_packages, setup

here = path.abspath(path.dirname(__file__))

# Get the long description from the README file
with open(path.join(here, "README.md"), encoding="utf-8") as f:
    long_description = f.read()

setup(
    name="evtol-power",
    version="0.0.2",
    description="eVTOL Vehicle Power Calculations for Trajectory Optimization",
    long_description=long_description,
    url="https://github.com/calebh94/evtol-power-range",
    author="Caleb Harris",
    author_email="harris.caleb84@gmail.com",
    license="MIT",
    classifiers=[
        "Development Status :: 5 - Production/Stable",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Topic :: Scientific/Engineering :: Trajectory Optimization",
    ],
    keywords="vtol, battery, power, trajectory optimization",
    package_dir={"": "src"},
    packages=find_packages(where="src"),
    python_requires=">= 3.5",
    install_requires=["pytest"],
)
