from skbuild import setup
from setuptools import find_packages

setup(
    name="DartRobots",
    version="0.2.5",
    description="A collection of robots using dartsim",
    author="Poh Zhi Ee",
    license="MIT",
    packages=find_packages(),
    cmake_args=['-DPACKAGE_PYTHON=TRUE'],
    python_requires='>=3.8'
)
