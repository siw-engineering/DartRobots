from skbuild import setup
from setuptools import find_packages

setup(
    name="DartRobots",
    version="0.5",
    description="A collection of robots using dartsim",
    authors="Poh Zhi Ee, Mohammed Ibrahim",
    license="MIT",
    packages=find_packages(),
    cmake_args=['-DPACKAGE_PYTHON=TRUE'],
    python_requires='>=3.8'
)
