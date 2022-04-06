from skbuild import setup

setup(
    name="DartRobots",
    version="0.1.0",
    description="A collection of robots using dartsim",
    author="Poh Zhi Ee",
    license="MIT",
    packages=['DartRobots'],
    cmake_args=['-DPACKAGE_PYTHON=TRUE'],
    package_dir={'': ''},
    python_requires='>=3.8'
)
