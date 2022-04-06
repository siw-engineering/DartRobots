from skbuild import setup

setup(
    name="Underdog",
    version="0.1.0",
    description="Project 1",
    author="Poh Zhi Ee",
    license="MIT",
    packages=['Underdog'],
    cmake_args=['-DPACKAGE_PYTHON=TRUE'],
    package_dir={'': ''},
    python_requires='>=3.8'
)
