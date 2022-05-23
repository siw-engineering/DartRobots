# Requirements

- Ubuntu 20.04
- conda env with python3.8 only

# Installation

pip install the wheel provided

# Design

## Concept

DartRobots is meant to provide a simple API for robots to be used in a python script
without exposing a lot of details of dart, while still maintaining some level of
efficiency when running to facilitate fast reinforcement learning training times.

Currently, it is split into mainly the World and Robot, with MiniCheetah being the
only implemented robot. The Robot should represent a robot that has kinematics,
dynamics and is controllable through its joints. The World should represent a
simulation instance with rendering capabilities including manipulating markers,
which also has ownership of terrain and robot.

## PImpl

Public headers in the main include folder will have no data members for the classes.
This is to ensure fewer dependencies need to be installed for downstream users.
For example, implementation details such as a datastruct of a certain library will
not be present in the public header, and as such the library does not need
to be included, and as long as the library is statically linked into the final binary
of DartRobots, the library does not need to be installed for any downstream users.

The private includes and implementations can be found in src/detail folder.


