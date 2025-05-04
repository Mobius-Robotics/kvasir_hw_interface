kvasir_hw_interface
=================

The `kvasir_hw_interface` package provides a `ros2_control` hardware interface
for our Kvasir robot. It communicates with a Nucleo-H503RB over a serial
connection.

> [!IMPORTANT]
> This repository contains only the hardware interface for Kvasir. The robot's description and other related files are maintained in the [`kvasir_description`](https://github.com/Mobius-Robotics/kvasir_description) repository under the Mobius-Robotics organization.

Acknowledgements
----------------

This package is based on the `diffdrive_arduino` package (see the repository from which this is forked from) and the `diffbot` example from the [ros2_control demos](https://github.com/ros-controls/ros2_control_demos/tree/master/example_2). Big thanks to the original authors for their excellent work, which provided a solid starting point for our interface.
