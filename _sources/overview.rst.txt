Overview
========

Reliable contact simulation is a key requirement for the community developing (semi-)autonomous robots.
Simulation, data collection, and straight-forward interfacing with robot hardware and multi-modal sensing will enable the development of robust machine learning algorithms and control approaches for real world applications.
The ROS-PyBullet Interface is a framework that provides a bridge between the popular Robot Operating System (ROS) with a reliable impact/contact simulator Pybullet.
Furthermore, this framework provides several interfaces that allow humans to interact with the simulator that facilitates Human-Robot Interaction in a virtual world.
Tutorial slides found `here <https://docs.google.com/presentation/d/1c7aYdl0kzYztaJyFgqGP7S1EfMuim9CGwq5VjEvhiQ8/edit?usp=sharing>`_.

The main features of the framework is summarized as follows.

1. Online, full-physics simulation of robots using a reliable simulator. The framework uses Pybullet to enable well founded contact simulation.
2. Integration with ROS ecosystem. Tracking the state of the world using Pybullet is integrated with ROS so that users can easily plugin their work in a similar way that a real system might be controlled.
3. Several interfaces for humans. Providing demonstrations from humans requires an interface to the simulated environment. In addition, utilizing a haptic interface the human can directly interact with the virtual world.
4. Modular and extensible design. Our proposed framework adopts a modular and highly extensible design paradigm using Python. This makes it easy for practitioners to develop and prototype their methods for several tasks.
5. Data collection with standard ROS tools. Since the framework provides an interface to ROS we can use common tools for data collection such as ROS bags and ``rosbag_pandas``.
6. Easily integrates with hardware. Tools are provided to easily remap the virtual system to physical hardware.



