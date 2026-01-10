Introduction To Veranda
=======================

What Is Veranda?
----------------

Congratulations! You are now the proud owner of the free software Veranda. Veranda is a 2-dimensional simulation tool which interfaces with other software through ROS 2 (Robot Operating System 2). Unlike Gazebo, which provides full 3D simulation, Veranda focuses on fast, efficient 2D simulation. Veranda is built mainly as a teaching and prototyping tool; it uses the Box2D physics engine and the Qt 5 toolkit, and can be run on any operating system that supports Qt 5 and ROS 2, including Ubuntu 24.04 and Windows. 

The goal of a Veranda simulation is to mimic a real environment which can be traversed by a virtual robot, producing sensor information as if the robot were in the real world. This will allow users to write and test control software which communicates with their robot through ROS without any access to hardware. Once the control code is ready to be run in the real world, the ROS channels can be redirected to the controllers and sensors on an actual robot. 

The simulation provided by Veranda is intended to be accurate enough that it looks realistic and can be used to test pathfinding algorithms. It is not intended to follow precise physics down to every minute detail. This should be taken into account when comparing the results of a simulation to the results produced by the real world.

Components of Veranda
---------------------

The Veranda project is comprised of one core piece of software and a number of small plugin packages. The core simulation provides the main application, including an object designer tool and the simulator itself as well as a virtual joystick which can be used to produce input for testing control code. Each plugin provides a piece that can be attached to an object in simulations. These objects can be built in the designer by assembling pieces added to the toolbox from plugins, and then saved in files to be used later or loaded directly into a simulation. This design allows for users to write their own plugin packages if they need a piece for a robot which is not provided by the project. A number of different pieces are included with the project to get you started if you just want to build and drive some basic bots.