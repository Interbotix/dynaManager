dynaManager
===========

Processing/Java application to set ID/Baud of DYNAMIXEL servos for InterbotiX Robot Kits

This application can be built with Processing 2.0
https://processing.org/download/

The dynaManager is a small application that will set the ID and Baud of various DYNAMIXEL servos. To do this, it requires an ArbotiX robocontroller
http://www.trossenrobotics.com/p/arbotix-robot-controller.aspx
running the ROS sketch. 

The dynaManager will send seriall commands to the ArbotiX to scan, set, and test DYNAMIXEL servos. 

Currently the dynaManager will only scan the 1000000 and 576000 baud. It will scan for servo IDs between 0 and 252.
