dynaManager
===========

Processing/Java application to set ID/Baud of DYNAMIXEL servos for InterbotiX Robot Kits


This application can be built with Processing 2.0
https://processing.org/download/

This application relies on the controlP5 library
http://www.sojamo.de/libraries/controlP5/

Binary releases can be found here
https://github.com/trossenrobotics/dynaManager/releases

Computer Compatibility:
ArmControl can be used on any system that supports
  1)Java
  2)Processing 2.0
  3)Java serial library (Included for Mac/Windows/Linux with processing 2.0)
ArmControl has been tested on the following systems
  1)Windows XP, Vista, 7, 8
  2)Mac 10.6+
  3)Linux

The dynaManager is a small application that will set the ID and Baud of various DYNAMIXEL servos. To do this, it requires an ArbotiX robocontroller
http://www.trossenrobotics.com/p/arbotix-robot-controller.aspx
running the ROS sketch. 

Instructions for setting up the ArbotiX can be found here
http://learn.trossenrobotics.com/arbotix/7-arbotix-quick-start-guide
Instructions for using the Dynamanager
http://learn.trossenrobotics.com/arbotix/1-using-the-tr-dynamixel-servo-tool#&panel1-1


The dynaManager will send serial commands to the ArbotiX to scan, set, and test DYNAMIXEL servos. Ids are user defined, but the baud rate will always be set to 1000000.


Currently the dynaManager will only scan the 1000000 and 576000 baud. It will scan for servo IDs between 0 and 252.

