# centauro_tools
This package provides tools for the control of the centauro robot!

## LegMovementAction Server
A ROS node implementing a server for a LegMovementAction. The easiest way of
building this package is to use the advr-superbuild build system, which will
take care of downloading and installing all required dependencies.
Before running the node, make sure that the NRT domain is able to control the robot
by turning on the XBotCommunicationPlugin.

```
rosservice call /XBotCommunicationPlugin_switch 1
```