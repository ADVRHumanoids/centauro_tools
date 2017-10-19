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

Then run the module as `rosrun centauro_tools wholebody_action_server --config path_to_config_file`