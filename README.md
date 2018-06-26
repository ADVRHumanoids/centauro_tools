# centauro_tools
This package provides tools for the control of the centauro robot!

## SimpleWheeledMotion
*CartesianInterface* implementation for the wheeled motion of the Centauro robot.
It supports:
 - control of waist frame X/Y/Z/rotZ through wheeled motion
 - control of wheels position X/Y w.r.t. waist frame
 - control of hands w.r.t. waist frame

## TerrainAdatpationPoses
Python script that sends references to the CartesianInterface *SimpleWheeledMotion* plugin 
in order to simultaneously change the support polygon shape as well as the waist height.
Due to Centauro's hip yaw joint limitations, it works best from a *spider-like* homing. 
The script `TerrainAdaptationStateMachine.py` consists of a basic *smach* state machine. 
From the `IDLE` state, the user can send three commands:
 - `up_stretch` 
 - `down_mid`
 - `down_low`

in order to change Centauro's support polygon and shape.
For **tuning such motions**, refer to the `TerrainAdaptationPoses.py` script.
In general, all motions are achieved by sending suitable goal messages to the CartesianInterface 
*ROS action server*; so, they can be easily incorporated in external python/c++ pieces of code (have 
a look at the code!)

**NOTE** to change the homing configuration, go to the current *SRDF file* and modify the *home* group state.
The "spider-like" homing can usually be found right after the mammal one (commented out).
Remember to apply the change both on the robot embedded pc AND on the control pc.
