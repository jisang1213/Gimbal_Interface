## Usage

The Gimbal class sets up the serial port and provides the methods to get the transformation matrix from the camera frame to the gimbal base frame.
The port name should be passed to its constructor.

```cpp
// initialiazation
Gimbal gimbal(portname);

// returns H matrix
gimbal.getHmatrix_L(); //for left camera
gimbal.getHmatrix_R(); //for right camera
```

The gimbal_com node is responsible for the actual serial communication.
It is subscribed to the "/gimbal_command" topic from which it receives roll/pitch/yaw commands (in the base frame) and publishes joint states to the "/gimbal_state" topic. It also updates the state in the Gimbal instance.
A pointer to the gimbal instance should be passed to its constructor.

```cpp
auto gimbal_com = std::make_shared<gimbal_com>(&gimbal);
```

The commander node publishes commands to the "/gimbal_command" topic. There are two ways it generates commands.
The first way is using a zero target (passive). The second way is by using the robot state (active).
The second way is preferable but it requires a world pointer or some way to get the robot state.

A custom node that publishes commands to the "/gimbal_command" topic can be implemented as well.

```cpp
// use this public method to set the world
void setWorld(raisim::World * world);

// alternatively, make a new method to get the robot pointer:
void setRobot(raisim::ArticulatedSystem * robot);
```
**UPDATE: The updated approach is to get the robot state directly through a topic and to send the H-matrix as an array of 12 doubles through another topic
