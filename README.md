![](images/gimbal_raisim.png)
## Overview

This is a standalone ros2 node that is designed to run independently from raisin.
It receives the robot state through the custom "robotstate" message and it sends the H matrix back to raisin through the "DoubleArray12" message.

# commander node
The commander node receives commands from raisin and publishes them to the "/gimbal_command" topic. There are two ways it generates commands.

# gimbal_com node
The gimbal_com node is responsible for the actual serial communication.
It is subscribed to the "/gimbal_command" topic from which it receives roll/pitch/yaw commands (in the base frame) and publishes the information necessary for coordinate transformation to the "/gimbal_state" topic.
The port name should be passed to its constructor.

```cpp
std::string portname = "/dev/ttyACM0";
auto gimbal_com = std::make_shared<gimbal_com>(portname);
```

The "gimbal" member of the gimbal_com node sets up the serial port and provides the methods to get the transformation matrix from the camera frame to the gimbal base frame.

```cpp
// first, the joint state is set
gimbal.setState(0.1,0.2,0.3);

// then we use these methods to get the transfomation matrix
gimbal.getHmatrix_L(); //for left camera
gimbal.getHmatrix_R(); //for right camera
```

The H matrix is published to the "railab_raibo/gimbal_state" topic as an array of 12 doubles.
The first 9 elements are the rows of the rotation matrix [R1 R2 R3]. The remaining three elements represent the position vector of the cameras with respect to the origin of the base [x y z].

The 4x4 H matrix is converted into an array as follows:
```cpp
//rotation
for(int i=0; i<3; i++){
    for(int j=0; j<3; j++){
        output_msg->transform_L[3*i + j] = H_matrix_left_cam(i,j);
        output_msg->transform_R[3*i + j] = H_matrix_right_cam(i,j);
    }
}

//position
for(int i=0; i<3; i++){
    output_msg->transform_L[9+i] = H_matrix_left_cam(i,3);
    output_msg->transform_R[9+i] = H_matrix_right_cam(i,3);
}
```
The origin of the base is the midpoint of the line connecting the two mounting holes.
![Base Origin](images/base_origin.png)
