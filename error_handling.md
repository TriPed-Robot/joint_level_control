# Error handling
## Idea
Since wrong sensor readings can cause catastrophic failure of the Triped, it is important to find out when such incorrect readings occur and to deal with them accordingly.   
The general idea of the error handling is to check each step of the spi transfer from the beaglebone to the sensor for errors and to set the hardware interface into an error state, should the errors accumulate. 
The error types can also be weighted differently, depending on how critical they are in operation of the motors.  
Also, since higher level control blocks depend on the lower levels, the state of the lower levels need to be broadcastet. This is done via ros diagnostic topics.

## What is tracked
The error handling of the controller can be seen in the following diagram:
![error_handling_flowchart](https://raw.githubusercontent.com/TriPed-Robot/joint_level_control/main/docs/triped_error_handling.svg)

The error handling is present in each layer of the joint level controller. It begins in the lowest layer with the spi transfer. 
The first check is, whether the spi device can be opened. If this isn't the case, then a flag indicating so, will be set. This flag is called `SPI_DEVICE_ERROR` and is set within the `error` parameter of the readSwingAngle function, which is a class variable of the HallSensor class.
Furthermore the spi mode is set, read and if an error occurs the `SPI_MODE_ERROR` flag will be set. 
Then after the spi transfer, the recieved message is checked for a set error bit, indicating a wrong last command frame to the sensor, and leading to the `SPI_CMD_ERROR` flag being set. The message is also checked for correct parity, since a wrong parity indicates incorrectly transfered sensor values. A wrong parity is indicated by the `SPI_PARITY_ERROR` flag.

The next layer is the HallSensor class. There the returned counts of the sensor are checked for feasibility. This means, the sensor returns a 16383 or 16384 value, if it is not correctly connected to the beaglebone. Therefore a reading of said values leads to another error called `ANGLE_OUT_OF_BOUNDS_ERROR`. The hall sensor now calculates the sensor angle from the counts and gives it, together with all error flags, to the next higher layer.

Finally, in the read function of the SwingJoint class the returned sensor angle is checked against the joint limits. Should the angle not be within the limits, specified in the joints.yaml file, the `JOINT_LIMIT_ERROR` flag is set.

### Weighing the errors
The SwingJoint class contains 

## What isn't tracked

## What happens if an error occurs

## Customisation


The Documentation of this repository can be seen [here](https://triped-robot.github.io/joint_level_control/html/index.html).
