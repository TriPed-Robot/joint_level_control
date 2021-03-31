#ifndef SWING_SENSOR_ROS
#define SWING_SENSOR_ROS

#define LEFT_SWING_SENSOR_ID 1
#define RIGHT_SWING_SENSOR_ID 2

// reads angle from a swing sensor, with given ID
double readSwingAngle(int sensorID);

#endif