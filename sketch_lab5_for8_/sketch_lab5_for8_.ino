#define USE_USBCON
#include <Wire.h>
#include <Zumo32U4.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt16.h>
ros::NodeHandle nh;
void ros_handler( const geometry_msgs::Twist& cmd_msg) {
float x = cmd_msg.linear.x;
float y = cmd_msg.linear.y;
if(x == 1.0) forward(100);
if(x == -1.0) backward(100);
if(y == 1.0) left(100);
if(y == -1.0) right(100);
stop();
}
ros::Subscriber<geometry_msgs::Twist> sub("/zumo/cmd_vel", ros_handler);
Zumo32U4Motors motors;
void setup()
{
// Uncomment if necessary to correct motor directions:
//motors.flipLeftMotor(true);
//motors.flipRightMotor(true);
nh.initNode();
nh.subscribe(sub);
}
void forward(int time)
{
motors.setLeftSpeed(100);
motors.setRightSpeed(100);
delay(time);
}
void backward(int time)
{
motors.setLeftSpeed(-100);
motors.setRightSpeed(-100);
delay(time);
}
void left(int time)
{
motors.setLeftSpeed(-100);
motors.setRightSpeed(100);
delay(time);
}
void right(int time)
{
motors.setLeftSpeed(100);
motors.setRightSpeed(-100);
delay(time);
}
void stop()
{
motors.setLeftSpeed(0);
motors.setRightSpeed(0);
}
void loop()
{
nh.spinOnce();
delay(1);
}
