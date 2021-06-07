#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include "r2serial_driver/Num.h"
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define RMID 183
#define TMID 184
// From the input array, return the checksum
uint8_t GetcheckSum( short nPacketSize, uint8_t *byArray)
{

  uint8_t byTmp = 0;
  for(short i = 0; i < nPacketSize; i++ ) byTmp += *(byArray + i);
  return (~byTmp +1);

} //GetcheckSum
 
// Make data to send to driver 
uint8_t * Run(int16_t id, int16_t data_Ctrl) 
{
  
  static uint8_t data_send[8] = {0,0,0,0,0,0,0,0}; 

  data_send[0] = RMID;
  data_send[1] = TMID;
  data_send[2] = id;
  data_send[3] = 130; 
  data_send[4] = 2;
  data_send[5] = data_Ctrl;
  data_send[6] = data_Ctrl >> 8;
  data_send[7] = GetcheckSum(7,data_send);
  return data_send;

}//Run

uint8_t * Stop_Free(int16_t id) 
{
  static uint8_t data_send[8] = {0,0,0,0,0,0,0,0}; 

  data_send[0] = RMID;
  data_send[1] = TMID;
  data_send[2] = id;
  data_send[3] = 5; 
  data_send[4] = 1;
  data_send[5] = 1;
  data_send[6] = GetcheckSum(6,data_send); 
  return data_send;

}//Stop_Free

uint8_t * Break(int16_t id) 
{
  static uint8_t data_send[8] = {0,0,0,0,0,0,0,0}; 

  data_send[0] = RMID;
  data_send[1] = TMID;
  data_send[2] = id;
  data_send[3] = 6; 
  data_send[4] = 1;
  data_send[5] = 1;
  data_send[6] = GetcheckSum(6,data_send); 
  return data_send;

}//Break

#define Pi 3.1415926535
#define rad_rpm 9.5492965964254
#define L  0.255 // wheelbase (in meters per radian)
#define R  0.075 //wheel radius (in meters per radian)
#define v_max  100 // speed maximum of moter behind gear

float V;  // forward velocity (ie meters per second)
float W;  // angular velocity (ie radians per second)
float v_r; // clockwise angular velocity of right wheel (ie radians per second)
float v_l; // counter-clockwise angular velocity of left wheel (ie radians per second)
float w_r, w_l; // speed rad/s of one
int16_t Wheel_left, Wheel_right; // speed befor gear  

//Process ROS command message, send to uController
void cmd_velCallback(const geometry_msgs::Twist& msg)
{

  V = msg.linear.x;      W = msg.angular.z;

  /* Van toc goc 2 banh */
  w_r = ((2 * V) + (W * L)) / (2 * R);   //(rad/s)
  w_l = ((2 * V) - (W * L)) / (2 * R);   //(rad/s)
  
  /* Van toc 2 banh */
  v_r = w_r*rad_rpm;  // (rpm)  
  v_l = w_l*rad_rpm;  // (rpm) 

  /* Kiem  tra van toc */
  if(v_r > v_max) v_r = v_max;
  if(v_l > v_max) v_l = v_max;

  /* van toc truoc hop so */
  Wheel_right = (v_r/v_max)*3000; 
  Wheel_left =  (v_l/v_max)*3000; 

  ROS_INFO("Wheel left: %d  Wheel right: %d", Wheel_left, Wheel_right);
  
} //cmd_velCallback

int main(int argc, char **argv)
{
  /**
   Khoi tao Node 
   */
   
  ros::init(argc, argv, "MD200_control");
  ros::NodeHandle n;
  
  ros::Publisher uc0dComman;
  ros::Subscriber cmd_vel;

  ros::Rate loop_rate(50);

  /* Publisher */
  uc0dComman = n.advertise<std_msgs::String>("uc0Command", 1000);

  /* Subscriber */
  cmd_vel = n.subscribe("cmd_vel", 10,cmd_velCallback);

  while (ros::ok())
  {
  /**
    * This is a message object. You stuff it with data, and then publish it.
    */
    
    
    for(uint8_t i = 1 ; i<3;i++)
    {
      int16_t V_buff;

      if(i == 1) V_buff = - Wheel_left; 
      else if(i == 2) V_buff = Wheel_right;

      std_msgs::String msg;
      std::stringstream ss;
      uint8_t *a;

      if(fabs(V_buff)>3){
        a = Run(i,V_buff);
        for(uint8_t j = 0; j < 8 ; j++) ss << *(a + j);   
      } 

      else if(fabs(V_buff) < 3){
        a = Stop_Free(i);
        for(uint8_t j = 0; j < 7 ; j++) ss << *(a + j);   
      }

      else {
        a = Break(i);
        for(uint8_t j = 0; j < 7 ; j++) ss << *(a + j);   
      }   

      msg.data = ss.str();
      uc0dComman.publish(msg);
      loop_rate.sleep();
    }
      
  ros::spinOnce();
  }
   
   return 0;
}
