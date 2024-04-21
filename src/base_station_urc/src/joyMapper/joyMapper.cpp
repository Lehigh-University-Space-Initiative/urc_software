#include "stdio.h"

#include "ros/ros.h"
#include <sstream>
#include "cross_pkg_messages/ManualDriveCMD.h"
#include "cross_pkg_messages/RoverComputerDriveCMD.h"
#include "sensor_msgs/Joy.h"

// #include "cs_libguarded/cs_libguarded.h"

#define TESTING_MODE 0

ros::Publisher manualDrive_pub;
ros::Publisher manualArm_pub;


// geometry_msgs::Vector3 differentialDriveMapper(double driveAxis, double turnAxis) {

// }

bool gotA0MSG = false;
bool gotA1MSG = false;


int op_mode;

sensor_msgs::Joy last0MSG;
sensor_msgs::Joy last1MSG;

void joy0Callback(const sensor_msgs::JoyConstPtr& msg) {
   gotA0MSG = true;
   last0MSG = *msg;
}
void joy1Callback(const sensor_msgs::JoyConstPtr& msg) {
   gotA1MSG = true;
   last1MSG = *msg;
}

void sendCMD() {
   if (!gotA0MSG || !gotA1MSG) return;

   float sensitivity = 0.6; //0.40f;
   // float sensitivity_turn = 0.4; //0.40f;


   cross_pkg_messages::ManualDriveCMD cmd;
   #if TESTING_MODE
   //test in a sin wive for x and cos wave for y
   float mag = 0.4f;
   float period = 10.0f;

   // cmd.value.x = 0.4f;
   // cmd.value.y = 0.4f;
   cmd.value.x = mag * sin(ros::Time::now().toSec() * 2.0f * M_PI / period);
   cmd.value.y = mag * cos(ros::Time::now().toSec() * 2.0f * M_PI / period);

   #else

   // float diff = abs(last0MSG.axes[1] - last1MSG.axes[1]);

   // float turn_t_val = diff / ((last0MSG.axes[1] + last1MSG.axes[1]) / 2);

   // float sensitivity = sensitivity_streight + ((sensitivity_turn - sensitivity_streight) * turn_t_val);

   cmd.value.x = last0MSG.axes[1] * sensitivity;
   //temp inverting before fixing motroctr
   cmd.value.y = last1MSG.axes[1] * -sensitivity;
   #endif
   cmd.value.z = 0; // this is ignored

   // send the command
   if (!op_mode) {
      manualDrive_pub.publish(cmd);
   }
   // ROS_INFO("sending msg");

   if (op_mode)
   {
      cross_pkg_messages::RoverComputerDriveCMD armCMD;
      // set the CMD_L x y z to the last0MSG axes 0 1 2
      // update, x is pitch, y = roll, z = yaw
      armCMD.CMD_L.x = last0MSG.axes[1];
      armCMD.CMD_L.y = last0MSG.axes[0];
      armCMD.CMD_L.z = last0MSG.axes[2];
      // set the CMD_R x y z to the last1MSG axes 0 1 2
      armCMD.CMD_R.x = last1MSG.axes[1];
      armCMD.CMD_R.y = last1MSG.axes[0];
      armCMD.CMD_R.z = last1MSG.axes[2];

      manualArm_pub.publish(armCMD);
   }
}

int main(int argc, char** argv) {
    
   ros::init(argc, argv, "JoyMapper"); 

   ros::NodeHandle n;

   //TOOO: set rate to correct amount
   ros::Rate loop_rate(30); 

   ROS_INFO("Joy Mapper is running");
   
   manualDrive_pub = n.advertise<cross_pkg_messages::ManualDriveCMD>("manualCommands", 10);
   manualArm_pub = n.advertise<cross_pkg_messages::RoverComputerDriveCMD>("manualArmControl", 10);

   //register callbacks 

   //the queue is set very small so the latest command is always used
   ros::Subscriber sub1 = n.subscribe("joy0", 1000, joy0Callback);
   ros::Subscriber sub2 = n.subscribe("joy1", 1000, joy1Callback);


   while (ros::ok()) {
      //proccsing

      ros::param::get("/op_mode", op_mode);

      sendCMD();

      //run loop
      ros::spinOnce();
      loop_rate.sleep();
   }

   return 0;
}