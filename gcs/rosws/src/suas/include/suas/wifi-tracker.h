/*****************************************************************************
 ********************************** LICENSE **********************************
 *****************************************************************************
 * Copyright 2016 Team Rakshak, IIT Bombay                                   *
 *                                                                           *
 * Licensed under the Apache License, Version 2.0 (the "License");           *
 * you may not use this file except in compliance with the License.          *
 * You may obtain a copy of the License at                                   *
 *                                                                           *
 *     http://www.apache.org/licenses/LICENSE-2.0                            *
 *                                                                           *
 * Unless required by applicable law or agreed to in writing, software       *
 * distributed under the License is distributed on an "AS IS" BASIS,         *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  *
 * See the License for the specific language governing permissions and       *
 * limitations under the License.                                            *
 *                                                                           *
 * File		: wifi-tracker.h                                             *
 * Author	: Kamal Galrani, Kunal Garg                                  *
 * Description	:                                                            *
 *****************************************************************************
 ******************************* DOCUMENTATION *******************************
 *****************************************************************************
 *                                                                           *
 *****************************************************************************/

#include <ros/ros.h>
#include <ros/time.h>
#include <csignal>
#include <communication/uart.h>

void Init();
void Reset(ros::NodeHandle& _nh);
void LoopPreCallback();
void LoopPostCallback();
void Shutdown(int signum);

int main(int argc, char **argv) {
  ros::init(argc, argv, "WiFi_Tracker");
  ros::NodeHandle _nh("suas");
  signal(SIGINT, Shutdown);
  ros::Rate loop_rate(1);

  Init();
  Reset(_nh);
  while(ros::ok()) {
    LoopPreCallback();
    ros::spinOnce();
    LoopPostCallback();
    loop_rate.sleep();
  }
  Shutdown(0);

  return 0;
}







#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/tfMessage.h>
#include <tf/transform_broadcaster.h>

#define COMMUNICATION_TIMEOUT 5.0f

ros::Subscriber imu_sub, pos_sub;
double roll, pitch, yaw, t_x, t_y, t_z, set_yaw, set_pitch;
Serial arduino;
bool hasIMU;
ros::Time posUpdated;

void imuUpdateCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  tf::Quaternion latestOrientation_;
  tf::quaternionMsgToTF(msg->orientation, latestOrientation_);
  tf::Matrix3x3 mat(latestOrientation_);
  mat.getRPY(roll, pitch, yaw);
  roll = roll * 180 / 3.14159265359;
  pitch = pitch * 180 / 3.14159265359;
  yaw = yaw * 180 / 3.14159265359;
  hasIMU = true;
}

void posUpdateCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  t_x = msg->pose.position.x;
  t_y = msg->pose.position.y;
  t_z = msg->pose.position.z;
  posUpdated = ros::Time::now();
}

void Init() {
  while(arduino.connect("/dev/ttyACM1", B57600) < 0 && ros::ok()) { ROS_ERROR("Communication with Arduino failed! Retrying every second..."); usleep(1000000);}
}

void Reset(ros::NodeHandle& _nh) {
  roll = pitch = yaw = 0;
  t_x = t_y = 1;
  t_z = -50;
  hasIMU = false;
  posUpdated = ros::Time::now() - ros::Duration(COMMUNICATION_TIMEOUT);
  imu_sub = _nh.subscribe("/wifi/imu/data", 1, imuUpdateCallback);
  pos_sub = _nh.subscribe("/uas/global_position/local", 1, posUpdateCallback);
}

void LoopPreCallback() {
}

void LoopPostCallback() {
  //if (!hasIMU_) return;
  //ros::Duration timeSincePosUpdate = ros::Time::now() - posUpdated;
  //if (timeSincePosUpdate.toSec() > COMMUNICATION_TIMEOUT) {
  //  t_x = t_y = 0;
  //  t_z = 50;
  //}
//roll - contains current roll. same for pitch and yaw <wifi_tracker>
//t_x, t_y, t_z - contain position of aircraft relative to tracker
  set_pitch = (atan(t_z/sqrt(t_y*t_y + t_x*t_x)))*180/3.1415926539;
  set_yaw = atan2(t_y,t_x)*180/3.1415926539-yaw;
  set_yaw = 127+set_yaw/360*127;

  std::ostringstream ss;
  ss << "y" << set_yaw << "\n";
  ROS_INFO_STREAM(ss.str() << "::" << ss.str().length());
  arduino.put(ss.str().c_str(), ss.str().length());
  ss.str("");
  ss.clear();

  ss << "p" << pitch << "\n";
  ROS_INFO_STREAM(ss.str() << "::" << ss.str().length());
  arduino.put(ss.str().c_str(), ss.str().length());
  ss.str("");
  ss.clear();

}

void Shutdown (int signum) {
  ros::shutdown();
}
