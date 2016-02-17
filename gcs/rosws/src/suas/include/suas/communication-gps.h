/*****************************************************************************
 ********************************** LICENSE **********************************
 *****************************************************************************
 * Copyright 2014 Mars Society India                                         *
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
 * File		: UBXMsgParser.h                                             *
 * Author	: Kamal Galrani                                              *
 * Description	:                                                            *
 *****************************************************************************
 ******************************* DOCUMENTATION *******************************
 *****************************************************************************
 *                                                                           *
 *****************************************************************************/

#include <ros/ros.h>
#include <ros/time.h>
#include <csignal>
#include <stdint.h>
#include <communication/uart.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <tf/transform_datatypes.h>
#include <tf/tfMessage.h>
#include <tf/transform_broadcaster.h>
#include <gps_common/conversions.h>
#include <Eigen/Dense>


void Init();
void Reset(ros::NodeHandle& _nh);
void LoopPreCallback();
void LoopPostCallback();
void Shutdown(int signum);

ros::Publisher  gps_odom_pub;
ros::Publisher  gps_fix_pub;
ros::Subscriber odom_sub;

//! @brief Signifies that we have an odometry message
//!
bool hasOdom_;
//! @brief Signifies that we have received an IMU message
//!
bool transformGood_;
//! @brief Timestamp of the latest good GPS message
//!
//! We assign this value to the timestamp of the odometry
//! message that we output
//!
uint32_t gpsUpdateTime_;
//! @brief Frame ID of the GPS odometry output
//!
//! This will just match whatever your odometry message has
//!
std::string worldFrameId_;
//! @brief Latest odometry data
//!
tf::Pose latestWorldPose_;
//! @brief Latest GPS data, stored as UTM coords
//!
tf::Pose latestUtmPose_;
//! @brief Latest IMU orientation
//!
tf::Quaternion latestOrientation_;
//! @brief Covariane for most recent GPS/UTM data
//!
Eigen::MatrixXd latestUtmCovariance_;
//! @brief Holds the UTM->odom transform
//!
tf::Transform utmWorldTransform_;
//! @brief Callback for the odom data
//!
void OdometryUpdateCallback(const nav_msgs::OdometryConstPtr& msg);
//! @brief Callback for the GPS fix data
//!
void computeTransform();
//! @brief Prepares the GPS odometry message before sending
//!
bool prepareGpsOdometry(nav_msgs::Odometry &gpsOdom);

Serial GPS;

void OdometryUpdateCallback(const nav_msgs::Odometry::ConstPtr& msg) {
 tf::quaternionMsgToTF(msg->pose.pose.orientation, latestOrientation_);
 tf::poseMsgToTF(msg->pose.pose, latestWorldPose_);
 worldFrameId_ = msg->header.frame_id;
 hasOdom_ = true;
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "UBXMsgParser");
  ros::NodeHandle _nh("rover");
  signal(SIGINT, Shutdown);

  gps_odom_pub = _nh.advertise<nav_msgs::Odometry>    ("/rover/sensors/gps/odom", 1);
  gps_fix_pub  = _nh.advertise<sensor_msgs::NavSatFix>("/rover/sensors/gps/fix",  1);

  Init();
  Reset(_nh);
  while(ros::ok()) {
    LoopPreCallback();
    ros::spinOnce();
    LoopPostCallback();
  }
  Shutdown(0);

  return 0;
}















bool verifyCK(std::string& msg) {
  const size_t len = msg.length();
  unsigned char CK_A = 0;
  unsigned char CK_B = 0;
  for (int i=0; i<len-2; i++) {
    CK_A = CK_A + msg[i];
    CK_B = CK_B + CK_A;
  }
  if (CK_A == msg[len-2] && CK_B == msg[len-1]) return true;
  else return false;
}

uint8_t ReceiveGPSNavMsg(std::string& msg, uint32_t& msgTime) {
  std::string str;
  uint8_t msgId;
  int16_t msgLen;

  while(ros::ok()) {
    GPS.fetchUntil(str, 0xB5);
    GPS.fetch(str, 2);
    if (str[0] == 0x62 && str[1] == 0x01) break;
  }
  GPS.fetch(str, 3);
  str.insert(0, 1, 0x01);
  msgId   = *((uint8_t*)(unsigned long)&str.c_str()[1]);
  msgLen  = *((int16_t*)(unsigned long)&str.c_str()[2]);
  GPS.fetch(msg, 4);
  msgTime = *((uint32_t*)(unsigned long)&msg.c_str()[0]);
  str = str + msg;
  GPS.fetch(msg, msgLen - 2);
  str = str + msg;
  if (verifyCK(str)) return msgId;
  else return -EBADMSG;
}

void computeTransform() {
  if(!transformGood_ && hasOdom_) {
    // Get the IMU's current RPY values. Need the raw values (for yaw, anyway).
    tf::Matrix3x3 mat(latestOrientation_);
    // Convert to RPY
    double imuRoll;
    double imuPitch;
    double imuYaw;
    mat.getRPY(imuRoll, imuPitch, imuYaw);
    // Compute the final yaw value that corrects for the difference between the
    // IMU's heading and the UTM grid's belief of where 0 heading should be (i.e.,
    // along the x-axis)
    imuYaw += (M_PI / 2.0);
    // Convert to tf-friendly structures
    tf::Quaternion imuQuat;
    imuQuat.setRPY(imuRoll, imuPitch, imuYaw);
    // The transform order will be orig_odom_pos * orig_utm_pos_inverse * cur_utm_pos.
    // Doing it this way will allow us to cope with having non-zero odometry position
    // when we get our first GPS message.
    tf::Pose utmPoseWithOrientation;
    utmPoseWithOrientation.setOrigin(latestUtmPose_.getOrigin());
    utmPoseWithOrientation.setRotation(imuQuat);
    utmWorldTransform_.mult(latestWorldPose_, utmPoseWithOrientation.inverse());
    double roll = 0;
    double pitch = 0;
    double yaw = 0;
    mat.setRotation(latestWorldPose_.getRotation());
    mat.getRPY(roll, pitch, yaw);
    ROS_INFO_STREAM("Latest world frame pose is: " << std::fixed <<
    "\nPosition: (" << latestWorldPose_.getOrigin().getX() << ", " <<
    latestWorldPose_.getOrigin().getY() << ", " <<
    latestWorldPose_.getOrigin().getZ() << ")" <<
    "\nOrientation: (" << roll << ", " <<
    pitch << ", " <<
    yaw << ")");
    mat.setRotation(utmWorldTransform_.getRotation());
    mat.getRPY(roll, pitch, yaw);
    ROS_INFO_STREAM("World frame->utm transform is " << std::fixed <<
    "\nPosition: (" << utmWorldTransform_.getOrigin().getX() << ", " <<
    utmWorldTransform_.getOrigin().getY() << ", " <<
    utmWorldTransform_.getOrigin().getZ() << ")" <<
    "\nOrientation: (" << roll << ", " <<
    pitch << ", " <<
    yaw << ")");
    transformGood_ = true;
  }
}

void Init() {
  while(GPS.connect("/dev/ttyHS3", B38400) < 0 && ros::ok()) { ROS_ERROR("GPS fail"); usleep(1000000);}
}

void Reset(ros::NodeHandle& _nh) {
  odom_sub = _nh.subscribe("/rover/loacalization/odom_fix", 1, OdometryUpdateCallback);
  hasOdom_       = false;
  transformGood_ = false;
  worldFrameId_  = "odom";
  latestUtmCovariance_.resize(6, 6);
//  while(!transformGood_ && ros::ok()) {
//    computeTransform();
//    if(transformGood_) odom_sub.shutdown();
//  }
}

void LoopPreCallback() {

}

void LoopPostCallback() {
  uint8_t gpsMsgId = 0x00;
  uint32_t gpsMsgTime;
  std::string message;
  nav_msgs::Odometry gpsOdomMsg;

WAIT_FOR_GPS_STATUS_MESSAGE:
  gpsMsgId = ReceiveGPSNavMsg(message, gpsMsgTime);
  if (gpsMsgId <= 0) ROS_ERROR("Error Receiving GPS message: %s!", strerror(-gpsMsgId));
  else if(gpsMsgId == 0x03) {
//    if (message[0] == 0x03 && message[1] == 0x0D) {
      gpsUpdateTime_ = gpsMsgTime;
      goto WAIT_FOR_GPS_FIX_MESSAGE;
//    }
//    else {
//      ROS_WARN("GPS fix not available since %u", gpsUpdateTime_);
//    }
  }
  goto WAIT_FOR_GPS_NEXT_TIME;
WAIT_FOR_GPS_FIX_MESSAGE:
  gpsMsgId = ReceiveGPSNavMsg(message, gpsMsgTime);
  if (gpsMsgId <= 0) ROS_ERROR("Error Receiving GPS message: %s!", strerror(-gpsMsgId));
  else if (gpsMsgId == 0x02 && gpsUpdateTime_ == gpsMsgTime) {
    double latitude  = (double)(*((int32_t*)(unsigned long)&message.c_str()[4]))/10000000;
    double longitude = (double)(*((int32_t*)(unsigned long)&message.c_str()[0]))/10000000;
    double height    = (double)(*((int32_t*)(unsigned long)&message.c_str()[8]))/1000;
    ROS_INFO_STREAM("Latitude " << latitude);
    ROS_INFO_STREAM("Longitude " << longitude);
    ROS_INFO_STREAM("Height " << height);
    double utmX = 0;
    double utmY = 0;
    std::string zone;
    gps_common::LLtoUTM(latitude, longitude, utmY, utmX, zone);
    latestUtmPose_.setOrigin(tf::Vector3(utmX, utmY, height));
    latestUtmCovariance_.setZero();
    // Copy the measurement's covariance matrix so that we can rotate it later
    latestUtmCovariance_(0, 0) = (double)(*((uint32_t*)(unsigned long)&message.c_str()[16]))/1000;
    latestUtmCovariance_(1, 1) = latestUtmCovariance_(0, 0);
    latestUtmCovariance_(2, 2) = (double)(*((uint32_t*)(unsigned long)&message.c_str()[20]))/1000;
    ROS_INFO_STREAM("Covariance " << latestUtmCovariance_);
    tf::Pose transformedUtm;
    transformedUtm.mult(utmWorldTransform_, latestUtmPose_);
    transformedUtm.setRotation(tf::Quaternion::getIdentity());
    // Rotate the covariance as well
    tf::Matrix3x3 rot(utmWorldTransform_.getRotation());
    Eigen::MatrixXd rot6d(6, 6);
    rot6d.setIdentity();
    for(size_t rInd = 0; rInd < 3; ++rInd) {
      rot6d(rInd,   0) = rot.getRow(rInd).getX();
      rot6d(rInd,   1) = rot.getRow(rInd).getY();
      rot6d(rInd,   2) = rot.getRow(rInd).getZ();
      rot6d(rInd+3, 3) = rot.getRow(rInd).getX();
      rot6d(rInd+3, 4) = rot.getRow(rInd).getY();
      rot6d(rInd+3, 5) = rot.getRow(rInd).getZ();
    }
    latestUtmCovariance_ = rot6d * latestUtmCovariance_.eval() * rot6d.transpose();
    // Now fill out the message. Set the orientation to the identity.
    tf::poseTFToMsg(transformedUtm, gpsOdomMsg.pose.pose);
    // Copy the measurement's covariance matrix so that we can rotate it later
    for (size_t i = 0; i < 3; i++) {
      for (size_t j = 0; j < 3; j++) {
        gpsOdomMsg.pose.covariance[3 * i + j] = latestUtmCovariance_(i, j);
      }
    }
    goto WAIT_FOR_GPS_VEL_MESSAGE;
  }
  goto WAIT_FOR_GPS_NEXT_TIME;
WAIT_FOR_GPS_VEL_MESSAGE:
  gpsMsgId = ReceiveGPSNavMsg(message, gpsMsgTime);
  if (gpsMsgId <= 0) ROS_ERROR("Error Receiving GPS message: %s!", strerror(-gpsMsgId));
  else if (gpsMsgId == 0x12 && gpsUpdateTime_ == gpsMsgTime) {
    gpsOdomMsg.twist.twist.linear.x   = + (double)(*((int32_t*)(unsigned long)&message.c_str()[4]))/100;
    gpsOdomMsg.twist.twist.linear.y   = + (double)(*((int32_t*)(unsigned long)&message.c_str()[0]))/100;
    gpsOdomMsg.twist.twist.linear.z   = - (double)(*((int32_t*)(unsigned long)&message.c_str()[8]))/100;
    gpsOdomMsg.twist.covariance[0, 0] =   (double)(*((int32_t*)(unsigned long)&message.c_str()[24]))/100;
    gpsOdomMsg.twist.covariance[1, 1] =   gpsOdomMsg.twist.covariance[0, 0];
    gpsOdomMsg.twist.covariance[2, 2] =   gpsOdomMsg.twist.covariance[0, 0];
    gpsOdomMsg.pose.pose.orientation  =   tf::createQuaternionMsgFromYaw ((double)(*((uint32_t*)(unsigned long)&message.c_str()[20]))/100000);
    gpsOdomMsg.pose.covariance[5,5]   =   (double)(*((uint32_t*)(unsigned long)&message.c_str()[28]))/100000;

    goto PUBLISH_GPS_DATA;
  }
  goto WAIT_FOR_GPS_NEXT_TIME;
PUBLISH_GPS_DATA:
  gps_odom_pub.publish(gpsOdomMsg);
WAIT_FOR_GPS_NEXT_TIME:
  ;
}

void Shutdown (int signum) {
  ros::shutdown();
}
