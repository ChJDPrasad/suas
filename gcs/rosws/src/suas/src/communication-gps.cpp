#include <suas/communication-gps.h>
#include <communication/uart.h>
#include <sensor_msgs/NavSatFix.h>

ros::Publisher  gps_fix_pub;
uint32_t gpsUpdateTime_;
Serial GPS;

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

void Init() {
  while(GPS.connect("/dev/ttyUSB0", B38400) < 0 && ros::ok()) { ROS_ERROR("GPS fail"); usleep(1000000);}
}

void Reset(ros::NodeHandle& _nh) {
  gps_fix_pub  = _nh.advertise<sensor_msgs::NavSatFix>("/gcs/gps/fix",  1);
}

void LoopPreCallback() {

}

void LoopPostCallback() {
  uint8_t gpsMsgId = 0x00;
  uint32_t gpsMsgTime;
  std::string message;
  sensor_msgs::NavSatFix gpsMsg;
  gpsMsg.status.service = 1;

//WAIT_FOR_GPS_STATUS_MESSAGE:
//  gpsMsgId = ReceiveGPSNavMsg(message, gpsMsgTime);
//  ROS_INFO_STREAM("status " << (unsigned int)gpsMsgId);
//  if (gpsMsgId <= 0) ROS_ERROR("Error Receiving GPS message: %s!", strerror(-gpsMsgId));
//  else if(gpsMsgId == 0x03) {
//    if (message[0] == 0x03 && message[1] == 0x0D) {
//      gpsMsg.status.status = 0;
//      gpsUpdateTime_ = gpsMsgTime;
//      gpsMsg.header.stamp = ros::Time(gpsMsgTime);
//      goto WAIT_FOR_GPS_FIX_MESSAGE;
//    }
//    else {
//      gpsMsg.status.status = -1;
//      gpsMsg.header.stamp = ros::Time(gpsMsgTime);
//      ROS_WARN("GPS fix not available");
//      goto PUBLISH_GPS_MSG;
//    }
//  }
//  return;
WAIT_FOR_GPS_FIX_MESSAGE:
  gpsMsgId = ReceiveGPSNavMsg(message, gpsMsgTime);
  gpsMsg.status.status = 0;
  gpsMsg.header.stamp = ros::Time(gpsMsgTime);
  if (gpsMsgId <= 0) ROS_ERROR("Error Receiving GPS message: %s!", strerror(-gpsMsgId));
  else if (gpsMsgId == 0x02) {
    gpsMsg.latitude  = (double)(*((int32_t*)(unsigned long)&message.c_str()[4]))/10000000;
    gpsMsg.longitude = (double)(*((int32_t*)(unsigned long)&message.c_str()[0]))/10000000;
    gpsMsg.altitude  = (double)(*((int32_t*)(unsigned long)&message.c_str()[8]))/1000;
    gpsMsg.position_covariance[0] = (double)(*((uint32_t*)(unsigned long)&message.c_str()[16]))/1000;
    gpsMsg.position_covariance[1] = 0;
    gpsMsg.position_covariance[2] = 0;
    gpsMsg.position_covariance[3] = 0;
    gpsMsg.position_covariance[4] = (double)(*((uint32_t*)(unsigned long)&message.c_str()[16]))/1000;
    gpsMsg.position_covariance[5] = 0;
    gpsMsg.position_covariance[6] = 0;
    gpsMsg.position_covariance[7] = 0;
    gpsMsg.position_covariance[8] = (double)(*((uint32_t*)(unsigned long)&message.c_str()[20]))/1000;
    gpsMsg.position_covariance_type = 3;
    goto PUBLISH_GPS_MSG;
  }
  return;
PUBLISH_GPS_MSG:
  gps_fix_pub.publish(gpsMsg);
}

void Shutdown (int signum) {
  ros::shutdown();
}

