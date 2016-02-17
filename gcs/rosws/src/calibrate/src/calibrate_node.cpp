#include <ros/ros.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/VFR_HUD.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <math.h>
#include <string>

#define PI 3.14159265

double roll, pitch, head, alt;
double troll, tpitch, thead, talt;
bool is_pos_ctl = false;
bool is_alt_ctl = false;
bool mode_changed = false;
std::string mode = "UNKNOWN";

void targetCallback(const mavros_msgs::AttitudeTarget::ConstPtr& msg) {
  double q0 = msg->orientation.x;
  double q1 = msg->orientation.y;
  double q2 = msg->orientation.z;
  double q3 = msg->orientation.w;
  tpitch = asin(2*(q0*q2 - q3*q1))*180/PI;
  troll = atan2(2*(q0*q3 + q1*q3), - 1 + 2*(q2*q2 + q3*q3))*180/PI;
  ROS_INFO("T:C\tROLL\t\t%f\t%f\t\tPITCH\t%f\t%f",troll,roll,tpitch,pitch);
}

void vfrhudCallback(const mavros_msgs::VFR_HUD::ConstPtr& msg) {
  alt = msg->altitude;
  head = msg->heading;
  if (is_pos_ctl || is_alt_ctl)
    ROS_INFO("T:C\tALT\t\t%f\t%f\t\tHEAD\t%f\t%f",talt,alt,thead,head);
}

void attitudeCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  double q0 = msg->pose.orientation.x;
  double q1 = msg->pose.orientation.y;
  double q2 = msg->pose.orientation.z;
  double q3 = msg->pose.orientation.w;
  pitch = asin(2*(q0*q2 - q3*q1))*180/PI;
  roll = atan2(2*(q0*q3 + q1*q3), - 1 + 2*(q2*q2 + q3*q3))*180/PI;
}

void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
  if (mode != msg->mode) {
    is_pos_ctl = false;
    is_alt_ctl = false;
    if (msg->mode.find(std::string("ALT")) != -1) is_alt_ctl = true;
    else if (msg->mode.find(std::string("POS")) != -1) is_pos_ctl = true;
    mode_changed = true;
    mode = msg->mode;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "calibration");
  ros::NodeHandle n;
  ros::Subscriber tatt_sub  = n.subscribe("/mavros/setpoint_raw/target_attitude", 1, targetCallback);
  ros::Subscriber att_sub = n.subscribe("/mavros/local_position/pose", 1, attitudeCallback);
  ros::Subscriber state_sub = n.subscribe("/mavros/state", 1, stateCallback);
  ros::Subscriber head_sub = n.subscribe("/mavros/vfr_hud", 1, vfrhudCallback);
  ros::Publisher roll_pub = n.advertise<std_msgs::Float32>("/calibration/roll", 100);
  ros::Publisher troll_pub = n.advertise<std_msgs::Float32>("/calibration/troll", 100);
  ros::Publisher pitch_pub = n.advertise<std_msgs::Float32>("/calibration/pitch", 100);
  ros::Publisher tpitch_pub = n.advertise<std_msgs::Float32>("/calibration/tpitch", 100);
  ros::Publisher head_pub = n.advertise<std_msgs::Float32>("/calibration/heading", 100);
  ros::Publisher thead_pub = n.advertise<std_msgs::Float32>("/calibration/theading", 100);
  ros::Publisher alt_pub = n.advertise<std_msgs::Float32>("/calibration/altitude", 100);
  ros::Publisher talt_pub = n.advertise<std_msgs::Float32>("/calibration/taltitude", 100);
  ros::Rate loop_rate(50);

  while (ros::ok()) {
    ros::spinOnce();
    if (mode_changed) {
      mode_changed = false;
      if (is_pos_ctl || is_alt_ctl) {
        thead = head;
        talt = alt;
      }
    }
    std_msgs::Float32 msg;
    msg.data = roll;
    roll_pub.publish(msg);
    msg.data = troll;
    troll_pub.publish(msg);
    msg.data = pitch;
    pitch_pub.publish(msg);
    msg.data = tpitch;
    tpitch_pub.publish(msg);
    msg.data = head;
    head_pub.publish(msg);
    msg.data = thead;
    thead_pub.publish(msg);
    msg.data = alt;
    alt_pub.publish(msg);
    msg.data = talt;
    talt_pub.publish(msg);
    loop_rate.sleep();
  }

  return 0;
}
