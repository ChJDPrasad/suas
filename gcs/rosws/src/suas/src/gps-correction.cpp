#include <suas/gps-correction.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>

ros::Subscriber gps_uas_sub;
ros::Subscriber gps_gcs_sub;
ros::Subscriber odom_uas_sub;
sensor_msgs::NavSatFix gpsGcsMsg;
sensor_msgs::NavSatFix gpsUasMsg;
nav_msgs::Odometry odomUasMsg;
bool gpsInitialised_; 
struct xyz{
	float x;
	float y;
	float z;
	}; 

struct LtLn{
	float lat;
	float lon;
	float alt;
	}; 

 	
struct LtLn base_0;
struct xyz base0;
float scale = 10^5;
int nm2m = 1852; 	

double norm(struct xyz pos){
	double ans  = sqrt(pos.x*pos.x + pos.x*pos.x + pos.x*pos.x);
	return ans;
}

struct xyz latlon2xyz(struct LtLn pos){
	struct xyz position; 
	position.x = (pos.lat - base_0.lat)*60*nm2m;
	position.y = (cos(base_0.lat)*(pos.lon - base_0.lon))*60*nm2m;
	position.z = pos.alt - base_0.alt;
    return position;
}
struct LtLn xyz2latlon(struct xyz pos){
	struct LtLn position;
	position.lat = pos.x/nm2m/60 + base_0.lat;
    position.lon = pos.y/nm2m/60/cos(base_0.lat) + base_0.lon;
    position.alt = pos.z + base_0.alt;
    return position;
}

struct xyz initialxyz(){
	 struct xyz pos;
	 pos.x = 0.0;
	 pos.y = 0.0;
	 pos.z = 0.0;
	 return pos;
}
struct xyz addxyz(struct xyz a, struct xyz b){
	struct xyz ans;
	ans.x = a.x+b.x;
	ans.y = a.y+b.y;
	ans.z = a.z+b.z;
	return ans;
}
struct xyz multiply(struct xyz a, float b){
	struct xyz ans;
	ans.x = a.x*b;
	ans.y = a.y*b;
	ans.z = a.z*b;
	return ans;
}


struct LtLn rostoLtLn(sensor_msgs::NavSatFix gps_data){
	struct LtLn data;
	data.lat = gps_data.latitude;
	data.lon = gps_data.longitude;
	data.alt = gps_data.altitude;
    return data;
}


struct xyz rostovel(nav_msgs::Odometry odom_data){
	struct xyz data;
	data.x = odom_data.twist.twist.linear.x;
	data.y = odom_data.twist.twist.linear.y;
	data.z = odom_data.twist.twist.linear.z;
    return data;
}


struct xyz rostoxyz(nav_msgs::Odometry odom_data){
	struct xyz data;
	data.x = odom_data.pose.pose.position.x;
	data.y = odom_data.pose.pose.position.y;
	data.z = odom_data.pose.pose.position.z;
    return data;
}

void odomUasUpdateCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  odomUasMsg.header = msg->header;
  odomUasMsg.child_frame_id = msg->child_frame_id;
  odomUasMsg.twist = msg->twist;
  odomUasMsg.pose = msg->pose;
}

void gpsGcsUpdateCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  gpsGcsMsg.latitude = msg->latitude;
  gpsGcsMsg.longitude = msg->longitude;
  gpsGcsMsg.altitude = msg->altitude;
  gpsGcsMsg.header.stamp = msg->header.stamp;
  gpsGcsMsg.position_covariance[0] = msg->position_covariance[0];
  gpsGcsMsg.position_covariance[1] = msg->position_covariance[1];
  gpsGcsMsg.position_covariance[2] = msg->position_covariance[2];
  gpsGcsMsg.position_covariance[3] = msg->position_covariance[3];
  gpsGcsMsg.position_covariance[4] = msg->position_covariance[4];
  gpsGcsMsg.position_covariance[5] = msg->position_covariance[5];
  gpsGcsMsg.position_covariance[6] = msg->position_covariance[6];
  gpsGcsMsg.position_covariance[7] = msg->position_covariance[7];
  gpsGcsMsg.position_covariance[8] = msg->position_covariance[8];
}

void gpsUasUpdateCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  gpsUasMsg.latitude = msg->latitude;
  gpsUasMsg.longitude = msg->longitude;
  gpsUasMsg.altitude = msg->altitude;
  gpsUasMsg.header.stamp = msg->header.stamp;
  gpsUasMsg.position_covariance[0] = msg->position_covariance[0];
  gpsUasMsg.position_covariance[1] = msg->position_covariance[1];
  gpsUasMsg.position_covariance[2] = msg->position_covariance[2];
  gpsUasMsg.position_covariance[3] = msg->position_covariance[3];
  gpsUasMsg.position_covariance[4] = msg->position_covariance[4];
  gpsUasMsg.position_covariance[5] = msg->position_covariance[5];
  gpsUasMsg.position_covariance[6] = msg->position_covariance[6];
  gpsUasMsg.position_covariance[7] = msg->position_covariance[7];
  gpsUasMsg.position_covariance[8] = msg->position_covariance[8];
}

void Init() {
    
}

void Reset(ros::NodeHandle& _nh) {
  gpsInitialised_ = 0;
  gps_gcs_sub  = _nh.subscribe("/gcs/gps/fix",  1, gpsGcsUpdateCallback);
  gps_uas_sub  = _nh.subscribe("/uas/global_position/raw/fix",  1, gpsUasUpdateCallback);
  odom_uas_sub = _nh.subscribe("/uas/global_position/local",  1, odomUasUpdateCallback);
}

void LoopPreCallback() {

}

void LoopPostCallback() {
    if (!gpsInitialised_) {
       base_0 = rostoLtLn(gpsGcsMsg);
       ROS_INFO_STREAM("init x: " << base_0.lat<<" inti y: "<<base_0.lon<<" init z: "<<base_0.alt);
     if (base_0.lat != 0) gpsInitialised_ = 1;
       } 
 //Position -> odomUasMsg.pose.pose.position.x,y,z (From home NED)
//Velocity -> odomUasMsg.twist.twist.linear.x,y,z (From home NED)
//GPSFix -> gpsXxxMsg.latitude,longitude,altitude (WGS84)
  
    int T = 5;
    struct xyz vel[T];
    struct xyz avg_vel;
    struct xyz GPS_xyz;
    struct xyz EKF_xyz0;
    struct xyz EKF_xyzT;
    struct LtLn base_pos;
    struct LtLn GPS_pos;
    struct LtLn GPS_pos_new;
    struct LtLn EKF_pos[T]; 
    struct xyz GPS_error;
    double drift;
    double tol1 = 0.0000001;
    double tol2 = 0.0000001;
    double a1   = 1;
    double a2   = 0.0;
    double del_t = 0.2;
    avg_vel = initialxyz();
		for(int i =0; i<T; i++)
		{   
			vel[i] = rostovel(odomUasMsg);		// from t to (t+T-1)
			EKF_pos[i] = xyz2latlon(rostoxyz(odomUasMsg));   // from t to (t+T-1)
			avg_vel = addxyz(avg_vel,vel[i]);
		}
		avg_vel = multiply(avg_vel, 1/T);
		GPS_pos = rostoLtLn(gpsUasMsg);         // at T
		GPS_xyz = latlon2xyz(GPS_pos);
		base_pos = rostoLtLn(gpsGcsMsg);
		drift = norm(latlon2xyz(base_pos));
ROS_INFO_STREAM("old lat: " << GPS_pos.lat<<" old lon: "<<GPS_pos.lon<<" old alt: "<<GPS_pos.alt);
ROS_INFO_STREAM("old x: " <<GPS_xyz.x<<" old y: "<<GPS_xyz.y<<" old z: "<< GPS_xyz.z);
		EKF_xyz0 = latlon2xyz(EKF_pos[0]);
		EKF_xyzT = latlon2xyz(EKF_pos[T-1]);
		tol2 = norm(multiply(vel[T-1],del_t*1.5));
// Updating GPS pos of rover
	if (drift< tol1)
	    ROS_INFO_STREAM("chill: error less than tol1");
	else
	    if(norm(addxyz(GPS_xyz,multiply(EKF_xyzT,-1)))< tol2)
			ROS_INFO_STREAM("chill: error less than tol2");
		else
<<<<<<< HEAD
			GPS_xyz = addxyz(addxyz(GPS_xyz ,multiply((addxyz(GPS_xyz,multiply(EKF_xyzT,-1))),del_t*a1*drift/norm((addxyz(GPS_xyz,multiply(EKF_xyzT,-1)))))), multiply(addxyz(addxyz(GPS_xyz,multiply(EKF_xyz0,-1)), multiply(avg_vel,T)),a2));

=======
			GPS_xyz = addxyz(addxyz(GPS_xyz ,multiply((addxyz(GPS_xyz,multiply(EKF_xyzT,-1))),del_t*a1*drift/norm((addxyz(GPS_xyz,multiply(EKF_xyzT,-1)))))), multiply(addxyz(addxyz(GPS_xyz,multiply(EKF_xyz0,-1)), multiply(avg_vel,T*del_t)),a2));
	
>>>>>>> 7ad2a640f4a204ec215da04ccb7dca84dc2fa925
	GPS_pos_new = xyz2latlon(GPS_xyz);

ROS_INFO_STREAM("new lat: " << GPS_pos_new.lat<<" new lon: "<<GPS_pos_new.lon<<" new alt: "<<GPS_pos_new.alt);
ROS_INFO_STREAM("new x: "<< GPS_xyz.x<<" new y: "<< GPS_xyz.y<<" new z: "<<GPS_xyz.z);
}

void Shutdown (int signum) {
  ros::shutdown();
}
