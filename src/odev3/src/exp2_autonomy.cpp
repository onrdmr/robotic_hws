/*
BU DOSYAYI ISTEDIGINIZ GIBI DUZENLEYEBILIRSINIZ. ANCAK
DOSYANIN ISMINI DEGISTIRMEYINIZ (GRUP NUMARASI EKLEMESI HARIC)
YAYINLAYACAGINIZ cmd_vel MESAJI BU PROGRAMDAN YAYINLANMALIDIR.
*/


#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/OccupancyGrid.h"

ros::Publisher cmd_vel_pub;
geometry_msgs::Twist cmd_vel;

void sub1CallBack(const nav_msgs::OccupancyGrid::ConstPtr& map){
	

cmd_vel_pub.publish(cmd_vel);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "man3_autonomy");
	ros::NodeHandle nh;

	cmd_vel.linear.y = 0.0;
	cmd_vel.linear.z = 0.0;
	cmd_vel.angular.x = 0.0;
	cmd_vel.angular.y = 0.0;
	
	ros::Subscriber sub1_ornek = nh.subscribe("/rtg/SIZDOLDURUNUZ", 1000, sub1CallBack);
	
	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/rtg/cmd_vel", 1000);

	ros::spin();

	return 0;
}
