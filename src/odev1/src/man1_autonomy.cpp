#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

ros::Publisher cmd_vel_pub;
geometry_msgs::Twist cmd_vel;

void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& laser){
	//ASAGIDA BULUNAN IF KOMUTU ORNEK OLARAK VERILMISTIR. SIZIN BURAYI DEGISTIRMENIZ BEKLENMEKTEDIR
	//BURDAN SONRASINI DEGISTIR
	if(laser->ranges[540] > 1.0){		//Lazerin tam orda noktasındaki olcum (540. indis) 1 metreden fazlaysa
		cmd_vel.linear.x = 0.25;		//cizgisel hiz 0.25 m/s olsun
		cmd_vel.angular.z = 0.0;		//acisal hiz 0.0 radyan/s olsun
	}else{								//fazla degilse
		cmd_vel.linear.x = 0.0;			//cizgisel hiz 0.0 m/s olsun
		cmd_vel.angular.z = 0.25;		//acisal hiz 0.25 radyan/s olsun
	}
	//BURDAN SONRASINA DOKUNMA

	cmd_vel_pub.publish(cmd_vel);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "man1_autonomy");
	ros::NodeHandle nh;

	cmd_vel.linear.y = 0.0;
	cmd_vel.linear.z = 0.0;
	cmd_vel.angular.x = 0.0;
	cmd_vel.angular.y = 0.0;

	ros::Subscriber laser_sub = nh.subscribe("/rtg/hokuyo", 1000, laserCallBack);
	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/rtg/cmd_vel", 1000);

	ros::spin();

	return 0;
}
