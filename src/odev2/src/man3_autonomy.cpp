#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Image.h"

#include <opencv2/opencv.hpp>

ros::Publisher cmd_vel_pub;
geometry_msgs::Twist cmd_vel;

void cameraCallBack(const sensor_msgs::Image::ConstPtr& camera){
	//ASAGIDA BULUNAN IF KOMUTU ORNEK OLARAK VERILMISTIR. SIZIN BURAYI DEGISTIRMENIZ BEKLENMEKTEDIR
	//BURDAN SONRASINI DEGISTIR
	
	cv::Mat mat(camera->height, camera->width, CV_8UC3, const_cast<uchar *>(&camera->data[0]),
    camera->step);


	// cmd_vel.linear.x = 0.20;
	
	//BURDAN SONRASINA DOKUNMA
	cv::imshow("OPENCV_WINDOW", mat);
    cv::waitKey(3);

	cmd_vel_pub.publish(cmd_vel);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "man3_autonomy");
	ros::NodeHandle nh;

	cmd_vel.linear.y = 0.0;
	cmd_vel.linear.z = 0.0;
	cmd_vel.angular.x = 0.0;
	cmd_vel.angular.y = 0.0;

	ros::Subscriber camera_sub = nh.subscribe("/rtg/camera/rgb/image_raw", 1000, cameraCallBack);
	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/rtg/cmd_vel", 1000);

	ros::spin();

	return 0;
}
