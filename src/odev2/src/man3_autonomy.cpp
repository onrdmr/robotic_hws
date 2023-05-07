#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Image.h"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>


ros::Publisher cmd_vel_pub;
geometry_msgs::Twist cmd_vel;

void cameraCallBack(const sensor_msgs::Image::ConstPtr& camera){
	//ASAGIDA BULUNAN IF KOMUTU ORNEK OLARAK VERILMISTIR. SIZIN BURAYI DEGISTIRMENIZ BEKLENMEKTEDIR
	//BURDAN SONRASINI DEGISTIR
	
	cv::Mat rgb_image(camera->height, camera->width, CV_8UC3, const_cast<uchar *>(&camera->data[0]),
    camera->step);
	cv::Mat hsv_image, mask, masked_image, edges;

	// gaussian blur
    cv::Mat blurred_image;
    cv::GaussianBlur(rgb_image, blurred_image, cv::Size(5, 5), 0.0, 0.0);

    // Display the original and blurred images side by side
    cv::Mat side_by_side;
    cv::hconcat(rgb_image, blurred_image, side_by_side);
    cv::imshow("Original vs. Blurred", side_by_side);
    // cv::waitKey(0);

    
    // // Convert RGB to HSV
    cv::cvtColor(blurred_image, hsv_image, cv::COLOR_RGB2HSV);
    // // Create mask for red color
    cv::Scalar lower_red(110, 50, 50);
    cv::Scalar upper_red(130, 255, 255);
    cv::inRange(hsv_image, lower_red, upper_red, mask);
    
    // Apply mask to HSV image
    // cv::bitwise_and(hsv_image, hsv_image, masked_image, mask);
    
    // // Convert HSV to RGB
    // cv::cvtColor(masked_image, masked_image, cv::COLOR_HSV2RGB);
    
    // Detect edges in masked RGB image
    // cv::Canny(masked_image, edges, 100, 200);
    
    // Display original RGB image, masked RGB image with edges, and wait for a key press
    // cv::imshow("RGB Image", rgb_image);
    cv::imshow("mask", mask);

    // cv::imshow("Masked RGB Image with Edges", edges);
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
