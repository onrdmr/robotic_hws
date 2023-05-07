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

	// gaussian blur for hsv conversion
    cv::Mat blurred_image;
    cv::GaussianBlur(rgb_image, blurred_image, cv::Size(5, 5), 0.0, 0.0);
    
    // // Convert RGB to HSV
    cv::cvtColor(blurred_image, hsv_image, cv::COLOR_RGB2HSV);
    // // Create mask for red color
    cv::Scalar lower_red(110, 50, 50);
    cv::Scalar upper_red(130, 255, 255);
    cv::inRange(hsv_image, lower_red, upper_red, mask);
    

	cv::Mat gray_image;
    cv::cvtColor(rgb_image, gray_image, cv::COLOR_BGR2GRAY);
	
	cv::Mat erodeElement = cv::getStructuringElement( cv::MORPH_RECT,cv::Size(10,10));
	cv::Mat dilateElement = cv::getStructuringElement( cv::MORPH_RECT,cv::Size(10,10));


	// states are
	// black reach state 
	// white reached state
	// on ramp state up
	// on ramp state down
	// going on ramp up
	// going on ramp down


    cv::erode(gray_image,gray_image,erodeElement);
    cv::dilate(gray_image,gray_image,dilateElement);
	
	// Apply Canny edge detection to the blurred image
    cv::Canny(gray_image, edges, 100, 300, 3);

	// cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    // cv::morphologyEx(edges, edges, cv::MORPH_OPEN, kernel);


	double rho = 1;  // distance precision in pixel, i.e. 1 pixel
    double angle = CV_PI / 180;  // angular precision in radian, i.e. 1 degree
    int min_threshold = 10;  // minimal of votes
  

	int height = mask.rows;	
    int width = mask.cols;
    cv::Mat maskHalf = cv::Mat::zeros(mask.size(), mask.type());

    // only focus bottom half of the screen
    cv::Point pts[4] = {
        cv::Point(0, height * 1 / 1.2),
        cv::Point(width, height * 1 / 1.2),
        cv::Point(width, height),
        cv::Point(0, height)
    };
    cv::fillConvexPoly(maskHalf, pts, 4, cv::Scalar(255, 0, 0));

    cv::Mat croppedEdges;
    cv::bitwise_and(mask, maskHalf, croppedEdges);

    std::vector<cv::Vec4i> line_segments;
    
	cv::imshow("cropped mask", croppedEdges);
	
	cv::HoughLinesP(croppedEdges, line_segments, rho, angle, min_threshold, 8, 4);

    // Draw lines on the original image
    cv::Mat line_image = gray_image.clone();
    for (auto line : line_segments)
    {
        cv::line(line_image, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(0, 0, 255), 2);
    }

	cv::imshow("direction of line", line_image);


    // Apply mask to HSV image
    // cv::bitwise_and(hsv_image, hsv_image, masked_image, mask);

    // // Convert HSV to RGB
    // cv::cvtColor(masked_image, masked_image, cv::COLOR_HSV2RGB);

    // Detect edges in masked RGB image
    // cv::Canny(masked_image, edges, 100, 200);

    // Display original RGB image, masked RGB image with edges, and wait for a key press
    // cv::imshow("RGB Image", rgb_image);

	// Display the original and blurred images side by side
    cv::Mat side_by_side;
    cv::hconcat(mask, edges, side_by_side);
    cv::imshow("blue coloured image  vs. canny edges of environment", side_by_side);

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
