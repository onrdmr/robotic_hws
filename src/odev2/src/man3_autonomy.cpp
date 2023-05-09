#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Image.h"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>


ros::Publisher cmd_vel_pub;
geometry_msgs::Twist cmd_vel;


// if robot in the ramp intervals are empty
std::vector<std::pair<int, int>> middleMaskIntervals(cv::Mat& maskImage, int row)
{
    int middleRow = row;
    cv::Mat middleRowMat = maskImage.row(middleRow).clone();

    std::vector<std::pair<int,int>> intervals;


    int intervalStart = 0, whiteCursor, blackCursor;
    bool lastlyWhite=false;
    for (int i = 0 ; i < middleRowMat.cols ; i++) {
        // ROS_INFO("image size : %d - %d", middleRowMat.rows, middleRowMat.cols);
        // ROS_INFO("%d ", middleRowMat.at<uchar>(0,i));
        
        if(middleRowMat.at<uchar>(0,i) > 100){
            whiteCursor = i;
            lastlyWhite = true;
        }else {
            if(lastlyWhite) 
            {
                intervals.push_back(std::pair<int,int>(intervalStart, whiteCursor));
                lastlyWhite = false;
            }
            blackCursor = i;
            intervalStart = blackCursor;
        }
    }
    
    if( intervalStart != middleRowMat.cols-1 ) {
        intervals.push_back(std::pair<int,int>(intervalStart, whiteCursor));

    }

    for( auto item : intervals)
    {
        // ROS_INFO("%d - %d interval is white", item.first, item.second);
    }

    return intervals;

}

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
    
    middleMaskIntervals(mask, 400);

    cv::Mat maskHalf = cv::Mat::zeros(mask.size(), mask.type());

    // only focus bottom half of the screen
    cv::Point pts[4] = {
        cv::Point(0, height * 1 / 1.2),
        cv::Point(width, height * 1 / 1.2),
        cv::Point(width, height),
        cv::Point(0, height)
    };
    cv::fillConvexPoly(maskHalf, pts, 4, cv::Scalar(255, 0, 0));

    cv::Mat croppedMask;
    cv::bitwise_and(mask, maskHalf, croppedMask);

    std::vector<cv::Vec4i> line_segments;
    
	cv::imshow("cropped mask", croppedMask);
	
    cv::Mat croppedEdges;
    cv::Canny(croppedMask, croppedEdges, 50, 200,3);

	cv::HoughLinesP(croppedEdges, line_segments, rho, angle, min_threshold, 8, 4);

    // Draw lines on the original image
    cv::Mat line_image = gray_image.clone();
    
    // take medium point of line segments
    int dataPointsLen = line_segments.size();
    cv::Mat dataPoints(dataPointsLen, 2, CV_32F);
    cv::Mat labels;
    cv::Mat centers;

    for (int i = 0, j=0 ; i < dataPointsLen ; i++, j++)
    {   
        dataPoints.at<float>(j,0) = static_cast<float> (line_segments[i][0]);
        dataPoints.at<float>(j,1) = static_cast<float> (line_segments[i][1]);
        dataPoints.at<float>(j+1,0) = static_cast<float> (line_segments[i][2]);
        dataPoints.at<float>(j+1,1) = static_cast<float> (line_segments[i][3]);
        ROS_INFO("point %f %f : %f %f", dataPoints.at<float>(i,0), dataPoints.at<float>(i,1), dataPoints.at<float>(i+1,0), dataPoints.at<float>(i+1,1) );
    }

    int K=2;

    if(line_segments.size() == 0) {
        return ;
    }
    ROS_INFO("N-K: %ld, %d" , line_segments.size(), K);

    cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 10, 1.0);
    int attempts = 3;

    int flags = cv::KMEANS_RANDOM_CENTERS;

    cv::kmeans(dataPoints, K, labels, criteria, attempts, flags, centers);

    ROS_INFO_STREAM ( "Cluster labels: " << std::endl << labels << std::endl << "Centroids: " << std::endl << centers << std::endl);

    unsigned long long x=0, y=0;
    for (auto line : line_segments)
    {
        ROS_INFO("%d,%d", line[2], line[3]);
        // throw std::runtime_error("error");
        // if( line[1] < line[3] ) {
        //     // ROS_INFO("13");
        //     // calculate cosine and bundle same groups
        
        cv::line(line_image, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(0, 0, 255), 2);
        

        // }
        // else{
            // ROS_INFO("23");
            // calculate cosine and bundle same groups
            

        //     cv::line(line_image, cv::Point(line[2], line[3]), cv::Point(line[0], line[1]), cv::Scalar(0, 0, 255), 2);
        // }

        x += line[0];
        x += line[2];
        y += line[1];
        y += line[3];
    }

    if(line_segments.size() > 0 ) {
        y = y / (line_segments.size() * 2);
        x = x / (line_segments.size() * 2);
    } else {
        x = 400;
        y = 700;
    }

    // ROS_INFO("%lld- %lld", x, y);

    // cv::drawMarker(line_image, cv::Point(static_cast<int>(centers.at<float>(0)),static_cast<int>(centers.at<float>(1))), 2, 255 ,-1);
    cv::drawMarker(line_image, cv::Point(static_cast<int>(centers.at<float>(0)),static_cast<int>(centers.at<float>(1))), 255, cv::MARKER_CROSS, 5, 2);
    cv::drawMarker(line_image, cv::Point(static_cast<int>(centers.at<float>(2)),static_cast<int>(centers.at<float>(3))), 255, cv::MARKER_CROSS, 5, 2);


    cv::circle(line_image, cv::Point(x,y), 2, 255 ,-1);

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
