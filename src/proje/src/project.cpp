#include "ros/ros.h"
#include "sensor_msgs/Image.h"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgcodecs.hpp>

// #include <visualisation_msgs/Marker.h>

void cameraCallBack(const sensor_msgs::Image::ConstPtr& camera)
{
    bool isLocked;
    std::ifstream fileLock("~/repos/robotic_hws/src/proje/file.lock");
    
    std::string boolString;
    fileLock >> boolString;
    isLocked = (boolString.at(0) == 48) ? false : true;
    ROS_INFO("Locked File content %s\n bool at 0 :%c file locked %d, value of 48 %c", boolString.c_str(), boolString.at(0), isLocked, 48);
    if(!isLocked) {

        cv::Mat rgbImage(camera->height, camera->width, CV_8UC3, const_cast<uchar*>(&camera->data[0]),
            camera->step);

        // cv::QRCodeDetector qrDecoder = cv::QRCodeDetector::QRCodeDetector();
        

        bool isSaved = cv::imwrite("~/repos/robotic_hws/src/proje/onur.jpg", rgbImage);
        if( isSaved == false )
        {   
            ROS_INFO("Saving the image, FAILED\n\n" );

            return;
        }



        cv::waitKey(3);
    }
    else {
        return;
    }
}

int main(int argc, char ** argv) 
{

    ros::init(argc, argv, "object detection");
    ros::NodeHandle nh;
    ros::Subscriber camera_sub = nh.subscribe("/rtg/camera/rgb/image_raw", 1000, cameraCallBack);

    ros::spin();

    return 0;
}