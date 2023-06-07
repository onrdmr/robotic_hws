#include <iostream>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// using namespace cv;



// std::pair<> DiffersInSegment() {

// }

int main(int argc , char * argv[]) 
{

    // cv::Mat image = cv::imread("/home/onur/robotic_hws/src/proje/barcode_example1.jpg");
    cv::Mat image = cv::imread("/home/onur/robotic_hws/src/proje/corrosive.jpg");
    // cv::Mat image = cv::imread("/home/onur/robotic_hws/src/proje/explosives.jpg");


    if (image.empty()) {
        std::cout << "Failed to load image!" << std::endl;
        return -1;
    }

    // cv::QRCodeDetector qrDet = cv::QRCodeDetector::QRCodeDetector();
    int kernelSize = 2;
    cv::Mat grayscaleImage;
    cv::cvtColor(image, grayscaleImage, cv::COLOR_BGR2GRAY);


    cv::Mat kernel = grayscaleImage(cv::Rect(0, 0, kernelSize, kernelSize)).clone();

    // Perform convoiution
    cv::Mat result;
    
    cv::filter2D(grayscaleImage, result, -1, kernel); 


    cv::imshow("Result", result);
    // cv::waitKey(0);

    cv::imshow("gs", grayscaleImage);
    cv::waitKey(0);


    return 0;
}
