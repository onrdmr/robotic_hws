#include <iostream>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tesseract/baseapi.h>


// using namespace cv;



// std::pair<> DiffersInSegment() {

// }



typedef struct Contour 
{
    // clokwise order
    cv::Point lu;
    cv::Point ru;
    cv::Point rb;
    cv::Point lb;

} ContourArea;


std::unique_ptr<ContourArea> createContour(cv::Mat& image, int cost)
{
    int minLeft=65536, maxRight=0, maxHeight=0, minHeight=65536 ;

    for (int row = 0 ; row < image.rows ; row++) 
    {
        for (int column = 0 ; column < image.cols ; column++)
        {
            uchar pixel = image.at<uchar>(row, column);
            
            if(pixel < 10)
            {
                if(column < minLeft) {
                    minLeft = column;
                }
                if(column > maxRight) {
                    maxRight = column;
                }

                if (row < minHeight) {
                    minHeight = row;
                }
                if (row > maxHeight) {
                    maxHeight = row;
                }
            }

        }

    }

    ContourArea * contour = new ContourArea { 
        { minLeft - cost, minHeight - cost}, // lu
        { maxRight + cost, minHeight - cost }, // ru
        { maxRight + cost, maxHeight + cost }, // rb
        { minLeft - cost, maxHeight + cost }, // lb
     };

    std::unique_ptr<ContourArea> contourPtr(contour);

    return contourPtr;
} 


int main(int argc , char * argv[]) 
{

    // cv::Mat image = cv::imread("/home/onur/robotic_hws/src/proje/barcode_example1.jpg");
    cv::Mat image = cv::imread("/home/onur/robotic_hws/src/proje/corrosive.jpg");
    // cv::Mat image = cv::imread("/home/onur/robotic_hws/src/proje/littleSlandedQr2.jpg");
    // cv::Mat image = cv::imread("/home/onur/robotic_hws/src/proje/explosives.jpg");




    if (image.empty()) {
        std::cout << "Failed to load image!" << std::endl;
        return -1;
    }

    // cv::QRCodeDetector qrDet = cv::QRCodeDetector::QRCodeDetector();
    int kernelSize = 1;
    cv::Mat grayscaleImage;
    cv::cvtColor(image, grayscaleImage, cv::COLOR_BGR2GRAY);


    cv::Mat kernel = grayscaleImage(cv::Rect(0, 0, kernelSize, kernelSize)).clone();

    // Perform convoiution
    cv::Mat result;
    cv::Mat blackErosion;
    
    cv::filter2D(grayscaleImage, result, -1, kernel);
    auto contourPtr = createContour(result, 15);


    int radius = 10;

    // Draw the circle on the image
    std::vector<cv::Point> points = {
        contourPtr.get()->lu,
        contourPtr.get()->ru,
        contourPtr.get()->rb,
        contourPtr.get()->lb,
        
    };
    cv::polylines(grayscaleImage, points, true, cv::Scalar(0, 255, 0), 2);


    // cv::circle(grayscaleImage, contourPtr.get()->lu, radius, cv::Scalar(0, 0, 255), 1);
    // cv::circle(grayscaleImage, contourPtr.get()->ru, radius, cv::Scalar(0, 0, 255), 1);
    // cv::circle(grayscaleImage, contourPtr.get()->rb, radius, cv::Scalar(0, 0, 255), 1);
    // cv::circle(grayscaleImage, contourPtr.get()->lb, radius, cv::Scalar(0, 0, 255), 1);

    cv::Rect roiRect = cv::boundingRect(points);
    cv::Mat croppedImage = grayscaleImage(roiRect);
    cv::bitwise_not(croppedImage, croppedImage);

    std::string tessdataPath = "/usr/share/tesseract-ocr/4.00/tessdata/";
    setenv("TESSDATA_PREFIX", tessdataPath.c_str(), 1);
    tesseract::TessBaseAPI ocr;
    ocr.Init(NULL, "eng", tesseract::OEM_DEFAULT);
    ocr.SetPageSegMode(tesseract::PSM_AUTO);



    // Set the image data for OCR
    ocr.SetImage(croppedImage.data, croppedImage.cols, croppedImage.rows, 1, croppedImage.step);

    // Perform OCR and retrieve the recognized text
    std::string text = ocr.GetUTF8Text();
    std::cout << "Recognized Text:\n" << text << std::endl;


    cv::imshow("Cropped Image",croppedImage);

 
    // if needed text detection and cropped
    




    cv::imwrite("/home/onur/robotic_hws/src/proje/src/Cropped_Image.jpg",croppedImage);


    cv::imshow("gs", grayscaleImage);
    cv::imshow("result", result);


    cv::waitKey(0);


    return 0;
}
