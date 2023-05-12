#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Image.h"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>

ros::Publisher cmd_vel_pub;
geometry_msgs::Twist cmd_vel;

unsigned long long int obstacleArea = 0;
int obstacleDirection = -1; // right -1 left +1 
unsigned long long int lineArea = 0;


// if robot in the ramp intervals are empty
// if robot in the ramp intervals are empty
// if robot in the ramp intervals are empty


std::vector<std::pair<int, int>> getMaskIntervals(cv::Mat& maskImage, int row, int threshold)
{
    int middleRow = row;
    cv::Mat middleRowMat = maskImage.row(middleRow).clone();

    std::vector<std::pair<int, int>> intervals;


    int intervalStart = 0, whiteCursor, blackCursor;
    bool lastlyWhite = false;
    for (int i = 0; i < middleRowMat.cols; i++) {
        // ROS_INFO("image size : %d - %d", middleRowMat.rows, middleRowMat.cols);
        // ROS_INFO("%d ", middleRowMat.at<uchar>(0,i));

        if (middleRowMat.at<uchar>(0, i) > threshold) {
            whiteCursor = i;
            lastlyWhite = true;
        }
        else {
            if (lastlyWhite)
            {
                intervals.push_back(std::pair<int, int>(intervalStart, whiteCursor));
                lastlyWhite = false;
            }
            blackCursor = i;
            intervalStart = blackCursor;
        }
    }

    if (intervalStart != middleRowMat.cols - 1) {
        intervals.push_back(std::pair<int, int>(intervalStart, whiteCursor));
    }

    for (auto item : intervals)
    {
        // ROS_INFO("%d - %d interval is white", item.first, item.second);
    }

    return intervals;
}

std::vector<std::pair<int, int>> getMaskIntervalsC(cv::Mat& maskImage, int column, int threshold)
{
    int middleColumn = column;
    cv::Mat middleColumndMat = maskImage.col(middleColumn).clone();

    std::vector<std::pair<int, int>> intervals;


    int intervalStart = 0, whiteCursor, blackCursor;
    bool lastlyWhite = false;
    for (int i = 0; i < middleColumndMat.rows; i++) {
        // ROS_INFO("image size : %d - %d", middleColumndMat.rows, middleColumndMat.cols);
        // ROS_INFO("%d ", middleColumndMat.at<uchar>(0,i));

        if (middleColumndMat.at<uchar>(i, 0) > threshold) {
            whiteCursor = i;
            lastlyWhite = true;
        }
        else {
            if (lastlyWhite)
            {
                intervals.push_back(std::pair<int, int>(intervalStart, whiteCursor));
                lastlyWhite = false;
            }
            blackCursor = i;
            intervalStart = blackCursor;
        }
    }

    if (intervalStart != middleColumndMat.cols - 1) {
        intervals.push_back(std::pair<int, int>(intervalStart, whiteCursor));
    }

    // for (auto item : intervals)
    // {
    //     // ROS_INFO("%d - %d interval is white", item.first, item.second);
    // }



    return intervals;
}



bool onTheRamp(cv::Mat& mask) {
    auto onRamp = getMaskIntervals(mask, 400, 100); // middle mask is used for detection of ramp 
    if (onRamp.empty()) {
        ROS_INFO("I am on ramp");
        return true;
    }
    else {
        ROS_INFO("I am not in ramp");
        return false;
    }

}

void applyColouredInterval(cv::Mat& mat, int i, std::tuple<std::pair<int, int>, int, int> colouredInterval)
{

    int colors[2] = { 125,255 };

    std::pair<int, int> interval = std::get<0>(colouredInterval);
    int colorCursor = std::get<1>(colouredInterval);

    for (int j = interval.first; j <= interval.second; j++)
    {
        if (!colorCursor) {
            obstacleArea += 1;

        }
        else {
            lineArea += 1; // TODO: can be removed
        }
        mat.at<uchar>(i, j) = colors[colorCursor];
    }
    if (colorCursor == 0) {   
        // ROS_INFO("interval %d %d", interval.second, interval.first);     
        if (static_cast<int>((interval.second + interval.first) / 2) > 400) {
            // ROS_INFO("obstacleDirection = -1");
            obstacleDirection = -1;
            // ROS_INFO("obstacleDirection = %d", obstacleDirection);

        } else {
            // ROS_INFO("+1");

            obstacleDirection = 1;
        }
    }
    // cv::imshow("appplied_colorization", mat);
    // cv::waitKey(3);
    // //std::cout << "waiting" << std::endl;
}

bool isInColouredIntervals(std::vector<std::tuple<std::pair<int, int>, int, int>>& colouredVector, std::pair<int, int>& interval) {
    if (colouredVector.empty()) {
        return false;
    }
    if (colouredVector.size() == 2) {
        return true;
    }
    else {
        for (auto item : colouredVector) {
            auto itemInterval = std::get<0>(item);
            if (itemInterval.second < interval.first && itemInterval.second < interval.second) {
                return false;
            }
            else if (interval.second < itemInterval.first && interval.second < itemInterval.second) {
                return false;
            }
        }

    }
    return true;
}

void applyAndUpdateColouredInterval(cv::Mat& mat, int i, std::vector<std::tuple<std::pair<int, int>, int, int>>& colouredIntervals, std::pair<int, int>& interval) {
    if (colouredIntervals.size() == 2) {
        int mean = (interval.first + interval.second) / 2;
        auto colouredInterval_0 = std::get<0>(colouredIntervals[0]);
        int colouredMean_0 = (colouredInterval_0.first + colouredInterval_0.second) / 2;
        int row_0 = std::get<2>(colouredIntervals[0]);

        auto colouredInterval_1 = std::get<0>(colouredIntervals[1]);
        int colouredMean_1 = (colouredInterval_1.first + colouredInterval_1.second) / 2;
        int row_1 = std::get<2>(colouredIntervals[1]);

        if (std::abs(colouredMean_0 - mean) + std::abs(i - row_0) < std::abs(colouredMean_1 - mean) + std::abs(i - row_1)) {
            colouredIntervals[0] = std::tuple<std::pair<int, int>, int, int>{ interval, std::get<1>(colouredIntervals[0]), i };
            applyColouredInterval(mat, i, colouredIntervals[0]);
        }
        else {
            colouredIntervals[1] = std::tuple<std::pair<int, int>, int, int>{ interval, std::get<1>(colouredIntervals[1]), i };
            applyColouredInterval(mat, i, colouredIntervals[1]);
        }
    }
    else {

        for (auto& item : colouredIntervals) {
            auto itemInterval = std::get<0>(item);
            if (itemInterval.second < interval.first && itemInterval.second < interval.second) {
                continue;
            }
            else if (interval.second < itemInterval.first && interval.second < itemInterval.second) {
                continue;
            }
            else {
                item = std::tuple<std::pair<int, int>, int, int>{ interval, std::get<1>(item), i };
                applyColouredInterval(mat, i, item);
                break;
            }
        }
    }

}

void segmentMask(cv::Mat& mat) {
    size_t rowLength = mat.rows;
    size_t colLength = mat.cols;

    int colorCursor = 0;


    std::vector<std::tuple<std::pair<int, int>, int, int>> coloredIntervals; // first pair column intervals, second int colorCursor, third int row
    for (int i = 1; i < rowLength; i++) {
        if (i == 700) {
            //  cv::drawMarker( mat, cv::Point(static_cast<int>((interval.first + interval.second)/2), i), 50, cv::MARKER_CROSS, 5, 2 );
            //     cv::waitKey(3);
            //std::cout << "hay aq" << std::endl;
        }
        std::vector<std::pair<int, int>> intervals = getMaskIntervals(mat, i, 100);
        if (intervals.size() == 1)
        {
            //std::cout << "interval" << intervals[0].second << " " << intervals[0].first << std::endl;
        }
        if (intervals.size() >= 2) {
            // union small segments
            //std::cout << "interval" << intervals[0].second << " " << intervals[0].first << std::endl;
            //std::cout << "interval" << intervals[1].second << " " << intervals[1].first << std::endl;
            std::vector<std::pair<int, int>> segments2Union;
            int first = 0, second = 0, j;
            for (j = 0; j < intervals.size() - 1; j++) {
                if (intervals[j + 1].first - intervals[j].second < 10) {
                    second = j + 1;
                    if (second == intervals.size() - 1) {
                        segments2Union.push_back({ first,second });
                    }
                }
                else {
                    if (first != second) {
                        segments2Union.push_back({ first,second });
                        first = second + 1;

                    }
                }
            }
            for (j = 0; j < segments2Union.size(); j++)
            {
                intervals[j].first = intervals[segments2Union[j].first].first;
                intervals[j].second = intervals[segments2Union[j].second].second;
            }

            if (segments2Union.size() != 0) {
                intervals.erase(intervals.begin(), intervals.end());
            }

            //std::cout << "error" << std::endl;
        }
        if (i == 700) {
            //  cv::drawMarker( mat, cv::Point(static_cast<int>((interval.first + interval.second)/2), i), 50, cv::MARKER_CROSS, 5, 2 );
            //     cv::waitKey(3);
            //std::cout << "hay aq" << std::endl;
        }
        for (auto interval : intervals) {
            if (i == 700) {
                // cv::drawMarker( mat, cv::Point(static_cast<int>((interval.first + interval.second)/2), i), 50, cv::MARKER_CROSS, 5, 2 );
                // cv::waitKey(3);
                ////std::cout << "errr" << std::endl;

            }
            if (!isInColouredIntervals(coloredIntervals, interval)) {
                std::tuple<std::pair<int, int>, int, int> colouredInterval {interval, colorCursor++, i};
                coloredIntervals.push_back(colouredInterval);
                applyColouredInterval(mat, i, colouredInterval);

            }
            else {
                applyAndUpdateColouredInterval(mat, i, coloredIntervals, interval);
                // cv::imshow("ColouredInterval", mat);
                // cv::waitKey(3);
            }

        }

    }



    // cv::imshow("segmented", mat);

    // throw std::runtime_excetion("");
    cv::waitKey(3);
}

bool isObstacle(cv::Mat& mat, const int&& length, int& area) {
    auto intervalsR = getMaskIntervals(mat, 400, 100);
    auto intervalsC = getMaskIntervalsC(mat, 0, 100);
    auto intervalsC_right = getMaskIntervalsC(mat, 799, 100);
    auto intervalsM = getMaskIntervalsC(mat, 300, 100);

    if (intervalsC.empty() && !intervalsC_right.empty())
    {
        intervalsC = intervalsC_right;
    }
    if (!intervalsM.empty() && (intervalsM[0].second - intervalsM[0].first) > intervalsC[0].second - intervalsC[0].first) {
        intervalsC = intervalsM;
    }
    ROS_INFO("%ld:%ld sizes ", intervalsC.size(), intervalsC.size());
    ROS_INFO("%ld:%ld sizes R", intervalsR.size(), intervalsR.size());

    if (intervalsR.size() != 1 || intervalsC.size() != 1) {
        return false;
    }

    if (intervalsR[0].second - intervalsR[0].first > length || intervalsC[0].second - intervalsC[0].first > length) {
        area = (intervalsR[0].second - intervalsR[0].first) * intervalsC[0].second - intervalsC[0].first;
        ROS_INFO("There is obstacle in road with area %d", area);

        return true;
    }
    return false;
}

// hay aq aynı kodu tekrar yazamıyom.
// bool searchForLine(cv::Mat& segmentedImage, cv::Point& point, int movementStep)
// {
//     int row = segmentedImage.cols - 1;

//     unsigned long long point_x = 0;
//     unsigned long long point_y = 0;

//     int len = 0;

//     int area;

//     // bool isObstacleSeen = isObstacle(segmentedImage, 300, area);
//     // bool isRobotOnRamp = onTheRamp(segmentedImage);

//     // if(isRobotOnRamp) {
//     //     ROS_INFO("robot on ramp");
//     // }

//     // if(isObstacleSeen) {
//     //     ROS_INFO("obstacle seen with area %d", area);
//     // }

//     ROS_INFO("obstacleArea %lld", obstacleArea);
//     ROS_INFO("obstacleDirection %d", obstacleDirection);
//     ROS_INFO("lineArea %lld", lineArea);

//     for (int i = 0; i < movementStep; i++) {
//         // ROS_INFO("getting interval %d" , i);

//         auto intervals = getMaskIntervals(segmentedImage, row - i, 200);
//         // ROS_INFO("OKEY");


//         if (intervals.size() == 1) {
//             int mean = (intervals[0].first + intervals[0].second) / 2;
//             point_x += mean;
//             point_y += row - i;
//             len++;
//         }


//     }
//     if(len!=0) {

//         ROS_INFO("area %d", area);
//         ROS_INFO("turn with %d", static_cast<int>(450 * (area / (double)(800 * 800))));
//         ROS_INFO("Vehicle now in above");
//         point.x += (point_x / len);//+ static_cast<int>(450 * (area / (double)(800 * 800)));
//         point.y = point_y / len;
//     }

//     return true;
// }

// int i = 0;

int vehicleStateTransition = 1; // it is above or below
bool previouslyRamp = false;

// // movement state can be set  by hand on ramp different on the line different etc.
bool searchForLine(cv::Mat& segmentedImage, cv::Point& point, int movementStep) {
    // ROS_INFO("searching for line");
    int row = segmentedImage.cols - 1;

    unsigned long long point_x = 0;
    unsigned long long point_y = 0;
    int len = 0;

    int area;

    bool isObstacleSeen = isObstacle(segmentedImage, 400, area);
    bool isRobotOnRamp = onTheRamp(segmentedImage);

    for (int i = 0; i < movementStep; i++) {
        // ROS_INFO("getting interval %d" , i);

        auto intervals = getMaskIntervals(segmentedImage, row - i, 200);
        // ROS_INFO("OKEY");


        if (intervals.size() == 1) {
            int mean = (intervals[0].first + intervals[0].second) / 2;
            point_x += mean;
            point_y += row - i;
            len++;
        }


    }
    if (previouslyRamp == true && isRobotOnRamp == false) {
        vehicleStateTransition = -vehicleStateTransition;
        ROS_INFO("vehicle translation changed  %d", vehicleStateTransition);
    }

    if (isRobotOnRamp)
    {
        previouslyRamp = true;
    }
    else {
        previouslyRamp = false;
    }


    // ROS_INFO("is something wrong here");
    if (len != 0) {
        ROS_INFO("vehcile %d", vehicleStateTransition );
        if (vehicleStateTransition == -1) {
            ROS_INFO("area %d", area);
            ROS_INFO("turn with %d", static_cast<int>(700 * (obstacleArea / (double)(800 * 800))));
            ROS_INFO("Vehicle now in above");
            point.x += (point_x / len) + static_cast<int>(700 * (obstacleArea / (double)(800 * 800)));
            point.y = point_y / len;

        }

        if (vehicleStateTransition == -1 && std::abs(static_cast<int>(point_x / len) - static_cast<int>(point.x)) > 30 && std::abs(static_cast<int>(point.y) - static_cast<int>(point_y / len)) > 30) {

            ROS_INFO("outlier with %d, %d : %d %d", point.x, point.y, static_cast<int>(point_x / len), static_cast<int>(point_y / len));
            cmd_vel.linear.x = 0.9;
        }
        else {
            point.x = point_x / len;
            point.y = point_y / len;
        }

        return true;
    }
    else {


        ROS_INFO("IS OBSTACLE SEEN %d: on ramp %d", isObstacleSeen, isRobotOnRamp);



        if (!isObstacleSeen || isRobotOnRamp) {
            ROS_INFO("Recovery Behaviour");
            point_x = 0;
            point_y = 0;
            for (int i = 0; i < movementStep; i++) {
                // ROS_INFO("getting interval %d" , i);

                auto intervals = getMaskIntervals(segmentedImage, row - i, 100);
                // ROS_INFO("OKEY");


                if (intervals.size() == 1) {
                    int mean = (intervals[0].first + intervals[0].second) / 2;
                    point_x += mean;
                    point_y += row - i;
                    len++;
                }

                // cv::imwrite("/home/onur/imageWithPoint.jpg", mask);

            }
            if (len != 0) {
                // outlier detection

                if (vehicleStateTransition == -1 && std::abs(static_cast<int>(point_x / len) - static_cast<int>(point.x)) > 40 && std::abs(static_cast<int>(point.y) - static_cast<int>(point_y / len)) > 40) {

                    ROS_INFO("outlier with %d, %d : %d %d", point.x, point.y, static_cast<int>(point_x / len), static_cast<int>(point_y / len));
                    cmd_vel.linear.x = 0.9;
                }
                else {
                    point.x = point_x / len;
                    point.y = point_y / len;
                }
            }
            cmd_vel.linear.x = 0;
            return false;

        } if (isObstacleSeen && !isRobotOnRamp) {

            ROS_INFO("Can i turn only see cylinder %d", static_cast<int>(750 * (obstacleArea / (double)(800 * 800))));
            point.x += static_cast<int>(obstacleArea * (area / (double)(800 * 800)));

            return true;
        }
        else {
            std::stringstream file;
            // file << "/home/onur/errorMask" << i << ".jpg";
            // i++;
            // cv::imwrite(file.str(), segmentedImage);

            ROS_INFO("BOŞ DÖNDÜ BURASI HATA OLUŞTURABİLİR!!!!");
            return false;
        }
    }

}

cv::Point point;
bool rotated=false;


void cameraCallBack(const sensor_msgs::Image::ConstPtr& camera)
{
    obstacleArea = 0; lineArea = 0;
    cv::Mat rgbImage(camera->height, camera->width, CV_8UC3, const_cast<uchar*>(&camera->data[0]),
        camera->step);

    cv::Mat hsv_image, mask, masked_image, edges, gray_image;

    // gaussian blur for hsv conversion
    cv::Mat blurred_image;
    cv::GaussianBlur(rgbImage, blurred_image, cv::Size(5, 5), 0.0, 0.0);

    // // Convert RGB to HSV
    cv::cvtColor(blurred_image, hsv_image, cv::COLOR_RGB2HSV);
    // // Create mask for red color
    cv::Scalar lower_red(110, 50, 50);
    cv::Scalar upper_red(130, 255, 255);
    cv::inRange(hsv_image, lower_red, upper_red, mask);


    // erode element
    cv::cvtColor(rgbImage, gray_image, cv::COLOR_BGR2GRAY);
    cv::Mat erodeElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10, 10));
    cv::Mat dilateElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10, 10));

    cv::erode(gray_image, gray_image, erodeElement);
    cv::dilate(gray_image, gray_image, dilateElement);

    // Apply Canny edge detection to the blurred image
    cv::Canny(gray_image, edges, 100, 300, 3);
    cv::imshow("edges", edges);

    auto intervals = getMaskIntervalsC(edges, 400, 100);

    // ROS_INFO("canny edge from aboce : %d %d", intervals[0].second, intervals[1].first);




    // cv::Mat gray_image;
    // cv::cvtColor(rgbImage, gray_image, cv::COLOR_BGR2GRAY);

    cv::imshow("RGB_IMAGE", mask);


    cv::imwrite("/home/onur/debug.jpg", mask);

    segmentMask(mask);
    // isObstacle(mask);


    searchForLine(mask, point, 50);

    // auto onRamp = getMaskIntervals(mask, 400, 100); // middle mask is used for detection of ramp 
    // if(onRamp.empty()) {
    //     ROS_INFO("I am on ramp");
    // } else {
    //     ROS_INFO("I am not in ramp");
    // }

    int dx = point.x - 400;
    int dy = point.y - 800;


    double radian = std::atan(dx / (double)dy);

    // std::vector<cv::Mat> hsv_channels;
    // split(hsv_image, hsv_channels);
    ROS_INFO("%d",cv::countNonZero(mask) );

    cv::Rect roi(0, 700, hsv_image.cols, 100);

    cv::Mat img_roi = hsv_image(roi).colRange(0,300);

    if( cv::countNonZero(mask) == 0 ) {
        
        // Calculate the average values of Hue, Saturation, and Value channels within the region of interest
        cv::Scalar avg_hue = mean(img_roi.rowRange(0, img_roi.rows).col(0));
        cv::Scalar avg_sat = mean(img_roi.rowRange(0, img_roi.rows).col(1));
        cv::Scalar avg_val = mean(img_roi.rowRange(0, img_roi.rows).col(2));


        if(avg_val[0] < 80.0 && avg_hue[0] > 14 && avg_sat[0] > 14  && avg_val[0] > 14) {
            ROS_INFO("rotating... ");
            rotated = true;
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = -0.9;

        }
        else {

            if(!rotated)
            {
                ROS_INFO("Go direct");
                cmd_vel.linear.x = 0.9;
                cmd_vel.angular.z = 0;
            }
        }
        // Print the average values of Hue, Saturation, and Value channels
        ROS_INFO_STREAM( "Average Hue: " << avg_hue[0] );
        ROS_INFO_STREAM( "Average Saturation: " << avg_sat[0] );
        ROS_INFO_STREAM( "Average Value: " << avg_val[0] );

        cv::imshow("ROI", img_roi);

    } else {
        // ROS_INFO("fuck");
        cmd_vel.linear.x = 0.9;
        cmd_vel.angular.z = radian;

    }

    cv::imshow("endimage", mask);
    cv::imshow("image",rgbImage);
    cv::drawMarker(mask, point, 50, cv::MARKER_CROSS, 5, 2);
    cv::imwrite("/home/onur/endimage.jpg", mask);


    cv::waitKey(3);

    // cmd_vel.linear.x = 1;


    // ROS_INFO("%d:%d -- %d:%d -- %f", point.x, point.y, dx, dy, radian * (180.0 / 3.141592653589793238463));


    // cmd_vel.linear.x = 1;
    // cmd_vel.angular.z = radian;

    cmd_vel_pub.publish(cmd_vel);
}

// void cameraCallBack(const sensor_msgs::Image::ConstPtr& camera){
// 	//ASAGIDA BULUNAN IF KOMUTU ORNEK OLARAK VERILMISTIR. SIZIN BURAYI DEGISTIRMENIZ BEKLENMEKTEDIR
// 	//BURDAN SONRASINI DEGISTIR
//     cv::Point lineDirection;


// 	cv::Mat rgb_image(camera->height, camera->width, CV_8UC3, const_cast<uchar *>(&camera->data[0]),
//     camera->step);
// 	cv::Mat hsv_image, mask, masked_image, edges;

// 	// gaussian blur for hsv conversion
//     cv::Mat blurred_image;
//     cv::GaussianBlur(rgb_image, blurred_image, cv::Size(5, 5), 0.0, 0.0);

//     // // Convert RGB to HSV
//     cv::cvtColor(blurred_image, hsv_image, cv::COLOR_RGB2HSV);
//     // // Create mask for red color
//     cv::Scalar lower_red(110, 50, 50);
//     cv::Scalar upper_red(130, 255, 255);
//     cv::inRange(hsv_image, lower_red, upper_red, mask);


// 	cv::Mat gray_image;
//     cv::cvtColor(rgb_image, gray_image, cv::COLOR_BGR2GRAY);

// 	cv::Mat erodeElement = cv::getStructuringElement( cv::MORPH_RECT,cv::Size(10,10));
// 	cv::Mat dilateElement = cv::getStructuringElement( cv::MORPH_RECT,cv::Size(10,10));


// 	// states are
// 	// black reach state 
// 	// white reached state
// 	// on ramp state up
// 	// on ramp state down
// 	// going on ramp up
// 	// going on ramp down


//     cv::erode(gray_image,gray_image,erodeElement);
//     cv::dilate(gray_image,gray_image,dilateElement);

// 	// Apply Canny edge detection to the blurred image
//     cv::Canny(gray_image, edges, 100, 300, 3);

// 	// cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
//     // cv::morphologyEx(edges, edges, cv::MORPH_OPEN, kernel);


// 	double rho = 1;  // distance precision in pixel, i.e. 1 pixel
//     double angle = CV_PI / 180;  // angular precision in radian, i.e. 1 degree
//     int min_threshold = 10;  // minimal of votes


// 	int height = mask.rows;	
//     int width = mask.cols;

//     auto onRamp = getMaskIntervals(mask, 400); // middle mask is used for detection of ramp 
//     if(onRamp.empty()) {
//         // ROS_INFO("I am on ramp");
//     } else {
//         // ROS_INFO("I am not in ramp");
//     }
//     cv::Mat maskHalf = cv::Mat::zeros(mask.size(), mask.type());

//     // only focus bottom half of the screen
//     cv::Point pts[4] = {
//         cv::Point(0, height * 1 / 1.2),
//         cv::Point(width, height * 1 / 1.2),
//         cv::Point(width, height),
//         cv::Point(0, height)
//     };
//     cv::fillConvexPoly(maskHalf, pts, 4, cv::Scalar(255, 0, 0));

//     cv::Mat croppedMask;
//     cv::bitwise_and(mask, maskHalf, croppedMask);

//     std::vector<cv::Vec4i> line_segments;

// 	cv::imshow("cropped mask", croppedMask);

//     cv::Mat croppedEdges;
//     cv::Canny(croppedMask, croppedEdges, 50, 200,3);

// 	cv::HoughLinesP(croppedEdges, line_segments, rho, angle, min_threshold, 8, 4);

//     // Draw lines on the original image
//     cv::Mat line_image = gray_image.clone();

//     // take medium point of line segments
//     int dataPointsLen = line_segments.size();
//     cv::Mat dataPoints(dataPointsLen, 2, CV_32F);
//     cv::Mat labels;
//     cv::Mat centers;

//     for (int i = 0, j=0 ; i < dataPointsLen ; i++, j++)
//     {   
//         dataPoints.at<float>(j,0) = static_cast<float> (line_segments[i][0]);
//         dataPoints.at<float>(j,1) = static_cast<float> (line_segments[i][1]);
//         dataPoints.at<float>(j+1,0) = static_cast<float> (line_segments[i][2]);
//         dataPoints.at<float>(j+1,1) = static_cast<float> (line_segments[i][3]);
//         // ROS_INFO("point %f %f : %f %f", dataPoints.at<float>(i,0), dataPoints.at<float>(i,1), dataPoints.at<float>(i+1,0), dataPoints.at<float>(i+1,1) );
//     }

//     int K=2;

//     if(line_segments.size() == 0) {
//         return ;
//     }
//     ROS_INFO("N-K: %ld, %d" , line_segments.size(), K);

//     cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 10, 1.0);
//     int attempts = 3;

//     int flags = cv::KMEANS_RANDOM_CENTERS;

//     cv::kmeans(dataPoints, K, labels, criteria, attempts, flags, centers);

//     ROS_INFO_STREAM ( "Cluster labels: " << std::endl << labels << std::endl << "Centroids: " << std::endl << centers << std::endl);

//     // if distance between centroid greater than 50
//     // applyIntervalElimination Logic

//     cv::Point centroid_1 = cv::Point(static_cast<int>(centers.at<float>(0)),static_cast<int>(centers.at<float>(1)));
//     cv::Point centroid_2 = cv::Point(static_cast<int>(centers.at<float>(2)),static_cast<int>(centers.at<float>(3)));

//     // cv::drawMarker(line_image, cv::Point(static_cast<int>(centers.at<float>(0)),static_cast<int>(centers.at<float>(1))), 2, 255 ,-1);
//     cv::drawMarker(line_image, centroid_1, 255, cv::MARKER_CROSS, 5, 2);
//     cv::drawMarker(line_image, centroid_2, 255, cv::MARKER_CROSS, 5, 2);

//     // find which one is eliminated
//     auto intervalsCentroid_1 = getMaskIntervals(mask, centroid_1.y); 
//     auto intervalsCentroid_2 = getMaskIntervals(mask, centroid_2.y);

//     // ROS_INFO("first centroid");
//     int min = 65536;  

//     int correctLabel=-1;
//     int isBothInterval=0;

//     std::pair<int,int> minPair;
//     for (auto interval : intervalsCentroid_1) {
//         // ROS_INFO("f%d-%d", interval.first, interval.second);
//         if((interval.second - interval.first) < min)
//         {
//             min = (interval.second - interval.first);
//             minPair = interval;
//         }
//     }

//     if (centroid_1.x > minPair.first-5 && centroid_1.x < minPair.second+5)
//     {
//         correctLabel = 0;
//         // ROS_INFO("label 0 ");
//         isBothInterval++;
//     }

//     // ROS_INFO("second centroid");
//     min = 65536;
//     for ( auto interval : intervalsCentroid_2) {
//         // ROS_INFO("s%d- %d", interval.first, interval.second);
//         if((interval.second - interval.first) < min)
//         {
//             min = (interval.second - interval.first);
//             minPair = interval;
//         }

//     }

//     if (centroid_2.x > minPair.first-5 && centroid_2.x < minPair.second+5)
//     {
//         correctLabel = 1;
//         // ROS_INFO("label 1");
//         isBothInterval++;
//     }

//     // use all line segments
//     if(isBothInterval == 2) {

//         unsigned long long x=0, y=0;
//         for (auto line : line_segments)
//         {
//             // ROS_INFO("%d,%d", line[2], line[3]);

//             cv::line(line_image, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(0, 0, 255), 2);

//             x += line[0];
//             x += line[2];
//             y += line[1];
//             y += line[3];
//         }

//         if(line_segments.size() > 0 ) {
//             y = y / (line_segments.size() * 2);
//             x = x / (line_segments.size() * 2);
//         } else {
//             x = 400;
//             y = 700;
//         }

//         // ROS_INFO("%lld- %lld", x, y);



//         lineDirection = cv::Point(x,y);
//         cv::circle(line_image, lineDirection, 2, 255 ,-1);

//     }
//     else 
//     {
//         if(correctLabel == 0)
//         {
//             unsigned long long size = 0;
//             unsigned long long x=0, y=0;


//             // for (auto line : line_segments)
//             // {
//             //     cv::line(line_image, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(0, 0, 255), 2);
//             // }

//             for ( int i = 0 ; i < dataPointsLen ; i++)
//             {
//                 if(labels.at<int>(i,0) == false) {
//                     size++;
//                     x += static_cast<int>(dataPoints.at<float>(i,0));
//                     y += static_cast<int>(dataPoints.at<float>(i,1));
//                     // ROS_INFO("label with points %d-%d", static_cast<int>(dataPoints.at<float>(i,0)), static_cast<int>(dataPoints.at<float>(i,1) ));
//                     cv::drawMarker(line_image, cv::Point(x,y), 255, cv::MARKER_DIAMOND, 25, 2);

//                 }


//             }
//             y = y / size;
//             x = x / size;
//             lineDirection = cv::Point(x, y);
//             cv::circle(line_image, lineDirection, 4, 255 ,-1);

//         } else {
//             unsigned long long x=0, y=0;
//             unsigned long long size = 0;

//             // for (auto line : line_segments)
//             // {
//             //     cv::line(line_image, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(0, 0, 255), 2);
//             // }


//             for ( int i = 0 ; i < dataPointsLen ; i++)
//             {
//                 if(labels.at<int>(i,0) == true) {
//                     size++;
//                     x += static_cast<int>(dataPoints.at<float>(i,0));
//                     y += static_cast<int>(dataPoints.at<float>(i,1));
//                     // ROS_INFO("label with points %d-%d", static_cast<int>(dataPoints.at<float>(i,0)), static_cast<int>(dataPoints.at<float>(i,1) ));
//                     cv::drawMarker(line_image, cv::Point(x,y), 255, cv::MARKER_TRIANGLE_UP, 25, 2);

//                 }


//             }
//             y = y / (size);
//             x = x / (size);
//             lineDirection = cv::Point(x, y);
//             cv::circle(line_image, lineDirection, 4, 255 ,-1);

//         }
//     }


// 	cv::imshow("direction of line", line_image);


//     // Apply mask to HSV image
//     // cv::bitwise_and(hsv_image, hsv_image, masked_image, mask);

//     // // Convert HSV to RGB
//     // cv::cvtColor(masked_image, masked_image, cv::COLOR_HSV2RGB);

//     // Detect edges in masked RGB image
//     // cv::Canny(masked_image, edges, 100, 200);

//     // Display original RGB image, masked RGB image with edges, and wait for a key press
//     // cv::imshow("RGB Image", rgb_image);

// 	// Display the original and blurred images side by side
//     cv::Mat side_by_side;
//     cv::hconcat(mask, edges, side_by_side);
//     cv::imshow("blue coloured image  vs. canny edges of environment", side_by_side);

//     // cv::imshow("Masked RGB Image with Edges", edges);
//     cv::waitKey(3);


//     // ROS_INFO("%d %d", lineDirection.x, lineDirection.y);


//     int dx =  lineDirection.x - 400;
//     int dy = lineDirection.y - 800;


//     double radian = std::atan(dy/(double)dx);

//     // ROS_INFO("%f", radian * (180.0/3.141592653589793238463));

//     // cmd_vel.linear.x = 0.1;
//     // cmd_vel.angular.z = radian;

// 	cmd_vel_pub.publish(cmd_vel);
// }

int main(int argc, char** argv) {
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
