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
        // //ROS_INFO("image size : %d - %d", middleColumndMat.rows, middleColumndMat.cols);
        // //ROS_INFO("%d ", middleColumndMat.at<uchar>(0,i));

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


    return intervals;
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
    if (!intervalsM.empty() && (intervalsM[0].second - intervalsM[0].first) > (intervalsC[0].second - intervalsC[0].first)) {
        intervalsC = intervalsM;
    }

    if (intervalsR.size() != 1 || intervalsC.size() != 1) {
        return false;
    }

    if (intervalsR[0].second - intervalsR[0].first > length || intervalsC[0].second - intervalsC[0].first > length) {
        area = (intervalsR[0].second - intervalsR[0].first) * intervalsC[0].second - intervalsC[0].first;
        //ROS_INFO("There is obstacle in road with area %d", area);

        return true;
    }
    return false;
}


bool onTheRamp(cv::Mat& mask) {
    auto onRamp = getMaskIntervals(mask, 400, 100); // middle mask is used for detection of ramp 
    if (onRamp.empty()) {
        //ROS_INFO("I am on ramp");
        return true;
    }
    else {
        //ROS_INFO("I am not in ramp");
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
        // //ROS_INFO("interval %d %d", interval.second, interval.first);     
        if (static_cast<int>((interval.second + interval.first) / 2) > 400) {
            // //ROS_INFO("obstacleDirection = -1");
            obstacleDirection = -1;
            // //ROS_INFO("obstacleDirection = %d", obstacleDirection);

        }
        else {
            // //ROS_INFO("+1");

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

bool isOnlyOneSegment(cv::Mat& mat) {
    int whitePixelThreshold = 0;
    for (int i = 0; i < mat.rows; i++) {
        for (int j = 0; j < mat.cols; j++) {
            if (static_cast<int>(mat.at<uchar>(i, j)) != 125 && static_cast<int>(mat.at<uchar>(i, j)) != 0) {
                // ROS_INFO("white exist at %d- %d",i,j);
                whitePixelThreshold++;
            }
            if(whitePixelThreshold == 250){
                return false;
            }
        }
    }
    // ROS_INFO("this image is only gray");
    return true;
}

void extraCheck(cv::Mat& mat) {
    int area;
    if (!isObstacle(mat, 125, area) && isOnlyOneSegment(mat)) {
        for (int i = 0; i < mat.rows; i++) {
            for (int j = 0; j < mat.cols; j++) {
                if (static_cast<int>(mat.at<uchar>(i, j)) == 125) {
                    // ROS_INFO("125 pixel tehere");

                    mat.at<uchar>(i, j) = 255;

                }
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
        std::vector<std::pair<int, int>> intervals = getMaskIntervals(mat, i, 100);
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

        for (auto interval : intervals) {
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

    extraCheck(mat);
    // throw std::runtime_excetion("");
    // cv::waitKey(3);
}


// TODO: hay aq aynı kodu tekrar yazamıyom.
bool searchForLine(cv::Mat& segmentedImage, cv::Point& point, int movementStep)
{
    int row = segmentedImage.cols - 1;

    unsigned long long point_x = 0;
    unsigned long long point_y = 0;

    int len = 0;

    int area;

    for (int i = 0; i < movementStep; i++) {
        // //ROS_INFO("getting interval %d" , i);

        auto intervals = getMaskIntervals(segmentedImage, row - i, 200);
        // //ROS_INFO("OKEY");


        if (intervals.size() == 1) {
            int mean = (intervals[0].first + intervals[0].second) / 2;
            point_x += mean;
            point_y += row - i;
            len++;
        }


    }
    if (len != 0) {

        point.x = (point_x / len);//+ static_cast<int>(450 * (area / (double)(800 * 800)));
        point.y = point_y / len;
    }
    else {
        cmd_vel.angular.z = 0;
        point.x = 400;
    }
    return true;
}


bool rotateState = false;

void rotateUntillLineInMiddle(cv::Point& point) {

    cmd_vel.linear.x = -0.1;
    cmd_vel.angular.z = -2.0;

    ROS_INFO("%d-%d = %d", point.x, point.y, std::abs(point.x - 400));

    if (std::abs(point.x - 600) < 200)
    {
        rotateState = false;
    }

}


int i = 0;
void cameraCallBack(const sensor_msgs::Image::ConstPtr& camera)
{
    cv::Point point;
    obstacleArea = 0; lineArea = 0;
    cv::Mat rgbImage(camera->height, camera->width, CV_8UC3, const_cast<uchar*>(&camera->data[0]),
        camera->step);

    cv::Mat hsv_image, mask;

    // gaussian blur for hsv conversion
    cv::Mat blurred_image;
    cv::GaussianBlur(rgbImage, blurred_image, cv::Size(5, 5), 0.0, 0.0);

    // // Convert RGB to HSV
    cv::cvtColor(blurred_image, hsv_image, cv::COLOR_RGB2HSV);
    // // Create mask for red color
    cv::Scalar lower_red(110, 50, 50);
    cv::Scalar upper_red(130, 255, 255);
    cv::inRange(hsv_image, lower_red, upper_red, mask);


    cv::imshow("RGB_IMAGE", mask);


    cv::imwrite("/home/onur/debug.jpg", mask);

    segmentMask(mask);


    searchForLine(mask, point, 100);

    ROS_INFO("obstacle area is %lld -- direct %d", obstacleArea, (obstacleDirection * static_cast<int>(350 * (obstacleArea / (double)(800 * 800)))));

    if (obstacleArea > 20000) {
        point.x += (obstacleDirection * 100);

    }

    ROS_INFO("point is at %d", point.x);

    // ROS_INFO("point %d -- %d", point.x, point.y);

    double dx = 400 - point.x;
    // int dy = point.y - 800;

    double radian = std::atan(dx / 100);

    ROS_INFO("radian is %lf", radian);


    cv::Rect roi(0, 700, hsv_image.cols, 100);

    cv::Mat img_roi = hsv_image(roi).colRange(250, 550);
    cv::imshow("ROI", img_roi);

    if (rotateState) {
        rotateUntillLineInMiddle(point);

    }
    else {

        if (cv::countNonZero(mask) == 0) {
            cv::Scalar avg_hue = mean(img_roi.rowRange(0, img_roi.rows).col(0));
            cv::Scalar avg_sat = mean(img_roi.rowRange(0, img_roi.rows).col(1));
            cv::Scalar avg_val = mean(img_roi.rowRange(0, img_roi.rows).col(2));
            // ROS_INFO_STREAM( "Average Hue: " << avg_hue[0] );
            // ROS_INFO_STREAM( "Average Saturation: " << avg_sat[0] );
            // ROS_INFO_STREAM( "Average Value: " << avg_val[0] );
            cmd_vel.linear.x = 0.4;
            cmd_vel.angular.z = 0;
            if (avg_hue[0] > 10 && avg_sat[0] > 10 && avg_val[0] > 10)
            {
                i++;
                if (i == 3) {
                    rotateState = true;
                    i = 0;
                }
            }

        }
        else {
            // //ROS_INFO("fuk");
            cmd_vel.linear.x = 0.80;
            cmd_vel.angular.z = radian;

        }
    }


    cv::drawMarker(mask, point, 100, cv::MARKER_CROSS, 5, 2);
    cv::imshow("endimage", mask);
    cv::imshow("image", rgbImage);
    cv::imwrite("/home/onur/endimage.jpg", mask);


    cv::waitKey(3);



    cmd_vel_pub.publish(cmd_vel);
}

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
