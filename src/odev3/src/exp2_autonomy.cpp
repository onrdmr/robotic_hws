#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "ros/console.h"

ros::Publisher cmd_vel_pub;
geometry_msgs::Twist cmd_vel;


// localisation is need for track trajectory we draw for localized lidar data

// gerekenler 
// 1. Prediction Model (olabilecek olası path e ait değerler alınır:)

// 2. Cost function (düz gitmesi için belirleyici olan )

// 3. Prediction horizon

// 4. Terminal Constraints 

void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& laser) {
    //ASAGIDA BULUNAN IF KOMUTU ORNEK OLARAK VERILMISTIR. SIZIN BURAYI DEGISTIRMENIZ BEKLENMEKTEDIR
    //BURDAN SONRASINI DEGISTIR

    // list of laser data that are evaluated in each cycle
    constexpr int points[]{  220, 240, 260, 280, 300, 320, 340,360, 380,400, 420, 440, 460, 480, 500,520, 540, 560, 580, 600, 620, 640 ,660, 680 ,700, 720, 740, 760, 780, 800, 820, 840, 860};

    // center of all points
    constexpr int center{ 540 };

    // gap between each point
    constexpr int gap{ points[1] - points[0] };

    constexpr int mgap{ center - points[0] };

    int maxPoint{ center };
    int minPoint{ center };

    double rightSum = 0;
    double leftSum = 0;

    

    for (int point : points) {
        if (laser->ranges[point] < 0.30) {
            continue;
        }

        if (laser->ranges[point] > laser->ranges[maxPoint]) {
            maxPoint = point;
        }

        

        if ( laser->ranges[point] < laser->ranges[minPoint]) { // laser->ranges[point] > 0.20 &&
            minPoint = point;
        }

        if (point < center) {
            rightSum += laser->ranges[point];
        }
        else if (point > center) {
            leftSum += laser->ranges[point];
        }
    }

    int maxDirection = ((rightSum > leftSum) ? -1 : 1);
    int minDirection;

    if (minPoint > center) {
        minDirection = 1;
    }
    else if (minPoint < center) {
        minDirection = -1;
    }
    else {
        minDirection = 0;
    }

    constexpr double linearBase = 0.50;
    constexpr double angularBase = 2.00;

    constexpr double threshold = 0.60;

    static int turnDirection = 1;

    // if (laser->ranges[center] > 100) {
    //     ROS_ERROR("looking in the air");
    //     cmd_vel.linear.x = -0.30;
    // }
    // else if (laser->ranges[center] > 100) {
    //     cmd_vel.linear.x = -0.30;
    // }
    else if (laser->ranges[center] < 0.50 && laser->ranges[center - gap * 2] < 0.50 && laser->ranges[center + gap * 2] < 0.50) {

        ROS_ERROR("going back");
        cmd_vel.linear.x = -0.20;
        cmd_vel.angular.z = -2.00 ;
    }
    else {
        ROS_ERROR("going forward");
        if(std::abs(rightSum - leftSum) < 0.2 ){
            ROS_ERROR("no angular");
            cmd_vel.angular.z = 0;

        }
        else {
            cmd_vel.angular.z = 1.50 * maxDirection;
        }

        cmd_vel.linear.x = 0.40;


        turnDirection = maxDirection;
        // * (0.3 + static_cast<double>(std::abs(center - maxPoint)) / gap);


        if (laser->ranges[minPoint] < threshold && maxDirection == minDirection) {
            ROS_ERROR("avoiding wall");

            if (laser->ranges[minPoint] < threshold / 2) {
                cmd_vel.linear.x = 0.00;
            }
            else {
                cmd_vel.linear.x = 0.20; /* * (0.4 + (laser->ranges[minDirection] > 1 ? 1 : laser->ranges[minDirection]));*/
            }
            cmd_vel.angular.z = 2.00 * -minDirection;

            turnDirection = -minDirection;

            // *(laser->ranges[minPoint] > 0.30 ? maxDirection : -minDirection)
            // * (1.2 - static_cast<double>(std::abs(center - minPoint)) / gap)
            // * (1.3 - static_cast<double>(laser->ranges[minPoint]) / threshold);
        }
    }

    // if (laser->ranges[center] < 0.20 || laser->ranges[center + gap * 2] < 0.20 || laser->ranges[center - gap * 2] < 0.20) {
    //     cmd_vel.linear.x = -0.30;
    // }


    // if (laser->ranges[center] < threshold) {
    //     cmd_vel.linear.x = 0.00;

    //     if (laser->ranges[center - 360] > threshold) {
    //         cmd_vel.angular.z = -angularBase;
    //     }
    //     else if (laser->ranges[center + 360] > threshold) {
    //         cmd_vel.angular.z = angularBase;
    //     }
    //     else {
    //         cmd_vel.angular.z = angularBase;
    //     }
    // }
    // else {
    //     cmd_vel.linear.x = linearBase;

    //     cmd_vel.angular.z += angularBase * -minSide
    //         * ((center - minDirection) ? (static_cast<double>(1) - static_cast<double>(std::abs(center - minDirection)) / gap) : static_cast<double>(1));
    // }

    // cmd_vel.linear.x = linearBase * (laser->ranges[minDirection] > 1 ? 1 : laser->ranges[minDirection]);

    // cmd_vel.angular.z = angularBase * maxSide * (static_cast<double>(std::abs(center - maxDirection)) / gap);

    // cmd_vel.angular.z += angularBase * -minSide
    //     * ((center - minDirection) ? (static_cast<double>(1) - static_cast<double>(std::abs(center - minDirection)) / gap) : static_cast<double>(1));


    ROS_ERROR("maxSide: %d", maxDirection);
    ROS_ERROR("linear: %lf", cmd_vel.linear.x);
    ROS_ERROR("angular: %lf", cmd_vel.angular.z);
    ROS_ERROR("max point: %lf", laser->ranges[maxPoint]);
    ROS_ERROR("min point: %lf\n", laser->ranges[minPoint]);

    //BURDAN SONRASINA DOKUNMA

    cmd_vel_pub.publish(cmd_vel);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "exp2_autonomy");
    ros::NodeHandle nh;

    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;

    ros::Subscriber laser_sub = nh.subscribe("/robot1/hokuyo", 1000, laserCallBack);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/robot1/cmd_vel", 1000);

    ros::spin();

    return 0;
}
