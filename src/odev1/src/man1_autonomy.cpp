#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

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
    constexpr int directions[]{ 340, 380, 420, 460, 500, 540, 600, 640, 680, 720, 760 };

    // center of all directions
    constexpr int centerDirection{ 540 };

    // gap between each direction
    constexpr int directionGap{ directions[1] - directions[0] };

    int maxDirection{ directions[0] };
    int minDirection{ directions[0] };

    for (int direction : directions) {
        if (laser->ranges[direction] > laser->ranges[maxDirection]) {
            maxDirection = direction;
        }

        if (laser->ranges[direction] < laser->ranges[minDirection]) {
            minDirection = direction;
        }
    }

    if (laser->ranges[maxDirection] < 1.00) {
        cmd_vel.linear.x = 0.00;
        cmd_vel.angular.z = 3.00;
    }
    else if (laser->ranges[minDirection] < 0.50) {
        cmd_vel.linear.x = 0.40;
        cmd_vel.angular.z = 1.00 * ((centerDirection - minDirection) / directionGap);
    }
    else {
        cmd_vel.linear.x = 1.00;
        cmd_vel.angular.z = 0.40 * ((maxDirection - centerDirection) / directionGap);
    }
    
    //BURDAN SONRASINA DOKUNMA

    cmd_vel_pub.publish(cmd_vel);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "man1_autonomy");
    ros::NodeHandle nh;

    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;

    ros::Subscriber laser_sub = nh.subscribe("/rtg/hokuyo", 1000, laserCallBack);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/rtg/cmd_vel", 1000);

    ros::spin();

    return 0;
}
