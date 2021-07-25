//
// Created by wang on 2021/7/25.
//


#include "IMUOdom.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_odom_node");
    ros::NodeHandle nh;

    IMUOdom imu_odom(nh);
    ros::spin();

    return 0;
}