//
// Created by wang on 2021/7/25.
//

#ifndef SRC_IMUODOM_H
#define SRC_IMUODOM_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "dbg.h"

class IMUOdom
{
    using V3d = Eigen::Vector3d;
    using Qd = Eigen::Quaterniond;
public:
    IMUOdom(ros::NodeHandle n);

private:
    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);

    void readROSParameters();

    V3d geoVec3toEigVec3(geometry_msgs::Vector3 geo_vec){
        return V3d(geo_vec.x, geo_vec.y, geo_vec.z);
    }

    Qd geoQtoEigQ(geometry_msgs::Quaternion geo_q){
        Qd q(geo_q.w, geo_q.x, geo_q.y, geo_q.z );
        q.normalize();
        return q;
    }

    geometry_msgs::Point eigVec3toGeoPoint(V3d p){
        geometry_msgs::Point point;
        point.x = p.x();
        point.y = p.y();
        point.z = p.z();
        return point;
    }

    geometry_msgs::Vector3 eigVec3toGeoVec3(V3d p){
        geometry_msgs::Vector3 point;
        point.x = p.x();
        point.y = p.y();
        point.z = p.z();
        return point;
    }

    geometry_msgs::Quaternion eigQtoGeoQ(Qd q){
        geometry_msgs::Quaternion quaternion;
        quaternion.w = q.w();
        quaternion.x = q.x();
        quaternion.y = q.y();
        quaternion.z = q.z();
        return quaternion;
    }
private:
    //// ROS Variables
    ros::NodeHandle n_;
    ros::Publisher pub_odom_;
    ros::Subscriber sub_imu_;

    //// Parameters
    std::string sub_imu_topic_name_;
    std::string pub_odom_topic_name_;
    int hz_;
    std::vector<double> g_vec_;
    bool enable_position_;

    //// Previous State
    V3d gw_;      // ENU frame
    sensor_msgs::Imu imu_prev_;         // 上一时刻的imu数据
    V3d Pwb_kk_ ;                       // 上一时刻imu相对world的位置
    Qd Qwb_kk_ ;                        // 上一时刻imu相对world的姿态
    V3d Vw_kk_ ;                        // 上一时刻imu相对于world的速度


};


#endif //SRC_IMUODOM_H
