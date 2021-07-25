//
// Created by wang on 2021/7/25.
//

#include "IMUOdom.h"

IMUOdom::IMUOdom(ros::NodeHandle n): n_(n){
    readROSParameters();
    gw_ = V3d( g_vec_[0], g_vec_[1], g_vec_[2]);
    imu_prev_.header.seq = UINT32_MAX;
    Pwb_kk_.setZero();
    Qwb_kk_.setIdentity();  // 认为第一时刻的imu坐标系为世界坐标系
    Vw_kk_.setZero();       // 保证初始化时imu是静止的

    pub_odom_ = n_.advertise<nav_msgs::Odometry>(pub_odom_topic_name_,1);
    sub_imu_ = n_.subscribe(sub_imu_topic_name_, 1, &IMUOdom::imuCallback, this);
}

void IMUOdom::readROSParameters()
{
    sub_imu_topic_name_ = "/imu/data";
    pub_odom_topic_name_ = "/imu/odom";
    hz_ = 200;
    g_vec_ = {0,0,-9.81};
    enable_position_ = false;
    if(ros::param::get("/imu_odom/sub_imu_topic", sub_imu_topic_name_))
        ROS_INFO("imu_odom/sub_imu_topic = %s", sub_imu_topic_name_.c_str());
    if(ros::param::get("/imu_odom/pub_odom_topic", pub_odom_topic_name_))
        ROS_INFO("imu_odom/pub_odom_topic = %s", pub_odom_topic_name_.c_str());
    if(ros::param::get("/imu_odom/hz", hz_))
        ROS_INFO("imu_odom/hz = %d", hz_);
    if(ros::param::get("/imu_odom/enable_position", enable_position_))
        ROS_INFO("imu_odom/enable_position = %d", enable_position_);
    if(ros::param::get("/imu_odom/g_vec", g_vec_)){
        ROS_INFO("imu_odom/g_vec = %f, %f, %f ", g_vec_[0], g_vec_[1], g_vec_[2]);
    } else {
        ROS_WARN("FAIL");
    }
}

/*
 * @说明： 已知上一时刻的imu数据，根据当前时刻的数据，通过中值法计算上一时刻到当前时刻的数据
 * */
void IMUOdom::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
//    static V3d acc_average ;
//    static int count = 0;
//    acc_average += geoVec3toEigVec3(msg->linear_acceleration);
//    ++count;
//    if(count % 3000 == 0) dbg( acc_average / count );
//    return;

    static int i = 0;
    nav_msgs::Odometry odom_msg;

    if( imu_prev_.header.seq != UINT32_MAX )
    {
        sensor_msgs::Imu imu_curr = *msg;
        V3d gyro_w = 0.5 *(geoVec3toEigVec3(imu_prev_.angular_velocity) + geoVec3toEigVec3(imu_curr.angular_velocity));
        double delta_t = (imu_curr.header.stamp - imu_prev_.header.stamp).toSec();
        V3d delta_vector = gyro_w * delta_t ;

        Eigen::AngleAxisd delta_angle_axis(delta_vector.norm(), delta_vector/delta_vector.norm());
        Qd delta_q( delta_angle_axis );
        // 下面这一种也可
//        Qd delta_q;
//        delta_q.w() = 1;
//        delta_q.x() = delta_vector.x() * 0.5;
//        delta_q.y() = delta_vector.y() * 0.5;
//        delta_q.z() = delta_vector.z() * 0.5;
//        delta_q.normalize();
        Qd Qwb_k = Qwb_kk_ * delta_q;       // k时刻姿态的预测
        V3d acc_w = 0.5*(Qwb_kk_ * geoVec3toEigVec3(imu_prev_.linear_acceleration) + gw_ + Qwb_k * geoVec3toEigVec3(imu_curr.linear_acceleration) + gw_) ;
        V3d Pwb_k = Pwb_kk_ + Vw_kk_ * delta_t + 0.5 * delta_t * delta_t * acc_w;
        V3d Vw_k = Vw_kk_ + acc_w * delta_t;
        Pwb_kk_ = Pwb_k;
        Qwb_kk_ = Qwb_k;
        Vw_kk_ = Vw_k;

        odom_msg.twist.twist.angular = eigVec3toGeoVec3(gyro_w);

    }
    imu_prev_ = *msg;

    if( i % std::max( (200 / hz_), 1) == 0 ){
        odom_msg.header = msg->header;
        if(enable_position_){
            odom_msg.pose.pose.position = eigVec3toGeoPoint(Pwb_kk_);
        }
        odom_msg.pose.pose.orientation = eigQtoGeoQ(Qwb_kk_);
        odom_msg.twist.twist.linear = eigVec3toGeoVec3(Vw_kk_);
        pub_odom_.publish(odom_msg);
//        dbg(Vw_kk_);
    }
    ++i;
    // Publish 位置 Pwb_kk_ ，姿态 Qwb_kk_，速度Vw_kk_
}