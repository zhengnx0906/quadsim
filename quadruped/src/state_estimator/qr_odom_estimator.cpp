#include "state_estimator/qr_odom_estimator.h"

#include "gazebo_msgs/GetModelState.h"


qrRobotOdometryEstimator::qrRobotOdometryEstimator(qrRobot *robotIn,
                                               ros::NodeHandle &nhIn)
    : robot(robotIn), nh(nhIn)
{
    pubOdometry = nh.advertise<nav_msgs::Odometry>("leggedodom", 1);
    //groundtruthpub = nh.advertise<nav_msgs::Odometry>("groundtruthOdom", 1);
    nh.param<bool>("publishOdomTF", publishOdomTF, false);
    lastTime = ros::Time::now();
    odomEstimateX = double(robot->GetBasePosition()[0]);
    odomEstimateY = robot->GetBasePosition()[1];
    odomEstimateZ = robot->GetBasePosition()[2]+0.02;
    //初始化机器人的位置和姿态
    ROS_INFO("robot_odom_estimator init success...");
}

void qrRobotOdometryEstimator::PublishOdometry()
{
    ros::spinOnce();
    currentTime = ros::Time::now();
    // ControlDataFlow* controlDataFlow = robot->controlDataFlow;
    const Vec3<float> estimatedVelocity = robot->GetBaseVelocity();
    const Vec3<float> &baserpy = robot->GetBaseRollPitchYaw();
    const Vec3<float> &baseRollPitchYawRate = robot->GetBaseRollPitchYawRate();
    //const Vec3<float> &baseOrientation = robot->GetBaseOrientation();
    float vX = estimatedVelocity[0];
    float vY = estimatedVelocity[1];
    float vZ = estimatedVelocity[2];

    double dt = (currentTime - lastTime).toSec();
    double dx, dy;
    dx = (vX * cos(baserpy[2]) - vY * sin(baserpy[2])) * dt;
    dy = (vX * sin(baserpy[2]) + vY * cos(baserpy[2])) * dt;
    odomEstimateX += dx;
    //std::cout<<dx<<" "<<odomEstimateX<<std::endl;
    odomEstimateY += dy;
    odomEstimateZ = robot->GetBasePosition()[2]+0.02;
    // since all odometry is 6DOF we'll need a quaternion created from yaw
    //geometry_msgs::Quaternion odomQuat = tf::createQuaternionMsgFromYaw(baserpy[2]);
    geometry_msgs::Quaternion odomQuat = tf::createQuaternionMsgFromRollPitchYaw(baserpy[0],baserpy[1],baserpy[2]);
    // publish the transform over tf
    geometry_msgs::TransformStamped odomTrans;
    odomTrans.header.stamp = currentTime;
    odomTrans.header.frame_id = "leggedodom";
    odomTrans.child_frame_id = "base";
    odomTrans.transform.translation.x = odomEstimateX;
    odomTrans.transform.translation.y = odomEstimateY;
    odomTrans.transform.translation.z = odomEstimateZ;
    odomTrans.transform.rotation = odomQuat;
    
    // send the transform
    ros::param::get("publishOdomTF", publishOdomTF);
    if(publishOdomTF)
    {
        odomBroadcaster.sendTransform(odomTrans);
    }

    /* 会卡住导致机器人不能正常控制
    ros::ServiceClient client2 = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    gazebo_msgs::GetModelState srv2;
    srv2.request.model_name = "base"; //指定要获取的机器人在gazebo中的名字；
    if (client2.call(srv2))
    {
          // 如果获取服务成功了的话，
        nav_msgs::Odometry odom;
        odom.pose.pose=srv2.response.pose;
        odom.twist.twist=srv2.response.twist;
        odom.header.stamp = currentTime;
        odom.header.frame_id = "leggedodom";

        odom.child_frame_id = "base";

        groundtruthpub.publish(odom);
    }
    */

    // publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = currentTime;
    odom.header.frame_id = "leggedodom";
    //set the position
    odom.pose.pose.position.x = odomEstimateX;
    odom.pose.pose.position.y = odomEstimateY;
    odom.pose.pose.position.z = odomEstimateZ;
    odom.pose.pose.orientation = odomQuat;
    //set the velocity 注意这个速度就是当前坐标系下的速度！！！！
    odom.child_frame_id = "base";
    odom.twist.twist.linear.x = vX;
    odom.twist.twist.linear.y = vY;
    odom.twist.twist.angular.z = baseRollPitchYawRate[2];

    //publish the message
    pubOdometry.publish(odom);


    lastTime = currentTime;
}
