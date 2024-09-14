#ifndef QR_ODOM_ESTIMATOR_H
#define QR_ODOM_ESTIMATOR_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include "robots/qr_robot.h"

class qrRobotOdometryEstimator {

public:
    qrRobotOdometryEstimator(qrRobot *robotIn, ros::NodeHandle &nhIn);

    void PublishOdometry();

private:
    qrRobot *robot;

//    RobotEstimator *robotEstimator;

//    LocomotionController *locomotionController;
    double odomEstimateX;

    double odomEstimateY;

    double odomEstimateZ;
    
    ros::NodeHandle &nh;

    ros::Publisher pubOdometry;

    //ros::Publisher groundtruthpub;

    bool publishOdomTF;
    // broadcast odom to base_link
    tf::TransformBroadcaster odomBroadcaster;

    ros::Time currentTime;

    ros::Time lastTime;
};

#endif // QR_ODOM_ESTIMATOR_H
