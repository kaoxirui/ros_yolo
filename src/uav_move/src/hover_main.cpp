/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo Classic SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>


// 回调位置信息
geometry_msgs::PoseStamped cur_pose;
void OdomCallback(const geometry_msgs::PoseStamped msg)
{
    cur_pose.header = msg.header;
    cur_pose.pose.position.x = - msg.pose.position.y;
    cur_pose.pose.position.y = msg.pose.position.x;
    cur_pose.pose.position.z = msg.pose.position.z;

    cur_pose.pose.orientation = msg.pose.orientation;
    std::cout <<"cur_pose =  [" << cur_pose.pose.position.x << ", "<<cur_pose.pose.position.y <<", "<<cur_pose.pose.position.z<<"]"<<std::endl;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "opti2odom");
    ros::NodeHandle nh;

    ros::Publisher pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
    
    //For OptiTrack
    ros::Subscriber odom_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/uav_new/pose", 10, OdomCallback);

    // // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(30.0);

    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
        pos_pub.publish(cur_pose);
    }

    return 0;
}
