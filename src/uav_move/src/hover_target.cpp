/**
 * @file hover_target.cpp
 * @brief 将target的坐标转换到机体系下并发布
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
    cur_pose = msg;
}

geometry_msgs::PoseStamped target_body;
geometry_msgs::PoseStamped target_world;
void TargetCallback(const geometry_msgs::PoseStamped msg)
{
    //机体系target相对飞机的坐标                                    
    target_body.pose.position.x=msg.pose.position.z;
    target_body.pose.position.y=-msg.pose.position.x;
    target_body.pose.position.z=-msg.pose.position.y;

    //机体坐标系方向下target的绝对坐标
    target_world.pose.position.x=cur_pose.pose.position.x+target_body.pose.position.x;
    //if(cur_pose.pose.position.x<=0)target_world.pose.position.x=-cur_pose.pose.position.x+target_body.pose.position.x;
    target_world.pose.position.y=cur_pose.pose.position.y+target_body.pose.position.y;
    target_world.pose.position.z=cur_pose.pose.position.z+target_body.pose.position.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hover_target");
    ros::NodeHandle nh;
    // For 定位信息
    ros::Subscriber odom_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, OdomCallback);
    // // the setpoint publishing rate MUST be faster than 2Hz
    ros::Subscriber target_position_sub=nh.subscribe<geometry_msgs::PoseStamped>("/detect_result_out",10,TargetCallback);
    //发布target在机体系下绝对位置的坐标
    ros::Publisher target_pub=nh.advertise<geometry_msgs::PoseStamped>("/target_position",10);
    ros::Rate rate(30.0);

    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
        target_pub.publish(target_world);
    }

    return 0;
}
