/**
 * @file hover_move.cpp
 * @brief 根据target坐标进行移动
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <iostream>
enum State{
    TAKE_OFF,
    MISSION,
    LAND
};

//获取飞机当前状态
mavros_msgs::State current_state;
//获取当前飞机位置
geometry_msgs::PoseStamped current_pose;
//得到target坐标
geometry_msgs::PoseStamped target_position;
//飞机初始位置
geometry_msgs::PoseStamped init_pose;
//飞机飞去的位置
geometry_msgs::PoseStamped compute_position;


void state_cb(const mavros_msgs::State::ConstPtr &msg){
    current_state=*msg;
}

void OdomCallback(const geometry_msgs::PoseStamped::ConstPtr &msg){
    current_pose=*msg;
}


geometry_msgs::PoseStamped target_position_cb(const geometry_msgs::PoseStamped::ConstPtr &position){
    target_position=*position;
    compute_position=current_pose;
    // compute_position.pose.position.x=fix_pose.pose.position.x+target_position.pose.position.x-2;
    //target_position.pose.position.x=target_position.pose.position.x-current_pose.pose.position.x;
    double dx=target_position.pose.position.x-current_pose.pose.position.x;//飞机和target的相对距离
    double dy=target_position.pose.position.y-current_pose.pose.position.y;
    double dz=target_position.pose.position.z-current_pose.pose.position.z;
    compute_position.pose.position.x=current_pose.pose.position.x+dx-2;
    //compute_position.pose.position.y=current_pose.pose.position.y-target_position.pose.position.y;
    compute_position.pose.position.y=current_pose.pose.position.y;
    compute_position.pose.position.z=0.6;

    //std::cout<<compute_position.pose.position.x<<std::endl;
    return compute_position;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "hover_move");
    ros::NodeHandle nh;
    // 获取mavros状态
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    // 给mavros发送位姿
    ros::Publisher target_position_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    // For 定位信息
    ros::Subscriber odom_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, OdomCallback);
    // 切换mavros状态
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    //订阅target坐标
    ros::Subscriber target_position=nh.subscribe<geometry_msgs::PoseStamped>("/target_position",10,target_position_cb);
    // the setpoint publishing rate MUST be faster than 2Hz
    //ros::Subscriber target_position_sub=nh.subscribe<geometry_msgs::PoseStamped>("/detect_result_out",10,TargetCallback);
        /*
    无人机有一个锁，如果不解锁，无人机虽然接受了命令但是不会动被锁住了，只有解锁了才能对无人机进行控制，下面这个服务调用就是用来请求解锁无人机。上面的current_state就包含了无人机是否解锁的信息，若没解锁就需要解锁，否则就不用，其用途在这就体现出来
    */
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");

    ros::Rate rate(10.0);

    // wait for FCU connection
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Connected to mavros!");

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = current_pose.pose.position.x;
    pose.pose.position.y = current_pose.pose.position.y;
    pose.pose.position.z = 0.6;

    for(int i = 200; ros::ok() && i > 0; --i){
        target_position_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

     // 请求的切换模式的消息，设置为OFFBOARD
    mavros_msgs::SetMode offb_set_mode;
    //offb_set_mode.request.custom_mode = "OFFBOARD";

    // 请求解锁的消息，arm表示解锁，设置为true，disarm是上锁
    mavros_msgs::CommandBool arm_cmd;
    //arm_cmd.request.value = true;

    // 记录上次请求的时间
    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        // 如果无人机模式不是OFFBOARD并且离上次操作时间大于5秒就发送请求切换，这里的5s是为了演示清楚设置的延时
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            // 更新本次请求的时间
            last_request = ros::Time::now();
        } else {
            // 如果当前未解锁且与请求时间大于5s，就发送请求解锁
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
		
        // 不断发送位置消息，但是只有解锁后才能真正开始运动，如果不发送就会退出OFFBOARD模式，因为请求发送速度要>=2HZ
        target_position_pub.publish(compute_position);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
