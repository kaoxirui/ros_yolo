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

#include <Eigen/Core>
#include <Eigen/Geometry>

ros::Publisher local_pos_pub;

// 回调mavros状态
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

// 回调位置信息
geometry_msgs::PoseStamped cur_pose;
void OdomCallback(const geometry_msgs::PoseStamped msg)
{
    cur_pose = msg;
    // 注意不用转位姿
    // cur_pose.header = msg.header;
    // cur_pose.pose.position.x = -msg.pose.position.y;
    // cur_pose.pose.position.y = msg.pose.position.x;
    // cur_pose.pose.position.z = msg.pose.position.z;

    // cur_pose.pose.orientation = msg.pose.orientation;
    // std::cout << "cur_pose =  [" << cur_pose.pose.position.x << ", " << cur_pose.pose.position.y << ", " << cur_pose.pose.position.z << "]" << std::endl;
}

// 起点，同时也是降落点
geometry_msgs::PoseStamped init_pose;

// 飞行状态(本任务中)
enum State
{
    TAKE_OFF,
    MISSION,
    LAND
};

// 判断飞机是否达到
bool check_target(geometry_msgs::PoseStamped set_pose, geometry_msgs::PoseStamped current_pose)
{
    double dx = set_pose.pose.position.x - current_pose.pose.position.x;
    double dy = set_pose.pose.position.y - current_pose.pose.position.y;
    double dz = set_pose.pose.position.z - current_pose.pose.position.z;

    if (dx * dx + dy * dy + dz * dz < 0.2)
        return true;
    else
        return false;
}

// 判断飞机是否达到
bool check_target(geometry_msgs::PoseStamped set_pose, geometry_msgs::PoseStamped current_pose,State& fly_state)
{
    double dx = set_pose.pose.position.x - current_pose.pose.position.x;
    double dy = set_pose.pose.position.y - current_pose.pose.position.y;
    double dz = set_pose.pose.position.z - current_pose.pose.position.z;

    if (dx * dx + dy * dy + dz * dz < 0.1)
        return true;
    else
        return false;
}



// 任务列表
double angle = 10.0;
int count =0;
geometry_msgs::PoseStamped mission_list(State& fly_state, geometry_msgs::PoseStamped current_pose)
{
    geometry_msgs::PoseStamped set_pose;
    switch (fly_state)
    {
    case TAKE_OFF:
        // 设置起飞点
        set_pose = init_pose;
	    set_pose.pose.position.x += 0.2;
        set_pose.pose.position.z = 0.7;
        // 如果当前起飞点达到
        if(check_target(set_pose,cur_pose))
        {
            // 切换状态到任务模式
            fly_state = MISSION;
            ROS_INFO("MISSION");
	    // 立即调一次任务(相当于且到MISSION下了)
            set_pose = mission_list(fly_state,cur_pose);
        }
        break;
    case MISSION:
        // 这儿画圆吧（做个灯光秀展示）
        switch(count)
        {
                case 0:
                        set_pose.pose.position.x = -5.6;
                        set_pose.pose.position.y = 0.878543;
                        set_pose.pose.position.z = 0.7;
                        ROS_INFO("Point 1");
			break;
                case 1:
                        set_pose.pose.position.x = -2.6379547119140625;
                        set_pose.pose.position.y = 0.5625098943710327;
                        set_pose.pose.position.z = 0.7;
                        ROS_INFO("Point 2");
			break;
                case 2:
                        set_pose.pose.position.x = 0.20602960884571075;
                        set_pose.pose.position.y = 0.021854689344763756;
                        set_pose.pose.position.z = 0.7;
                        ROS_INFO("Point 3");
			break;
                 case 3:
                        set_pose.pose.position.x = -2.6379547119140625;
                        set_pose.pose.position.y = 0.5625098943710327;
                        set_pose.pose.position.z = 0.7;
                        ROS_INFO("Point 2");
			break;
                 case 4:
                        set_pose.pose.position.x = -5.6;
                        set_pose.pose.position.y = 0.878543;
                        set_pose.pose.position.z = 0.7;
                        ROS_INFO("Point 1");
			break;
		case 5:
			fly_state = LAND;
			ROS_INFO("LAND");		
                        break;
        }
        break;
    case LAND:
        set_pose = init_pose;
        set_pose.pose.position.z = current_pose.pose.position.z;
    default:
        break;
    }
    local_pos_pub.publish(set_pose);
    return set_pose;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hover_circle");
    ros::NodeHandle nh;

    // 获取mavros状态
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    // 给mavros发送位姿
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("local_planner/goal_position", 10);
    // For 定位信息
    ros::Subscriber odom_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, OdomCallback);
    // 切换mavros状态
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    // the setpoint publishing rate MUST be faster than 2Hz
    //ros::Subscriber target_position_sub=nh.subscribe<geometry_msgs::PoseStamped>("/detect_result_out",10,TargetCallback);
    ros::Rate rate(10.0);

    // wait for FCU connection
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Connected to mavros!");

    // 设置起点坐标(3秒以内要求保证定位可靠)
    ros::Time init_time = ros::Time::now();
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
        init_pose = cur_pose;
        ros::Time init_last_time = ros::Time::now();

        if ((init_last_time - init_time).toSec() > 3.0f)
            break;
    }

    State fly_state = TAKE_OFF;
ROS_INFO("TAKE_OFF");
    // 向缓冲器中发数
    geometry_msgs::PoseStamped set_pose = mission_list(fly_state, cur_pose);
    //for (int i = 100; ros::ok() && i > 0; --i)
    //{
       // local_pos_pub.publish(set_pose);
       // ros::spinOnce();
       // rate.sleep();
    //}

    // 准备切换到OFFBOARD模式
    // mavros_msgs::SetMode offb_set_mode;
    // offb_set_mode.request.custom_mode = "OFFBOARD";

    // 手起解锁
    // mavros_msgs::CommandBool arm_cmd;
    // arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    bool change_mode = false;
    while (ros::ok())
    {
        //尝试且offboard (未降落)
        /*if (current_state.mode != "OFFBOARD" && current_state.mode != "AUTO.LAND" &&
            (ros::Time::now() - last_request > ros::Duration(3.0)))
        {
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else*/
	if(current_state.mode == "OFFBOARD")
        {
	   /* change_mode = true;
            // 飞机未解锁，需要等待
            if (!current_state.armed)
            {
                ROS_INFO("Vehicle disarmed");
            }
            else            //飞机解锁
            {*/
                if(fly_state == LAND)
                {
		            ROS_INFO("LAND");
                    set_pose = mission_list(fly_state,cur_pose);
                    // 如果到达了指定点
                    if(check_target(set_pose,cur_pose,fly_state))
                    {
                        //等待1s，保证精准降落
                        last_request = ros::Time::now();
                        while((ros::Time::now() - last_request).toSec() < 1.0)
                        {
                            //定位不能丢
                            ros::spinOnce();
                            rate.sleep();
                        }

                        //开始降落
                        mavros_msgs::SetMode land_set_mode;
                        land_set_mode.request.custom_mode = "AUTO.LAND";
                        set_mode_client.call(land_set_mode);
                    }
                    
                }

                //到达后且下一个点,且点
                if(check_target(set_pose,cur_pose))
		        {
                    set_pose = mission_list(fly_state,cur_pose);
		            if(fly_state == MISSION)
		    	    count++;
		        }	
            //}
        }
	    //local_pos_pub.publish(set_pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
