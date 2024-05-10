#include <ros/ros.h>
#include <std_msgs/Bool.h>			        //  消息类型    订阅 v2i节点 信号灯停车 
                                            //              订阅  激光节点 停车
#include "common_msgs/v2c_control.h"		//  消息类型    订阅 v2c节点 总控指令
#include <geometry_msgs/PoseStamped.h>	    //  消息类型    订阅 imu节点 定位数据 
#include <geometry_msgs/PoseArray.h>		//  消息类型    发布 目标路径 
#include <std_msgs/Float32.h>			    //  消息类型    发布 车辆目标速度 
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <decision/common_map.h>		    //  地图

#include <dynamic_reconfigure/client.h>
#include "teb_local_planner/TebLocalPlannerReconfigureConfig.h"

#include <vector>
// #include <actionlib/client/simple_action_client.h>
// #include <move_base_msgs/MoveBaseAction.h>
#define STRAIGHT 1
#define LEFT 2
#define RIGHT 3


// #define TEST__

using namespace std;

//******************************
//          全局变量
//******************************
// ros::Publisher pose_pub;	// 把三个发布器都写成全局变量，目的是在回调函数中使用
ros::Publisher speed_pub;
ros::Publisher path_pub;
// actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac;
int vehicle_id;             // 本车ID

double target_speed;        // 记录主控发来的目标速度
double pose_x, pose_y;  // 记录融合定位数据
bool v2i_brake, lidar_brake;// 记录信号灯和激光是否要求停车，不论停车与否两节点将一直发出消息
teb_local_planner::TebLocalPlannerReconfigureConfig config_;
geometry_msgs::Pose pose_cruise;
bool v2i_brake_flag = false;
bool cruise_flag = false;
bool lidar_brake_flag = false;
int lidar_brake_count = 0;// 记录在因为障碍物停止后，lidar_brake变为false(障碍物消失后)次数，需要连续5次后才重新启动
//******************************
//          功能函数
//******************************
vector<vector<double>> path_planning(int start_vertex, int target_vertex);						                    // 路径规划主函数：包括规划和插值
void interpolation(vector<int> &path, vector<vector<double>> &densePath, bool isAP);					            // 路径规划插值函数：计算稠密路径
void calcuCircleCenter(double x1, double y1, double x2, double y2, double r, int dir, double &x0, double &y0);	    // 路径规划工具函数：计算圆弧的圆心坐标
	
//******************************
//          回调函数
//******************************
// void fusePoseCallBack(const nav_msgs::Odometry& p);	    // 订阅imu节点发布的原始定位数据
void v2iCallBack(const std_msgs::Bool& p);			            // 订阅v2i节点发布的刹车指令
void v2cCallBack(const common_msgs::v2c_control& p); 			// 订阅v2c节点发布的数据
void lidarCallBack(const std_msgs::Bool& p);					// 订阅激光节点发布的刹车指令
void poseCallBack(const geometry_msgs::PoseWithCovarianceStamped& p);//接收决策发布的融合位姿
// void dynCallBack(const teb_local_planner::TebLocalPlannerReconfigureConfig& config_)
// {
//     ROS_INFO("double: %f", config_.max_vel_x);
// }


//******************************
//          主程序
//******************************
int main(int argc, char** argv)
{
//ros相关内容初始化
	ros::init(argc, argv, "decision");
	ros::NodeHandle nh;
	ROS_INFO_STREAM("initialing");
	// ros::Subscriber raw_pose_sub = nh.subscribe("/odom", 1, fusePoseCallBack);        // 订阅其他节点发布的原始定位数据
	ros::Subscriber lidar_sub = nh.subscribe("lidar_brake", 1, lidarCallBack);          // 订阅激光节点发布的刹车指令
	ros::Subscriber v2i_sub = nh.subscribe("v2i_brake", 1, v2iCallBack);                // 订阅v2i节点发布的刹车指令
	ros::Subscriber v2c_sub = nh.subscribe("v2c", 1, v2cCallBack);                      // 订阅v2c节点发布的数据
	ros::Subscriber pose_sub = nh.subscribe("/robot_pose_ekf/odom_combined", 1, poseCallBack);                //订阅决策发布的位姿

	// pose_pub = nh.advertise<geometry_msgs::PoseStamped>("fusion_pose", 1);    	        // 发布融合定位数据
	speed_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1); 		            // 发布车辆目标速度
	path_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);    	        // 发布车辆目标路径
	// dynamic_reconfigure::Client<teb_local_planner::TebLocalPlannerReconfigureConfig> client("/move_base/TebLocalPlannerROS", dynCallBack);
	dynamic_reconfigure::Client<teb_local_planner::TebLocalPlannerReconfigureConfig> client("/move_base/TebLocalPlannerROS");
	// ac = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>(nh,"move_base",true);
	
	#ifdef TEST__
		// 测试版
		if(!nh.getParam("vehicle_id", vehicle_id))                                          // 获取本车ID
		{
			ROS_INFO_STREAM("Get vehicle_id param fail... default = 1.");
			vehicle_id = 1;
		}
		ROS_INFO_STREAM("Get vehicle_id param:"<<vehicle_id);
		
		int decision_frequency_Hz=0;
		if(!nh.getParam("decision_frequency_Hz", decision_frequency_Hz)) 
		{
			ROS_INFO_STREAM("Get decision_frequency_Hz param fail ... default = 20");
			decision_frequency_Hz = 20;
		}
	#else
		if(!nh.getParam("vehicle_id", vehicle_id))                                          // 获取本车ID
		{
			ROS_INFO_STREAM("Get vehicle_id param fail... return -1.");
			return -1;
		}
		ROS_INFO_STREAM("Get vehicle_id param:"<<vehicle_id);
		
		int decision_frequency_Hz=0;
		if(!nh.getParam("decision_frequency_Hz", decision_frequency_Hz)) 
		{
			ROS_INFO_STREAM("Get decision_frequency_Hz param fail ... return");
			return -1;
		}
	#endif


// 初始化全局变量
    target_speed = 0.12;
    v2i_brake = false;
    lidar_brake = false;
    
    pose_x = -10000;
    pose_y = -10000;  // 记录融合定位数据
	ros::Rate loop_rate(decision_frequency_Hz);
    ROS_INFO("start to navigation.");
	bool getConfigFlag = false;
	bool initialVelFlag = false;

	//主程序部分  	
	while(ros::ok())
	{	
		ros::spinOnce();
		if (!getConfigFlag) {
			if (client.getCurrentConfiguration(config_, ros::Duration(1))) { // 无回复
				ROS_WARN_STREAM("success to get current config: "<< config_.max_vel_x);
				getConfigFlag = true;
			} else {
				ROS_WARN("failed to get current config");
			}
		}
		if (getConfigFlag && !initialVelFlag) {
			config_.max_vel_x = target_speed;
			config_.max_vel_x_backwards = target_speed;
			// config_.penalty_epsilon = target_speed<=config_.penalty_epsilon ? target_speed-0.01 : config_.penalty_epsilon;
			config_.penalty_epsilon = 0.1;
			if (client.setConfiguration(config_)) {
				initialVelFlag = true;
			}
			ROS_INFO_STREAM("success to initial vel: "<< config_.max_vel_x);
		}
		loop_rate.sleep();
	}
	
	return 0;	
}

//******************************
//          函数实现
//******************************
//************************************************************************************************************************************************************************回调函数**************
// 订阅v2c节点发布的数据
void v2cCallBack(const common_msgs::v2c_control& p)
{
//这是接收主控的指令的回调函数，主要包括：功能选择（循迹/泊车/队列...）、路径规划...	注：路径规划通过 path_pub 发布出去
    if(p.ID != vehicle_id)
        return;
        
    if(p.type == 150)
    {
        if(round(pose_x)==-10000 || round(pose_y)==-10000)
        {
            ROS_INFO_STREAM("Position do not init");
            return;
        }
		// ROS_INFO_STREAM("Cancel previous Goal");
		// ac.cancelGoal();
		ROS_INFO_STREAM("Sending Goal");
		geometry_msgs::Point point;
		geometry_msgs::Quaternion quaternion;
		geometry_msgs::Pose pose;
		geometry_msgs::PoseStamped goal;
		// move_base_msgs::MoveBaseGoal goal;
		point.x = p.data1/1000.0 + dx*0.001;
		point.y = p.data2/1000.0 + dy*0.001;
		// point.x = 0.107;
		// point.y = 0.840;
		// point.x = 0.333 + dx*0.001;
		// point.y = 0.840 + dy*0.001;
		point.z = 0;
		quaternion.x = 0;
		quaternion.y = 0;
		quaternion.z = 0;
		quaternion.w = 1;
		pose.position = point;
		pose.orientation = quaternion;
		goal.header.stamp = ros::Time::now();
		goal.header.frame_id = "map";
		pose_cruise = pose;
		goal.pose = pose;
		// goal.target_pose.header.stamp = ros::Time::now();
		// goal.target_pose.header.frame_id = "map";
		// goal.target_pose.pose = pose;

		// ac.sendGoal(goal);
		if (v2i_brake_flag == false) {
			cruise_flag = true;
			path_pub.publish(goal);
		}
		// ROS_INFO_STREAM("Send Goal Success: "<< point.x << "," << point.y);
    }
	else if(p.type == 151) {
		if(round(pose_x)==-10000 || round(pose_y)==-10000)
        {
            ROS_INFO_STREAM("Position do not init");
            return;
        }
		// ROS_INFO_STREAM("Cancel previous Goal");
		// ac.cancelGoal();
		ROS_INFO_STREAM("Sending Goal");
		geometry_msgs::Point point;
		geometry_msgs::Quaternion quaternion;
		geometry_msgs::Pose pose;
		geometry_msgs::PoseStamped goal;
		// move_base_msgs::MoveBaseGoal goal;

		point.x = parkVertexInfoS[int(p.data3)-1].vertexPos.x;
		point.y = parkVertexInfoS[int(p.data3)-1].vertexPos.y;
		point.z = 0;
		quaternion.x = 0;
		quaternion.y = 0;
		quaternion.z = 0;
		quaternion.w = 1;
		pose.position = point;
		pose.orientation = quaternion;
		goal.header.stamp = ros::Time::now();
		goal.header.frame_id = "map";
		goal.pose = pose;
		// goal.target_pose.header.stamp = ros::Time::now();
		// goal.target_pose.header.frame_id = "map";
		// goal.target_pose.pose = pose;

		// ac.sendGoal(goal);
		path_pub.publish(goal);
		
		ROS_INFO_STREAM("Send Parking Goal Success");
	}
    else if(p.type == 152)
    {
		dynamic_reconfigure::Client<teb_local_planner::TebLocalPlannerReconfigureConfig> client("/move_base/TebLocalPlannerROS");
        switch(int(p.data1))
        {
            case 2:
                {//加速，限速0.4r/s
                    target_speed = (target_speed + p.data2/1000.0)<2 ? (target_speed + p.data2/1000.0) : 0.4;
					v2i_brake_flag = false;
                    break;
                }
            case 1:
                {//减速，最低0
                    target_speed = (target_speed - p.data2/1000.0)>0.11 ? (target_speed - p.data2/1000.0) : 0.11;
					v2i_brake_flag = false;
                    break;
                }
            case 0:
				{	//制动
					if (cruise_flag==true && v2i_brake_flag==false) {
						v2i_brake_flag = true;
						ROS_WARN_STREAM("V2i Goal");
						geometry_msgs::Point point;
						geometry_msgs::Quaternion quaternion;
						geometry_msgs::Pose pose;
						geometry_msgs::PoseStamped goal;
						point.x = -10001;
						point.y = -10001;
						point.z = 0;
						quaternion.x = 0;
						quaternion.y = 0;
						quaternion.z = 0;
						quaternion.w = 1;
						pose.position = point;
						pose.orientation = quaternion;
						goal.header.stamp = ros::Time::now();
						goal.header.frame_id = "map";
						goal.pose = pose;
						path_pub.publish(goal);
					}
					break;
				}
			ROS_INFO_STREAM("Configuring target speed: "<< target_speed);
			config_.max_vel_x = target_speed;
			config_.max_vel_x_backwards = min(target_speed, 0.11);
			// config_.penalty_epsilon = config_.penalty_epsilon<target_speed ? config_.penalty_epsilon : target_speed-0.01;

			client.setConfiguration(config_);
			ROS_INFO_STREAM("Finish config target speed: "<< target_speed);
        }
    } else if(p.type == 154) {
		if(round(pose_x)==-10000 || round(pose_y)==-10000)
        {
            ROS_INFO_STREAM("Position do not init");
            return;
        }
		// ROS_INFO_STREAM("Cancel previous Goal");
		// ac.cancelGoal();
		ROS_INFO_STREAM("Sending Formation Goal");
		geometry_msgs::Point point;
		geometry_msgs::Quaternion quaternion;
		geometry_msgs::Pose pose;
		geometry_msgs::PoseStamped goal;
		point.x = mapVertexInfoS[int(p.data3)-1].vertexPos.x/1000.0;
		point.y = mapVertexInfoS[int(p.data3)-1].vertexPos.y/1000.0;
		point.z = 0;
		quaternion.x = 0;
		quaternion.y = 0;
		quaternion.z = 0;
		quaternion.w = 1;
		pose.position = point;
		pose.orientation = quaternion;
		goal.header.stamp = ros::Time::now();
		goal.header.frame_id = "map";
		pose_cruise = pose;
		goal.pose = pose;

		if (v2i_brake_flag == false) {
			cruise_flag = true;
			path_pub.publish(goal);
			ROS_INFO_STREAM("Have Sent Formation Goal Success: "<< point.x << "," << point.y);
		}
	}
	else if(p.type == 155){
		ROS_INFO_STREAM("Emergency stop car "<< p.ID);
		geometry_msgs::Point point;
		geometry_msgs::Quaternion quaternion;
		geometry_msgs::Pose pose;
		geometry_msgs::PoseStamped goal;
		point.x = -10001;
		point.y = -10001;
		point.z = 0;
		quaternion.x = 0;
		quaternion.y = 0;
		quaternion.z = 0;
		quaternion.w = 1;
		pose.position = point;
		pose.orientation = quaternion;
		goal.header.stamp = ros::Time::now();
		goal.header.frame_id = "map";
		goal.pose = pose;
		path_pub.publish(goal);
	}
	else if(p.type == 156){
		ROS_INFO_STREAM("parking car is "<< p.ID);
	}
}

void poseCallBack(const geometry_msgs::PoseWithCovarianceStamped& p)
{
    
    // 接收位置数据
    pose_x = p.pose.pose.position.x;     //xy位置正常接收
    pose_y = p.pose.pose.position.y;	// m->mm

}

// 订阅激光节点发布的刹车指令					
void lidarCallBack(const std_msgs::Bool& p)	
{
//这个回调函数与下面v2iCallBack作用相同，即接收激光的刹车指令，以目标速度是否为0的方式，通过 speed_pub 发布出去	
    lidar_brake = p.data;                                                       // 接收激光停障指令

	if (lidar_brake == true) {
		lidar_brake_count = 0;
		if (lidar_brake_flag == false) {
			lidar_brake_flag = true;
			ROS_WARN_STREAM("stop for obstacle!");
			geometry_msgs::Point point;
			geometry_msgs::Quaternion quaternion;
			geometry_msgs::Pose pose;
			geometry_msgs::PoseStamped goal;
			point.x = -10001;
			point.y = -10001;
			point.z = 0;
			quaternion.x = 0;
			quaternion.y = 0;
			quaternion.z = 0;
			quaternion.w = 1;
			pose.position = point;
			pose.orientation = quaternion;
			goal.header.stamp = ros::Time::now();
			goal.header.frame_id = "map";
			goal.pose = pose;
			path_pub.publish(goal);
		}
		
	} else {
		if(lidar_brake_flag==true){
			lidar_brake_count++;
		}
		if (lidar_brake_count > 10) {// 直到lidar_brake稳定为false，即无障碍物时再重新启动
			ROS_WARN_STREAM("obstacle remove!continue move!");
			geometry_msgs::PoseStamped goal;
			lidar_brake_flag = false;
			lidar_brake_count = 0;
			goal.pose = pose_cruise;
			goal.header.stamp = ros::Time::now();
			goal.header.frame_id = "map";
			path_pub.publish(goal);
		}
	}
}			
		
// 订阅v2i节点发布的刹车指令
void v2iCallBack(const std_msgs::Bool& p)
{
	// dynamic_reconfigure::Client<teb_local_planner::TebLocalPlannerReconfigureConfig> client("/move_base/TebLocalPlannerROS");

//这里接收信号灯的刹车指令，以目标速度是否为0的方式，通过 speed_pub 发布出去
    v2i_brake = p.data;                                                         // 接收激光停障指令
    std_msgs::Float32 pub_target_speed;
    pub_target_speed.data = (lidar_brake || v2i_brake) ? 0 : target_speed;      // 当激光 信号灯 二者任一为true（即刹车），就刹车

	if (v2i_brake == true) {
		if (cruise_flag==true && v2i_brake_flag==false) {
			v2i_brake_flag = true;
			ROS_WARN_STREAM("V2i Goal");
			geometry_msgs::Point point;
			geometry_msgs::Quaternion quaternion;
			geometry_msgs::Pose pose;
			geometry_msgs::PoseStamped goal;
			point.x = -10001;
			point.y = -10001;
			point.z = 0;
			quaternion.x = 0;
			quaternion.y = 0;
			quaternion.z = 0;
			quaternion.w = 1;
			pose.position = point;
			pose.orientation = quaternion;
			goal.header.stamp = ros::Time::now();
			goal.header.frame_id = "map";
			goal.pose = pose;
			path_pub.publish(goal);
		}
	} else {
		if (v2i_brake_flag==true && cruise_flag==true) {
			ROS_WARN_STREAM("V2i Release");
			geometry_msgs::PoseStamped goal;
			v2i_brake_flag = false;
			goal.pose = pose_cruise;
			goal.header.stamp = ros::Time::now();
			goal.header.frame_id = "map";
			path_pub.publish(goal);
		}
	}

	// geometry_msgs::Twist pub_speed_msg;
	// if (v2i_brake == true) {
	// 	pub_speed_msg.linear.x =  pub_target_speed.data;
	// 	speed_pub.publish(pub_speed_msg);   // 把实际速度发给纵向控制节点
	// 	ROS_INFO_STREAM("Finish config target speed: "<< pub_speed_msg.linear.x);
	// }
	// ROS_INFO_STREAM("Configuring target speed: "<< pub_target_speed.data);
	// config_.max_vel_x = pub_target_speed.data;
	// config_.max_vel_x_backwards = pub_target_speed.data;
	// config_.penalty_epsilon = config_.penalty_epsilon<target_speed ? config_.penalty_epsilon : target_speed-0.01;
	// client.setConfiguration(config_);
	// ROS_INFO_STREAM("Finish config target speed: "<< pub_target_speed.data);
}

// 订阅其他节点发布的原始定位数据
// void fusePoseCallBack(const nav_msgs::Odometry& p)
// {
// //这里主要接收imu节点的定位数据，同时接收相机的一些定位信息，然后计算出一个融合定位数据，最后通过 pose_pub 发布出去。
//     // pose_pub.publish(p);                                                        // 没处理直接发出去了
    
// 	// 框架更改后，直接取得 lio 的融合定位结果
//     // pose_x = p.pose.position.x;
//     // pose_y = p.pose.position.y;
// 	ROS_INFO_STREAM("Receive pose success");
// 	pose_x = p.pose.pose.position.x * 1000;     //xy位置正常接收
//     pose_y = p.pose.pose.position.y * 1000; // m->mm
    
//     //四元数 转 欧拉角             偏航角需要经四元数转化而来
//     // tf2::Quaternion quat(
//     //     p.pose.pose.orientation.x,
//     //     p.pose.pose.orientation.y,
//     //     p.pose.pose.orientation.z,
//     //     p.pose.pose.orientation.w);
// }

//************************************************************************************************************************************************************************路径规划**************
// 路径规划主函数：包括规划和插值		
vector<vector<double>> path_planning(int start_vertex, int target_vertex)
{
//路径规划
	//定义存储可能的路径及其代价值
	vector<vector<int>> allPath;       	        //定义所有的可能路径
    	vector<int> cost;                    	//对应每条路径的代价值

    	vector<int> firstPath;               	//添加一条新的路径
    	firstPath.push_back(start_vertex);      //添加起点
    	allPath.push_back(firstPath);
    	cost.push_back(0);             	        //添加新路径的代价值

	//定义一些标志
	bool finishPlan = false;     		        //规划完成标志
	int finalCost = 60000;    		            //预定义最终代价值
	int hasReachTarget = false;  		        //定义是否已经有路线找到了终点

	vector<int> endingNode; 		            //断路预处理:记录一些断头路
	endingNode = {0,16,21};

	//遍历规划：
	//  不断遍历所有的可能路线，直到到达目标位置
	//  当某一条可能的路径到达目标位置后，开始与其他可能的路径比较长度，保留 到达终点的路径中 最短路径长度的 路径
    	while(!finishPlan)
    	{
        	//ROS_INFO_STREAM("allPath.size():"<<allPath.size());

        	int outNum = 0; //定义每次遍历时淘汰的路径数
        	for(int m = 0; m < allPath.size(); m++)
        	{
            		int i = m-outNum;
        		//1、路径淘汰
            		vector<int> oriPath;
            		vector<int> T = allPath[i];
	    		for(int k = 0; k < T.size(); k++)
	        		oriPath.push_back(T[k]);

        		int fromNodeIndex = oriPath.back();
        		if(target_vertex == fromNodeIndex)                 		//1.1、对于达到目标点的路径：比较产生最小的代价值，淘汰较大代价值的路径
        		{
            		hasReachTarget = true;

            		if(1 == allPath.size())                 	//1.1.1、当到达目标点，同时经过淘汰环节，只剩一条可能的路径时，完成规划
            		{
                    		ROS_INFO_STREAM("finishPlan");
                			finishPlan = true;
                			break;
            		}

            		if(cost[i]<(finalCost+1))                	//1.1.2、到达目标点，但还有未淘汰的路径时，进行最小代价值更新或淘汰操作
            		{
                			finalCost = cost[i];
            		}
            		else
            		{//同时移除路径及其代价
            			allPath.erase(allPath.begin()+i);
            			cost.erase(cost.begin()+i);
			    	    outNum++;
                    	//ROS_INFO_STREAM("remove route type 1");
            		}
            		continue;
        		}
        		else                                        		//1.2、对于未达到目标点的路径：若已经有路径到达终点，则淘汰代价值较大的 未到终点的路径
        		{
            		if(hasReachTarget)
	            		if(cost.at(i)>finalCost)
	            		{//同时移除路径及其代价
	                		allPath.erase(allPath.begin()+i);
            				cost.erase(cost.begin()+i);
	                		outNum++;
			        	    //ROS_INFO_STREAM("remove route type 2");
	                		continue;
	            		}
        		}

        		//2、路径延伸
		    	Vertex fromNode = mapVertexInfoS[fromNodeIndex-1];				//2.1、取出当前路径最后一个点
		    	vector<int> nextNodes = fromNode.relatedVertexes;
		    	vector<int> nextEdges = fromNode.relatedEdges;
		    	int curCost = cost[i];
                         	
		    	if(find(endingNode.begin(), endingNode.end(), nextNodes[0]) != endingNode.end()) //2.2、淘汰断路
		    	{//同时移除路径及其代价
		        	    allPath.erase(allPath.begin()+i);
                		cost.erase(cost.begin()+i);
		        	    outNum++;
		            	//ROS_INFO_STREAM("remove route type 3");
		        	    continue;
		    	}

		    	for(int j = 0; j < nextNodes.size(); j++)                               	//2.3、确定分叉情况
		    	{
		        	//更新路径
		        	vector<int> newOne(oriPath);
		        	newOne.push_back(nextNodes[j]);
		        	//更新代价
		        	int incCost = mapEdgeInfoS[nextEdges[j]-1].attr_cost;

		        	if(0 == j)                                                          	//2.3.1、对于第一个子节点，顺延当前路径
		        	{
		            		allPath[i].swap(newOne);
		            		cost[i] = curCost+incCost;
		        	}
		        	else                                                                	//2.3.2、对于第2/3个子节点，则新建路径
		        	{
		            		allPath.push_back(newOne);
		            		cost.push_back(curCost+incCost);
		        	}
		    	}

        	}
    	}

    	vector<int> path(allPath[0]);
    	/*string outcome = "";
    	for(int i=0; i<path.size(); i++)
    	    outcome += (std::to_string(path[i]) + " -> ");
    	    
    	ROS_INFO_STREAM("plan result:"<<outcome);
 */   	    
    	
//路径插值
	vector<vector<double>> densePath;
	interpolation(path, densePath, 0);//插值，得到稠密路径 
	
	return densePath;	
}	

// 路径规划插值函数：计算稠密路径
void interpolation(vector<int> &path, vector<vector<double>> &densePath, bool isAP)
{
	int resolution = 50;									//插值的分辨率，单位：mm，下述距离单位均为 mm
	vector<int> repairEdge{12,18,39,50,52,53,55,104,109,125,136,149,163,167,168};	//修复一些边

	for(int i = 0; i < path.size()-1; i++)
	{
		//确定该节点坐标
		Vertex rNode= isAP ? parkVertexInfoS[path[i]-201] : mapVertexInfoS[path[i]-1];
		double x1 = rNode.vertexPos.x;
		double y1 = rNode.vertexPos.y;

		//确定相邻下一节点
		int nextNode = path[i+1];
		Vertex nNode = isAP ? parkVertexInfoS[nextNode-201] : mapVertexInfoS[nextNode-1];
		double x2 = nNode.vertexPos.x;
		double y2 = nNode.vertexPos.y;

		//确定两节点之间的路线属性
		int index = find(rNode.relatedVertexes.begin(), rNode.relatedVertexes.end(), nextNode) - rNode.relatedVertexes.begin();
		int edgeIndex = rNode.relatedEdges[index];
		Edge rEdge = isAP ? parkEdgeInfoS[edgeIndex-201] : mapEdgeInfoS[edgeIndex-1];
		double dis = rEdge.attr_dis;
		double angle = rEdge.attr_angle;
		int edgeType = rEdge.attr_type;

		if(rEdge.attr_isReverse && (edgeType!=1)) //表示此时为泊车路径插值，要考虑是否是倒车：若是倒车，在插值算法下，需要将左右互换
			edgeType = (2 == edgeType) ? 3 : 2;

		//添加该路段首节点
		vector<double> add0 = {x1,y1,double(path[i]),double(path[i+1]),double(edgeIndex)};
		densePath.push_back(add0);

		vector<double> add1;

		//开始插值
		if(edgeType == STRAIGHT)   //直行
		{          
			int cnt = int(dis)/resolution;
			for(int j = 0; j < cnt; j++)
			{
				double xx = x1 + (x2 - x1)*(j+1)/cnt;
				double yy = y1 + (y2 - y1)*(j+1)/cnt;

				vector<double> add1{xx,yy,double(path[i]),double(path[i+1]),double(edgeIndex)};
				densePath.push_back(add1);
			}
		}
		else if(edgeType == LEFT)  //左转
		{
			//计算v圆心坐标
			double x0=0,y0=0;
			calcuCircleCenter(x1,y1,x2,y2,dis,edgeType,x0,y0);

			//分段
			double radAngle = angle * 3.14159 / 180;        //角度用弧度表示
			double curveDis = dis * radAngle;               //弧长
			int cnt = int(curveDis)/resolution;             //按弧长分段
			double aveAngle = radAngle/cnt;                 //按角度分段

			//插值
			for(int j = 0; j < cnt-1; j++)
			{
				double angle = (j+1)*aveAngle;               //圆心角
				double isoscelesAngle = (3.14159 - angle)/2; //等腰底角
				double chord = 2 * dis * sin(angle/2);       //弦长

				double a = atan((y0-y1)/(x0-x1));  
				double b = 3.14159 - (isoscelesAngle +a);

				int flag = (y1>y2) ? 1 : -1;
				if(find(repairEdge.begin(), repairEdge.end(), edgeIndex) != repairEdge.end())//修复一些边
					flag = flag *(-1);

				double xx = x1 + flag*chord*cos(b);
				double yy = y1 - flag*chord*sin(b);

				vector<double> add1{xx,yy,double(path[i]),double(path[i+1]),double(edgeIndex)};
				densePath.push_back(add1);
			}
		}
		else if(edgeType == RIGHT)  //右转
		{
			//计算v圆心坐标
			double x0=0,y0=0;
			calcuCircleCenter(x1,y1,x2,y2,dis,edgeType,x0,y0);
	
			//分段
			double radAngle = angle * 3.14159 / 180;        //角度用弧度表示
			double curveDis = dis * radAngle;               //弧长
			int cnt = int(curveDis)/resolution;             //按弧长分段
			double aveAngle = radAngle/cnt;                 //按角度分段

			//插值
			for(int j = 0; j < cnt-1; j++)
			{

				double angle = (j+1)*aveAngle;               //圆心角
				double isoscelesAngle = (3.14159 - angle)/2; //等腰底角
				double chord = 2 * dis * sin(angle/2);       //弦长

				double a = atan((y0-y1)/(x0-x1));
				double b = 3.14159 - (isoscelesAngle - a);

				int flag = (y1>y2) ? 1 : -1;
				if(find(repairEdge.begin(), repairEdge.end(), edgeIndex) != repairEdge.end())//修复一些边
					flag = flag *(-1);

				double xx = x1 - flag*chord*cos(b);
				double yy = y1 - flag*chord*sin(b);

				vector<double> add1{xx,yy,double(path[i]),double(path[i+1]),double(edgeIndex)};
				densePath.push_back(add1);
			}  
		}
	}
}

// 路径规划工具函数：计算圆弧的圆心坐标
void calcuCircleCenter(double x1, double y1, double x2, double y2, double r, int dir, double &x0, double &y0)
{
	double c1 = (x2*x2 - x1*x1 + y2*y2 - y1*y1) / (2*(x2 - x1));
	double c2 = (y2 - y1) / (x2 - x1);
	double A = c2*c2 + 1;
	double B = 2*x1*c2 - 2*c1*c2 - 2*y1;
	double C = x1*x1 - 2*x1*c1 + c1*c1 + y1*y1 - r*r;

	if(dir==LEFT)
	{
		if(x1>x2)
			y0 = (-B + sqrt(B*B - 4*A*C)) / (2*A);
		else
			y0 = (-B - sqrt(B*B - 4*A*C)) / (2*A);
	}
	else if(dir==RIGHT)
	{
		if(x1>x2)
			y0 = (-B - sqrt(B*B - 4*A*C)) / (2*A);
		else
			y0 = (-B + sqrt(B*B - 4*A*C)) / (2*A);
	}
	else
		return;

	x0 = c1 - c2*y0;
}


/*





*/
