#include<ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>  //四元数与欧拉角互转
#include <tf2/LinearMath/Matrix3x3.h>

#include <common_msgs/v2c_control.h>//定義發布消息類型
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <nav_msgs/Odometry.h>
//tcp
#include <sys/socket.h>
#include <arpa/inet.h>
#include <fcntl.h>  //接收数据阻塞与否

#include <boost/algorithm/string/classification.hpp>  //處理接收的數據，解析
#include <boost/algorithm/string/split.hpp>

#include <sstream>//使用stringStream類
#include <vector>  
#include <string>
#include <iostream>
using namespace std;

int sockfd;
float curV=0; 
int vehicle_id;
string target_ip;
string target_port;
double pose_x = -10000;
double pose_y = -10000;
double pose_yaw = -10000;

void positionCallBack(const nav_msgs::Odometry& p);
void poseCallBack(const geometry_msgs::PoseWithCovarianceStamped& p);
void speedCallBack(const std_msgs::Float32& v);
string DoubleToString(double d);

// #define TEST__

int main(int argc,char** argv)
{
	ros::init(argc,argv,"v2c");
	ros::NodeHandle nh;
	ros::Publisher v2c_pub=nh.advertise<common_msgs::v2c_control>("v2c",10);
	ros::Subscriber sub1=nh.subscribe("current_speed",1,speedCallBack);
	ros::Subscriber sub2=nh.subscribe("/odom",1,positionCallBack);
	ros::Subscriber pose_sub = nh.subscribe("/robot_pose_ekf/odom_combined", 1, poseCallBack);                //订阅决策发布的位姿

	int v2c_frequency_Hz=0;

	// 测试
	#ifdef TEST__
		v2c_frequency_Hz = 20;
		vehicle_id = 1;
	#else
		if(!nh.getParam("v2c_frequency_Hz", v2c_frequency_Hz)) 
		{
			ROS_INFO_STREAM("Get v2c_frequency_Hz param fail ... return");
			// 测试注释
			return -1;
		}
		if(!nh.getParam("vehicle_id", vehicle_id))                                          // 获取本车ID
		{
			ROS_INFO_STREAM("Get vehicle_id param fail... default = 1.");
			vehicle_id = 1;
		}
		if(!nh.getParam("target_ip", target_ip))                                          // 获取本车ID
		{
			ROS_INFO_STREAM("Get vehicle_id param fail... default = 192.168.0.11.");
			target_ip = "192.168.0.11";
		}
		if(!nh.getParam("target_port", target_port))                                          // 获取本车ID
		{
			ROS_INFO_STREAM("Get vehicle_id param fail... default = 4000.");
			target_ip = "4000";
		}
	#endif
	

//TCP配置
	//1、创建socket套接字：AF_INET（地址域：IPV4），SOCK_STREAM（套接字类型 流式套接字），0（协议类型 默认），返回-1表示创建失败
	sockfd=socket(AF_INET,SOCK_STREAM,0);
	//配置IP和端口
	struct sockaddr_in ser; //统一接口sockaddr_in 对应 IPV4
	ser.sin_family=AF_INET;
	ser.sin_port=htons(atoi(target_port.c_str()));
	ser.sin_addr.s_addr=inet_addr(target_ip.c_str());
	//2、连接
	socklen_t len=sizeof(struct sockaddr_in);
	
	//连接失败，尝试重新链接
	int connnectTime = 10;
	while(connnectTime--)
	{
	    ROS_INFO_STREAM("v2c try connect..."<<connnectTime);
		// 测试注释
		#ifdef TEST__

		#else
			if(!(connect(sockfd, (struct sockaddr*)&ser, len)<0))
	        break;
		#endif
	    
	}
	if(connnectTime<0)
	{
		ROS_INFO_STREAM("v2c connect error");
		// 测试注释
		#ifdef TEST__
			
		#else
			return -1;
		#endif
		
	}
	ROS_INFO_STREAM("v2c connect success");
	//3、配置为非阻塞模式（不用ioctl）
    int flag_fcntl = fcntl(sockfd, F_GETFL, 0);
    fcntl(sockfd, F_SETFL, flag_fcntl|O_NONBLOCK);

	ros::Rate loop_rate(v2c_frequency_Hz);

	// 测试使用
	#ifdef TEST__
		int test_flag = 0;
		int count = 20;
	#else
		
	#endif
	

	while(ros::ok())
	{
		ros::spinOnce();
		
		//socket接收
		char buff[1024]={0};
		memset(buff,0x00,1024);
		int ret = recv(sockfd,buff,1024,0);// 返回值>0表示正确接收。 ==0表示另一端关闭 ==-1发生错误
		
		// 测试使用
		#ifdef TEST__
			if (test_flag == 0 && count >=0)
			{
				count--;
				if (count<=0) {
					test_flag = 1;
				}
				common_msgs::v2c_control msg;
				msg.data1 = 2.0; // 加减速，加速2、减速1、制动0
				msg.data2 = 0.0; // 速度增值
				msg.data3 = 4.0; // target_vertex
				msg.ID = 1;
				msg.type = 150;
				v2c_pub.publish(msg);
				ROS_INFO_STREAM("V2C sending success");
			}
		#else
			if(ret>0)
			{
				string A=buff;

				vector<string> vStr;
				boost::split(vStr,A,boost::is_any_of(","),boost::token_compress_on);
				if(vStr.size()<2)
					continue;
					
				int car_number=stoi(vStr[1].data(),NULL);
				ROS_INFO_STREAM("The car number is "<<car_number);
				int type=stoi(vStr[2].data(),NULL);
				if(type==150){
					ROS_INFO_STREAM("NOW the appliation scene1: Tracking");
				}
				else if(type==151){
					ROS_INFO_STREAM("NOW the appliation scene2: Parking");
				}
				else if(type==152){
					ROS_INFO_STREAM("NOW the appliation scene3: Single car speed control");
				}
				else if(type == 155){
					ROS_INFO_STREAM("NOW the appliation scene5: Emergency stop");
				}
				else if(type == 156){
					ROS_INFO_STREAM("NOW the appliation scene2: Parking");
				}
				else{
					ROS_INFO_STREAM("NOW the appliation scene4: Multi cars control");
				}
				
				common_msgs::v2c_control msg;

				double control1=strtod(vStr[3].data(),NULL);
				ROS_INFO_STREAM("control1="<<control1);
				msg.data1=control1;
				double control2=strtod(vStr[4].data(),NULL);
				ROS_INFO_STREAM("control2="<<control2);
				msg.data2=control2;
				double control3=strtod(vStr[5].data(),NULL);
				ROS_INFO_STREAM("control3="<<control3);
				msg.data3=control3;
				
				msg.ID=car_number;
				msg.type=type;
				v2c_pub.publish(msg);
			}
		#endif
		loop_rate.sleep();	
	}
}

string DoubleToString(double d)
{
    string str;
    stringstream ss;
    ss<<d;
    ss>>str;
    return str;
}

void positionCallBack(const nav_msgs::Odometry& p)
{
	curV = p.twist.twist.linear.x * 1000;

    string position_x= std::to_string(pose_x);
    string position_y= std::to_string(pose_y);
    string position_yaw= std::to_string(pose_yaw);
    string position_speed= std::to_string(curV);
	string vehicle_num = std::to_string(vehicle_id);
    string Position="$S,"+vehicle_num+","+position_x+","+position_y+","+position_yaw+",00,"+position_speed+",000,0000,*BB\r\n";

    const char* position=Position.c_str();

    send(sockfd,position,Position.size(),0);
	// ROS_INFO_STREAM("pos: "<<pose_x << "," << pose_y << "," << curV);

}

void poseCallBack(const geometry_msgs::PoseWithCovarianceStamped& p)
{
    
	double dx = 0.928;
	double dy = 0.126;
	// 接收位置数据
    pose_x = (p.pose.pose.position.x + dx) * 1000;     //xy位置正常接收
    pose_y = (p.pose.pose.position.y + dy) * 1000;	// m->mm
	    //四元数 转 欧拉角             偏航角需要经四元数转化而来
    tf2::Quaternion quat(
        p.pose.pose.orientation.x,
        p.pose.pose.orientation.y,
        p.pose.pose.orientation.z,
        p.pose.pose.orientation.w);
        
    double roll, pitch, yaw;
    tf2::Matrix3x3 m(quat);
    m.getRPY(roll, pitch, yaw);
    
    pose_yaw = yaw;
    
	pose_yaw = fmod(pose_yaw + M_PI, 2.0 * M_PI);
	if (pose_yaw < 0) {
		pose_yaw += 2.0 * M_PI;
	}
	pose_yaw = pose_yaw - M_PI;
	pose_yaw = pose_yaw * 1000;
}

void speedCallBack(const std_msgs::Float32& v)
{
    curV=v.data;
}










