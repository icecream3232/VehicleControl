//ros
#include <ros/ros.h>
#include <v2i/common_map.h>
#include <geometry_msgs/PoseStamped.h>  // 消息类型 订阅决策节点 融合定位数据 
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


#include <std_msgs/Bool.h>              // 消息类型 发布 信号灯停车命令
#include <tf2/LinearMath/Quaternion.h>  //四元数与欧拉角互转
#include <tf2/LinearMath/Matrix3x3.h>

//socket
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <fcntl.h>  //接收数据阻塞与否

//c++
#include <iostream>
#include <vector>
#include <string>

using namespace std;

//******************************
//          全局变量
//******************************
int sockfd = -1;                            // socket id
vector<vector<int>> traffic_light_info;     // 定义信号灯信息，每个灯为一个vector，包括：灯色(1红 2黄 3绿 0初始)和剩余时间
ros::Publisher tl_pub;                      //
int num_LigCtr_1 = 9;
int num_LigCtr_2 = 0;
int num_TraLight = num_LigCtr_1 + num_LigCtr_2;
//******************************
//          功能函数
//******************************
void decode(char* ip, char* buffer);        // 解析接收的信号

//******************************
//          回调函数
//******************************
void poseCallBack(const geometry_msgs::PoseWithCovarianceStamped& p);//接收决策发布的融合位姿

//******************************
//          主程序
//******************************
int main(int argc, char** argv)
{
    ros::init(argc, argv, "v2i");
    ros::NodeHandle nh;
    
    tl_pub = nh.advertise<std_msgs::Bool>("v2i_brake", 1);                   //向决策发布，因交通灯而起的车辆起停
    ros::Subscriber pose_sub = nh.subscribe("/robot_pose_ekf/odom_combined", 1, poseCallBack);                //订阅决策发布的位姿
    
    int v2i_frequency_Hz=0;
	if(!nh.getParam("v2i_frequency_Hz", v2i_frequency_Hz)) 
	{
        ROS_INFO_STREAM("Get v2i_frequency_Hz param fail ... return");
        return -1;
    }	

// *******************通信配置*************************   
// 创建socket
    //AF_INET：IPV4，AF_INET6：IPV6
    //SOCK_DGRAM：UDP套接字，STREAM：TCP套接字
    //0：根据前两个参数确定使用的协议
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if(-1==sockfd)
        return -1;

// 设置本地地址与广播接收端口
    struct sockaddr_in addr_local;
    socklen_t addr_local_len=sizeof(addr_local);
    memset(&addr_local, 0, addr_local_len);
    addr_local.sin_family      = AF_INET;               // Use IPV4
    addr_local.sin_port        = htons(6000);           
    addr_local.sin_addr.s_addr = htonl(INADDR_ANY);     //接收任意IP发来的数据

// 绑定本地地址与广播接收端口
    int bind_result = bind(sockfd, (struct sockaddr*)&addr_local, addr_local_len); 
    if (bind_result == -1)
    {
        close(sockfd);
        return -1;
    }

// 定义数据缓存
    char buffer[30];
    memset(buffer, 0, 30);
 
//配置为非阻塞模式（不用ioctl）
    int flag_fcntl = fcntl(sockfd, F_GETFL, 0);
    fcntl(sockfd, F_SETFL, flag_fcntl|O_NONBLOCK);
  
// **********************基本变量配置********************** 
//信号灯初始状态：全部初始化为初始化灯色3，时间为0，但是最后一个灯除外，最后一个灯为常绿灯
    for(int i=0; i<num_TraLight; i++)  // 修改
    {
        vector<int> traffic_light_info_i;
        // if(i!=11)
        // {
        //     traffic_light_info_i.push_back(0);//初始灯色
        //     traffic_light_info_i.push_back(0);//初始时间
        // }
        // else
        // {
        //     traffic_light_info_i.push_back(3);//初始灯色
        //     traffic_light_info_i.push_back(40);//初始时间
        // }
        traffic_light_info_i.push_back(0);//初始灯色
        traffic_light_info_i.push_back(0);//初始时间
        
        traffic_light_info.push_back(traffic_light_info_i);
    }
    
    
// **********************实时接收与处理数据**********************    
    ros::Rate loop_rate(v2i_frequency_Hz);
    while(ros::ok())
    {
//接收数据
        struct sockaddr_in addr_from;                       // 定义 记录消息来源的IP
        socklen_t addr_from_len=sizeof(addr_from);
        memset(&addr_from, 0, addr_from_len);               // 初始化为空
        int len = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr*)&addr_from, &addr_from_len); //接收，消息来源的IP将存到addr_from中
        
//解析与存储
        if (len>0) {
            decode(inet_ntoa(addr_from.sin_addr), buffer);      // 按协议解析数据
            memset(buffer, 0, 20);
        }
        
        
//回调
        ros::spinOnce();
        
        
        // for(int i=0; i<num_TraLight; i++)
            // ROS_INFO_STREAM("Traffic_light_info_"<<i<<": color:"<<traffic_light_info[i][0]<<", time:"<<traffic_light_info[i][1]);
        
        
        loop_rate.sleep();
    }
    
    close(sockfd);

    return 0;
}

//******************************
//          函数实现
//******************************

/*
    功能函数：解析接收的信号
    --------------------------------------------
    road_id     light_id    light_id    light_id 
     路口       信号灯号   信号灯号   信号灯号
    --------------------------------------------   
        1           0           1           2
        2           3           4
        3           5           6
        4           7           8
        5           9           10
    --------------------------------------------
*/
void decode(char* ip, char* buffer)
{
    // ROS_INFO_STREAM("Get traffic_light_info from:"<<ip<<", buf:"<<buffer);
//取出IP的字段
    char* c = strrchr(ip, '.'); // 找到最后一个“.”的位置（指针）
    int index = c - ip + 1;     // 对应的第几个字符
    
    string ip_str = ip;       
    string ip_str_sub_front3 = ip_str.substr(0, index);                     // 取出前三个字段 即：192.168.11. 
    string ip_str_sub_back1 = ip_str.substr(index, ip_str.size()-index);    // 取出最后一个字段 即：200等
    int ip_int = stoi(ip_str_sub_back1);  // 转换成整型
    if(ip_str_sub_front3.compare("192.168.0.") != 0)
        return;

//解析   
    int start_decode_index = 9; // 开始解析的索引
    if(1 == (ip_int-199))       // 1号控制板
    {
        for(int i=0; i<num_LigCtr_1; i++)
        {
            //同一个信号灯，红黄绿三个灯珠只有一种灯珠的时间不为零，基于此，将非零的灯色和时间保存下来
            // if(int green = buffer[start_decode_index + 3*i])
            if(int green = buffer[start_decode_index])
            {
                traffic_light_info[i][0] = 3;
                traffic_light_info[i][1] = green;
            }
            // else if(int yellow = buffer[start_decode_index + 1 + 3*i])
            else if(int yellow = buffer[start_decode_index + 1])
            {
                traffic_light_info[i][0] = 2;
                traffic_light_info[i][1] = yellow;
            }
            // else if(int red = buffer[start_decode_index + 2 + 3*i])
            else if(int red = buffer[start_decode_index + 2])
            {
                traffic_light_info[i][0] = 1;
                traffic_light_info[i][1] = red;
            }
        }
    }
    else if(2 == (ip_int-199))   // 2号路口控制板
    {
        // int road_id = ip_int-199;
        for(int i=num_LigCtr_1; i<num_LigCtr_1+num_LigCtr_2; i++)
        {
            //同一个信号灯，红黄绿三个灯珠只有一种灯珠的时间不为零，基于此，将非零的灯色和时间保存下来
            if(int green = buffer[start_decode_index + 3*i])
            {
                traffic_light_info[i][0] = 3;
                traffic_light_info[i][1] = green;
            }
            else if(int yellow = buffer[start_decode_index + 1 + 3*i])
            {
                traffic_light_info[i][0] = 2;
                traffic_light_info[i][1] = yellow;
            }
            else if(int red = buffer[start_decode_index + 2 + 3*i])
            {
                traffic_light_info[i][0] = 1;
                traffic_light_info[i][1] = red;
            }
        }
    }
}

/*
    回调函数：接收决策发布的融合位姿
*/
void poseCallBack(const geometry_msgs::PoseWithCovarianceStamped& p)
{
    
    // 接收位置数据
    // double pose_x = p.pose.position.x;     //xy位置正常接收
    // double pose_y = p.pose.position.y;
    double pose_x = p.pose.pose.position.x;     //xy位置正常接收
    double pose_y = p.pose.pose.position.y;	// m->mm
    
    //四元数 转 欧拉角             偏航角需要经四元数转化而来
    tf2::Quaternion quat(
        p.pose.pose.orientation.x,
        p.pose.pose.orientation.y,
        p.pose.pose.orientation.z,
        p.pose.pose.orientation.w);
        
    double roll, pitch, yaw;
    tf2::Matrix3x3 m(quat);
    m.getRPY(roll, pitch, yaw);
    
    double pose_yaw = yaw;
    
// 判断是否位于判定区   
    int index = -1;
    // ROS_INFO_STREAM("V2i Judge");
    // ROS_INFO_STREAM("Var LOC:" << pose_x*1000.0 << " , " << pose_y*1000.0);
    for(int i=0;i<num_TraLight;i++)
    {
        vector<double> tl = TLInfoS[i];
        double xmin = tl[0];
        double xmax = tl[1];
        double ymin = tl[2];
        double ymax = tl[3];
        // ROS_INFO_STREAM("RECEIVE LOC" << pose_x*1000 << " , " << pose_y*1000);
        // ROS_INFO_STREAM("Area LOC" << xmin << " , " << xmax << " , " << ymin << " , " << ymax);
        if((pose_x*1000.0 >= xmin) && (pose_x*1000.0 <= xmax) && (pose_y*1000.0 >= ymin) && (pose_y*1000.0 <= ymax))
        {
            index = i;
            break;
        }
    }

// 输出结果    
    std_msgs::Bool d;
    d.data = false;
    if(index < 0)
    {
        tl_pub.publish(d);
        // ROS_INFO_STREAM("v2c over 1: " << d);
        return;
    }
     
    if(traffic_light_info[index][0] == 1 || traffic_light_info[index][0] == 2)  // 灯色是红色跟黄色 都为停车
        { d.data = true; tl_pub.publish(d); }
    else
        tl_pub.publish(d);
    // ROS_INFO_STREAM("v2c over 2: " << d);
    /*
    //欧拉角 转 四元数
        tf2::Quaternion orien;
        orien.setRPY(1,1,1);
    */
}

