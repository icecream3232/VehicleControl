import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
# 此处参数以激光雷达正前方为中心线,angle_range_left表示中心线以左考虑的角度，angle_range_right表示中心线以右考虑的角度，此处以角度计，单位为度(o)
angle_range_left = 30
angle_range_right = 30
rad_range_left = angle_range_left / 180 * 3.1415927410125732
rad_range_right = angle_range_right / 180 * 3.1415927410125732
# 此处参数为判断停障时障碍物距离激光雷达的最小距离
min_obs_dis = 0.2
# 此处参数为判断停障时满足障碍物距离激光雷达的最小距离的激光点个数
min_num_lidar = 3
lidar_brake_pub = rospy.Publisher("lidar_brake", Bool, queue_size=1)

def laser_callback(msg : LaserScan):
    # 处理接收到的LaserScan数据
    # rospy.loginfo("Received laser scan data: range data size = %d", len(msg.ranges))
    lidar_brake = Bool()
    range_left_num = int(rad_range_left / msg.angle_increment)
    range_right_num = int(rad_range_right / msg.angle_increment)
    total_lidar_num = len(msg.ranges)
    # dis_list_left = []
    # angle_list_left = []
    # dis_list_right = []
    # angle_list_right = []
    # for i in range(range_left_num):
    #     dis_list_left.append(msg.ranges[i])
    #     angle_list_left.append(-3.1415927410125732 + i * msg.angle_increment)
    # for i in range(range_right_num):
    #     dis_list_right.append(msg.ranges[total_lidar_num - i - 1])
    #     angle_list_right.append(3.1415927410125732 - i * msg.angle_increment)
    # dis_list = dis_list_left
    # dis_list.extend(dis_list_right)
    # angle_list = angle_list_left
    # angle_list.extend(angle_list_right)
    
    dis_angle_list_left = []
    dis_angle_list_right = []
    for i in range(range_left_num):
        dis_angle_list_left.append([msg.ranges[i], -3.1415927410125732 + i * msg.angle_increment, i])
    for i in range(range_right_num):
        dis_angle_list_right.append([msg.ranges[total_lidar_num - i - 1], 3.1415927410125732 - i * msg.angle_increment, -i])
    dis_angle_list = dis_angle_list_left
    dis_angle_list.extend(dis_angle_list_right)
    dis_angle_list.sort(reverse=False, key=lambda x:x[0])
    # print(dis_angle_list)
    lidar_brake.data = False# 默认为不触发停障命令
    print(dis_angle_list[0][0])
    if(dis_angle_list[0][0] <= min_obs_dis):# 若离得最近的激光点距离小于min_obs_dis
        i = 0
        obs_list = []
        while(dis_angle_list[i][0] <= min_obs_dis):# 存储所有距离小于min_obs_dis的激光点到obs_list
            obs_list.append(dis_angle_list[i])
            i+=1
        obs_list.sort(key=lambda x:x[2])# 按照序列进行排序
        print(obs_list)
        for i in range(len(obs_list)):
            obs_flag = True
            for j in range(min_num_lidar-1):
                if(i + j + 2 > len(obs_list)):# 若满足连续激光点数超出了obs_list范围
                    obs_flag = False
                    break
                else:
                    if(abs(obs_list[i+j+1][2] - obs_list[i+j][2]) > 2):# 若激光点序列不连续
                        obs_flag = False
                        break
            if(obs_flag):# 若以i起始的激光点序列连续满足min_num_lidar，则认为前方有障碍物，进行停障
                lidar_brake.data = True
                break
    
    # plt.ion()
    # plt.clf()  #清除上一幅图像
    # ax = plt.figure().add_subplot(111, polar=True)
    # ax.scatter(angle_list, dis_list)
    # ax.set_title('Laser Scan Data Visualization')
    # ax.set_xlabel('Angle (radians)')
    # ax.set_ylabel('Distance (meters)')
    # plt.pause(0.001)
    # plt.ioff()
    # print(dis_list_left)
    # print(dis_list_right)
    
    
    # rospy.loginfo("lidar brake flag is {}\n".format(lidar_brake.data))
    lidar_brake_pub.publish(lidar_brake)

def lidar_listener():
	# 初始化节点
    rospy.init_node('laser_scan_listener', anonymous=True)
    
    # 订阅LaserScan数据
    rospy.Subscriber("/scan", LaserScan, laser_callback)
    
    # 维持节点运行，直到被关闭
    rospy.spin()
    
if __name__ == '__main__':
    lidar_listener()