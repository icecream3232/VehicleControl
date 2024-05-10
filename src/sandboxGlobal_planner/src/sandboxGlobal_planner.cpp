/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Authors: Eitan Marder-Eppstein, Sachin Chitta
*********************************************************************/
#include <angles/angles.h>
#include <sandboxGlobal_planner/sandboxGlobal_planner.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// 更改此处引用的头文件来进行test和正式地图之间的切换
#include <sandboxGlobal_planner/common_map.h>
// #include <sandboxGlobal_planner/common_map_test.h>
#define PARKING 2 // 正常地图中PARKING = 2为倒车边，测试地图中PARKING = 1

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(sandboxGlobal_planner::sandboxGlobal_planner, nav_core::BaseGlobalPlanner)

namespace sandboxGlobal_planner {

  sandboxGlobal_planner::sandboxGlobal_planner()
  : costmap_ros_(NULL), costmap_(NULL), world_model_(NULL), initialized_(false){}

  sandboxGlobal_planner::sandboxGlobal_planner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  : costmap_ros_(NULL), costmap_(NULL), world_model_(NULL), initialized_(false){
    initialize(name, costmap_ros);
  }

  sandboxGlobal_planner::~sandboxGlobal_planner() {
    // deleting a nullptr is a noop
    delete world_model_;
  }
  
  void sandboxGlobal_planner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    if(!initialized_){
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();

      ros::NodeHandle private_nh("~/" + name);
      private_nh.param("step_size", step_size_, costmap_->getResolution());
      private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
      world_model_ = new base_local_planner::CostmapModel(*costmap_); 

      initialized_ = true;
    }
    else
      ROS_WARN("This planner has already been initialized... doing nothing");
  }

  //we need to take the footprint of the robot into account when we calculate cost to obstacles
  double sandboxGlobal_planner::footprintCost(double x_i, double y_i, double theta_i){
    if(!initialized_){
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return -1.0;
    }

    std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
    //if we have no footprint... do nothing
    if(footprint.size() < 3)
      return -1.0;

    //check if the footprint is legal
    double footprint_cost = world_model_->footprintCost(x_i, y_i, theta_i, footprint);
    return footprint_cost;
  }

  // 查询某一坐标的最近邻地图点的编号
  int sandboxGlobal_planner::nearest_dot(double x,double y){
    x = x * 1000;
    y = y * 1000;
      int curVertex = 1;
      double minDis = pow(mapVertexInfoS[0].vertexPos.x - x , 2) + pow(mapVertexInfoS[0].vertexPos.y - y , 2);
      minDis = sqrt(minDis);
      for(auto each : mapVertexInfoS){
          double dis = pow(each.vertexPos.x - x , 2) + pow(each.vertexPos.y - y , 2);
          dis = sqrt(dis);
          if(dis < minDis)
          {
              curVertex = each.vertexPos.id;
              minDis = dis;
          }
      }
      return curVertex;
  }

  //获取两个节点之间的估计代价,使用两点坐标位置连线距离作为估计代价
  double sandboxGlobal_planner::getHeu(nodePtr node1, nodePtr node2){
      // double hScore;
      // double x_node1 = mapVertexInfoS[node1->idx].vertexPos.x;
      // double y_node1 = mapVertexInfoS[node1->idx].vertexPos.y;
      // double x_node2 = mapVertexInfoS[node2->idx].vertexPos.x;
      // double y_node2 = mapVertexInfoS[node2->idx].vertexPos.y;
      // hScore = sqrt(pow(x_node1-x_node2, 2) + pow(y_node1 - y_node2, 2));
      // return hScore;
      return 0;
  }

  //获得两个节点之间的cost,输入为两个节点之间的边的编号，编号对应mapEdgeInfoS中的元素
  double sandboxGlobal_planner::getCost(int Edge_idx){
      Edge currentEdge = mapEdgeInfoS[Edge_idx - 1];
      if(currentEdge.attr_type == 1){
          return currentEdge.attr_dis;
      }
      else if (currentEdge.attr_type == 2 || currentEdge.attr_type == 3){
          return currentEdge.attr_cost;
      }
      return -1;
  }

  //获取路径上点的编号，并以vector<int>存储，从起始点到终点
  vector<int> sandboxGlobal_planner::getPath(nodePtr end_node){
      nodePtr currentPtr = end_node;
      vector<int> path;
      while(currentPtr->comeFrom != nullptr){
          path.push_back(currentPtr->idx);
          currentPtr = currentPtr->comeFrom;
      }
      path.push_back(currentPtr->idx);
      reverse(path.begin(),path.end());
      return path;
  }

  //对于当前点能够扩展的点，计算分别对应的cost，并将能扩展的点加入neighborPtrSets，对应的cost加入neighborPtrSets
  void sandboxGlobal_planner::AstarGetSeries(nodePtr currentPtr, vector<nodePtr> & neighborPtrSets, vector<double> & edgeCostSets){
      neighborPtrSets.clear();
      edgeCostSets.clear();
      vector<int> nearNode = mapVertexInfoS[currentPtr->idx - 1].relatedVertexes;
      if(nearNode[0] != 0){
          vector<int> nearEdge = mapVertexInfoS[currentPtr->idx - 1].relatedEdges;
          for(int i = 0; i < nearNode.size(); i++){
              if(astarMap[nearNode[i] - 1]->state == -1){//若存在closelist中则跳过
                  continue;
              }
              neighborPtrSets.push_back(astarMap[nearNode[i] - 1]);
              edgeCostSets.push_back(getCost(nearEdge[i]));
          }
      } 
  }   

  //初始化地图
  void sandboxGlobal_planner::initMap(){
      astarMap = new nodePtr [mapVertexInfoS.size()];
      for(int i = 0; i < mapVertexInfoS.size(); i++){
          astarMap[i] = new node(i+1);
      }
  }

// 路径插值工具函数：对于右为x正，上为y正的坐标系，判断由点(x1,y1)指向点(x2,y2)的向量与x轴正方向的夹角
  double sandboxGlobal_planner::calculateAngle(int y2, int y1, int x2, int x1){
    double angle,tangent;
    if(x1 == x2 && y2 > y1){
        angle = M_PI / 2.0;
    }
    else if (x1 == x2 && y2 < y1)
    {
        angle = -M_PI / 2.0;
    }
    else if (y2 == y1 && x2 > x1){
        angle = 0;
    }
    else if (y2 == y1 && x2 < x1)
    {
        angle = M_PI;
    }
    else{
        tangent = double((y2 - y1))/double((x2 - x1));
        angle = atan(tangent);
        if ((y2-y1 < 0 && x2-x1 < 0) || (y2-y1 > 0 && x2-x1 < 0)){
            angle += M_PI;
        }
        else if (x2-x1 > 0 && y2-y1 < 0){
            angle += 2*M_PI;
        }
    }
    return angle;
}

// 路径插值工具函数：计算圆弧的圆心坐标
  void sandboxGlobal_planner::calcuCircleCenter(double x1, double y1, double x2, double y2, double r, int dir, double &x0, double &y0){   
    //几何方法，求解该段圆弧的中间点与原点的距离以及该两点连线与x轴的角度，从而求解出圆心坐标
    double x_mid = (x1 + x2)/2;
    double y_mid =(y1 + y2)/2;
    double angle = calculateAngle(y2, y1, x2, x1);
    double d = sqrt(r*r-(x1-x_mid)*(x1-x_mid)-(y1-y_mid)*(y1-y_mid));
    double x_r1 = x_mid + d*cos(angle + M_PI / 2.0);
    double y_r1 = y_mid + d*sin(angle + M_PI / 2.0);
    double x_r2 = x_mid + d*cos(angle - M_PI / 2.0);
    double y_r2 = y_mid + d*sin(angle - M_PI / 2.0);
    if(dir == LEFT){
        x0 = x_r1;
        y0 = y_r1;
    }
    else if (dir == RIGHT){
        x0 = x_r2;
        y0 = y_r2;  
    }
  }

  //路径插值得到稠密路径
  void sandboxGlobal_planner::interpolationPath(vector<int> &path, vector<vector<double>> &densePath, bool isAP){
      int resolution = 50;									//插值的分辨率，单位：mm，下述距离单位均为 mm
      for(int i = 0; i < path.size()-1; i++)
    {
      //确定该节点坐标
      Vertex rNode=  mapVertexInfoS[path[i]-1];
      double x1 = rNode.vertexPos.x;
      double y1 = rNode.vertexPos.y;

      //确定相邻下一节点
      int nextNode = path[i+1];
      Vertex nNode = mapVertexInfoS[path[i+1]-1];
      double x2 = nNode.vertexPos.x;
      double y2 = nNode.vertexPos.y;

      //确定两节点之间的路线属性
      int index = find(rNode.relatedVertexes.begin(), rNode.relatedVertexes.end(), nextNode) - rNode.relatedVertexes.begin();
      int edgeIndex = rNode.relatedEdges[index];
      Edge rEdge = mapEdgeInfoS[edgeIndex-1];
      double dis = rEdge.attr_dis;
      double angle = rEdge.attr_angle;
      int edgeType = rEdge.attr_type;

      if(rEdge.attr_isReverse && (edgeType!=1)) //表示此时为泊车路径插值，要考虑是否是倒车：若是倒车，在插值算法下，需要将左右互换
        edgeType = (2 == edgeType) ? 3 : 2;

      //添加该路段首节点
      vector<double> add0 = {x1*0.001,y1*0.001,double(path[i]),double(path[i+1]),double(edgeIndex)};
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

          vector<double> add1{xx*0.001,yy*0.001,double(path[i]),double(path[i+1]),double(edgeIndex)};
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

          double a = calculateAngle(y1, y0, x1, x0);  
          double b = 3.14159 - isoscelesAngle + a;

          double xx = x1 + chord*cos(b);
          double yy = y1 + chord*sin(b);

          vector<double> add1{xx*0.001,yy*0.001,double(path[i]),double(path[i+1]),double(edgeIndex)};
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

          double a = calculateAngle(y1, y0, x1, x0);
          double b = isoscelesAngle + a - 3.14159;

          double xx = x1 + chord*cos(b);
          double yy = y1 + chord*sin(b);

          vector<double> add1{xx*0.001,yy*0.001,double(path[i]),double(path[i+1]),double(edgeIndex)};
          densePath.push_back(add1);
        }  
      }
    }
  }

// replanning时，获取离当前车辆位置最近点之前的航向角，输入单位为m,输入为重规划路径得到的第1个点的坐标，返回对应的yaw。若找不到对应的，则返回0.0
  double sandboxGlobal_planner::get_yaw_from_record(double x, double y){
    for(auto point : points_record){
      if(abs(point[0] - x) < 0.001 && abs(point[1] - y) < 0.001){
        return point[2];
      }
    }
    return 0.0;
  }


// 对将要发布的路径点，做角度偏差的矫正
  void sandboxGlobal_planner::correct_angle_deviation(vector<vector<double>> &densePath, double theta_bias){
    vector<vector<double>> polar_points;// 转换到极坐标系下
    vector<double> polar_point = {0.0, 0.0};// 极坐标系下一个点的坐标，第一个是与x轴正半轴角度，第二个是半径
    for(int i = 0; i < densePath.size(); i++){
      vector<double> point = densePath[i];
      double r = sqrt(pow(point[0], 2.0) + pow(point[1], 2.0));
      double theta = calculateAngle(point[1]*1000.0, 0.0, point[0]*1000.0, 0.0) + theta_bias;
      double x = r * cos(theta);
      double y = r * sin(theta);
      densePath[i][0] = x;
      densePath[i][1] = y;
    }
  }

  //Astar的主函数
  vector<int> sandboxGlobal_planner::Astar_planner(int start_nearest_idx,int end_nearest_idx){
      multimap<double,nodePtr> openlist;//存储所有已访问但未扩展的节点
      nodePtr start_node = astarMap[start_nearest_idx - 1];//起始节点
      nodePtr end_node = astarMap[end_nearest_idx - 1];//目标节点
      start_node->set(1,nullptr,0,getHeu(start_node,end_node));//初始化目标节点
      openlist.insert(make_pair(start_node->gScore + start_node->hScore,start_node));//将初始节点加入openlist

      vector<nodePtr> neighborPtrSets;//存储每次对某个节点扩展得到的临近节点
      vector<double> edgeCostSets;//存储两个临近节点之间的cost
      nodePtr currentPtr;//每轮扩展的节点

      while(!openlist.empty()){
          currentPtr = openlist.begin()->second;//获取openlist中代价f最小的点
          currentPtr->state = -1;//设置当前点已访问过，放入closelist
          openlist.erase(openlist.begin());//将该点从openlist中弹出

          if( currentPtr->idx == end_node->idx ){//如果扩展到目标点，则返回查找到的路径        
              vector<int> path = getPath(currentPtr);
              return path;
          }

          AstarGetSeries(currentPtr,neighborPtrSets,edgeCostSets);//获取当前点可扩展的点集及扩展代价cost集
          if(! neighborPtrSets.empty()){
          int neighborNums = neighborPtrSets.size();
              for(int i = 0; i < neighborNums; i++){
                  nodePtr neighborPtr = neighborPtrSets[i];
                  if(neighborPtr->state == 0){//若当前可扩展点不在openlist中，设置其参数，并加入openlist
                      neighborPtr->set(1,currentPtr,currentPtr->gScore + edgeCostSets[i],getHeu(neighborPtr,end_node));
                      openlist.insert(make_pair(neighborPtr->gScore + neighborPtr->hScore, neighborPtr));
                  }
                  else if (neighborPtr->state == 1){//若当前可扩展点在openlist中，比较已有扩展方式与新的扩展方式的总cost的大小，选择扩展方式
                      if(currentPtr->gScore + edgeCostSets[i] < neighborPtr->gScore){
                          openlist.erase(currentPtr->gScore+currentPtr->hScore);
                          neighborPtr->set(1,currentPtr,currentPtr->gScore + edgeCostSets[i],getHeu(neighborPtr,end_node));
                          openlist.insert(make_pair(neighborPtr->gScore + neighborPtr->hScore, neighborPtr));
                      }
                  }
                  
              }
          }
      }
      vector<int> path;
      path.push_back(-1);
      return path;//若无法找到路径，则返回path中的第一个元素为-1
  }

  bool sandboxGlobal_planner::makePlan(const geometry_msgs::PoseStamped& start, 
      const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){

    if(!initialized_){
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return false;
    }

    ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

    plan.clear();
    costmap_ = costmap_ros_->getCostmap();
    sandboxGlobal_planner::initMap();

    if(goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
      ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.", 
          costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
      return false;
    }
    
    if((goal.pose.position.x + 10000) < 0 && (goal.pose.position.y + 10000) < 0){// 红灯时停车
      plan.push_back(start);
      return true;
    }

    int start_nearest_idx = nearest_dot(start.pose.position.x, start.pose.position.y);
    int end_nearest_idx = nearest_dot(goal.pose.position.x, goal.pose.position.y);

    if(end_nearest_idx != goal_last){// 判断当前是否是同一目标点下的重新规划
      goal_last = end_nearest_idx;// 更新终点id
      is_replanning = false;
      points_record.clear();// 清空记录的路径点
    }
    else{
      is_replanning = true;
    }

    // 若为重规划，则直接使用上次规划得到的点，再发一遍
    if(is_replanning){
      ROS_DEBUG("---------START REPLANNING----------");
      int nearest_record_id;
      double nearest_dis = 10000000.0;
      for(int i = 0; i < points_record.size(); i++){
        double dis_now_point = sqrt(pow((start.pose.position.x - points_record[i][0]),2.0) + pow((start.pose.position.y - points_record[i][1]), 2.0));
        if(dis_now_point - nearest_dis < 0){
          nearest_dis = dis_now_point;
          nearest_record_id = i;
        }
      }
      vector<vector<double>> densePath(points_record.begin()+nearest_record_id,points_record.end());
      // 记录每一次取出点的上一个点的坐标
        vector<double> pos_last = {0.0, 0.0};
        for(int i = 0; i < densePath.size(); i++){
          // 如果当前点和上一个点的坐标相同，则不添加该点
          if(abs(pos_last[0]-densePath[i][0]) < 0.0001 && abs(pos_last[1]-densePath[i][1]) < 0.0001){
            continue;
          }
          pos_last[0] = densePath[i][0];
          pos_last[1] = densePath[i][1];
          geometry_msgs::PoseStamped pose_stamped;
          tf2::Quaternion goal_quat;
          //设置坐标系与时间
          pose_stamped.header.frame_id = "map";
          pose_stamped.header.stamp = ros::Time::now();
          //设置坐标
          pose_stamped.pose.position.x = densePath[i][0];
          pose_stamped.pose.position.y = densePath[i][1]; 
          //设置方向角，为两点连线方向
          double target_yaw;
          if(i == 0){
            // target_yaw = atan2(densePath[i][0]-start.pose.position.x,densePath[i][1]-start.pose.position.y);
            if(densePath.size() > 1){
              double yaw1 = calculateAngle(densePath[i][1]*1000, start.pose.position.y*1000, densePath[i][0]*1000, start.pose.position.x*1000);
              double yaw2 = densePath[i+1][2];
              if(3*M_PI/2 > abs(yaw1-yaw2) && abs(yaw1-yaw2) > M_PI/2){
                continue;
              }
              else{
                target_yaw = yaw1;
              }
            }
            else{
              target_yaw = densePath[i][2];
            }
          }
          else if(i < densePath.size()-1){
            // 在重规划中，densePath[i][2]为记录的对应点的yaw角，直接赋给target_yaw
            target_yaw = densePath[i][2];
          }
          // else if(i = densePath.size()-1){
          //     target_yaw = tf2::getYaw(goal.pose.orientation);
          // }
          goal_quat.setRPY(0, 0, target_yaw);
          pose_stamped.pose.orientation.x = goal_quat.x();
          pose_stamped.pose.orientation.y = goal_quat.y();
          pose_stamped.pose.orientation.z = goal_quat.z();
          pose_stamped.pose.orientation.w = goal_quat.w();

          ROS_DEBUG("pose_x:%f, pose_y:%f, pose_angle:%f",pose_stamped.pose.position.x,pose_stamped.pose.position.y,target_yaw);

          plan.push_back(pose_stamped);
        }
        return true;
    }

    ROS_DEBUG("---------START ASTARPLANNING--------");
    vector<int> idx_path = Astar_planner(start_nearest_idx,end_nearest_idx);
    if(idx_path[0] == -1){
        // cout<<"fail to path"<<endl;
        return false;
    }
    else{
        vector<vector<double>> densePath;
        interpolationPath(idx_path,densePath,false);
        //把目标点也加入到路径点中
        vector<double> end{goal.pose.position.x,goal.pose.position.y,0,0,0};
        densePath.push_back(end);
        // 矫正坐标的角度偏差
        correct_angle_deviation(densePath, theta_bias);
        // 记录每一次取出点的上一个点的坐标
        vector<double> pos_last = {0.0, 0.0};
        for(int i = 0; i < densePath.size(); i++){
          // 如果当前点和上一个点的坐标相同，则不添加该点
          if(abs(pos_last[0]-densePath[i][0]) < 0.0001 && abs(pos_last[1]-densePath[i][1]) < 0.0001){
            continue;
          }
          pos_last[0] = densePath[i][0];
          pos_last[1] = densePath[i][1];
          geometry_msgs::PoseStamped pose_stamped;
          tf2::Quaternion goal_quat;
          //设置坐标系与时间
          pose_stamped.header.frame_id = "map";
          pose_stamped.header.stamp = ros::Time::now();
          //设置坐标
          pose_stamped.pose.position.x = densePath[i][0];
          pose_stamped.pose.position.y = densePath[i][1]; 
          //设置方向角，为两点连线方向
          double target_yaw;
          if(i == 0){
            // target_yaw = atan2(densePath[i][0]-start.pose.position.x,densePath[i][1]-start.pose.position.y);
            if(is_replanning){
              target_yaw = get_yaw_from_record(densePath[i][0], densePath[i][1]);
            }
            else{
              if(densePath.size() > 1){
                double yaw1 = calculateAngle(densePath[i][1]*1000, start.pose.position.y*1000, densePath[i][0]*1000, start.pose.position.x*1000);
                double yaw2 = calculateAngle(densePath[i+1][1]*1000, densePath[i][1]*1000, densePath[i+1][0]*1000, densePath[i][0]*1000);
                if(3*M_PI/2 > abs(yaw1-yaw2) && abs(yaw1-yaw2) > M_PI/2){
                  continue;
                }
                else{
                  target_yaw = yaw1;
                }
              }
              else{
                target_yaw = calculateAngle(densePath[i][1]*1000, start.pose.position.y*1000, densePath[i][0]*1000, start.pose.position.x*1000);
              }
            }
          }
          else if(i < densePath.size()-1){
            // densePath[i][4]为该加密后获得的点对应的edge的id号，将其与宏PARKING进行对比判断当前点是否在倒车边上，正常地图中edge_id=1,2为倒车边，测试地图中1为倒车边
            if (densePath[i][4] > PARKING){
                // 当edge_id大于PARKING时，为正向行驶，此时该点方向 = 上一个点指向当前点的方向
                // target_yaw = atan2(densePath[i][0]-densePath[i-1][0],densePath[i][1]-densePath[i-1][1]);
                target_yaw = calculateAngle(densePath[i][1]*1000, densePath[i-1][1]*1000, densePath[i][0]*1000, densePath[i-1][0]*1000);
            }
            else{
                // 当edge_id小于PARKING时，为倒车行驶，此时该点方向 = 当前点指向上一个点的方向
                // target_yaw = atan2(densePath[i][0]-densePath[i-1][0],densePath[i][1]-densePath[i-1][1])+M_PI;
                target_yaw = calculateAngle(densePath[i-1][1]*1000, densePath[i][1]*1000, densePath[i-1][0]*1000, densePath[i][0]*1000);
            }
          }
          // else if(i = densePath.size()-1){
          //     target_yaw = tf2::getYaw(goal.pose.orientation);
          // }
          goal_quat.setRPY(0, 0, target_yaw);
          pose_stamped.pose.orientation.x = goal_quat.x();
          pose_stamped.pose.orientation.y = goal_quat.y();
          pose_stamped.pose.orientation.z = goal_quat.z();
          pose_stamped.pose.orientation.w = goal_quat.w();

          ROS_DEBUG("pose_x:%f, pose_y:%f, pose_angle:%f",pose_stamped.pose.position.x,pose_stamped.pose.position.y,target_yaw);

          plan.push_back(pose_stamped);
          if(!is_replanning){// 当不是重规划时，记录规划得到的点
            points_record.push_back({densePath[i][0], densePath[i][1], target_yaw});
          }
        }
        ROS_DEBUG("global plan over");
        return true;
    }

    const double start_yaw = tf2::getYaw(start.pose.orientation);
    const double goal_yaw = tf2::getYaw(goal.pose.orientation);

  }

};
