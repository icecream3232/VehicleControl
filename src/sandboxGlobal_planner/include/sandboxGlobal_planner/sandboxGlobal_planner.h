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
* Author: Eitan Marder-Eppstein
*********************************************************************/
#ifndef CARROT_PLANNER_H_
#define CARROT_PLANNER_H_
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>

#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

#include <vector>
#include <common_msgs/v2c_control.h>
#define inf 1>>20
#define STRAIGHT 1
#define LEFT 2
#define RIGHT 3
struct node;
typedef node* nodePtr;
struct node//节点结构体，用于Astar规划算法
{
    int idx;//节点在拓扑地图中的编号
    int state;//节点状态，为1是在openlist中，为-1是在closelist中，为0是还未扩展
    double gScore;//记录节点代价，gScore为起点到到目前节点累积的代价cost
    double hScore;//记录节点代价，hScore为当前节点到目标点的代价估计
    nodePtr comeFrom;//记录从哪个节点扩展到此节点的
    node(){}
    node(int _idx){
        idx = _idx;
        state = 0;
        gScore = inf;
        hScore = inf;
        comeFrom = nullptr;
    }
    //设置node参数
    void set(int _state, nodePtr _comeFrom, int _gScore, int _hScore){
        state = _state;
        comeFrom = _comeFrom;
        gScore = _gScore;
        hScore = _hScore;
    }
    ~node(){};
};

namespace sandboxGlobal_planner{
  /**
   * @class CarrotPlanner
   * @brief Provides a simple global planner that will compute a valid goal point for the local planner by walking back along the vector between the robot and the user-specified goal point until a valid cost is found.
   */
  class sandboxGlobal_planner : public nav_core::BaseGlobalPlanner {
    public:
      /**
       * @brief  Constructor for the CarrotPlanner
       */
      sandboxGlobal_planner();
      /**
       * @brief  Constructor for the CarrotPlanner
       * @param  name The name of this planner
       * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
       */
      sandboxGlobal_planner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief Destructor
       */
      ~sandboxGlobal_planner();

      /**
       * @brief  Initialization function for the CarrotPlanner
       * @param  name The name of this planner
       * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
       */
      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      // 查询某一坐标的最近邻地图点的编号
      int nearest_dot(double x,double y);
      //获取两个节点之间的估计代价,使用两点坐标位置连线距离作为估计代价
      double getHeu(nodePtr node1, nodePtr node2);
      //获得两个节点之间的cost,输入为两个节点之间的边的编号，编号对应mapEdgeInfoS中的元素
      double getCost(int Edge_idx);
      //获取路径上点的编号，并以vector<int>存储，从起始点到终点
      std::vector<int> getPath(nodePtr end_node);
      //对于当前点能够扩展的点，计算分别对应的cost，并将能扩展的点加入neighborPtrSets，对应的cost加入neighborPtrSets
      void AstarGetSeries(nodePtr currentPtr, std::vector<nodePtr> & neighborPtrSets, std::vector<double> & edgeCostSets);
      //初始化地图
      void initMap();
      // 路径插值工具函数：对于右为x正，上为y正的坐标系，判断由点(x1,y1)指向点(x2,y2)的向量与x轴正方向的夹角
      double calculateAngle(int y2, int y1, int x2, int x1);
      // 路径插值工具函数：计算圆弧的圆心坐标
      void calcuCircleCenter(double x1, double y1, double x2, double y2, double r, int dir, double &x0, double &y0);
      //路径插值得到稠密路径
      void interpolationPath(std::vector<int> &path, std::vector<std::vector<double>> &densePath, bool isAP);
      //Astar的主函数
      std::vector<int> Astar_planner(int start_nearest_idx,int end_nearest_idx);

      double get_yaw_from_record(double x, double y);
      // 将规划出的路径点从前轴中心为坐标系转为以车后轴中心为坐标系,路径点的坐标单位为m，wheelBase为轴距，单位为m
      void trans_coord(std::vector<std::vector<double>> &PathWithTheta, double wheelBase);
      // 将规划得到的点添加到plan中
      void add_points2plan(std::vector<std::vector<double>> &PathWithTheta, std::vector<geometry_msgs::PoseStamped> &paln);
      /**
       * @brief Given a goal pose in the world, compute a plan
       * @param start The start pose 
       * @param goal The goal pose 
       * @param plan The plan... filled by the planner
       * @return True if a valid plan was found, false otherwise
       */
      bool makePlan(const geometry_msgs::PoseStamped& start, 
          const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

      int  goal_last = -1;
      double goal_last_angle = 0.0;
      bool is_replanning = false;// 标记本次全局规划是否是重规划
      std::vector<std::vector<double>> points_record; // 记录正常规划时规划得到的点信息，格式为{x, y, yaw}，用于重规划时获取离车最近的路径点的航向角

    private:
      void correct_angle_deviation(std::vector<std::vector<double>> &densePath, double theta_bias);
      nodePtr *astarMap;//拓扑地图

      costmap_2d::Costmap2DROS* costmap_ros_;
      double step_size_, min_dist_from_robot_;
      costmap_2d::Costmap2D* costmap_;
      base_local_planner::WorldModel* world_model_; ///< @brief The world model that the controller will use

      /**
       * @brief  Checks the legality of the robot footprint at a position and orientation using the world model
       * @param x_i The x position of the robot 
       * @param y_i The y position of the robot 
       * @param theta_i The orientation of the robot
       * @return 
       */
      double footprintCost(double x_i, double y_i, double theta_i);

      bool initialized_;
  };
};  
#endif
