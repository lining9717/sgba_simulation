#ifndef _HECTOR_SIMULATION_H_
#define _HECTOR_SIMULATION_H_

#include "sgba_bug_planner.h"
#include "sgba_back_planner.h"
#include "sgba_exception.h"
#include "sgba_simulation/EnableMotors.h"
#include "sgba_uav.h"

#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>
#include <thread>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

typedef std::shared_ptr<Bug::BugPlanner> BugPlannerPtr;

class Simulation
{
public:
    Simulation();
    ~Simulation();

    //启动
    void start();

    //每架无人机开始飞行的回调函数
    void startUAVCallback(int id, position p);

    //轨迹显示回调函数
    void trajectoryCallback();

    //地图绘制回调函数
    void obstacleMapCallback();

private:
    ros::NodeHandle nh;

    //前一架无人机的位置
    position moving_target = {0, -1, 0};

    // 无人机数组
    std::vector<std::shared_ptr<UAV>> uavs;

    std::vector<std::pair<int, int>> move_directions;
    //无人机数量
    int uav_num;

    //开始追击时，上一架无人机剩余的电量
    // int sleep_battery = 30;

    //地图宽度和高度
    int width = 0, height = 0;

    //中心原点和左上角原点转换偏移值
    int x_bias = 0, y_bias = 0;

    //从左上角原点转化为中心原点
    int getXFromCol(int col);

    //从左上角原点转化为中心原点
    int getYFromRow(int row);

    std::vector<std::string> lab;
};

#endif