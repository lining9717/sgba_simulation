#ifndef _HECTOR_UAV_H_
#define _HECTOR_UAV_H_

#include "utiltool.h"
#include "sgba_exception.h"
#include "sgba_bug_planner.h"
#include "sgba_back_planner.h"
#include "sgba_simulation/EnableMotors.h"
#include <cmath>
#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sstream>
#include <tf/transform_datatypes.h>

#define FRONT 0x1110
#define LEFT 0x1111
#define BACK 0x1112
#define RIGHT 0x1113

#define NO_PATH 0xf321
#define GET_GOAL 0xf322

#define TRACKING 0x1114
#define WALL_FLLOWING 0x1115
#define IDLE 0x1116

//话题及服务名称
const std::string controller_topic_name = "/cmd_vel";
const std::string motor_service_name = "/enable_motors";
const std::string position_topic_name = "/ground_truth_to_tf/pose";

class UAV
{
private:
    ros::NodeHandle nh;
    ros::NodeHandle nh_private;
    ros::Publisher publisher;  //用于发布cmd_vel话题
    ros::ServiceClient client; //用于驱动无人机马达的服务客户端
    ros::Subscriber sub;       //用于订阅获取无人机位置的话题

    //Twist 变量
    float x;
    float y;
    float z;
    float th;
    float speed;
    float turn;
    bool done;

    double yaw;

    //电量
    int battery;
    int battery_limit;

    //循环频率
    int rate;

    //话题消息
    geometry_msgs::Twist msg;

    //无人机机头朝向
    int head_toward;

    //无人机实际位置
    position quadrotor_position;
    //无人机坐标位置
    position precise_position;
    //路径规划的目标点
    position target_position;

    position init_position;

    // 是否有需要追逐的目标
    bool is_catch;

    //无人机编号
    int id;

    //记录路径
    std::ofstream logger;

public:
    //无人机当前运行状态
    int state;
    geometry_msgs::PoseStamped pose;

    UAV(position init_po, int _id);
    ~UAV();

    void initForDrive();
    void setTarget(position t);

    position getUAVPosition();

    void positionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void setBattery(int b);
    int getBattery();

    void waitForSubscribles();
    void update(float _x, float _y, float _z,
                float _th, float _speed, float _turn);

    int caculateDirection(std::pair<int, int> next_position);

    // 操作无人机
    void rotateQuadrotor(double angular, ros::Rate &loop_rate);
    void moveQuadrotor(float distance, ros::Rate &loop_rate, int power);
    void hoverQuadrotor(ros::Rate &loop_rate);
    void fixQuadrotorRoute(ros::Rate &loop_rate);
    void fixQuadrotorAngle(ros::Rate &loop_rate);

    //无人际导航策略
    void drive(std::shared_ptr<Bug::BugPlanner> &planner);
    void back(std::shared_ptr<IARAStar::IARAStarPlanner> &planner);

    // 开启或关闭无人机马达
    void enableMotors();
    void disableMotors();

    // 停止无人机
    void stop();


    // 电源消耗
    bool powerConsumption(int percent);

    //记录无人机行驶轨迹
    bool setLoggerFile();

    //是否在飞行
    bool isFlying();
};

#endif