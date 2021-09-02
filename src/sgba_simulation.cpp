#include "sgba_simulation.h"

//无人机起始点
std::vector<position> uav_positons = {{1, -1, 0}, {1, 0, 0}, {0, 0, 0}, {0, -1, 0}};

int full_battery = 100;

// 记录日志
std::string log_file_path;

//网格地图路径
std::string map_file;

// 无人机数量
int g_uav_num;

Simulation::Simulation()
{
    nh = ros::NodeHandle();
    std::ifstream myfile(map_file);
    std::string input;
    while (getline(myfile, input))
    {
        lab.push_back(input);
        width = std::max(width, (int)input.size());
    }
    height = lab.size();
    x_bias = width / 2;
    y_bias = height / 2;
    //无人机初始运动方向
    move_directions = {{x_bias, 0}, {0, y_bias}, {-x_bias, 0}, {0, -y_bias}};
    uav_num = g_uav_num;
    uavs = std::vector<std::shared_ptr<UAV>>(uav_num);
    for (int i = 0; i < uav_num; ++i)
        uavs[i] = std::make_shared<UAV>(uav_positons[i], i);
}
Simulation::~Simulation() {}

//从左上角原点转化为中心原点
int Simulation::getXFromCol(int col)
{
    return col - x_bias;
}

//从左上角原点转化为中心原点
int Simulation::getYFromRow(int row)
{
    return y_bias - row;
}

void Simulation::startUAVCallback(int id, position p)
{
    std::shared_ptr<UAV> curr_uav = uavs[id];
    if (!curr_uav->setLoggerFile())
        return;
    curr_uav->initForDrive();
    BugPlannerPtr p_bug = std::make_shared<Bug::BugPlanner>();
    p_bug->init({uav_positons[id].x, uav_positons[id].y}, move_directions[id], map_file.c_str());
    curr_uav->drive(p_bug);
    curr_uav->disableMotors();
}

/*
// void Simulation::trajectoryCallback()
// {
//     ros::Publisher bug_path_pub = nh.advertise<nav_msgs::Path>("bug_trajectory", 10, true);
//     ros::Publisher uav_marker_pub = nh.advertise<visualization_msgs::Marker>("UAV_Position", 10);
//     ros::Publisher track_path_pub = nh.advertise<nav_msgs::Path>("track_trajectory", 10, true);
//     ros::Publisher track_marker_pub = nh.advertise<visualization_msgs::Marker>("track_Position", 10);
//     nav_msgs::Path bug_path;
//     visualization_msgs::Marker uav_marker;
//     nav_msgs::Path track_path;
//     visualization_msgs::Marker track_marker;
//     bug_path.header.stamp = ros::Time::now();
//     bug_path.header.frame_id = "/simulation_frame";
//     track_path.header.stamp = ros::Time::now();
//     track_path.header.frame_id = "/simulation_frame";
//     uav_marker.header.frame_id = "/simulation_frame";
//     uav_marker.header.stamp = ros::Time::now();
//     uav_marker.ns = "UAV_Point";
//     uav_marker.action = visualization_msgs::Marker::ADD;
//     uav_marker.type = visualization_msgs::Marker::SPHERE;
//     uav_marker.id = 0;
//     uav_marker.pose.orientation.w = 1.0;
//     uav_marker.scale.x = 0.2;
//     uav_marker.scale.y = 0.2;
//     uav_marker.scale.z = 0.2;
//     uav_marker.color.r = 1.0f;
//     uav_marker.color.a = 1.0;
//     track_marker.header.frame_id = "/simulation_frame";
//     track_marker.header.stamp = ros::Time::now();
//     track_marker.ns = "Track_Point";
//     track_marker.action = visualization_msgs::Marker::ADD;
//     track_marker.type = visualization_msgs::Marker::SPHERE;
//     track_marker.id = 0;
//     track_marker.pose.orientation.w = 1.0;
//     track_marker.scale.x = 0.2;
//     track_marker.scale.y = 0.2;
//     track_marker.scale.z = 0.2;
//     track_marker.color.b = 1.0f;
//     track_marker.color.a = 1.0;
//     ros::Rate loop_rate(10);
//     int last_wall_follow_ind = 0;
//     int wall_follow_ind = 0;
//     int tracking_ind = 0;
//     while (ros::ok())
//     {
//         for (int i = 0; i < uav_num; ++i)
//         {
//             if (uavs[i]->state == WALL_FLLOWING)
//             {
//                 wall_follow_ind = i;
//                 if (last_wall_follow_ind != wall_follow_ind)
//                 {
//                     track_path.poses.clear();
//                     track_path_pub.publish(track_path);
//                     ros::spinOnce();
//                 }
//                 last_wall_follow_ind = wall_follow_ind;
//             }
//             if (uavs[i]->state == TRACKING)
//             {
//                 tracking_ind = i;
//             }
//         }
//         if (uavs[wall_follow_ind]->state == WALL_FLLOWING)
//         {
//             geometry_msgs::PoseStamped trajectory_path;
//             trajectory_path = uavs[wall_follow_ind]->pose;
//             bug_path.poses.push_back(trajectory_path);
//             uav_marker.pose.position = trajectory_path.pose.position;
//             uav_marker_pub.publish(uav_marker);
//             bug_path_pub.publish(bug_path);
//             ros::spinOnce();
//         }
//         if (uavs[tracking_ind]->state == TRACKING)
//         {
//             geometry_msgs::PoseStamped trajectory_path;
//             trajectory_path = uavs[tracking_ind]->pose;
//             track_path.poses.push_back(trajectory_path);
//             track_marker.pose.position = trajectory_path.pose.position;
//             track_marker_pub.publish(track_marker);
//             track_path_pub.publish(track_path);
//             ros::spinOnce();
//         }
//         loop_rate.sleep();
//     }
// }

void Simulation::obstacleMapCallback()
{
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("obstacle_map", 10);
    visualization_msgs::MarkerArray obstacles;

    ros::Rate loop_rate(10);

    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            if (lab[y][x] == '#')
            {
                visualization_msgs::Marker marker;
                marker.header.frame_id = "/simulation_frame";
                marker.header.stamp = ros::Time::now();
                marker.ns = "obstacle_map";
                marker.id = y * width + x;
                marker.type = visualization_msgs::Marker::CUBE;
                marker.action = visualization_msgs::Marker::ADD;

                marker.pose.position.x = getXFromCol(x);
                marker.pose.position.y = getYFromRow(y);
                marker.pose.position.z = 1;

                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;

                marker.scale.x = 1.0;
                marker.scale.y = 1.0;
                marker.scale.z = 2.0;

                marker.color.r = 0.5f;
                marker.color.g = 0.5f;
                marker.color.b = 0.5f;
                marker.color.a = 1.0;

                marker.lifetime = ros::Duration();
                obstacles.markers.push_back(marker);
            }
        }
    }
    while (ros::ok())
    {
        marker_pub.publish(obstacles);
        loop_rate.sleep();
    }
}
*/

void Simulation::start()
{
    std::vector<std::thread> jobs(uav_num);
    for (int i = 0; i < uav_num; ++i)
    {
        jobs[i] = std::thread(&Simulation::startUAVCallback, this, i, uav_positons[i]);
    }
    for (auto &thread : jobs)
        thread.join();
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sgba_simulation");
    std::string path = ros::package::getPath("sgba_simulation");
    std::cout << path << std::endl;
    int temp_uav_num;
    std::string temp_log_file_path;
    std::string temp_map_file;

    if (ros::param::get("~log_file_path", temp_log_file_path))
        log_file_path = path + temp_log_file_path;
    else
        log_file_path = path + "/path_log/20_20_world/two_uavs";

    if (ros::param::get("~map_file", temp_map_file))
        map_file = path + temp_map_file;
    else
        map_file = path + "/maps/map_20_20.txt";

    if (ros::param::get("~uav_num", temp_uav_num))
        g_uav_num = temp_uav_num;
    else
        g_uav_num = 2;

    std::cout << "g_uav_num:" << g_uav_num << std::endl;
    std::cout << "log_file_path:" << log_file_path << std::endl;
    std::cout << "map_file:" << map_file << std::endl;

    std::shared_ptr<Simulation> p_simulation = std::make_shared<Simulation>();
    p_simulation->start();
    return 0;
}
