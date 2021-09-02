#include "sgba_uav.h"

extern std::string log_file_path;
extern std::string map_file;
extern std::vector<position> uav_positons;
extern int full_battery;

static int getNextDirection(int d)
{
    switch (d)
    {
    case FRONT:
        return LEFT;
    case LEFT:
        return BACK;
    case BACK:
        return RIGHT;
    case RIGHT:
        return FRONT;
    default:
        return 0;
    }
}

static int getPreDirection(int d)
{
    switch (d)
    {
    case FRONT:
        return RIGHT;
    case LEFT:
        return FRONT;
    case BACK:
        return LEFT;
    case RIGHT:
        return BACK;
    default:
        return 0;
    }
}

static int getOppositeDirection(int d)
{
    switch (d)
    {
    case FRONT:
        return BACK;
    case LEFT:
        return RIGHT;
    case BACK:
        return FRONT;
    case RIGHT:
        return LEFT;
    default:
        return 0;
    }
}

/**
 * @brief Construct a new UAV::UAV object
 * 
 * @param init_po 无人机初始位置
 * @param _id     无人机的编号
 */
UAV::UAV(position init_po, int _id)
{
    nh = ros::NodeHandle();
    nh_private = ros::NodeHandle("~");
    id = _id;

    std::stringstream topic_name1;
    topic_name1 << "/uav" << id << controller_topic_name;
    publisher = nh.advertise<geometry_msgs::Twist>(topic_name1.str(), 1);

    std::stringstream service_name;
    service_name << "/uav" << id << motor_service_name;
    client = nh.serviceClient<sgba_simulation::EnableMotors>(service_name.str());

    std::stringstream topic_name2;
    topic_name2 << "/uav" << id << position_topic_name;
    sub = nh.subscribe(topic_name2.str(), 10, &UAV::positionCallback, this);

    x = 0.0;
    y = 0.0;
    z = 0.0;
    th = 0.0;
    speed = 1.0;
    turn = 2.0;
    done = false;
    quadrotor_position = init_po;
    precise_position = init_po;
    init_position = init_po;

    head_toward = RIGHT;
    rate = 1;
    battery = full_battery;
    battery_limit = full_battery * 0.6;
    is_catch = false;

    state = IDLE;
}

UAV::~UAV()
{
}

void UAV::setBattery(int b)
{
    battery = b;
}

int UAV::getBattery()
{
    return battery;
}

/**
 * @brief 为飞行做准备
 * 
 */
void UAV::initForDrive()
{
    try
    {
        waitForSubscribles();
        enableMotors();
        ros::Rate loop_rate(rate);
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = 0;
        precise_position.z = 1.0;

        //起飞, 距离地面1m
        msg.linear.x = 0;
        msg.linear.y = 0;
        msg.linear.z = speed;
        moveQuadrotor(2.0, loop_rate, 0);
        hoverQuadrotor(loop_rate);

        fixQuadrotorAngle(loop_rate);
        hoverQuadrotor(loop_rate);
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
}

bool UAV::setLoggerFile()
{
    std::stringstream ss;
    ss << log_file_path << "/uav" << id << ".txt";
    std::string logger_file;
    ss >> logger_file;
    logger.open(logger_file, std::ios::out);
    if (!logger.is_open())
    {
        std::cout << "Open " << logger_file << " file failure." << std::endl;
        return false;
    }
    return true;
}

void UAV::setTarget(position t)
{
    target_position = t;
    is_catch = true;
}

/**
 * @brief 获取无人机实时位置的回调函数
 * 
 * @param msg 
 */
void UAV::positionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    quadrotor_position.x = msg->pose.position.x;
    quadrotor_position.y = msg->pose.position.y;
    quadrotor_position.z = msg->pose.position.z;
    yaw = tf::getYaw(msg->pose.orientation) * 180 / M_PI;
    pose = *msg;
}

/**
 * @brief 用于/cmd_vel话题的等待订阅 
 * 
 */
void UAV::waitForSubscribles()
{
    int count = 0;
    while (ros::ok() and publisher.getNumSubscribers() == 0)
    {
        if (count == 4)
            std::cout << "[INFO] uav" << id << " Waiting for subscriber to connect to " << controller_topic_name << std::endl;
        sleep(1);
        ++count;
        count %= 5;
    }
    if (!ros::ok())
        throw HectotException("Got shutdown request before subscribers connected");
}

/**
 * @brief 更新无人机状态信息
 * 
 * @param _x 
 * @param _y 
 * @param _z 
 * @param _th 
 * @param _speed 
 * @param _turn 
 */
void UAV::update(float _x, float _y, float _z,
                 float _th, float _speed, float _turn)
{
    x = _x;
    y = _y;
    z = _z;
    th = _th;
    speed = _speed;
    turn = _turn;
}

/**
 * @brief 计算无人机前进方向
 * 
 * @param next_position  下一个移动位置的坐标
 * @return int  方向
 */
int UAV::caculateDirection(std::pair<int, int> next_position)
{
    if (next_position.first == (int)precise_position.x)
    {
        if (next_position.second > precise_position.y)
            return FRONT;
        if (next_position.second < precise_position.y)
            return BACK;
    }

    if (next_position.second == (int)precise_position.y)
    {
        if (next_position.first > precise_position.x)
            return RIGHT;
        if (next_position.first < precise_position.x)
            return LEFT;
    }
    return 0;
}

/**
 * @brief 旋转无人机
 * 
 * @param angular  角度
 * @param loop_rate  频率
 */
void UAV::rotateQuadrotor(double angular, ros::Rate &loop_rate)
{
    msg.linear.x = 0;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = angular > 0 ? turn : -turn;
    int ticks = int(abs(angular) / turn * rate);
    for (int i = 0; i < ticks; ++i)
    {
        powerConsumption(1);
        publisher.publish(msg);
        loop_rate.sleep();
    }
}

/**
 * @brief 移动无人机
 * 
 * @param distance  距离
 * @param loop_rate  频率
 */
void UAV::moveQuadrotor(float distance, ros::Rate &loop_rate, int power)
{
    int ticks = int(distance / speed * rate);
    for (int i = 0; i < ticks; ++i)
    {
        powerConsumption(power);
        ros::spinOnce();
        publisher.publish(msg);
        loop_rate.sleep();
    }
}

/**
 * @brief 悬浮无人机
 * 
 * @param loop_rate 
 */
void UAV::hoverQuadrotor(ros::Rate &loop_rate)
{
    if (quadrotor_position.z < precise_position.z)
    {
        msg.linear.x = 0;
        msg.linear.y = 0;
        msg.linear.z = speed;
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = 0;
        int ticks = int((precise_position.z - quadrotor_position.z) / speed * rate);
        ticks = ticks == 0 ? 1 : ticks;
        for (int i = 0; i < ticks; ++i)
        {
            // powerConsumption(1);
            ros::spinOnce();
            publisher.publish(msg);
            loop_rate.sleep();
        }
    }
    else
    {
        msg.linear.x = 0;
        msg.linear.y = 0;
        msg.linear.z = 0;
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = 0;
        powerConsumption(1);
        publisher.publish(msg);
        loop_rate.sleep();
    }
}

/**
 * @brief 开启无人机的马达
 * 
 */
void UAV::enableMotors()
{
    sgba_simulation::EnableMotors srv;
    srv.request.enable = true;
    if (client.call(srv))
    {
        if (srv.response.success)
            ROS_INFO("Success to enable motors");
        else
            ROS_INFO("Fail to enable motors");
    }
    else
    {
        std::stringstream ss;
        ss << "Failed to call service /uav" << id << "/enable_motors";
        throw HectotException(ss.str().c_str());
    }
}

bool UAV::isFlying()
{
    if (!done)
        return true;
    return false;
}

/**
 * @brief 关闭无人机的马达
 * 
 */
void UAV::disableMotors()
{
    sgba_simulation::EnableMotors srv;
    srv.request.enable = false;
    if (client.call(srv))
    {
        if (srv.response.success)
            ROS_INFO("Success to disable UAV%d motors", id);
        else
            ROS_INFO("Fail to disable UAV%d motors", id);
    }
    else
    {
        std::stringstream ss;
        ss << "Failed to call service /uav" << id << "/enable_motors";
        throw HectotException(ss.str().c_str());
    }
}

// 停止无人机
void UAV::stop()
{
    done = true;
}

// 电源消耗
bool UAV::powerConsumption(int percent)
{
    battery -= percent;

    if (battery > 0 and battery <= battery_limit)
    {
        printf("[UAV%d] BATTERY STATE: Low %d%%\n", id, battery);
        return true;
    }
    else if (battery > battery_limit)
    {
        printf("[UAV%d] BATTERY STATE: %d%%\n", id, battery);
        return true;
    }
    else
    {
        printf("[UAV%d] BATTERY STATE: Empty 0%%\n", id);
        return false;
    }
}

position UAV::getUAVPosition()
{
    return precise_position;
}

void UAV::fixQuadrotorAngle(ros::Rate &loop_rate)
{
    if (fabs(yaw) > 3)
    {
        msg.linear.y = 0;
        msg.linear.z = 0;
        msg.linear.x = 0;
        ros::Rate loopRate(10);
        printf("----------[UAV%d] fix Quadrotor yaw------------\n", id);

        int ticks = int(3 / 0.1 * 10);
        for (int i = 0; i < ticks and fabs(yaw) > 0.5; ++i)
        {
            ros::spinOnce();
            printf("\033[47;31m [UAV%d] yaw: %f\033[0m\n", id, yaw);
            msg.angular.z = yaw > 0 ? -0.1 : 0.1;
            publisher.publish(msg);
            loopRate.sleep();
        }
    }
}
void UAV::fixQuadrotorRoute(ros::Rate &loop_rate)
{

    if (quadrotor_position.isClose(precise_position, 0.25))
        return;
    while (ros::ok() and fabs(quadrotor_position.x - precise_position.x) > 0.1)
    {
        ros::spinOnce();
        msg.linear.x = quadrotor_position.x - precise_position.x > 0 ? -0.1 : 0.1;
        msg.linear.z = 0;
        msg.linear.y = 0;
        publisher.publish(msg);
        loop_rate.sleep();
    }

    while (ros::ok() and fabs(quadrotor_position.y - precise_position.y) > 0.1)
    {
        ros::spinOnce();
        msg.linear.y = quadrotor_position.y - precise_position.y > 0 ? -0.1 : 0.1;
        msg.linear.z = 0;
        msg.linear.x = 0;
        publisher.publish(msg);
        loop_rate.sleep();
    }
}

/**
 * @brief 用于绕墙导航
 * 
 * @param planner 规划器
 */
void UAV::drive(std::shared_ptr<Bug::BugPlanner> &planner)
{
    state = WALL_FLLOWING;
    bool flag = true;
    ros::Rate loop_rate(rate);
    try
    {
        while (!done and ros::ok())
        {
            ros::spinOnce(); //获取无人机位置
            std::pair<int, int> next_x_y = planner->getNextPosition();
            if (next_x_y.first == GET_GOAL)
            {
                printf("Get the goal!\n");
                return;
            }
            if (next_x_y.first == NO_PATH)
                throw HectotException("No path");

            printf("[UAV%d] next position(%d, %d)\n", id, next_x_y.first, next_x_y.second);

            int next_direction = caculateDirection(next_x_y);
            if (next_direction == 0)
                throw HectotException("Next direction caculation error.");

            fixQuadrotorRoute(loop_rate);
            //当前方向于需要前进的方向不同时,进行方向调整
            if (next_direction != head_toward)
            {
                //左转90度,y轴正方向
                if (getNextDirection(head_toward) == next_direction)
                {
                    msg.linear.x = 0;
                    msg.linear.y = speed;
                    msg.linear.z = 0;
                    moveQuadrotor(1.0, loop_rate, 1);
                }
                //右转90度，y轴负方向
                else if (getPreDirection(head_toward) == next_direction)
                {
                    msg.linear.x = 0;
                    msg.linear.y = -speed;
                    msg.linear.z = 0;
                    moveQuadrotor(1.0, loop_rate, 1);
                }
                //向后转180度，x轴负方向
                else if (getOppositeDirection(head_toward) == next_direction)
                {
                    msg.linear.x = -speed;
                    msg.linear.y = 0;
                    msg.linear.z = 0;
                    moveQuadrotor(1.0, loop_rate, 1);
                }
            }
            else //x轴正方向
            {
                msg.linear.x = speed;
                msg.linear.y = 0;
                msg.linear.z = 0;
                moveQuadrotor(1.0, loop_rate, 1);
            }

            hoverQuadrotor(loop_rate);
            precise_position.x = next_x_y.first;
            precise_position.y = next_x_y.second;
            logger << precise_position.x << " " << precise_position.y << std::endl;
            if (battery <= 0)
            {
                done = !done;
                printf("[WARNNING] Stop by battery empty\n");
            }
            if (flag and battery < battery_limit)
            {
                // 注释部分直接使用最短路径返航
                // std::shared_ptr<IARAStar::IARAStarPlanner> back_planner = std::make_shared<IARAStar::IARAStarPlanner>();
                // back_planner->init({precise_position.x, precise_position.y},
                //                    {uav_positons[id].x, uav_positons[id].y},
                //                    map_file.c_str());
                // printf("Change goal\n");
                // back(back_planner);
                // break;

                //以下为使用bug算法返航
                planner = std::make_shared<Bug::BugPlanner>();
                planner->init({precise_position.x, precise_position.y},
                              {uav_positons[id].x, uav_positons[id].y},
                              map_file.c_str());
                printf("Change goal\n");
                flag = false;
            }
            if (init_position == precise_position)
            {
                done = !done;
                printf("Finish!\n");
            }
        }
        msg.linear.x = 0;
        msg.linear.y = 0;
        msg.linear.z = 0;
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = 0;
        publisher.publish(msg);
    }
    catch (const std::exception &e)
    {
        logger.close();
        std::cerr << e.what() << '\n';
    }
    state = IDLE;
}

void UAV::back(std::shared_ptr<IARAStar::IARAStarPlanner> &planner)
{
    ros::Rate loop_rate(rate);
    try
    {
        while (!done and ros::ok())
        {
            ros::spinOnce(); //获取无人机位置
            std::pair<int, int> next_x_y = planner->getNextPosition();
            if (next_x_y.first == GET_GOAL)
            {
                printf("Get the goal!\n");
                return;
            }
            if (next_x_y.first == NO_PATH)
                throw HectotException("No path");

            printf("[UAV%d] next position(%d, %d)\n", id, next_x_y.first, next_x_y.second);

            int next_direction = caculateDirection(next_x_y);
            if (next_direction == 0)
                throw HectotException("Next direction caculation error.");

            fixQuadrotorRoute(loop_rate);
            //当前方向于需要前进的方向不同时,进行方向调整
            if (next_direction != head_toward)
            {
                //左转90度,y轴正方向
                if (getNextDirection(head_toward) == next_direction)
                {
                    msg.linear.x = 0;
                    msg.linear.y = speed;
                    msg.linear.z = 0;
                    moveQuadrotor(1.0, loop_rate, 1);
                }
                //右转90度，y轴负方向
                else if (getPreDirection(head_toward) == next_direction)
                {
                    msg.linear.x = 0;
                    msg.linear.y = -speed;
                    msg.linear.z = 0;
                    moveQuadrotor(1.0, loop_rate, 1);
                }
                //向后转180度，x轴负方向
                else if (getOppositeDirection(head_toward) == next_direction)
                {
                    msg.linear.x = -speed;
                    msg.linear.y = 0;
                    msg.linear.z = 0;
                    moveQuadrotor(1.0, loop_rate, 1);
                }
            }
            else //x轴正方向
            {
                msg.linear.x = speed;
                msg.linear.y = 0;
                msg.linear.z = 0;
                moveQuadrotor(1.0, loop_rate, 1);
            }

            hoverQuadrotor(loop_rate);
            precise_position.x = next_x_y.first;
            precise_position.y = next_x_y.second;
            logger << precise_position.x << " " << precise_position.y << std::endl;
            if (battery <= 0)
            {
                done = !done;
                printf("[WARNNING] Stop by battery empty\n");
            }
            if (init_position == precise_position)
            {
                done = !done;
                printf("Finish!\n");
            }
        }
        msg.linear.x = 0;
        msg.linear.y = 0;
        msg.linear.z = 0;
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = 0;
        publisher.publish(msg);
    }
    catch (const std::exception &e)
    {
        logger.close();
        std::cerr << e.what() << '\n';
    }
}