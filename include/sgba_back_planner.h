#ifndef _I_ARA_STAR_PLANNER_H_
#define _I_ARA_STAR_PLANNER_H_
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <set>
#include <string>
#include <unordered_set>
#include <vector>

#include "base_planner.h"

#define iara_m_p(a, b) std::make_pair(a, b)
#define INF 1e8 + 7
#define HAVE_PATH 0x123f
#define NO_PATH 0xf321
#define GET_GOAL 0xf322

namespace IARAStar
{
    struct SetElem
    {
        int id;
        double fvalue;
        SetElem(int id, double fv) : id(id), fvalue(fv) {}
        SetElem()
        {
            id = -1;
            fvalue = INF;
        }

        bool operator<(const SetElem &d) const
        {
            return (fvalue != d.fvalue) ? (fvalue < d.fvalue) : (id < d.id);
        }
    };

    class IARAStarPlanner 
    {
    public:
        IARAStarPlanner() {}
        ~IARAStarPlanner() {}
        //将迷宫中的坐标转换为顶点编号
        int GetIndex(int x, int y);

        //获取曼哈顿距离
        int Heruestic(int x, int y);

        //计算顶点优先级（迭代启发式）
        double GetFvalue(int i);

        // ARA*算法的迭代
        bool ImprovePath();

        //更新所选SetElem fvalue的函数
        void UpdateFvalues(std::set<SetElem> &s);

        //在更改迭代epsilon的同时启动ARA *迭代的函数
        bool ComputePath();

        //删除
        void DfsDelete(int current);

        void Step1();
        void Step2();
        void Step3();
        void Step4();

        // 获取路径
        void GetCurentPath();

        std::string getPlannerName();
        bool init(std::pair<int, int> nstart, std::pair<int, int> ngoal,
                  const char *map);
        std::pair<int, int> getNextPosition();
        void updateNextTarget(int x, int y);

        int getXFromCol(int col);
        int getYFromRow(int row);
        int getColFromCoordinate(int x);
        int getRowFromCoordinate(int y);

        int sum(std::vector<int> &v);

    private:
        const std::string planner_name = "IARAStar";
        //创建算法所需的集合
        std::set<SetElem> OPEN;
        std::set<SetElem> CLOSED;
        std::set<SetElem> INCONS;
        std::set<int> DELETED;

        int x_bias;
        int y_bias;

        //地图
        std::vector<std::vector<int>> graph;
        //四联通
        std::vector<std::pair<int, int>> neigh = {iara_m_p(0, 1), iara_m_p(0, -1),
                                                  iara_m_p(1, 0), iara_m_p(-1, 0)};
        double eps_max = 1.8;
        double eps = eps_max;

        // I-ara* 步骤
        double step = 0.3;
        // 迷宫顶点编号，起始变量，上次访问
        int map_size, start, goal, last_start;
        //迷宫的大小。 输入后更改，m为列数，n为行数
        int m = 0;
        int n = 0;
        //文字图
        std::vector<std::string> lab;

        int path_len = INF;
        int old_goal = INF;

        //重定向的运行时间
        clock_t time_cur;
        //图顶点的v值和g值
        std::vector<int> v;
        std::vector<int> g;
        //还原路径的父向量
        std::vector<int> parents;
        //最后找到的路径
        std::vector<int> path;
        //迭代时间
        std::vector<int> times;
    };
} // namespace IARAStar
#endif