#include "sgba_back_planner.h"

bool IARAStar::IARAStarPlanner::init(std::pair<int, int> nstart, std::pair<int, int> ngoal, const char *map)
{
    std::ifstream myfile(map);
    std::string input;

    while (getline(myfile, input))
    {
        lab.push_back(input);
        m = std::max(m, (int)input.size());
    }
    int i_new, j_new;
    n = lab.size();
    x_bias = m / 2;
    y_bias = n / 2;
    std::cout << "iarastar Map: Rows=" << n << ",Cols=" << m << std::endl;
    graph.resize(n * m);
    map_size = n * m;
    start = m * getRowFromCoordinate(nstart.second) + getColFromCoordinate(nstart.first);
    goal = m * getRowFromCoordinate(ngoal.second) + getColFromCoordinate(ngoal.first);
    std::cout << "iarastar start: (" << start % m << ", " << start / m << ")" << std::endl;
    std::cout << "iarastar goal: (" << goal % m << ", " << goal / m << ")" << std::endl;

    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j < m; ++j)
        {
            if (lab[i][j] != '.')
                continue;
            for (auto it : neigh)
            {
                i_new = i + it.first;
                j_new = j + it.second;
                if (i_new > -1 and i_new < n and j_new > -1 and j_new < m)
                {
                    if (lab[i_new][j_new] == '.')
                    {
                        //每个节点存放与自身相邻的无障碍节点
                        graph[GetIndex(i, j)].push_back(GetIndex(i_new, j_new));
                    }
                }
            }
        }
    }
    last_start = -1;
    // 用无穷大填充v值g值
    v.resize(map_size, INF);
    g.resize(map_size, INF);
    parents.resize(map_size, -1);
    // 运行算法
    g[start] = 0;
    OPEN.insert(SetElem(start, GetFvalue(start)));
    return true;
}

int IARAStar::IARAStarPlanner::GetIndex(int x, int y)
{
    return (x * m + y);
}

int IARAStar::IARAStarPlanner::Heruestic(int x, int y)
{
    int x_x = x / m;
    int x_y = x % m;
    int y_x = y / m;
    int y_y = y % m;
    return (abs(x_x - y_x) + abs(x_y - y_y));
}

double IARAStar::IARAStarPlanner::GetFvalue(int i)
{
    return std::min((double)INF, g[i] + eps * Heruestic(i, goal));
}

bool IARAStar::IARAStarPlanner::ImprovePath()
{
    //OPEN size不为0 并且fvalue（goal）> OPEN中的最小fvalue
    while (OPEN.size() and GetFvalue(goal) > GetFvalue(OPEN.begin()->id))
    {
        SetElem s = *OPEN.begin(); //取出OPEN中最小的fvale节点（迭代器）存到CLOSE中
        CLOSED.insert(s);
        OPEN.erase(OPEN.begin()); // 删除OPEN中最小值

        v[s.id] = g[s.id];
        for (auto new_s : graph[s.id]) //访问当前节点s的邻居
        {
            //s和new_s相邻，所以距离为1
            if (g[new_s] > v[s.id] + 1)
            {
                parents[new_s] = s.id; //new_s的父节点为s
                auto next = SetElem(new_s, GetFvalue(new_s));
                if (CLOSED.find(next) == CLOSED.end()) //若new_s不在CLOSE中
                {
                    OPEN.erase(next);       //删除OPEN中的new_s,确保不在OPEN中，若存在则删除
                    g[new_s] = v[s.id] + 1; //更新g[new_s]
                    if (INCONS.find(next) == INCONS.end())
                    {
                        //第一次发现new_s，INCONS和CLSOE中都没有new_s才加入到OPEN中
                        OPEN.insert(SetElem(new_s, GetFvalue(new_s)));
                    }
                    else
                    {
                        INCONS.erase(next);
                        INCONS.insert(SetElem(new_s, GetFvalue(new_s)));
                    }
                }
                else
                {
                    CLOSED.erase(SetElem(new_s, GetFvalue(new_s)));
                    g[new_s] = v[s.id] + 1;
                    INCONS.insert(SetElem(new_s, GetFvalue(new_s)));
                }
            }
        }
    }

    if (g[goal] == INF)
    {
        return false;
    }
    return true;
}

void IARAStar::IARAStarPlanner::UpdateFvalues(std::set<SetElem> &s)
{

    std::set<SetElem> save;
    for (auto i : s)
    {
        save.insert(SetElem(i.id, GetFvalue(i.id)));
    }
    s = save;
}

bool IARAStar::IARAStarPlanner::ComputePath()
{
    while (true)
    {
        auto result = ImprovePath();
        if (result == false)
        {
            return false;
        }
        if (eps == 1)
        {
            return true;
        }
        //将INCONS中所有元素加入到OPEN中
        OPEN.insert(INCONS.begin(), INCONS.end());
        INCONS.clear();
        CLOSED.clear();
        eps = fmax(1, eps - step);
        UpdateFvalues(OPEN);
    }
}

void IARAStar::IARAStarPlanner::Step1()
{
    if (g[start] != v[start])
    {
        OPEN.erase(SetElem(start, GetFvalue(start)));
        INCONS.erase(SetElem(start, GetFvalue(start)));
        g[start] = v[start];
    }
}

void IARAStar::IARAStarPlanner::DfsDelete(int current)
{
    INCONS.erase(SetElem(current, GetFvalue(current)));
    OPEN.erase(SetElem(current, GetFvalue(current)));
    v[current] = INF;
    g[current] = INF;
    parents[current] = -1;
    DELETED.insert(current);
    for (auto i : graph[current])
    {
        if (parents[i] == current)
        {
            DfsDelete(i);
        }
    }
}

void IARAStar::IARAStarPlanner::Step2()
{
    if (start != last_start)
    {
        parents[start] = -1;
        DfsDelete(last_start);
    }
}

void IARAStar::IARAStarPlanner::Step3()
{
    for (auto i : DELETED)
    {
        for (auto j : graph[i])
        {
            if (g[i] > v[j] + 1)
            {
                g[i] = v[j] + 1;
                parents[i] = j;
            }
        }
        if (g[i] != INF)
        {
            OPEN.insert(SetElem(i, GetFvalue(i)));
        }
    }
    UpdateFvalues(INCONS);
    OPEN.insert(INCONS.begin(), INCONS.end());
    CLOSED.clear();
    INCONS.clear();
    DELETED.clear();
}

void IARAStar::IARAStarPlanner::Step4()
{
    if (GetFvalue(goal) > OPEN.begin()->fvalue)
    {
        eps = eps_max;
    }
    else
    {
        eps = fmax(eps - step, 1);
    }
    UpdateFvalues(OPEN);
}

void IARAStar::IARAStarPlanner::GetCurentPath()
{
    path.clear();
    int current = goal;
    // paths << current << ' ';
    path.push_back(goal);
    while (current != start)
    {
        current = parents[current];
        // paths << current << ' ';
        path.push_back(current);
    }
    path_len = path.size();
}

int IARAStar::IARAStarPlanner::sum(std::vector<int> &v)
{
    int summ = 0;
    for (auto i : v)
    {
        summ += i;
    }
    return summ;
}

std::pair<int, int> IARAStar::IARAStarPlanner::getNextPosition()
{
    if (start == goal)
        return iara_m_p(GET_GOAL, GET_GOAL);
    if (old_goal == goal and !path.empty())
    {
        start = *(path.rbegin() + 1);
        path.pop_back();
        return iara_m_p(getXFromCol(start % m), getYFromRow(start / m));
    }
    if (ComputePath() == false)
    {
        std::cout << "Get target!" << std::endl;
        return iara_m_p(GET_GOAL, GET_GOAL);
    }
    //获取找到的路径
    GetCurentPath();
    last_start = start;
    start = *(path.rbegin() + 1);
    old_goal = goal;
    path.pop_back();
    if (start == goal)
    {
        std::cout << "Get target!" << std::endl;
        return iara_m_p(GET_GOAL, GET_GOAL);
    }
    UpdateFvalues(OPEN);
    Step1();
    Step2();
    Step3();
    Step4();
    return iara_m_p(getXFromCol(start % m), getYFromRow(start / m));
}

void IARAStar::IARAStarPlanner::updateNextTarget(int x, int y)
{
    goal = getRowFromCoordinate(y) * m + getColFromCoordinate(x);
}

std::string IARAStar::IARAStarPlanner::getPlannerName()
{
    return planner_name;
}

int IARAStar::IARAStarPlanner::getXFromCol(int col)
{
    return col - x_bias;
}
int IARAStar::IARAStarPlanner::getYFromRow(int row)
{
    return y_bias - row;
}

int IARAStar::IARAStarPlanner::getColFromCoordinate(int x)
{
    if (x + x_bias < 0 or x + x_bias >= m)
        printf("[ERROR] X out of bound\n");
    return x + x_bias;
}
int IARAStar::IARAStarPlanner::getRowFromCoordinate(int y)
{
    if (y_bias - y < 0 or y_bias - y >= n)
        printf("[ERROR] Y out of bound\n");
    return y_bias - y;
}