#ifndef _BUG_PLANNER_H_
#define _BUG_PLANNER_H_
#include "sgba_bug_main.h"
#include "base_planner.h"

namespace Bug
{
    class BugPlanner
    {
    private:
        Grid bugGrid;
        Bug bug;
        bool onWall;
        int direction;
        int startRow;
        int startCol;
        bool is_at_start = true;
        int nextDirection;
        int prevDirection;
        int oppDirection;
        const std::string planner_name = "Bug";

    public:
        BugPlanner();
        ~BugPlanner();
        void setGoal(int x, int y);
        bool init(std::pair<int, int> nstart, std::pair<int, int> ngoal, const char *map);
        std::string getPlannerName();
        std::pair<int, int> getNextPosition();
        void updateNextTarget(int x, int y);
    };
} // namespace Bug
#endif