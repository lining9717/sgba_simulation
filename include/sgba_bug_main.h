#ifndef _BUG_MAIN_H_
#define _BUG_MAIN_H_
#include "sgba_bug_grid.h"

namespace Bug
{
    class Bug
    {
    private:
        Grid grid;
        int stepsMoved = 0;
        int rowPos = 0, colPos = 0;
        int goalRow, goalCol;
        std::vector<std::pair<int, int>> path;
        bool displayGrid = false;
        int pathVal = BUG_PATH;

    public:
        void printGrid();
        Bug();
        ~Bug();
        Bug(int start_x, int start_y, int goal_x, int goal_y, Grid &grid);
        void setGoal(int goal_x, int goal_y);
        void printPath();
        // std::vector<std::pair<int, int>> getPath();
        std::pair<int, int> move(int direction);
        int sense(int direction);
        int calculateDirection();
        void setDisplayGrid(bool displayGrid);
        int getStepsMoved();
        int getRowPos();
        int getColPos();
        int getPathVal();
        void setPathVal(int pathVal);
    };
} // namespace Bug

#endif