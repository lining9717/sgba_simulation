#ifndef _BUG_GRID_H_
#define _BUG_GRID_H_
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>

#define bug_m_p(a, b) std::make_pair(a, b)
#define BUG 5
#define MLINE 6

#define BUG_START 3
#define BUG_GOAL 4
#define BUG_PATH 2
#define BUG_WALL 1
#define BUG_PASSABLE 0

#define NORTH 1
#define EAST 2
#define SOUTH 3
#define WEST 4

namespace Bug
{
    class Grid
    {
    private:
        std::vector<int> gridArr;
        int width;
        int length;
        int x_bias;
        int y_bias;

    public:
        Grid();
        ~Grid();
        Grid(const char *name);
        int getWidth();
        int getLength();
        int index(int row, int col);
        void setValue(int row, int col, int value);
        int getValue(int row, int col);
        int getValueCoordinate(int x, int y);
        int getColFromCoordinate(int x);
        int getRowFromCoordinate(int y);
        int getXFromCol(int col);
        int getYFromRow(int row);
        void setValueCoordinate(int x, int y, int value);
        void printGrid();
    };
} // namespace Bug

#endif