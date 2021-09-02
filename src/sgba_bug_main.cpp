#include "sgba_bug_main.h"

Bug::Bug::Bug() {}
Bug::Bug::~Bug() {}
Bug::Bug::Bug(int start_x, int start_y, int goal_x, int goal_y, Grid &grid)
{

    int start_row = grid.getRowFromCoordinate(start_y);
    int start_col = grid.getColFromCoordinate(start_x);
    int goal_row = grid.getRowFromCoordinate(goal_y);
    int goal_col = grid.getColFromCoordinate(goal_x);
    this->rowPos = start_row;
    this->colPos = start_col;
    this->goalRow = goal_row;
    this->goalCol = goal_col;
    this->grid = grid;
    
    grid.setValue(rowPos, colPos, BUG);
    grid.setValue(goalRow, goalCol, BUG_GOAL);

    printGrid();
}

void Bug::Bug::setGoal(int goal_x, int goal_y)
{
    grid.setValue(goalRow, goalCol, BUG_PASSABLE);
    int goal_row = grid.getRowFromCoordinate(goal_y);
    int goal_col = grid.getColFromCoordinate(goal_x);

    this->goalRow = goal_row;
    this->goalCol = goal_col;
    grid.setValue(goalRow, goalCol, BUG_GOAL);
}

void Bug::Bug::printPath()
{
    for (std::pair<int, int> p : path)
    {
        std::cout << "(" << p.first << "," << p.second << ")"
                  << "--->";
    }
}

std::pair<int, int> Bug::Bug::move(int direction)
{
    grid.setValue(rowPos, colPos, pathVal);
    switch (direction)
    {
    case NORTH:
        if (rowPos == grid.getLength())
        {
            break;
        }
        rowPos++;
        break;
    case EAST:
        if (colPos == grid.getWidth())
        {
            break;
        }
        colPos++;
        break;
    case SOUTH:
        if (rowPos == 0)
        {
            break;
        }
        rowPos--;
        break;
    case WEST:
        if (colPos == 0)
        {
            break;
        }
        colPos--;
        break;
    default:
        break;
    }
    grid.setValue(rowPos, colPos, BUG);
    path.push_back(bug_m_p(rowPos, colPos));

    stepsMoved++;
    printGrid();
    int coordinat_x = grid.getXFromCol(colPos);
    int coordinat_y = grid.getYFromRow(rowPos);

    // return bug_m_p(rowPos, colPos);
    return bug_m_p(coordinat_x, coordinat_y);
}
int Bug::Bug::sense(int direction)
{
    switch (direction)
    {
    case NORTH:
        if (rowPos == grid.getLength())
        {
            break;
        }
        return grid.getValue(rowPos + 1, colPos);
    case EAST:
        if (colPos == grid.getWidth())
        {
            break;
        }
        return grid.getValue(rowPos, colPos + 1);
    case SOUTH:
        if (rowPos == 0)
        {
            break;
        }
        return grid.getValue(rowPos - 1, colPos);
    case WEST:
        if (colPos == 0)
        {
            break;
        }
        return grid.getValue(rowPos, colPos - 1);
    default:
        return -1;
    }
    return -1;
}
int Bug::Bug::calculateDirection()
{
    int xDist = goalCol - colPos;
    int yDist = goalRow - rowPos;

    int xChanges[5] = {0, 0, -1, 0, 1};
    int yChanges[5] = {0, -1, 0, 1, 0};
    double shortestDist = sqrt(xDist * xDist + yDist * yDist);
    int direction = 0;
    for (int i = 0; i < 5; i++)
    {
        if (sqrt((xDist + xChanges[i]) * (xDist + xChanges[i]) + (yDist + yChanges[i]) * (yDist + yChanges[i])) < shortestDist)
        {
            shortestDist = sqrt((xDist + xChanges[i]) * (xDist + xChanges[i]) + (yDist + yChanges[i]) * (yDist + yChanges[i]));
            direction = i;
        }
    }

    return direction;
}

void Bug::Bug::setDisplayGrid(bool displayGrid)
{
    this->displayGrid = displayGrid;
}
int Bug::Bug::getStepsMoved()
{
    return stepsMoved;
}
int Bug::Bug::getRowPos()
{
    return rowPos;
}
int Bug::Bug::getColPos()
{
    return colPos;
}
int Bug::Bug::getPathVal()
{
    if (displayGrid)
    {
        grid.printGrid();
    }
}
void Bug::Bug::setPathVal(int pathVal)
{
    this->pathVal = pathVal;
}

void Bug::Bug::printGrid()
{
    if (displayGrid)
        grid.printGrid();
}
