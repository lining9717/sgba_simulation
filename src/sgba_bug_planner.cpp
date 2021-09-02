#include "sgba_bug_planner.h"

static int getOppositeCoordinate(int coordinate)
{
    switch (coordinate)
    {
    case NORTH:
        return SOUTH;
    case SOUTH:
        return NORTH;
    case EAST:
        return WEST;
    case WEST:
        return EAST;
    default:
        return -1;
    }
}
static int getNextCoordinate(int coordinate)
{
    switch (coordinate)
    {
    case NORTH:
        return EAST;
    case EAST:
        return SOUTH;
    case SOUTH:
        return WEST;
    case WEST:
        return NORTH;
    default:
        return -1;
    }
}
static int getPrevCoordinate(int coordinate)
{
    switch (coordinate)
    {
    case NORTH:
        return WEST;
    case EAST:
        return NORTH;
    case SOUTH:
        return EAST;
    case WEST:
        return SOUTH;
    default:
        return -1;
    }
}

Bug::BugPlanner::BugPlanner() {}

Bug::BugPlanner::~BugPlanner() {}

std::string Bug::BugPlanner::getPlannerName()
{
    return planner_name;
}

void Bug::BugPlanner::updateNextTarget(int x, int y)
{
}

bool Bug::BugPlanner::init(std::pair<int, int> nstart, std::pair<int, int> ngoal, const char *map)
{
    bugGrid = Grid(map);
    bug = Bug(nstart.first, nstart.second, 0, 0, bugGrid);
    bug.setDisplayGrid(false);
    // direction = bug.calculateDirection();
    // onWall = bug.sense(direction) == 1;
    setGoal(ngoal.first, ngoal.second);
    return true;
}

void Bug::BugPlanner::setGoal(int x, int y)
{
    bug.setGoal(x, y);
    direction = bug.calculateDirection();
    onWall = bug.sense(direction) == 1;
}

std::pair<int, int> Bug::BugPlanner::getNextPosition()
{
    std::pair<int, int> result = bug_m_p(-1, -1);
    while (bug.calculateDirection() != 0)
    {
        if (onWall)
        {
            if (is_at_start)
            {
                nextDirection = getNextCoordinate(direction);
                prevDirection = getPrevCoordinate(direction);
                oppDirection = getOppositeCoordinate(direction);
                is_at_start = false;
            }
            if (bug.sense(direction) == BUG_WALL &&
                bug.sense(nextDirection) != BUG_WALL)
            {
                result = bug.move(nextDirection);
                return result;
            }
            else if (bug.sense(direction) != BUG_WALL)
            {
                result = bug.move(direction);
                nextDirection = direction;
                oppDirection = getNextCoordinate(direction);
                prevDirection = getOppositeCoordinate(direction);
                direction = getPrevCoordinate(direction);
                return result;
            }
            else if (bug.sense(direction) == BUG_WALL &&
                     bug.sense(nextDirection) == BUG_WALL &&
                     bug.sense(oppDirection) != BUG_WALL)
            {
                result = bug.move(oppDirection);
                nextDirection = getOppositeCoordinate(direction);
                oppDirection = getPrevCoordinate(direction);
                prevDirection = direction;
                direction = getNextCoordinate(direction);
                return result;
            }
            else if (bug.sense(direction) == BUG_WALL &&
                     bug.sense(nextDirection) == BUG_WALL &&
                     bug.sense(oppDirection) == BUG_WALL)
            {
                result = bug.move(prevDirection);
                nextDirection = getPrevCoordinate(direction);
                prevDirection = getNextCoordinate(direction);
                oppDirection = direction;
                direction = getOppositeCoordinate(direction);
                return result;
            }
        }
        else
        {
            direction = bug.calculateDirection();
            if (bug.sense(direction) == 1)
            {
                onWall = true;
                continue;
            }
            result = bug.move(direction);
            return result;
        }
    }
    return result;
}