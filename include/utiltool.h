#pragma once

#include <cmath>

struct position
{
    float x;
    float y;
    float z;

    bool operator==(const position &p) const
    {
        if (x == p.x and y == p.y)
            return true;
        return false;
    }
    bool isClose(const position &p, float distance)
    {
        if (fabs(x - p.x) <= distance and fabs(y - p.y) <= distance)
            return true;
        return false;
    }
};