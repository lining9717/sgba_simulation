#pragma once
#include <iostream>
#include <string>

class BasePlanner
{
public:
    virtual bool init(std::pair<int, int> nstart, std::pair<int, int> ngoal, const char *map) = 0;
    virtual std::pair<int, int> getNextPosition() = 0;
    virtual std::string getPlannerName() = 0;
    virtual void updateNextTarget(int x, int y) = 0;
    BasePlanner() {}
    virtual ~BasePlanner() {}
};