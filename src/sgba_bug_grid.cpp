#include "sgba_bug_grid.h"

Bug::Grid::Grid() {}
Bug::Grid::~Grid() {}
Bug::Grid::Grid(const char *name)
{
    std::vector<std::string> lab;
    std::ifstream myfile(name);
    std::string input;
    width = 0;
    length = 0;

    while (getline(myfile, input))
    {
        lab.push_back(input);
        width = std::max(width, (int)input.size());
    }
    length = lab.size();

    x_bias = width / 2;
    y_bias = length / 2;
    // printf("set bias x:%d y:%d\n", x_bias, y_bias);
    
    gridArr.resize(width * length, 0);
    for (int i = 0; i < length; ++i)
    {
        for (int j = 0; j < width; ++j)
        {
            if (lab[i][j] == '#')
                setValue(i, j, BUG_WALL);
        }
    }
}

int Bug::Grid::getWidth()
{
    return this->width;
}

int Bug::Grid::getLength()
{
    return this->length;
}

int Bug::Grid::index(int row, int col)
{
    return col + row * width;
}

void Bug::Grid::setValue(int row, int col, int value)
{
    gridArr[index(row, col)] = value;
}

int Bug::Grid::getValue(int row, int col)
{
    return gridArr[index(row, col)];
}

void Bug::Grid::setValueCoordinate(int x, int y, int value)
{
    int row = getRowFromCoordinate(y);
    int col = getColFromCoordinate(x);
    gridArr[index(row, col)] = value;
}

int Bug::Grid::getValueCoordinate(int x, int y)
{
    int row = getRowFromCoordinate(y);
    int col = getColFromCoordinate(x);
    return gridArr[index(row, col)];
}

int Bug::Grid::getColFromCoordinate(int x)
{
    if (x + x_bias < 0 or x + x_bias > width)
        printf("[ERROR] X out of bound\n");
    return x + x_bias;
}

int Bug::Grid::getRowFromCoordinate(int y)
{
    if (y_bias - y < 0 or y_bias - y > length)
        printf("[ERROR] Y out of bound\n");
    return y_bias - y;
}

int Bug::Grid::getXFromCol(int col)
{
    return col - x_bias;
}

int Bug::Grid::getYFromRow(int row)
{
    return y_bias - row;
}

void Bug::Grid::printGrid()
{
    for (int i = 0; i < this->length; i++)
    {
        for (int j = 0; j < this->width; j++)
        {
            if (gridArr[index(i, j)] == BUG_WALL)
                std::cout << "# ";
            else if (gridArr[index(i, j)] == BUG_PASSABLE)
                std::cout << ". ";
            else if (gridArr[index(i, j)] == BUG)
                std::cout << "A ";
            else if (gridArr[index(i, j)] == BUG_GOAL)
                std::cout << "Z ";
            else if (gridArr[index(i, j)] == BUG_PATH)
                std::cout << "@ ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}
