#ifndef HECTOT_EXCEPTION_H_
#define HECTOT_EXCEPTION_H_

#include <exception>

class HectotException : public std::exception
{
private:
    const char *msg;

public:
    explicit HectotException(const char *m) : msg(m) {}
    const char *what() const throw()
    {
        return msg;
    }
};

#endif