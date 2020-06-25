#ifndef HUBO_PLANNER_CONFIGURATION_H
#define HUBO_PLANNER_CONFIGURATION_H

#include <quadmap/quadmap.h>

struct Configuration {
    Configuration () { data[0] = data[1] = data[2] = 0.0; }

    Configuration (const Configuration& _other) {
        data[0] = _other(0);
        data[1] = _other(1);
        data[2] = _other(2);
    }

    Configuration (float _x, float _y, float _r) {
        data[0] = _x;
        data[1] = _y;
        data[2] = _r;
    }

    inline Configuration& operator= (const Configuration& _other)  {
        data[0] = _other(0);
        data[1] = _other(1);
        data[2] = _other(2);
        return *this;
    }

    inline const float& operator() (unsigned int i) const
    {
        return data[i];
    }
    inline float& operator() (unsigned int i)
    {
        return data[i];
    }

    inline float& x()
    {
        return operator()(0);
    }

    inline float& y()
    {
        return operator()(1);
    }

    inline float& r()
    {
        return operator()(2);
    }

    inline const float& x() const
    {
        return operator()(0);
    }

    inline const float& y() const
    {
        return operator()(1);
    }

    inline const float& r() const
    {
        return operator()(2);
    }

    float data[3];
};

#endif //HUBO_PLANNER_CONFIGURATION_H
