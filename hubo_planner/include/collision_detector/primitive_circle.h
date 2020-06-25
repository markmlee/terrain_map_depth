#ifndef HUBO_PLANNER_PRIMITIVE_CIRCLE_H
#define HUBO_PLANNER_PRIMITIVE_CIRCLE_H

#include <quadmap/quadmap.h>

struct Circle {
    /*
     * Constructor without any information
     */
    Circle() : center(0.0f,0.0f), radius(0.0f) {}

    /*
     * Constructor on the assumption that the center is (0.0, 0.0)
     */
    explicit Circle(const float _r) : center(0.0f, 0.0f), radius(_r) {}

    /*
     * Constructor of axis-aligned bounding box with the size at the center point.
     */
    Circle(const quadmap::point2d& _c, const float _r) : center(_c), radius(_r) {}

    quadmap::point2d center;
    float            radius;
};

#endif //HUBO_PLANNER_PRIMITIVE_CIRCLE_H
