#ifndef HUBO_PLANNER_PRIMITIVE_BOX_H
#define HUBO_PLANNER_PRIMITIVE_BOX_H

#include <quadmap/quadmap.h>

struct Box {
    /*
     * Constructor without any information
     */
    Box() : center(0.0f, 0.0f), size(0.0f) { }

    /*
     * Constructor on the assumption that the center is (0.0, 0.0)
     */
    explicit Box(const double& _size)  : center(0.0f, 0.0f), size(_size) { }

    /*
     * Constructor of box with the size at the center point
     */
    Box(const quadmap::point2d& _center, const float& _size)  : center(_center), size(_size) { }

    quadmap::point2d center;
    float            size;
};

#endif //HUBO_PLANNER_PRIMITIVE_BOX_H
