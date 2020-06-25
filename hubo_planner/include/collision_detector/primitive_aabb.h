#ifndef HUBO_PLANNER_PRIMITIVE_AABB_H
#define HUBO_PLANNER_PRIMITIVE_AABB_H

#include <quadmap/quadmap.h>

struct AABB {
    /*
     * Constructor without any information
     */
    AABB() : min_bbx(0.0f, 0.0f), max_bbx(0.0f, 0.0f) { }

    /*
     * Constructor on the assumption that the center is (0.0, 0.0)
     */
    explicit AABB(const quadmap::point2d& _size) {
        max_bbx =  (_size * 0.5);
        min_bbx = -(_size * 0.5);   // == -1.0 * max_bbx
    }

    /*
     * Constructor of axis-aligned bounding box with the size at the center point.
     */
    AABB(const quadmap::point2d& _center, const quadmap::point2d& _size) {
        max_bbx = _center + (_size * 0.5);
        min_bbx = _center - (_size * 0.5);  // == max_bbx - _size
    }

    quadmap::point2d min_bbx;
    quadmap::point2d max_bbx;
};

#endif //HUBO_PLANNER_PRIMITIVE_AABB_H
