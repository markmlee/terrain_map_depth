#ifndef HUBO_PLANNER_COLLISION_DETECTOR_H
#define HUBO_PLANNER_COLLISION_DETECTOR_H

// Environment
#include <collision_detector/primitive_box.h>

// Robot model
#include <collision_detector/primitive_obb.h>
// #include <hubo_planner/primitive_circle.h>

class CollisionChecker{
public:
    static bool doCollide(const quadmap::QuadTree* _environment, const OBB& _obb);

protected:

    static bool doCollideRecursive(const quadmap::QuadTree* _environment, const OBB& _obb,
                                   const quadmap::QuadTreeNode* _node, const Box& _node_box);


    static bool doCollidePrimitives(const OBB& _obb, const Box& _box);
    static bool doCollidePrimitives(const OBB& _obb, const AABB& _aabb);
};

#endif //HUBO_PLANNER_COLLISION_DETECTOR_H
