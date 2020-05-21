#ifndef __COLLISION_CHECKER_H_
#define __COLLISION_CHECKER_H_

#include "configuration.h"
#include <quadmap/quadmap.h>
#include <nav_msgs/GridCells.h>

#define SQUARE_ROOT_2   1.41421356237

struct RobotModel{
    RobotModel(void) : center(0,0), radius(0.0) {}
    RobotModel(const quadmap::point2d& _c, const double _r) : center(_c), radius(_r) {}
    RobotModel(const double _x, const double _y, const double _r) : center(_x, _y), radius(_r) {}
    RobotModel(const Configuration& _configuration, const double _r) : center(_configuration.x(), _configuration.y()), radius(_r) {}

    quadmap::point2d center;
    double           radius;
};

class CollisionChecker{
public:
    CollisionChecker() {}
    ~CollisionChecker() {}

    bool doCollide(const quadmap::QuadTree* _environment, const RobotModel& _robot);
    bool doCollideInner(const quadmap::QuadTree* _environment, const RobotModel& _robot);

protected:
    bool doCollideRecur(const quadmap::QuadTree* _environment, const RobotModel& _robot, quadmap::QuadTreeNode* _node, float _node_radius, quadmap::point2d _node_center);
    bool doCollideRecurInner(const quadmap::QuadTree* _environment, const RobotModel& _robot, quadmap::QuadTreeNode* _node, float _node_radius, quadmap::point2d _node_center);
    bool doCollidePrimitive(const RobotModel& _robot, quadmap::point2d& _bbx_center, float _bbx_radius);
    bool doCollidePrimitiveInner(const RobotModel& _robot, quadmap::point2d& _bbx_center, float _bbx_radius, float inner_factor);
};

#endif
