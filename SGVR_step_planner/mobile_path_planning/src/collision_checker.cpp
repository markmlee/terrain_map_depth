#include <mobile_path_planning/collision_checker.h>

bool CollisionChecker::doCollide(const quadmap::QuadTree* _environment, const RobotModel& _robot)
{
    return doCollideRecur(_environment, _robot, _environment->getRoot(), (float)_environment->getNodeSize(1), quadmap::point2d(0.0f, 0.0f));
}

bool CollisionChecker::doCollideInner(const quadmap::QuadTree* _environment, const RobotModel& _robot)
{
    return doCollideRecurInner(_environment, _robot, _environment->getRoot(), (float)_environment->getNodeSize(1), quadmap::point2d(0.0f, 0.0f));
}

bool CollisionChecker::doCollideRecur(const quadmap::QuadTree* _environment, const RobotModel& _robot, quadmap::QuadTreeNode* _node, float _node_radius, quadmap::point2d _node_center)
{
    for(unsigned int i = 0; i < 4; i++){
        if(!_environment->nodeChildExists(_node, i))
            continue;   // Unknown space

        quadmap::QuadTreeNode* childNode = _environment->getNodeChild(_node, i);
        if(!_environment->isNodeOccupied(childNode))
            continue;   // Free space

        // Collision check
        float childNodeRadius = _node_radius / 2;
        quadmap::point2d childNodeCenter = _node_center;
        if(i & 1) childNodeCenter.x() += childNodeRadius;
        else      childNodeCenter.x() -= childNodeRadius;
        if(i & 2) childNodeCenter.y() += childNodeRadius;
        else      childNodeCenter.y() -= childNodeRadius;

        // Collision check between circle and bounding box
        if(doCollidePrimitive(_robot, childNodeCenter, childNodeRadius)){
            // Collision at leaf node
            if(!_environment->nodeHasChildren(childNode))
                return true;
            else if(doCollideRecur(_environment, _robot, childNode, childNodeRadius, childNodeCenter)){
                return true;
            }
        }
    }

    return false;
}

bool CollisionChecker::doCollideRecurInner(const quadmap::QuadTree* _environment, const RobotModel& _robot, quadmap::QuadTreeNode* _node, float _node_radius, quadmap::point2d _node_center)
{
    for(unsigned int i = 0; i < 4; i++){
        if(!_environment->nodeChildExists(_node, i))
            continue;   // Unknown space

        quadmap::QuadTreeNode* childNode = _environment->getNodeChild(_node, i);
        if(!_environment->isNodeOccupied(childNode))
            continue;   // Free space

        // Collision check
        float childNodeRadius = _node_radius / 2;
        quadmap::point2d childNodeCenter = _node_center;
        if(i & 1) childNodeCenter.x() += childNodeRadius;
        else      childNodeCenter.x() -= childNodeRadius;
        if(i & 2) childNodeCenter.y() += childNodeRadius;
        else      childNodeCenter.y() -= childNodeRadius;

        // Collision check between circle and bounding box
        if(doCollidePrimitiveInner(_robot, childNodeCenter, childNodeRadius, _robot.radius/2.)){
            // Collision at leaf node
            if(!_environment->nodeHasChildren(childNode))
                return true;
            else if(doCollideRecurInner(_environment, _robot, childNode, childNodeRadius, childNodeCenter)){
                return true;
            }
        }
    }

    return false;
}

bool CollisionChecker::doCollidePrimitive(const RobotModel& _robot, quadmap::point2d& _bbx_center, float _bbx_radius)
{
    double distX = fabs(_robot.center.x() - _bbx_center.x());
    double distY = fabs(_robot.center.y() - _bbx_center.y());

    if(distX > (_bbx_radius + _robot.radius))
        return false;
    if(distY > (_bbx_radius + _robot.radius))
        return false;

    if(distX <= _bbx_radius){
        return true;
    }
    if(distY <= _bbx_radius){
        return true;
    }

    const double r = SQUARE_ROOT_2 * _bbx_radius + _robot.radius;
    double cornerDist = distX*distX + distY*distY;
    if(cornerDist < (r * r)){
        return true;
    }
    else{
        return false;
    }
}

bool CollisionChecker::doCollidePrimitiveInner(const RobotModel& _robot, quadmap::point2d& _bbx_center, float _bbx_radius, float inner_factor)
{
    double distX = fabs(_robot.center.x() - _bbx_center.x());
    double distY = fabs(_robot.center.y() - _bbx_center.y());

    if(distX > (_bbx_radius - inner_factor))
        return false;
    if(distY > (_bbx_radius - inner_factor))
        return false;

    if(distX <= _bbx_radius){
        return true;
    }
    if(distY <= _bbx_radius){
        return true;
    }

    const double r = SQUARE_ROOT_2 * _bbx_radius - _robot.radius;
    double cornerDist = distX*distX + distY*distY;
    if(cornerDist < (r * r)){
        return true;
    }
    else{
        return false;
    }
}
