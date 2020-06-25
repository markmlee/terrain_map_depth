#include <collision_detector/collision_detector.h>

bool CollisionChecker::doCollide(const quadmap::QuadTree* _environment, const OBB& _obb)
{
    if(_environment == nullptr || _environment->size() <= 0)
        return false;

    double root_size = _environment->getNodeSize(0);
    Box root_box(quadmap::point2d(0.0f, 0.0f), (float)root_size);
    return doCollideRecursive(_environment, _obb, _environment->getRoot(), root_box);
}

bool CollisionChecker::doCollideRecursive(const quadmap::QuadTree* _environment, const OBB& _obb, const quadmap::QuadTreeNode* _node, const Box& _node_box)
{
    for(unsigned int i = 0; i < 4; i++){
        if(!_environment->nodeChildExists(_node, i))
            continue;   // Unknown space

        const quadmap::QuadTreeNode* childNode = _environment->getNodeChild(_node, i);
        if(!_environment->isNodeOccupied(childNode))
            continue;   // Free space

        // Collision check
        Box child_node_box(_node_box.center, _node_box.size * 0.5f);
        float center_offset = _node_box.size * 0.25f;
        if(i & 1U) child_node_box.center.x() += center_offset;
        else       child_node_box.center.x() -= center_offset;
        if(i & 2U) child_node_box.center.y() += center_offset;
        else       child_node_box.center.y() -= center_offset;

        // Collision check between circle and bounding box
        if(doCollidePrimitives(_obb, child_node_box)) {
            // Collision at leaf node (child node)
            if(!_environment->nodeHasChildren(childNode))
                return true;
            // Recursive collision check using a child node
            if(doCollideRecursive(_environment, _obb, childNode, child_node_box)) {
                return true;
            }
        }
    }

    return false;
}

bool CollisionChecker::doCollidePrimitives(const OBB& _obb, const Box& _box)
{
    double box_half_size = _box.size * 0.5;
    quadmap::point2d t = _obb.center - _box.center;

    // X-axis of AABB
    {
        double ra = box_half_size;
        double rb = (_obb.aabb.max_bbx.x() - _obb.aabb.min_bbx.x()) * 0.5;
        if(std::fabs(t.x()) > ra + rb)
            return false;
    }
    // Y-axis of AABB
    {
        double ra = box_half_size;
        double rb = (_obb.aabb.max_bbx.y() - _obb.aabb.min_bbx.y()) * 0.5;
        if(std::fabs(t.y()) > ra + rb)
            return false;
    }
    // X-axis of OBB
    {
        // double ra = box_half_size * std::fabs(_obb.axis[0].x()) + box_half_size * std::fabs(_obb.axis[1].x());
        double ra = box_half_size * std::fabs(_obb.axis[0].x()) + box_half_size * std::fabs(_obb.axis[0].y());
        double rb = _obb.length.x() * 0.5;
        // if(std::fabs(t.x() * _obb.axis[0].x() + t.y() * _obb.axis[1].x()) > ra + rb)
        if(std::fabs(t.x() * _obb.axis[0].x() + t.y() * _obb.axis[0].y()) > ra + rb)
            return false;
    }
    // Y-axis of OBB
    {
        // double ra = box_half_size * std::fabs(_obb.axis[0].y()) + box_half_size * std::fabs(_obb.axis[1].y());
        double ra = box_half_size * std::fabs(_obb.axis[1].x()) + box_half_size * std::fabs(_obb.axis[1].y());
        double rb = _obb.length.y() * 0.5;
        // if(std::fabs(t.x() * _obb.axis[0].y() + t.y() * _obb.axis[1].y()) > ra + rb)
        if(std::fabs(t.x() * _obb.axis[1].x() + t.y() * _obb.axis[1].y()) > ra + rb)
            return false;
    }

    return true;
}

bool CollisionChecker::doCollidePrimitives(const OBB& _obb, const AABB& _aabb)
{
    quadmap::point2d aabb_radius = (_aabb.max_bbx - _aabb.min_bbx) * 0.5f;
    quadmap::point2d aabb_center = (_aabb.max_bbx + _aabb.min_bbx) * 0.5f;
    quadmap::point2d t = _obb.center - aabb_center;

    // X-axis of AABB
    {
        double ra = aabb_radius.x();
        double rb = (_obb.aabb.max_bbx.x() - _obb.aabb.min_bbx.x()) * 0.5;
        if(std::fabs(t.x()) > ra + rb)
            return false;
    }
    // Y-axis of AABB
    {
        double ra = aabb_radius.y();
        double rb = (_obb.aabb.max_bbx.y() - _obb.aabb.min_bbx.y()) * 0.5;
        if(std::fabs(t.y()) > ra + rb)
            return false;
    }
    // X-axis of OBB
    {
        double ra = aabb_radius.x() * std::fabs(_obb.axis[0].x()) + aabb_radius.y() * std::fabs(_obb.axis[1].x());
        double rb = _obb.length.x() * 0.5;
        if(std::fabs(t.x() * _obb.axis[0].x() + t.y() * _obb.axis[1].x()) > ra + rb)
            return false;
    }
    // Y-axis of OBB
    {
        double ra = aabb_radius.x() * std::fabs(_obb.axis[0].y()) + aabb_radius.y() * std::fabs(_obb.axis[1].y());
        double rb = _obb.length.y() * 0.5;
        if(std::fabs(t.x() * _obb.axis[0].y() + t.y() * _obb.axis[1].y()) > ra + rb)
            return false;
    }

    return true;
}