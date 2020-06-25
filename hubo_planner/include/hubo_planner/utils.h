#ifndef HUBO_PLANNER_UTILS_H
#define HUBO_PLANNER_UTILS_H

#include <octomap/octomap.h>
#include <quadmap/quadmap.h>

#include <geometry_msgs/Quaternion.h>

namespace utils{
    /*
     *
     */
    bool project_environment(octomap::OcTree* _environment_3d, quadmap::QuadTree* _environment_2d, double _MIN_HEIGHT, double _MAX_HEIGHT) {
        if(_environment_3d == nullptr)
            return false;

        _environment_3d->expand();
        for(octomap::OcTree::leaf_iterator it = _environment_3d->begin_leafs(); it != _environment_3d->end_leafs(); it++){
            octomap::point3d point = _environment_3d->keyToCoord(it.getKey());
            if(point.z() < _MIN_HEIGHT && point.z() > _MAX_HEIGHT)
                continue;
            _environment_2d->updateNode(point.x(), point.y(), true);
        }

        return true;
    }

    /*
     *
     */
    double get_rotation(const geometry_msgs::Quaternion& _orientation) {
        double n = std::sqrt(_orientation.x*_orientation.x + _orientation.y*_orientation.y + _orientation.z*_orientation.z + _orientation.w*_orientation.w);
        double s = n > 0 ? 2.0/(n*n) : 0.0;

        double v1 = s * (_orientation.x*_orientation.y + _orientation.w*_orientation.z);
        double v2 = 1.0 - (s * (_orientation.y*_orientation.y + _orientation.z*_orientation.z));

        return std::atan2(v1, v2);
    }
}


#endif //HUBO_PLANNER_UTILS_H
