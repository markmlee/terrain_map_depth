#ifndef HUBO_PLANNER_PRIMITIVE_OBB_H
#define HUBO_PLANNER_PRIMITIVE_OBB_H

#include <collision_detector/primitive_aabb.h>

struct OBB {
    /*
     * Constructor without any information
     */
    OBB() : center(0.0f, 0.0f), length(0.0f, 0.0f) {
        axis[0] = quadmap::point2d(1.0, 0.0);
        axis[1] = quadmap::point2d(0.0, 1.0);

        aabb = AABB();
    }

    /*
     * Constructor on the assumption that the center is (0.0, 0.0) and no rotation
     */
    explicit OBB(const quadmap::point2d& _size) : center(0.0f, 0.0f) {
        axis[0] = quadmap::point2d(1.0, 0.0);
        axis[1] = quadmap::point2d(0.0, 1.0);
        length  = _size;

        aabb = AABB(_size);
    }

    /*
     * Constructor without rotation
     */
    OBB(const quadmap::point2d& _center, const quadmap::point2d& _size) {
        center = _center;
        axis[0] = quadmap::point2d(1.0, 0.0);
        axis[1] = quadmap::point2d(0.0, 1.0);
        length = _size;

        aabb = AABB(_center, _size);
    }

    /*
     * Constructor of oriented bounding box with the size at the center point and the rotation
     */
    OBB(const quadmap::point2d& _center, const quadmap::point2d& _size, const double _rotation) {
        center = _center;
        // axis[0] = quadmap::point2d((float)std::cos(_rotation), (float)(-std::sin(_rotation)));
        // axis[1] = quadmap::point2d((float)std::sin(_rotation), (float)std::cos(_rotation));
        axis[0] = quadmap::point2d((float)std::cos(_rotation), (float)(std::sin(_rotation)));
        axis[1] = quadmap::point2d(-(float)std::sin(_rotation), (float)std::cos(_rotation));
        length = _size;

        std::array<quadmap::point2d, 4> vertices;
        get_vertices(vertices);
//        quadmap::point2d half_size = _size * 0.5;
//        quadmap::point2d rot_half_width  = axis[0] * half_size(0);
//        quadmap::point2d rot_half_height = axis[1] * half_size(1);
//        vertices[0] = _center + rot_half_width;
//        vertices[1] = _center - rot_half_width;
//        vertices[2] = _center + rot_half_height;
//        vertices[3] = _center - rot_half_height;

        aabb = AABB(_center, quadmap::point2d(0.0, 0.0));
        /*for(int i = 0; i < vertices.size(); i++) {
            // min_bbx
            if(aabb.min_bbx.x() > vertices[i].x())  aabb.min_bbx.x() = vertices[i].x();
            if(aabb.min_bbx.y() > vertices[i].y())  aabb.min_bbx.y() = vertices[i].y();
            // max_bbx
            if(aabb.max_bbx.x() < vertices[i].x())  aabb.max_bbx.x() = vertices[i].x();
            if(aabb.max_bbx.y() < vertices[i].y())  aabb.max_bbx.y() = vertices[i].y();
        }*/
        for(auto v : vertices) {
            // min_bbx
            if(aabb.min_bbx.x() > v.x())  aabb.min_bbx.x() = v.x();
            if(aabb.min_bbx.y() > v.y())  aabb.min_bbx.y() = v.y();
            // max_bbx
            if(aabb.max_bbx.x() < v.x())  aabb.max_bbx.x() = v.x();
            if(aabb.max_bbx.y() < v.y())  aabb.max_bbx.y() = v.y();
        }
    }

    void get_vertices(std::array<quadmap::point2d, 4>& _vertices) const {
        quadmap::point2d half_size = this->length * 0.5;
        quadmap::point2d rot_half_width  = axis[0] * half_size(0);
        quadmap::point2d rot_half_height = axis[1] * half_size(1);
        _vertices[0] = this->center - rot_half_width - rot_half_height;
        _vertices[1] = this->center + rot_half_width - rot_half_height;
        _vertices[2] = this->center + rot_half_width + rot_half_height;
        _vertices[3] = this->center - rot_half_width + rot_half_height;
    }


    void get_stepping_vertices(std::array<quadmap::point2d, 12>& _vertices) const {
        quadmap::point2d half_size = this->length * 0.5;
        quadmap::point2d rot_half_width  = axis[0] * half_size(0);
        quadmap::point2d rot_half_height = axis[1] * half_size(1);
        _vertices[0] = this->center - rot_half_width - rot_half_height;
        _vertices[1] = this->center + rot_half_width - rot_half_height;
        _vertices[2] = this->center + rot_half_width + rot_half_height;
        _vertices[3] = this->center - rot_half_width + rot_half_height;

        _vertices[4] = this->center + rot_half_width;
        _vertices[5] = this->center - rot_half_width;
        _vertices[6] = this->center + rot_half_height;
        _vertices[7] = this->center - rot_half_height;

        _vertices[8]  = this->center - rot_half_width * 0.5 - rot_half_height * 0.5;
        _vertices[9]  = this->center + rot_half_width * 0.5 - rot_half_height * 0.5;
        _vertices[10] = this->center + rot_half_width * 0.5 + rot_half_height * 0.5;
        _vertices[11] = this->center - rot_half_width * 0.5 + rot_half_height * 0.5;
    }

    // OBB
    quadmap::point2d center;
    quadmap::point2d axis[2];   // == rotation matrix of which each column vector represents a rotated axis
    quadmap::point2d length;

    // Bounding volume: AABB
    AABB aabb;
};

#endif //HUBO_PLANNER_PRIMITIVE_OBB_H
