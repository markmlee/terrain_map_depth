#ifndef SYNTHETIC_DATASET_OBJECT_H
#define SYNTHETIC_DATASET_OBJECT_H

#include <octomap/octomap_types.h>
#include <limits>

/*
 * Geometric information of an object
 */
class Object{
public:
    // Constructors
    Object() {
        min_bbx = octomap::point3d(0.0f, 0.0f, 0.0f);
        max_bbx = octomap::point3d(0.0f, 0.0f, 0.0f);
    }
    Object(const octomap::point3d& _center, const octomap::point3d& _length) {
        set_bbx(_center, _length);
    }

    /*
     * Check the collision between an object and a box
     *
     * @param min_bbx: min point of the cell
     * @param max_bbx: max point of the cell
     * @return collision
     */
    bool is_collision(const octomap::point3d& _min_bbx, const octomap::point3d& _max_bbx) const {
        if (max_bbx.x() < _min_bbx.x() || min_bbx.x() > _max_bbx.x())   return false;
        if (max_bbx.y() < _min_bbx.y() || min_bbx.y() > _max_bbx.y())   return false;
        if (max_bbx.z() < _min_bbx.z() || min_bbx.z() > _max_bbx.z())   return false;
        return true;
    }

    /*
     * Check the collision between an object and a cell
     *
     * @param center: center of the cell
     * @param size: size of the cell
     * @return collision
     */
    bool is_collision(const octomap::point3d& _center, const float _size) const {
        octomap::point3d half_size(_size / 2.0f, _size / 2.0f, _size / 2.0f);
        return is_collision(_center - half_size, _center + half_size);
    }

    /*
     * Check the collision between an object and a point
     *
     * @param x: X coordinate of point
     * @param y: Y coordinate of point
     * @param z: Z coordinate of point
     * @return collision
     */
    bool is_collision(const float _x, const float _y, const float _z) const {
        if (min_bbx.x() > _x || max_bbx.x() < _x)
            return false;
        if (min_bbx.y() > _y || max_bbx.y() < _y)
            return false;
        if (min_bbx.z() > _z || max_bbx.z() < _z)
            return false;
        return true;
    }

    /*
     * Check the collision between an object and a point
     *
     * @param _point: 3D coordinate of point
     * @return collision
     */
    bool is_collision(const octomap::point3d& _point) const {
        return is_collision(_point.x(), _point.y(), _point.z());
    }

    // Set the information of an object
    void set_bbx(const octomap::point3d& _center, const octomap::point3d& _length){
        min_bbx = _center - (_length * 0.5f);
        max_bbx = _center + (_length * 0.5f);
    }

    // Get the information of an object
    octomap::point3d get_min_bbx() const { return min_bbx; }
    octomap::point3d get_max_bbx() const { return max_bbx; }
    octomap::point3d get_center() const { return (min_bbx + max_bbx) * 0.5f; }
    octomap::point3d get_length() const { return max_bbx - min_bbx; }
    //float get_min_x() const { return min_bbx.x; }
    //float get_min_y() const { return min_bbx.y; }
    //float get_min_z() const { return min_bbx.z; }
    //float get_max_x() const { return max_bbx.x; }
    //float get_max_y() const { return max_bbx.y; }
    //float get_max_z() const { return max_bbx.z; }

protected:
    octomap::point3d min_bbx;
    octomap::point3d max_bbx;
};


#endif //SYNTHETIC_DATASET_OBJECT_H
