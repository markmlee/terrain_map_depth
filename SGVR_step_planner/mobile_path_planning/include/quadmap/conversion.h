#ifndef MOBILE_PATH_PLANNING_CONVERSION_H
#define MOBILE_PATH_PLANNING_CONVERSION_H

#include <octomap/octomap.h>
#include <quadmap/quadmap.h>

octomap::OcTree* quadmap2octomap(quadmap::QuadTree* _quadtree, const double _height = 0.0)
{
    octomap::OcTree* octree = new octomap::OcTree(_quadtree->getResolution());

    _quadtree->expand();
    for(auto it = _quadtree->begin_leafs(); it != _quadtree->end_leafs(); it++){
        octomap::point3d center(it.getX(), it.getY(), _height);
        octree->updateNode(it.getX(), it.getY(), _height, (float)it->getLogOdds());
    }
    _quadtree->prune();

    return octree;
}

quadmap::QuadTree* octomap2quadmap(octomap::OcTree* _octree, const double _height = 0.0)
{
    quadmap::QuadTree* quadtree = new quadmap::QuadTree(_octree->getResolution());

    octomap::key_type projection_height_key = _octree->coordToKey(_height);

    _octree->expand();
    for(auto it = _octree->begin_leafs(); it != _octree->end_leafs(); it++){
        if((it.getKey())[2] == projection_height_key){
            quadtree->updateNode(it.getX(), it.getY(), (float)it->getLogOdds());
        }
    }
    _octree->prune();

    return quadtree;
}


#endif //MOBILE_PATH_PLANNING_CONVERSION_H
