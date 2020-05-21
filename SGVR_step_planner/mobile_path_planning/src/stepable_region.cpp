//
// Created by heechanshin on 19. 6. 19.
//

#include <algorithm>
#include <nav_msgs/OccupancyGrid.h>
#include "mobile_path_planning/stepable_region.h"
#include <octomap_msgs/Octomap.h>

#include <quadmap/conversion.h>
#include <octomap_msgs/conversions.h>

StepableRegion::StepableRegion() {
    stepable_region_quadtree = new quadmap::QuadTree(blockLength);

    //TODO : Change nav_msgs::OccupancyGrid message to octomap message
    // stepable_region_publisher = nh.advertise<nav_msgs::OccupancyGrid>("mobile_hubo/stepable_region", 1);
    stepable_region_publisher = nh.advertise<octomap_msgs::Octomap>("mobile_hubo/stepable_region", 1);
}

StepableRegion::StepableRegion(double RESOLUTION){
    stepable_region_quadtree = new quadmap::QuadTree(RESOLUTION);

    //TODO : Change nav_msgs::OccupancyGrid message to octomap message
    // stepable_region_publisher = nh.advertise<nav_msgs::OccupancyGrid>("mobile_hubo/stepable_region", 1);
    stepable_region_publisher = nh.advertise<octomap_msgs::Octomap>("mobile_hubo/stepable_region", 1);
}

void StepableRegion::UpdateStepableRegion(const quadmap::point2d &_center, const quadmap::point2d &_length) {
    // Change cartessian coordinate value in double to key(index) value of quadtree
    quadmap::QuadTreeKey minKey = stepable_region_quadtree->coordToKey(_center - (_length * 0.5));
    quadmap::QuadTreeKey maxKey = stepable_region_quadtree->coordToKey(_center + (_length * 0.5));
    for(uint16_t kx = minKey[0]; kx <= maxKey[0]; kx++) {
        for (uint16_t ky = minKey[1]; ky <= maxKey[1]; ky++) {
            stepable_region_quadtree->updateNode(quadmap::QuadTreeKey(kx, ky), true);
        }
    }
}

void StepableRegion::GenerateSampleStepableRegion() {

    quadmap::point2d _center(0, 0);
    quadmap::point2d _length(0.5, 0.5);
    UpdateStepableRegion(_center, _length);

    double spacing = 1.8;
    double width = 1.6;
    double height = 1.6;
    for(int i = 0; i < vertLength/8; i++){
        for(int j = 0; j < horiLength/8; j++){
            _center = quadmap::point2d(j * spacing, spacing * i);
            _length = quadmap::point2d(width, height);
            UpdateStepableRegion(_center, _length);

            _center = quadmap::point2d(-j * spacing, spacing * i);
            _length = quadmap::point2d(width, height);
            UpdateStepableRegion(_center, _length);

            _center = quadmap::point2d(j * spacing, -spacing * i);
            _length = quadmap::point2d(width, height);
            UpdateStepableRegion(_center, _length);

            _center = quadmap::point2d(-j * spacing, -spacing * i);
            _length = quadmap::point2d(width, height);
            UpdateStepableRegion(_center, _length);
        }
    }

    _center = quadmap::point2d(-0.15, 2.1);
    _length = quadmap::point2d(0.15, 0.125);
    UpdateStepableRegion(_center, _length);
}

void StepableRegion::PublishStepableRegionMsg() {
    octomap::OcTree* stepable_region_octree = quadmap2octomap(stepable_region_quadtree, -stepable_region_quadtree->getResolution() / 2.);

    octomap_msgs::Octomap stepable_region_msg;
    stepable_region_msg.header.frame_id = "map";
    stepable_region_msg.header.stamp = ros::Time::now();
    octomap_msgs::binaryMapToMsg(*stepable_region_octree, stepable_region_msg);

    stepable_region_publisher.publish(stepable_region_msg);

    delete stepable_region_octree;
}

quadmap::QuadTree StepableRegion::GetQuadtree() {
    return *stepable_region_quadtree;
}