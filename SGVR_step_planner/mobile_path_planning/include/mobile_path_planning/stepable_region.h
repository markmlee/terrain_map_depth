//
// Created by heechanshin on 19. 6. 19.
//

#ifndef MOBILE_PATH_PLANNING_STEPABLE_REGION_H
#define MOBILE_PATH_PLANNING_STEPABLE_REGION_H

#include <ros/ros.h>

#include <mobile_path_planning/collision_checker.h>


class StepableRegion{
public:
    /*****************************
     *
     * Stepable region constructor without designated resolution.
     * In this case, resolution equals to blockLength.
     *
     *****************************/
    StepableRegion();

    /*****************************
     *
     * Stepable region constructor with designated resolution.
     *
     *****************************/
    StepableRegion(double RESOLUTION);

    /*****************************
     *
     * Insert new node to stepable region
     * This method is called many times from GenerateSampleStepableRegion function
     *
     *****************************/
    void UpdateStepableRegion(const quadmap::point2d& _center, const quadmap::point2d& _length);

    /*****************************
     *
     * Generate stepable region for testing walking on stepping bridge
     *
     *****************************/
    void GenerateSampleStepableRegion();

    /*****************************
     *
     * Publish stepable region as ros message
     *
     *****************************/
    void PublishStepableRegionMsg();

    quadmap::QuadTree GetQuadtree();

private:
    quadmap::QuadTree* stepable_region_quadtree;
    ros::NodeHandle nh;
    ros::Publisher stepable_region_publisher;

    double horiLength = 20.; // Entire horizontal map size in meter
    double vertLength = 20.; // Entire vertical map size in meter
    double blockLength = 0.05; // Default resolution of quadmap in meter
};

#endif //MOBILE_PATH_PLANNING_STEPABLE_REGION_H
