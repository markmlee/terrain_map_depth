//
// Created by heechanshin on 19. 5. 7.
//

#ifndef MOBILE_PATH_PLANNING_STEP_PLANNER_H
#define MOBILE_PATH_PLANNING_STEP_PLANNER_H

#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <mobile_path_planning/path_planner.h>
#include <mobile_path_planning/StepsStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <mobile_path_planning/collision_checker.h>
#include <random>

class Foot {
public:
    Foot(bool isRightFoot, double d, double mf, double ms, double mb, double af, double ab){
        _isRightFoot = isRightFoot;
        _d = d;
        _mf = mf;
        _ms = ms;
        _mb = mb;
        _af = af;
        _ab = ab;
    }

    /*****************************
     *
     * Set support region values
     *
     *****************************/
    void setSupportRegion(double d, double mf, double ms, double mb, double af, double ab){
        _d = d;
        _mf = mf;
        _ms = ms;
        _mb = mb;
        _af = af;
        _ab = ab;
    }

    /*****************************
     *
     * Check whether given pose is inside of stepable region
     *
     *****************************/
    bool checkInsideStepableRegion(geometry_msgs::Pose& pose, quadmap::QuadTree* stepable_region);

    /*****************************
     *
     * Check whether given pose is inside of support region
     *
     *****************************/
    bool checkInsideSupportRegion(geometry_msgs::Pose& pose);

    bool checkNearToNominalPath(geometry_msgs::Pose& pose, nav_msgs::Path &nominalPath, int currIdxOnNominalPath);

    /*****************************
     *
     * Get foot information according to nominal path, which is global path and supporting foot
     *
     *****************************/
    Foot getNextStep(nav_msgs::Path& nominalPath, int& currIdxOnNominalPath, int& prevIdxOnNominalPath, geometry_msgs::Pose& supportingFootPose, quadmap::QuadTree* stepable_region, bool randomized);

    /*****************************
     *
     * Set foot pose according to current point on nominal path. Foot is displaced by d.
     *
     *****************************/
    void setPoseAccordingToPathPose(geometry_msgs::Pose& pathPose);

    /*****************************
     *
     * Get yaw value from pose orientation.
     *
     *****************************/
    static double getYawFromPose(geometry_msgs::Pose& pose);

    static void setYawToPose(geometry_msgs::Pose& pose, double yaw);

    /*****************************
     *
     * Transform global coordinate to local coordinate or local coordinate to global coordinate
     *
     *****************************/
    void global2local(double g_ori_x, double g_ori_y, double theta, double g_obj_x, double g_obj_y, double& out_x, double& out_y);
    void local2global(double g_ori_x, double g_ori_y, double theta, double l_obj_x, double l_obj_y, double& out_x, double& out_y);

    /*****************************
     *
     * linspace function like a function in matlab
     *
     *****************************/
    template <typename T>
    std::vector<T> linspace(T a, T b, size_t N) {
        T h = (b - a) / static_cast<T>(N-1);
        std::vector<T> xs(N);
        typename std::vector<T>::iterator x;
        T val;
        for (x = xs.begin(), val = a; x != xs.end(); ++x, val += h)
            *x = val;
        if(xs.at(xs.size()-1) != b){
            xs.at(xs.size()-1) = b;
        }
        return xs;
    }

    geometry_msgs::Pose getPose(){return this->pose;}
    void setPose(geometry_msgs::Pose& pose){this->pose = pose;}

    /*************************************************
     *
     * isRightFoot : If this foot is right foot, then true. Otherwise false.
     * hasError : If this foot has any error, this flag becomes true.
     * d : Displacement between center of body and center of foot.
     * mf : Front point of support region
     * ms : Side point of support region
     * mb : Back point of support region
     * af : Front angle of inner support region
     * ab : Back angle of inner support region
     * lookAhead : Number of how many times to check candidate steps
     * pose : Pose(position and orientation) of foot in global coordinate
     * supportRegionCenter : Pose of support region
     * @ support region : Candidate region of where to place moving foot. It shaped partially ellipse.
     *
     *************************************************/
    bool _isRightFoot = true;
    bool _hasError = false;
    double _d;
    double _mf;
    double _ms;
    double _mb;
    double _af;
    double _ab;
    int lookAhead = 100;

    double footRadius = 0.1;
    double stepAreaRatio = 0.7;

    geometry_msgs::Pose pose;
    geometry_msgs::Pose supportRegionCenter;
    CollisionChecker collisionChecker;
    RobotModel centerOfFoot;
};

class StepPlanner {
public:
    StepPlanner(double d, double mf, double ms, double mb, double af, double ab, quadmap::QuadTree* _stepable_region);

    /*****************************
     *
     * Plan steps according to nominal path.
     * If isRightFootMoved true, which means currently moving foot is left, supporting foot is right foot. Vice versa.
     * Return of this function is success of planning.
     *
     *****************************/
    bool planning(bool isRightFootMoved, Foot* supportingFoot, nav_msgs::Path& nominalPath, mobile_path_planning::StepsStamped& steps, mobile_path_planning::StepsStamped &supportRegions);
    bool planning(bool isRightFootMoved, Foot* supportingFoot, nav_msgs::Path& nominalPath, mobile_path_planning::StepsStamped& steps, mobile_path_planning::StepsStamped &supportRegions, int numPlanningSteps);

    /*************************************************
     *
     * rightFoot : Right foot of robot. isRightFoot = true
     * leftFoot : Left foot of robot. isRightFoot = false
     * numPlanningSteps : How many steps to plan.
     * ErrPathPose : Indicates the point where step planning was failed
     *
     *************************************************/
    Foot rightFoot;
    Foot leftFoot;
    quadmap::QuadTree* stepable_region;
    geometry_msgs::Pose ErrPathPose;

    int numPlanningSteps = 200;

    double stepRandomizeValue = 0.1;
    bool randomizedPlanning = true;
};


#endif //MOBILE_PATH_PLANNING_STEP_PLANNER_H
