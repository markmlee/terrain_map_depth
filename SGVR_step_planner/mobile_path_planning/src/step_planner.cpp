//
// Created by heechanshin on 19. 5. 7.
//

#include "mobile_path_planning/step_planner.h"

void Foot::setPoseAccordingToPathPose(geometry_msgs::Pose& pathPose){
    double yaw = getYawFromPose(pathPose);

    if(_isRightFoot){
        yaw -= M_PI_2;
    }
    else{
        yaw += M_PI_2;
    }

    // displace by d, but same orientation
    pose.position.x = pathPose.position.x + _d * cos(yaw);
    pose.position.y = pathPose.position.y + _d * sin(yaw);
    pose.position.z = 10.;
    pose.orientation = pathPose.orientation;
    supportRegionCenter.position.x = pathPose.position.x - _d * cos(yaw);
    supportRegionCenter.position.y = pathPose.position.y - _d * sin(yaw);
    setYawToPose(supportRegionCenter, getYawFromPose(pathPose) - M_PI_2);
}

double Foot::getYawFromPose(geometry_msgs::Pose &pose) {
    tf2::Quaternion q = tf2::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    tf2::Matrix3x3 mat(q);
    tf2Scalar yaw;
    tf2Scalar pitch;
    tf2Scalar roll;
    mat.getEulerYPR(yaw, pitch, roll);

    return yaw;
}

void Foot::setYawToPose(geometry_msgs::Pose &pose, double yaw) {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    pose.orientation.w = q.w();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
}

void Foot::global2local(double g_ori_x, double g_ori_y, double theta, double g_obj_x, double g_obj_y, double &out_x,
                        double &out_y) {
    out_x = (g_obj_x - g_ori_x) * cos(theta) + (g_obj_y - g_ori_y) * sin(theta);
    out_y = -(g_obj_x - g_ori_x) * sin(theta) + (g_obj_y - g_ori_y) * cos(theta);
}

void Foot::local2global(double g_ori_x, double g_ori_y, double theta, double l_obj_x, double l_obj_y, double &out_x,
                        double &out_y) {
    out_x = l_obj_x * cos(theta) - l_obj_y * sin(theta) + g_ori_x;
    out_y = l_obj_x * sin(theta) + l_obj_y * cos(theta) + g_ori_y;
}

bool Foot::checkInsideStepableRegion(geometry_msgs::Pose &pose, quadmap::QuadTree* stepable_region) {
    // Check whether the candidate foot is stepable or not by checking collision with stepable region.
    // Collision means stepable, because that means foot overlaps the region.
    centerOfFoot.center = quadmap::point2d(pose.position.x, pose.position.y);
    centerOfFoot.radius = footRadius;

    RobotModel lt;
    RobotModel lb;
    RobotModel rt;
    RobotModel rb;

    double ratio = stepAreaRatio;
    double yaw = getYawFromPose(pose);
    double vector_lt_x = ratio * (footRadius/2. * cos(yaw - M_PI_2) + footRadius * cos(yaw));
    double vector_lt_y = ratio * (footRadius/2. * sin(yaw - M_PI_2) + footRadius * sin(yaw));
    double vector_lb_x = ratio * (footRadius/2. * cos(yaw - M_PI_2) + footRadius * cos(yaw + M_PI));
    double vector_lb_y = ratio * (footRadius/2. * sin(yaw - M_PI_2) + footRadius * sin(yaw + M_PI));
    double vector_rt_x = ratio * (footRadius/2. * cos(yaw + M_PI_2) + footRadius * cos(yaw));
    double vector_rt_y = ratio * (footRadius/2. * sin(yaw + M_PI_2) + footRadius * sin(yaw));
    double vector_rb_x = ratio * (footRadius/2. * cos(yaw + M_PI_2) + footRadius * cos(yaw + M_PI));
    double vector_rb_y = ratio * (footRadius/2. * sin(yaw + M_PI_2) + footRadius * sin(yaw + M_PI));

    lt.center = quadmap::point2d(pose.position.x + vector_lt_x, pose.position.y + vector_lt_y);
    lt.radius = stepable_region->getResolution()/2.;
    lb.center = quadmap::point2d(pose.position.x + vector_lb_x, pose.position.y + vector_lb_y);
    lb.radius = stepable_region->getResolution()/2.;
    rt.center = quadmap::point2d(pose.position.x + vector_rt_x, pose.position.y + vector_rt_y);
    rt.radius = stepable_region->getResolution()/2.;
    rb.center = quadmap::point2d(pose.position.x + vector_rb_x, pose.position.y + vector_rb_y);
    rb.radius = stepable_region->getResolution()/2.;

    bool occu_lt = false;
    occu_lt = collisionChecker.doCollideInner(stepable_region, lt);
    for(int i = 0; i < 4; i++){
        if(occu_lt)
            break;
        lt.center = lt.center + quadmap::point2d(stepable_region->getResolution() * cos(pow(-1, i/2) * yaw), stepable_region->getResolution() * sin(pow(-1, i%2) * yaw));
        occu_lt = collisionChecker.doCollideInner(stepable_region, lt);
    }

    bool occu_lb = false;
    occu_lb = collisionChecker.doCollideInner(stepable_region, lb);
    for(int i = 0; i < 4; i++){
        if(occu_lb)
            break;
        lb.center = lb.center + quadmap::point2d(stepable_region->getResolution() * cos(pow(-1, i/2) * yaw), stepable_region->getResolution() * sin(pow(-1, i%2) * yaw));
        occu_lb = collisionChecker.doCollideInner(stepable_region, lb);
    }

    bool occu_rt = false;
    occu_rt = collisionChecker.doCollideInner(stepable_region, rt);
    for(int i = 0; i < 4; i++){
        if(occu_rt)
            break;
        rt.center = rt.center + quadmap::point2d(stepable_region->getResolution() * cos(pow(-1, i/2) * yaw), stepable_region->getResolution() * sin(pow(-1, i%2) * yaw));
        occu_rt = collisionChecker.doCollideInner(stepable_region, rt);
    }

    bool occu_rb = false;
    occu_rb = collisionChecker.doCollideInner(stepable_region, rb);
    for(int i = 0; i < 4; i++){
        if(occu_rb)
            break;
        rb.center = rb.center + quadmap::point2d(stepable_region->getResolution() * cos(pow(-1, i/2) * yaw), stepable_region->getResolution() * sin(pow(-1, i%2) * yaw));
        occu_rb = collisionChecker.doCollideInner(stepable_region, rb);
    }

    return (occu_lt && occu_rb) || (occu_lb && occu_rt);
    // return collisionChecker.doCollide(stepable_region, centerOfFoot);

}

bool Foot::checkInsideSupportRegion(geometry_msgs::Pose &pose) {
    // Orientation of support region local coordinate.
    // Fundamentally, orientation of support region is considering y axis, so we need to subtract by pi/2 to consider x axis.
    double orientationSupportRegionCoordinate = getYawFromPose(supportRegionCenter);

    double cos_angle = cos(orientationSupportRegionCoordinate);
    double sin_angle = sin(orientationSupportRegionCoordinate);

    double xc = pose.position.x - supportRegionCenter.position.x;
    double yc = pose.position.y - supportRegionCenter.position.y;

    //Rotational transform
    double xct = xc * cos_angle - yc * sin_angle;
    double yct = xc * sin_angle + yc * cos_angle;

    // Check given pose of foot is inside of support region according to foot information and quadrant
    // Because support region defers according to quadrant
    // Right and left foot has different base axis ( right foot has path axis - PI/2, left foot has path axis + PI/2 ).
    // So, it works according to which foot it is.
    if(_isRightFoot){ //Right
        if(xct >= 0){
            // Quadrant 1, 4
            if(yct >= 0){
                // Quadrant 1
                if(pow(xct,2)/pow(_ms,2) + pow(yct,2)/pow(_mf,2) <= 1){
                    return true;
                }
                else{
                    return false;
                }
            }
            else{
                // Quadrant 4
                if(pow(xct,2)/pow(_ms,2) + pow(yct,2)/pow(_mb,2) <= 1){
                    return true;
                }
                else{
                    return false;
                }
            }
        }
        else{
            // Quadrant 2, 3
            if(yct >= 0){
                // Quadrant 2
                if((pow(xct,2)/pow(_ms,2) + pow(yct,2)/pow(_mf,2) <= 1) && (atan2(-xct,yct) <= _af)){
                    return true;
                }
                else{
                    return false;
                }
            }
            else{
                // Quadrant 3
                if((pow(xct,2)/pow(_ms,2) + pow(yct,2)/pow(_mb,2) <= 1) && (atan2(xct,yct) <= _ab)){
                    return true;
                }
                else{
                    return false;
                }
            }
        }
    }
    else{ //Left
        if(xct >= 0){
            // Quadrant 1, 4
            if(yct >= 0){
                // Quadrant 1
                if((pow(xct,2)/pow(_ms,2) + pow(yct,2)/pow(_mf,2) <= 1) && (atan2(xct,yct) <= _af)){
                    return true;
                }
                else{
                    return false;
                }
            }
            else{
                // Quadrant 4
                if((pow(xct,2)/pow(_ms,2) + pow(yct,2)/pow(_mb,2) <= 1) && (atan2(xct,-yct) <= _ab)){
                    return true;
                }
                else{
                    return false;
                }
            }
        }
        else{
            // Quadrant 2, 3
            if(yct >= 0){
                // Quadrant 2
                if(pow(xct,2)/pow(_ms,2) + pow(yct,2)/pow(_mf,2) <= 1){
                    return true;
                }
                else{
                    return false;
                }
            }
            else{
                // Quadrant 3
                if(pow(xct,2)/pow(_ms,2) + pow(yct,2)/pow(_mb,2) <= 1){
                    return true;
                }
                else{
                    return false;
                }
            }
        }
    }
}

bool Foot::checkNearToNominalPath(geometry_msgs::Pose &pose, nav_msgs::Path &nominalPath, int currIdxOnNominalPath) {
    for(int i = currIdxOnNominalPath; i < nominalPath.poses.size(); i++){
        double distance = sqrt(pow(pose.position.x - nominalPath.poses[i].pose.position.x, 2) + pow(pose.position.y - nominalPath.poses[i].pose.position.y, 2));

        if(distance < 2*_d)
            return true;
    }
    return false;
}

Foot Foot::getNextStep(nav_msgs::Path &nominalPath, int &currIdxOnNominalPath, int &prevIdxOnNominalPath,
                       geometry_msgs::Pose &supportingFootPose, quadmap::QuadTree* stepable_region, bool randomized) {
    // Set support region pose according to supporting foot
    double yawOfSupportingFoot = getYawFromPose(supportingFootPose);
    if(_isRightFoot){
        yawOfSupportingFoot -= M_PI_2;
    }
    else{
        yawOfSupportingFoot += M_PI_2;
    }
    supportRegionCenter.position.x = supportingFootPose.position.x + 2.*_d * cos(yawOfSupportingFoot);
    supportRegionCenter.position.y = supportingFootPose.position.y + 2.*_d * sin(yawOfSupportingFoot);
    supportRegionCenter.position.z = 10.;
    setYawToPose(supportRegionCenter, getYawFromPose(supportingFootPose) - M_PI_2);

    //std::cout << "supportingFootPose : " << supportingFootPose.position.x << ", " << supportingFootPose.position.y << std::endl;
    // Get orientation of path pose at initial currIdxOnNominalPath
    geometry_msgs::Pose initPathPose = nominalPath.poses.at(currIdxOnNominalPath).pose;

    // Candidate foot and pose of next step
    geometry_msgs::Pose candidateFootPose;
    Foot candidateFoot(_isRightFoot, _d, _mf, _ms, _mb, _af, _ab);

    // Flag whether find candidate or not
    bool foundCandidate = false;

    // Last index of path pose
    int mostFarIdxPathPose = prevIdxOnNominalPath + lookAhead;
    if(mostFarIdxPathPose > nominalPath.poses.size()-1){
        mostFarIdxPathPose = nominalPath.poses.size()-1;
    }

    for(;mostFarIdxPathPose > prevIdxOnNominalPath;){
        // Get current yaw value of path pose and set candidate foot pose
        // Candidate foot is displaced by d from nominal path
        geometry_msgs::Pose currPathPose = nominalPath.poses.at(mostFarIdxPathPose).pose;
        double currPathYaw = getYawFromPose(currPathPose);
        if(_isRightFoot){
            currPathYaw -= M_PI_2;
        }
        else{
            currPathYaw += M_PI_2;
        }

        double gap = _d;
        if(randomized){
            std::random_device rn;
            std::mt19937_64 rnd(rn());
            std::uniform_real_distribution<double> rand_d_range(-_d, _d);
            gap = _d + rand_d_range(rnd);
        }

        candidateFootPose.position.x = currPathPose.position.x + gap * cos(currPathYaw);
        candidateFootPose.position.y = currPathPose.position.y + gap * sin(currPathYaw);
        candidateFootPose.position.z = 10.;
        candidateFootPose.orientation = currPathPose.orientation;

        // Check whether the candidate is inside of support region and stepable region
        if(checkInsideSupportRegion(candidateFootPose) && checkInsideStepableRegion(candidateFootPose, stepable_region)){
            //std::cout << "candidateFootPose : " << candidateFootPose.position.x << ", " << candidateFootPose.position.y << std::endl;
            //std::cout << "supportRegionPose : " << supportRegionCenter.position.x << ", " << supportRegionCenter.position.y << ", " << getYawFromPose(supportRegionCenter) << std::endl;
            candidateFoot.setPose(candidateFootPose);
            double yaw = getYawFromPose(candidateFootPose);
            if(candidateFoot._isRightFoot){
                yaw += M_PI_2;
            }
            else{
                yaw -= M_PI_2;
            }
            candidateFoot.supportRegionCenter.position.x = candidateFootPose.position.x + 2.*_d * cos(yaw);
            candidateFoot.supportRegionCenter.position.y = candidateFootPose.position.y + 2.*_d * sin(yaw);
            candidateFoot.supportRegionCenter.position.z = 10.;
            setYawToPose(candidateFoot.supportRegionCenter, getYawFromPose(candidateFootPose) - M_PI_2);
            foundCandidate = true;
            break;
        }
        else{
            mostFarIdxPathPose--;
        }
    }

    if(!foundCandidate){
      candidateFoot._hasError = true;
    }
    // Set prevIdxOnNominalPath for next step
    prevIdxOnNominalPath = currIdxOnNominalPath;
    // Set currIdxOnNominalPath for next step
    currIdxOnNominalPath = mostFarIdxPathPose;

    return candidateFoot;
}

StepPlanner::StepPlanner(double d, double mf, double ms, double mb, double af, double ab,
                         quadmap::QuadTree* _stepable_region) : rightFoot(Foot(true, d, mf, ms, mb, af, ab)), leftFoot(Foot(false, d, mf, ms, mb, af, ab)), stepable_region(_stepable_region) {
}

bool StepPlanner::planning(bool isRightFootMoved, Foot* supportingFoot, nav_msgs::Path& nominalPath,
                           mobile_path_planning::StepsStamped& steps, mobile_path_planning::StepsStamped &supportRegions) {
    int currIdxOnNominalPath = 0;
    int prevIdxOnNominalPath = 0;

    std::vector<Foot> plannedStep;

    //Actual task loop, numPlanningSteps is basically 200
    for(int step = 0; step < numPlanningSteps; step++){

        if(supportingFoot->_hasError){  //Can`t find next step

            //Send error path point
            //this->ErrPathPose = nominalPath.poses.at(currIdxOnNominalPath).pose;
        }
        else{
            // Foot information for executing robot
            mobile_path_planning::Foot foot;
            foot.pose = supportingFoot->pose;
            foot.pose.position.x = foot.pose.position.x;
            foot.pose.position.y = foot.pose.position.y;
            foot.is_right = supportingFoot->_isRightFoot;
            mobile_path_planning::Foot supportRegion;
            supportRegion.pose.position.x = supportingFoot->supportRegionCenter.position.x;
            supportRegion.pose.position.y = supportingFoot->supportRegionCenter.position.y;
            supportRegion.pose.position.z = 10.;
            //std::cout << "noError : " << supportRegion.pose.position.x << ", " << supportRegion.pose.position.y << ", " << supportRegion.pose.position.z << ", " << std::endl;
            supportRegion.pose.orientation.w = supportingFoot->supportRegionCenter.orientation.w;
            supportRegion.pose.orientation.x = supportingFoot->supportRegionCenter.orientation.x;
            supportRegion.pose.orientation.y = supportingFoot->supportRegionCenter.orientation.y;
            supportRegion.pose.orientation.z = supportingFoot->supportRegionCenter.orientation.z;

            steps.steps.push_back(foot);
            plannedStep.push_back(*supportingFoot);
            if((!std::isnan(supportRegion.pose.position.x)) &&
               (!std::isinf(supportRegion.pose.position.x)) &&
               (!std::isnan(supportRegion.pose.position.y)) &&
               (!std::isinf(supportRegion.pose.position.y)))
                supportRegions.steps.push_back(supportRegion);
        }

        geometry_msgs::Pose supportingFootPose = supportingFoot->getPose();
        double stepAreaRatio = supportingFoot->stepAreaRatio;
        // Select which foot should be moved
        if(isRightFootMoved){
            // Move left foot
            leftFoot = Foot(!supportingFoot->_isRightFoot, supportingFoot->_d, supportingFoot->_mf, supportingFoot->_ms, supportingFoot->_mb, supportingFoot->_af, supportingFoot->_ab);
            if(randomizedPlanning){
                *supportingFoot = leftFoot.getNextStep(nominalPath, currIdxOnNominalPath, prevIdxOnNominalPath, supportingFootPose, stepable_region, true);
            }
            else{
                *supportingFoot = leftFoot.getNextStep(nominalPath, currIdxOnNominalPath, prevIdxOnNominalPath, supportingFootPose, stepable_region, false);
            }

            isRightFootMoved = false;
        }
        else{
            // Move right foot
            rightFoot = Foot(!supportingFoot->_isRightFoot, supportingFoot->_d, supportingFoot->_mf, supportingFoot->_ms, supportingFoot->_mb, supportingFoot->_af, supportingFoot->_ab);
            if(randomizedPlanning){
                *supportingFoot = rightFoot.getNextStep(nominalPath, currIdxOnNominalPath, prevIdxOnNominalPath, supportingFootPose, stepable_region, true);
            }
            else{
                *supportingFoot = rightFoot.getNextStep(nominalPath, currIdxOnNominalPath, prevIdxOnNominalPath, supportingFootPose, stepable_region, false);
            }

            isRightFootMoved = true;
        }
        supportingFoot->stepAreaRatio = stepAreaRatio;

        double distance_to_goal = sqrt(pow(nominalPath.poses[currIdxOnNominalPath].pose.position.x - nominalPath.poses[nominalPath.poses.size()-1].pose.position.x, 2) + pow(nominalPath.poses[currIdxOnNominalPath].pose.position.y - nominalPath.poses[nominalPath.poses.size()-1].pose.position.y, 2));
        if(distance_to_goal < 0.2){
            break;
        }
    }

    if((steps.steps.size() > 0) && !(supportingFoot->_hasError) ){
        return true;
    }
    else{
        return false;
    }
}

bool StepPlanner::planning(bool isRightFootMoved, Foot *supportingFoot, nav_msgs::Path &nominalPath,
                           mobile_path_planning::StepsStamped &steps, mobile_path_planning::StepsStamped &supportRegions, int numPlanningSteps) {
    this->numPlanningSteps = numPlanningSteps;

    return this->planning(isRightFootMoved, supportingFoot, nominalPath, steps, supportRegions);
}
