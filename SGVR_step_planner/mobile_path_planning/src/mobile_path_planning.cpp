#include <ros/ros.h>

#include <tf2_msgs/TFMessage.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <mobile_path_planning/path_planner.h>
#include <mobile_path_planning/step_planner.h>

#include <quadmap/conversion.h>
#include <tf/transform_listener.h>

// Global variable for subscribing the data
geometry_msgs::Pose start_pose;
geometry_msgs::Pose goal_pose;
geometry_msgs::Pose err_pose[20];
octomap::OcTree* environment; // contains obstacle information
octomap::OcTree* stepable_region; // contains stepable information, in this case, occupied means stepable.

bool required_plan = false;

void set_start_pose(const tf2_msgs::TFMessage::ConstPtr& _msg); // set start pose, which is current robot pose
void set_goal_pose(const geometry_msgs::PoseStamped::ConstPtr& _msg);  // set goal pose. This callback function not yet used by the planner. The goal pose needs to be published at some where. We hard coded the goal pose at line 33~34.
void set_octomap(const octomap_msgs::Octomap::ConstPtr& _msg);
void set_stepable_region(const octomap_msgs::Octomap::ConstPtr& _msg);

bool project_environment(octomap::OcTree* _environment_3D, quadmap::QuadTree* _environment_2D, float MINIMUM_HEIGHT, float MAXIMUM_HEIGHT); //convert 3D octomap to 2D quadmap

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mobile_path_planning");
    ros::NodeHandle nh;

    // Initialize the global variables
    start_pose.orientation.w = 1.0;
    // Goal pose is set as arbitrary values
    goal_pose.orientation.w = 1.0;
    goal_pose.position.x = 0.0;
    goal_pose.position.y = 0.0;
    stepable_region = nullptr;
    environment = nullptr;

    // Publish output of path and step planning
    ros::Publisher path_publisher = nh.advertise<nav_msgs::Path>("mobile_hubo/navigation_path", 1);
    ros::Publisher steps_publisher = nh.advertise<mobile_path_planning::StepsStamped>("mobile_hubo/steps", 1);
    ros::Publisher marker_publisher = nh.advertise<visualization_msgs::MarkerArray>("mobile_hubo/step_markers", 1);
    ros::Publisher marker_publisher2 = nh.advertise<visualization_msgs::MarkerArray>("mobile_hubo/support_region_markers", 1);

    // Subscribe the current pose, goal pose, obstacles map and stepable region
    ros::Subscriber start_pose_subscriber = nh.subscribe("/tf", 1, set_start_pose);
    ros::Subscriber goal_pose_subscriber = nh.subscribe("move_base_simple/goal", 1, set_goal_pose);
    ros::Subscriber map_subscriber = nh.subscribe("mobile_hubo/environment", 1, set_octomap);
    ros::Subscriber stepable_region_subscriber = nh.subscribe("mobile_hubo/stepable", 1, set_stepable_region);

    // Default setting of support region of step planning
    /*************************************************************
    * d : Displacement between center of body and center of foot.
    * mf : Front point of support region
    * ms : Side point of support region
    * mb : Back point of support region
    * af : Front angle of inner support region
    * ab : Back angle of inner support region
    **************************************************************/
    float d = 0.2;
    float mf = 0.7;
    float ms = 0.3;
    float mb = 0.2;
    float af = M_PI/12.;
    float ab = M_PI_4;

    /********************************** Main sequence ***********************************/
    ros::Rate rate(1);//1Hz
    while(nh.ok()){
        // wait until the 'environment' and 'stepable_region' are set
        if(environment == nullptr || stepable_region == nullptr || !required_plan){
            ros::spinOnce();
            continue;
        }

        // 1. Prepare a data for path planning
        ros::Time start = ros::Time::now();

        quadmap::QuadTree* environment_2D = new quadmap::QuadTree(environment->getResolution());
        project_environment(environment, environment_2D, 0.1, 5.0);
        quadmap::QuadTree* stepable_region_2D = new quadmap::QuadTree(stepable_region->getResolution());
        project_environment(stepable_region, stepable_region_2D, 0.f, 1.f);

        // 2. Path planning
        nav_msgs::Path path;
        mobile_path_planning::StepsStamped steps;
        mobile_path_planning::StepsStamped supportRegions;
        AstarPlanner path_planner(environment_2D, RobotModel(0.0, 0.0, 0.25)); // RobotModel : coordinate (0,0), radius 0.25m
        path_planner.planning(start_pose, goal_pose, path);
        ros::Time end = ros::Time::now();

        double path_planning_time = (end - start).toSec() * 1000;
        //std::cout << "Planning Time: " << path_planning_time << " [ms]" << std::endl;
        std::cout << "Path size: " << path.poses.size() << std::endl;

        // if global path is found
        if(path.poses.size() > 0){
            // 2-2. Foot step planning
            ros::Time step_start = ros::Time::now();
            // Generate foot instance, whenever the path is found, we set right foot as supporting foot.
            Foot starting_foot(false, d, mf, ms, mb, af, ab);
            // Align starting foot with start position of path.
            starting_foot.setPoseAccordingToPathPose(path.poses.at(0).pose);

            // Generate step planner. it has support region information(d, ..., ab) and stepable region information.
            StepPlanner step_planner(d, mf, ms, mb, af, ab, stepable_region_2D);


            bool _isStepPathExist = true;
            int numPlanningSteps; //If number of planning step is specified, plan the step with the value. If not, plan with default number of steps, default is 200

                // Step path planning
            starting_foot.setPoseAccordingToPathPose(path.poses.at(0).pose);//Initiate starting foot
            ros::param::get("/mobile_path_planning/step_area_ratio", starting_foot.stepAreaRatio);
            ros::param::get("/mobile_path_planning/step_randomize_value", step_planner.stepRandomizeValue);
            ros::param::get("/mobile_path_planning/randomized_planning", step_planner.randomizedPlanning);
            if(ros::param::get("/mobile_path_planning/num_planning_steps", numPlanningSteps)){
                //ROS_INFO("Got param num_planning_steps : %d", numPlanningSteps);
                _isStepPathExist = step_planner.planning(true, &starting_foot, path, steps, supportRegions, numPlanningSteps);
            }
            else{
                _isStepPathExist = step_planner.planning(true, &starting_foot, path, steps, supportRegions);
            }

            steps.header = path.header;
            ros::Time step_end = ros::Time::now();

            double step_planning_time = (step_end - step_start).toSec() * 1000;
            std::cout << "Step Planning Time: " << step_planning_time << " [ms]" << std::endl;
            std::cout << "Step size: " << steps.steps.size() << std::endl;
            ROS_INFO("Step Planning Time : %f", step_planning_time);
        }

        delete environment_2D;

        // 3. Publish the path
        if(path.poses.size() > 0){
            path_publisher.publish(path);
        }
        if(steps.steps.size() > 0){
            steps_publisher.publish(steps);
        }

        // 4. Publish markers for visualization


        // Marker for foot step visualization
        visualization_msgs::MarkerArray markerArray;
        int numPlanningSteps;
        if(!ros::param::get("/mobile_path_planning/num_planning_steps", numPlanningSteps)){
            numPlanningSteps = 50;
        }
        for(int i = 0; i < numPlanningSteps; i++){
            visualization_msgs::Marker marker;
            marker.header.stamp = ros::Time::now();
            marker.ns = "foot_step";
            marker.id = i;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::DELETEALL;
            marker.lifetime = ros::Duration();

            markerArray.markers.push_back(marker);
        }

        for(int i = 0; i < steps.steps.size(); i++){
            visualization_msgs::Marker marker;
            marker.header = path.header;
            marker.ns = "foot_step";
            marker.id = i;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose = steps.steps.at(i).pose;
            marker.scale.x = 0.2;
            marker.scale.y = 0.15;
            marker.scale.z = 0.01;
            marker.color.r = 1.f * (1. - float(i) / steps.steps.size());
            marker.color.g = 0.f;
            marker.color.b = 1.f * (float(i) / steps.steps.size());
            marker.color.a = 1.f;
            marker.lifetime = ros::Duration();

            markerArray.markers.push_back(marker);
        }
        marker_publisher.publish(markerArray);


        //ros::spinOnce();


        // Marker for foot step visualization
        visualization_msgs::MarkerArray markerArray2;
        int numPlanningSteps2;
        if(!ros::param::get("/mobile_path_planning/num_planning_steps", numPlanningSteps2)){
            numPlanningSteps2 = 50;
        }
        for(int i = 0; i < numPlanningSteps2; i++){
            visualization_msgs::Marker marker;
            marker.header.stamp = ros::Time::now();
            marker.ns = "foot_step";
            marker.id = i + steps.steps.size();
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::DELETEALL;
            marker.lifetime = ros::Duration();

            markerArray2.markers.push_back(marker);
        }

        for(int i = 0; i < supportRegions.steps.size(); i++){
            visualization_msgs::Marker marker;
            marker.header = path.header;
            marker.ns = "support_regions";
            marker.id = i + steps.steps.size();
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose = supportRegions.steps.at(i).pose;
            if(std::isnan(marker.pose.position.x) ||
               std::isinf(marker.pose.position.x) ||
               std::isnan(marker.pose.position.y) ||
               std::isinf(marker.pose.position.y) ||
                std::isnan(marker.pose.position.z) ||
                std::isinf(marker.pose.position.z)){
                std::cout << marker.pose.position.x << ", " << marker.pose.position.y << ", " << marker.pose.position.z << ", " << std::endl;
            }
            //marker.pose.position.z = -0.01;
            marker.scale.x = ms*2;
            marker.scale.y = mf*2;
            marker.scale.z = 0.01;
            marker.color.r = 1.f * (1. - float(i) / supportRegions.steps.size());
            marker.color.g = 0.f;
            marker.color.b = 1.f * (float(i) / supportRegions.steps.size());
            marker.color.a = 0.4f;
            marker.lifetime = ros::Duration();

            markerArray2.markers.push_back(marker);
        }
        marker_publisher2.publish(markerArray2);



        ros::spinOnce();
    }

    /************************************************************************************/
    return 0;
}

void set_start_pose(const tf2_msgs::TFMessage::ConstPtr& _msg)
{
    // Convert tf2_msgs to geometry_msgs::Pose
    for(int i = 0; i < _msg->transforms.size(); i++){
        if(std::string("base_link_estimate").compare(_msg->transforms.at(i).child_frame_id) == 0){
            start_pose.position.x = _msg->transforms.at(i).transform.translation.x;
            start_pose.position.y = _msg->transforms.at(i).transform.translation.y;
            start_pose.position.z = 10.0;

            start_pose.orientation.w = _msg->transforms.at(i).transform.rotation.w;
            start_pose.orientation.x = _msg->transforms.at(i).transform.rotation.x;
            start_pose.orientation.y = _msg->transforms.at(i).transform.rotation.y;
            start_pose.orientation.z = _msg->transforms.at(i).transform.rotation.z;

            break;
        }
    }

//    std::cout << "Set start pose" << std::endl;
}

void set_goal_pose(const geometry_msgs::PoseStamped::ConstPtr& _msg){
    goal_pose = _msg->pose;
    required_plan = true;
    std::cout << "Set goal pose" << std::endl;
}

void set_octomap(const octomap_msgs::Octomap::ConstPtr& _msg){
    delete environment;

    environment = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(*_msg);
//    std::cout << "Set octomap" << std::endl;
}

void set_stepable_region(const octomap_msgs::Octomap::ConstPtr& _msg){
    delete stepable_region;

    stepable_region = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(*_msg);
}

bool project_environment(octomap::OcTree* _environment_3D, quadmap::QuadTree* _environment_2D, float MINIMUM_HEIGHT, float MAXIMUM_HEIGHT){
    if(_environment_3D == nullptr)
        return false;

    _environment_3D->expand();
    for(octomap::OcTree::leaf_iterator it = _environment_3D->begin_leafs(); it != _environment_3D->end_leafs(); it++){
        octomap::point3d point = _environment_3D->keyToCoord(it.getKey());
        if(point.z() < MINIMUM_HEIGHT && point.z() > MAXIMUM_HEIGHT)
            continue;
        _environment_2D->updateNode(point.x(), point.y(), true);
    }

    return true;
}
