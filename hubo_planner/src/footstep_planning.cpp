#include <ros/ros.h>

#include <tf/transform_listener.h>

#include <footstep_planner/footstep_planner.h>
#include <hubo_planner/utils.h>
#include <hubo_planner/visualizer.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <std_msgs/String.h>

// #include <hubo_planner/Footstep.h>
#include <hubo_planner/FootstepsStamped.h>
//rosPODO action msg
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <gogo_gazelle/MotionAction.h>
#include <actionlib/client/terminal_state.h>

#include <gogo_gazelle/update.h>

#define VIS_START_POSE
#define VIS_GOAL_POSE
#define VIS_ENVIRONMENT
#define VIS_STEPPING_STONES
#define VIS_FOOTSTEPS
#define VIS_SUPPORT_REGION

class FootstepPlanningServer {
protected:
    // OPTIONS =========================================================================================================
    const std::string   FIXED_FRAME_ID  = "base_link";
    const std::string   VIS_FRAME_ID    = "map";          // only for visualization

    const double        ROBOT_WIDTH  = 0.7;
    const double        ROBOT_HEIGHT = 0.8;

    //This is for narrowpath

    const double        FOOTSTEP_WIDTH  = 0.25;// 0.25;  // [m]
    const double        FOOTSTEP_HEIGHT = 0.18;//0.18;  // [m]


// This is for stepping stone
/*
    const double        FOOTSTEP_WIDTH  = 0.26;  // [m] X
    const double        FOOTSTEP_HEIGHT = 0.25;  // [m] Y //0.26 was great
*/

    const double        GOAL_SEARCH_MAX_RADIUS = 1.2;
    const double        GOAL_SEARCH_MIN_RADIUS = 0.2;
    const double        GOAL_SEARCH_LEFT_RIGHT = 0.5;
    const double        GOAL_SEARCH_STEP       = 0.02;

public:
    FootstepPlanningServer() : nh(), planner(FOOTSTEP_WIDTH, FOOTSTEP_HEIGHT) {
#ifdef VIS_START_POSE
        start_pose_marker_publisher             = nh.advertise<visualization_msgs::Marker>("hubo_planner/start_pose", 1);
#endif
#ifdef VIS_GOAL_POSE
        goal_pose_marker_publisher              = nh.advertise<visualization_msgs::Marker>("hubo_planner/goal_pose", 1);
#endif
#ifdef VIS_SUPPORT_REGION
        support_region_marker_publisher         = nh.advertise<visualization_msgs::Marker>("hubo_planner/support_region", 1);
#endif
#ifdef VIS_ENVIRONMENT
        environment_markerarray_publisher       = nh.advertise<visualization_msgs::MarkerArray>("hubo_planner/environment", 1);
#endif
#ifdef VIS_STEPPING_STONES
        stepping_stones_markerarray_publisher   = nh.advertise<visualization_msgs::MarkerArray>("hubo_planner/stepping_stones", 1);
#endif
#ifdef VIS_FOOTSTEPS
        footstep_markerarray_publisher          = nh.advertise<visualization_msgs::MarkerArray>("hubo_planner/footstep", 1);
#endif

        footsteps_publisher        = nh.advertise<hubo_planner::FootstepsStamped>("hubo_planner/footsteps_stamped", 1);
        environment_subscriber     = nh.subscribe("environment", 1, &FootstepPlanningServer::set_environment, this);
        stepping_stones_subscriber = nh.subscribe("stepping_stones", 1, &FootstepPlanningServer::set_stepping_stones, this);
        begin_subscriber           = nh.subscribe("begin_footstep_planning", 1, &FootstepPlanningServer::begin_footstep_planning, this);
        result_subscriber          = nh.subscribe("walking/result", 1, &FootstepPlanningServer::goal_result_callback, this);
        support_step_subscriber    = nh.subscribe("robot_states", 1, &FootstepPlanningServer::support_step_callback,this);

        robot_size      = quadmap::point2d((float)ROBOT_WIDTH, (float)ROBOT_HEIGHT);
        footstep_size   = quadmap::point2d((float)FOOTSTEP_WIDTH, (float)FOOTSTEP_HEIGHT);

        start_pose      = new Configuration(0.0, 0.0, M_PI);
        goal_pose       = nullptr;
        environment     = nullptr;
        stepping_stones = nullptr;

        begin_flag           = false;
        environment_flag     = false;
        stepping_stones_flag = false;

        target_num_footsteps = 7; //TODO : This value means that how many footsteps you want to obtain from planner.

#ifdef VIS_START_POSE
        update_tf();
        Configuration* transformed_start_pose = start_pose->get_transformed_Configuration(tf_transform_baselink2map);  //transform
        RobotModel robot_model(quadmap::point2d(start_pose->x(), start_pose->y()), robot_size, start_pose->r());
        visualization::robot_model_to_marker(robot_model, start_pose_marker, VIS_FRAME_ID);
        delete transformed_start_pose;
#endif
    }
    ~FootstepPlanningServer() {
        delete start_pose;
        delete goal_pose;
        delete environment;
        delete stepping_stones;
    }

    /*
     *
     */
    void begin_footstep_planning(const std_msgs::String::ConstPtr& _msg){
        begin_flag = true;
    }


    //update flag from /Result topic
    bool first_command_flag = true;
    bool first_result_flag = true;
    void goal_result_callback(const gogo_gazelle::MotionActionResultConstPtr &result) {

        // TODO : Requirement2
        delete start_pose;
        update_tf();
        start_pose = Configuration(0, 0, M_PI).get_transformed_Configuration(tf_transform_baselink2map);           //  fixed_frame : map


        if (first_command_flag) {
            ROS_INFO("ignore first result");
            first_command_flag = false;
        }
        else {
            ROS_INFO("Start Planning!");
            ROS_INFO("Next Footstep : %s", (planner.get_current_footstep()) ? "left" : "right");
            ROS_INFO("Target number of footstep : %d", target_num_footsteps);
            begin_flag = true;
        }
//#ifdef VIS_FOOTSTEPS
//        if(!footstep_markerarray.markers.empty() && footstep_markerarray_publisher.getNumSubscribers() > 0) {
//            footstep_markerarray_publisher.publish(footstep_markerarray);
//        }
//#endif
//        if(!footsteps_stamped.steps.empty()) {
//            footsteps_publisher.publish(footsteps_stamped);
//        }
    }
    void support_step_callback(const gogo_gazelle::updateConstPtr &robot_state){
        // current_support_step is right
        if(robot_state->step_phase % 2 == 0){
            planner.set_current_footstep(true);
        }
        // current_support_step is left
        else {
            planner.set_current_footstep(false);
        }
    }

    /*
     *
     */
    void set_environment(const octomap_msgs::Octomap::ConstPtr& _msg) {
        // Only if begin_flag is true, this function is executed
//        if(!begin_flag || environment_flag)
//            return;
        if(environment_flag)
            return;

        auto environment_3d = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(*_msg);

        delete environment;
        environment = new quadmap::QuadTree(environment_3d->getResolution());
        utils::project_environment(environment_3d, environment, 0.1, 5.0);
        planner.set_environment(environment);
//        std::cout << "Set environment: " << environment->size() << std::endl;

#ifdef VIS_ENVIRONMENT
        visualization::environment_to_markerarray(environment, environment_markerarray, VIS_FRAME_ID);
#endif

        delete environment_3d;

        environment_flag = true;
    }
    /*
     *
     */
    void set_stepping_stones(const octomap_msgs::Octomap::ConstPtr& _msg) {
        // Only if (begin_flag && environment_flag) is true, this function is executed
//        if(!begin_flag || stepping_stones_flag)
//            return;
        if(stepping_stones_flag)
            return;
//      std::cout << "inside callback "  << std::endl;
        auto stepping_stones_3d = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(*_msg);
//        auto stepping_stones_3d = (octomap::OcTree*)octomap_msgs::fullMsgToMap(*_msg);
//        std::cout << "OctoMap Size: " << stepping_stones_3d->size() << std::endl;

        delete stepping_stones;
        stepping_stones = new quadmap::QuadTree(stepping_stones_3d->getResolution());
        utils::project_environment(stepping_stones_3d, stepping_stones, -0.5, 0.1);
        planner.set_stepping_stones(stepping_stones);
        //std::cout << "Set stepping stones: " << stepping_stones->size() << std::endl;

#ifdef VIS_STEPPING_STONES
        visualization::stepping_stones_to_markerarray(stepping_stones, stepping_stones_markerarray, VIS_FRAME_ID);
#endif

        delete stepping_stones_3d;

        stepping_stones_flag = true;
    }

    /*
     *
     */
    bool planning() {

//      if(stepping_stones == nullptr || start_pose == nullptr || goal_pose == nullptr) TODO : original
//            return false;
      if(stepping_stones == nullptr || start_pose == nullptr)
            return false;


        ros::Time start = ros::Time::now();
//        bool found = planner.planning(*start_pose, *goal_pose);   TODO: original
        bool found = planner.planning(*start_pose, target_num_footsteps);


#ifdef VIS_SUPPORT_REGION
        // start footstep
        FootstepNode* start_footstep = planner.get_current_footstep_node();
        //::cout << "Start footstep plan: " << (start_footstep->isRight ? "Left" : "Right") << std::endl;
        //std::cout << "Start footstep conf: " << start_footstep->step_conf.x() << " , " << start_footstep->step_conf.y() << std::endl;
        // Support region
        quadmap::point2d support_region_viz_size = quadmap::point2d((float)planner.SUPPORT_REGION_MAX_X-(float)planner.SUPPORT_REGION_MIN_X+(float)FOOTSTEP_WIDTH,
                                                                    (float)planner.SUPPORT_REGION_MAX_Y-(float)planner.SUPPORT_REGION_MIN_Y+(float)FOOTSTEP_HEIGHT);
        float support_region_viz_center_x;
        float support_region_viz_center_y;
        if(start_footstep->isRight){
             support_region_viz_center_x = -planner.SUPPORT_REGION_MIN_X + start_pose->x() - support_region_viz_size.x() / 2.0;
             support_region_viz_center_y = -planner.SUPPORT_REGION_MIN_Y + start_pose->y() - support_region_viz_size.y() / 2.0;
        }
        else{
            support_region_viz_center_x = -planner.SUPPORT_REGION_MIN_X + start_pose->x() - support_region_viz_size.x() / 2.0;
            support_region_viz_center_y = planner.SUPPORT_REGION_MIN_Y + start_pose->y() + support_region_viz_size.y() / 2.0;
        }
        // visualization data
        OBB support_region_model(quadmap::point2d(support_region_viz_center_x, support_region_viz_center_y), support_region_viz_size, start_footstep->step_conf.r());
        visualization::robot_model_to_marker(support_region_model, support_region_marker, VIS_FRAME_ID);
        delete start_footstep;
#endif

        if(!found) {
            //ros::Time end = ros::Time::now();
            //std::cout << "Cannot find the goal: " << (end - start).toSec() << " [sec]" << std::endl;

            return false;
        }
        ros::Time end = ros::Time::now();
       //std::cout << "Found the goal: " << (end - start).toSec() << " [sec]" << std::endl;
        ROS_INFO("Number of footsteps: %d", static_cast<int>(planner.get_footsteps().size()));

#ifdef VIS_FOOTSTEPS
        visualization::footsteps_to_markerarray(planner.get_footsteps(), footstep_size, footstep_markerarray, VIS_FRAME_ID);
#endif

        // Generate the footsteps message
        planner.transform_footsteps(tf_transform_map2baselink);
        footsteps_to_message(planner.get_footsteps());

        return true;
    }

    /*
     *
     */

    bool planning_with_goal_search() {

      if(begin_flag){

        int goal_search_count = 0;

        ros::Time start = ros::Time::now();

        for(float goal_search_radius = GOAL_SEARCH_MAX_RADIUS; goal_search_radius > GOAL_SEARCH_MIN_RADIUS ; goal_search_radius-=GOAL_SEARCH_STEP){
            float y    = 0.0;     // to search left,right direction
            float sign = 1.0;     // to calculate y

            for(float goal_search_left_right = 0.0; goal_search_left_right <= 2*GOAL_SEARCH_LEFT_RIGHT; goal_search_left_right+=GOAL_SEARCH_STEP){
                y += sign * goal_search_left_right;
                sign *= -1.0;     // change the sign

                delete start_pose;
                delete goal_pose;

                goal_search_count++;

                update_tf();
                start_pose = Configuration(0, 0, M_PI).get_transformed_Configuration(tf_transform_baselink2map);           //  fixed_frame : map
                goal_pose = Configuration(-goal_search_radius, y, M_PI - y/goal_search_radius).get_transformed_Configuration(tf_transform_baselink2map);  // fixed_frame : map
//                std::cout << "Start pose: " << start_pose->x() << " / " << start_pose->y() << " / " << start_pose->r() << std::endl;
//                std::cout << "Goal pose: " << goal_pose->x() << " / " << goal_pose->y() << " / " << goal_pose->r() << std::endl;


                if(planning()) {
                    std::cout << "Goal pose: " << goal_pose->x() << " / " << goal_pose->y() << " / " << goal_pose->r() << std::endl;
                    ros::Time end = ros::Time::now();
                    //std::cout << "" << (end - start).toSec() << " [sec]" << std::endl;
                    ROS_WARN("total plan time: %f [sec]", (end - start).toSec());
                    std::cout << "Goal search: " << goal_search_count << " [iteration]" << std::endl;
                    std::cout<<"---------------------------------------------------------"<<std::endl;


#ifdef VIS_START_POSE
                    RobotModel start_robot_model(quadmap::point2d(start_pose->x(), start_pose->y()), robot_size, start_pose->r());
                    visualization::robot_model_to_marker(start_robot_model, start_pose_marker, VIS_FRAME_ID);
#endif
#ifdef VIS_GOAL_POSE
//                    Configuration* goal_pose_baselink = goal_pose->get_transformed_Configuration(tf_transform_map2baselink);
                    RobotModel goal_robot_model(quadmap::point2d(goal_pose->x(), goal_pose->y()), robot_size, goal_pose->r());
                    visualization::robot_model_to_marker(goal_robot_model, goal_pose_marker, VIS_FRAME_ID);
#endif
#ifdef VIS_FOOTSTEPS
                    if(!footstep_markerarray.markers.empty() && footstep_markerarray_publisher.getNumSubscribers() > 0) {
                      footstep_markerarray_publisher.publish(footstep_markerarray);
                    }
#endif
                    if(!footsteps_stamped.steps.empty()) {
                      footsteps_publisher.publish(footsteps_stamped);
                    }
                    begin_flag = false;

                    return true;
                }


            }
        }

        ROS_ERROR("Cannot find the goal pose and the footsteps.");
        //std::cout << "Cannot find the goal pose and the footsteps." << std::endl;
        begin_flag = false;
        return false;
      }
    }
    /*
     *
     */
    bool planning_target_num_footsteps(){

        if (begin_flag){
            ros::Time start = ros::Time::now();

//            TODO : remove this for requirement2
            delete start_pose;
            update_tf();
            start_pose = Configuration(0, 0, M_PI).get_transformed_Configuration(tf_transform_baselink2map);           //  fixed_frame : map

            if(planning()) {

//                auto goal_step = planner.get_footsteps().front();
//                delete goal_pose;
//                if((planner.get_current_footstep()+target_num_footsteps) % 2 == 0)
//                    goal_pose = Configuration(goal_step.x(), goal_step.y()+0.7, goal_step.r()+M_PI).get_transformed_Configuration(tf_transform_baselink2map);  // fixed_frame : map
//                else
//                    goal_pose = Configuration(goal_step.x(), goal_step.y(), goal_step.r()+M_PI).get_transformed_Configuration(tf_transform_baselink2map);  // fixed_frame : map

                //std::cout << "Goal pose: " << goal_pose->x() << " / " << goal_pose->y() << " / " << goal_pose->r() << std::endl;
                ros::Time end = ros::Time::now();
                //std::cout << "" << (end - start).toSec() << " [sec]" << std::endl;
                ROS_INFO("total plan time: %f [sec]", (end - start).toSec());
                std::cout<<"---------------------------------------------------------"<<std::endl;


#ifdef VIS_START_POSE
                RobotModel start_robot_model(quadmap::point2d(start_pose->x(), start_pose->y()), robot_size, start_pose->r());
                visualization::robot_model_to_marker(start_robot_model, start_pose_marker, VIS_FRAME_ID);
#endif
//#ifdef VIS_GOAL_POSE
//                RobotModel goal_robot_model(quadmap::point2d(goal_pose->x(), goal_pose->y()), robot_size, goal_pose->r());
//                visualization::robot_model_to_marker(goal_robot_model, goal_pose_marker, VIS_FRAME_ID);
//#endif
#ifdef VIS_FOOTSTEPS
                if(!footstep_markerarray.markers.empty() && footstep_markerarray_publisher.getNumSubscribers() > 0) {
                    footstep_markerarray_publisher.publish(footstep_markerarray);
                }
#endif
                if(!footsteps_stamped.steps.empty()) {
                    footsteps_publisher.publish(footsteps_stamped);
                }
                begin_flag = false;
                if(target_num_footsteps >=2 )
                  target_num_footsteps--;
                return true;
            }
            ROS_ERROR("Cannot find the goal pose and the footsteps.");
        }
    }
    /*
     *
     */
    void publish_messages() {
#ifdef VIS_START_POSE
        if(start_pose != nullptr && start_pose_marker_publisher.getNumSubscribers() > 0) {
            start_pose_marker_publisher.publish(start_pose_marker);
        }
#endif
#ifdef VIS_GOAL_POSE
        if(goal_pose != nullptr && goal_pose_marker_publisher.getNumSubscribers() > 0) {
            goal_pose_marker_publisher.publish(goal_pose_marker);
        }
#endif
#ifdef VIS_SUPPORT_REGION
        if(support_region_marker_publisher.getNumSubscribers() > 0) {
            support_region_marker_publisher.publish(support_region_marker);
        }
#endif
#ifdef VIS_ENVIRONMENT
        if(environment != nullptr && environment_markerarray_publisher.getNumSubscribers() > 0) {
            environment_markerarray_publisher.publish(environment_markerarray);
        }
#endif
#ifdef VIS_STEPPING_STONES
        if(stepping_stones != nullptr && stepping_stones_markerarray_publisher.getNumSubscribers() > 0) {
            stepping_stones_markerarray_publisher.publish(stepping_stones_markerarray);
        }
#endif
//#ifdef VIS_FOOTSTEPS
//        if(!footstep_markerarray.markers.empty() && footstep_markerarray_publisher.getNumSubscribers() > 0) {
//            footstep_markerarray_publisher.publish(footstep_markerarray);
//        }
//#endif
//
//        if(!footsteps_stamped.steps.empty()) {
//            footsteps_publisher.publish(footsteps_stamped);
//        }
    }

    /*
     *
     */
    void set_current_footstep_configuration() {
        const Configuration& footstep_configuration = planner.get_next_footstep();
        bool is_right = planner.is_next_footstep_right();
        planner.set_current_footstep_configuration(footstep_configuration, is_right);
    }
    /*
     *
     */
    bool footsteps_to_message(const std::vector<Configuration>& _footsteps, const ros::Time& _time = ros::Time::now()) {
        if(planner.get_footsteps().size() <= 0)
            return false;

        footsteps_stamped.header.frame_id = FIXED_FRAME_ID;
        footsteps_stamped.header.stamp = _time;
        footsteps_stamped.steps.clear();
        footsteps_stamped.steps.reserve(_footsteps.size());

        bool is_current_right = !planner.is_next_footstep_right();
        for(int i = (int)_footsteps.size()-1; i >= 0; i--) {
            hubo_planner::Footstep footstep_msg;

            footstep_msg.is_right = is_current_right;
            footstep_msg.pose.position.x = _footsteps[i].x();
            footstep_msg.pose.position.y = _footsteps[i].y();
            footstep_msg.pose.position.z = 0.0;

            octomath::Quaternion q(0.0f, 0.0f, _footsteps[i].r());
            footstep_msg.pose.orientation.x = q.x();
            footstep_msg.pose.orientation.y = q.y();
            footstep_msg.pose.orientation.z = q.z();
            footstep_msg.pose.orientation.w = q.u();
            footsteps_stamped.steps.push_back(footstep_msg);

            is_current_right = !is_current_right;

            //std::cout << "i: " << i << "y: " << _footsteps[i].y() << std::endl;
        }

        return true;
    }

    bool isValid_flag(){
        //return begin_flag && environment_flag && stepping_stones_flag;
        //return begin_flag && stepping_stones_flag;  TODO
        //return begin_flag;
//        return environment_flag && stepping_stones_flag;
        return stepping_stones_flag;
    }

    bool reset_flag(){
//        begin_flag = false;
        environment_flag = false;
        stepping_stones_flag = false;
    }
    void update_tf(){
        try{
            tf_listener.lookupTransform("/map", "/base_link", ros::Time(0), tf_transform_baselink2map);
            tf_listener.lookupTransform("/base_link", "/map",  ros::Time(0), tf_transform_map2baselink);
        }
        catch (tf::TransformException ex){
            ROS_WARN("%s",ex.what());
        }
    }
protected:
    Configuration*          start_pose;
    Configuration*          goal_pose;
    // hubo_planner::Footstep  next_footstep;
    hubo_planner::FootstepsStamped  footsteps_stamped;

    quadmap::point2d        robot_size;
    quadmap::point2d        footstep_size;

    quadmap::QuadTree*      environment;
    quadmap::QuadTree*      stepping_stones;

    FootstepPlanner         planner;

    ros::NodeHandle         nh;
    ros::Subscriber         environment_subscriber;
    ros::Subscriber         stepping_stones_subscriber;
    ros::Subscriber         begin_subscriber;
    ros::Subscriber         result_subscriber;
    ros::Subscriber         support_step_subscriber;
    ros::Publisher          footsteps_publisher;

    tf::TransformListener   tf_listener;
    tf::StampedTransform    tf_transform_baselink2map;
    tf::StampedTransform    tf_transform_map2baselink;

    bool                    begin_flag;
    bool                    environment_flag;
    bool                    stepping_stones_flag;

    int                     target_num_footsteps;

#ifdef VIS_START_POSE
    ros::Publisher              start_pose_marker_publisher;
    visualization_msgs::Marker  start_pose_marker;
#endif
#ifdef VIS_GOAL_POSE
    ros::Publisher              goal_pose_marker_publisher;
    visualization_msgs::Marker  goal_pose_marker;
#endif
#ifdef VIS_ENVIRONMENT
    ros::Publisher                  environment_markerarray_publisher;
    visualization_msgs::MarkerArray environment_markerarray;
#endif
#ifdef VIS_STEPPING_STONES
    ros::Publisher                  stepping_stones_markerarray_publisher;
    visualization_msgs::MarkerArray stepping_stones_markerarray;
#endif
#ifdef VIS_FOOTSTEPS
    ros::Publisher                  footstep_markerarray_publisher;
    visualization_msgs::MarkerArray footstep_markerarray;
#endif
#ifdef VIS_SUPPORT_REGION
    ros::Publisher                  support_region_marker_publisher;
    visualization_msgs::Marker      support_region_marker;
#endif
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "footstep_planning");
    ros::NodeHandle nh;

    FootstepPlanningServer footstep_planning_server;
    ros::Rate loop_rate(100);
    ROS_ERROR("new version w 26/25");

    try {
        while (true) {
            // Begin the footstep planning
            if (footstep_planning_server.isValid_flag()) {

                //ros::Time start = ros::Time::now();
                //footstep_planning_server.planning_with_goal_search();
                footstep_planning_server.planning_target_num_footsteps();
                footstep_planning_server.reset_flag();
                //ros::Time end = ros::Time::now();

                //std::cout<<"---------------------------------------------------------"<<std::endl;
//                std::cout << "" << (end - start).toSec() << " [sec]" << std::endl;
                footstep_planning_server.publish_messages();
//                ROS_WARN("published footstep");   TODO
            }
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // If the robot move the foot to the next stamp successfully,
            // change "current_footstep_configuration" and "is_current_footstep_right" of footstep planner
            // before next footstep planning. Refer "set_current_footstep_configuration()" function above."
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////

            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    catch (std::runtime_error &e) {
        ROS_ERROR("Exception: %s", e.what());
        return -1;
    }

    return 0;
}
