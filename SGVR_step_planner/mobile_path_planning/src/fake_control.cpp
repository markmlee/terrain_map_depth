#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>

// Global variable for subscribe the data
geometry_msgs::Pose current_pose;
geometry_msgs::Pose target_pose;

/*
 * Subscribe the message providing the navigation path, and then copy the next target pose of mobile base
 *
 * _msg: navigation path
 */
void set_target_pose_callback(const nav_msgs::Path::ConstPtr& _msg);

/*
 * Compute the linear and angular velocities to control the mobile base
 *
 * _current_pose: current pose
 * _target_pose:  pose to move toward
 */
geometry_msgs::Twist compute_target_velocity(geometry_msgs::Pose& _current_pose, geometry_msgs::Pose& _target_pose);

/*
 * Control the mobile base in virtual environment, and compute the next pose in a short time
 *
 * _velocity: linear and angular velocity to move the target pose
 * _timestep: time to move the mobile base on the given velocity
 */
geometry_msgs::Pose fake_control_mobile_base(const geometry_msgs::Twist& _velocity, const double _timestep = 0.1);

/*
 * Publish a new pose after fake control
 *
 * _pose_publisher: publisher
 * _current_pose:   new pose
 */
void publish_current_pose(ros::Publisher& _pose_publisher, const geometry_msgs::Pose& _current_pose);

int main(int argc, char** argv){
    ros::init(argc, argv, "fake_control");
    ros::NodeHandle nh;

    // Publisher to provide the data having type "geometry_msgs/PoseStamped"
    ros::Publisher pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("mobile_hubo/current_pose", 1);

    // Subscriber to get the data having type "nav_msgs/Path" or "geometry_msgs/PoseStamped"
    ros::Subscriber path_subscriber = nh.subscribe("mobile_hubo/path", 1, set_target_pose_callback);

    // Publisher
    while(nh.ok()){
        // 1. Get a next target pose
        // Subscriber

        // 2. Compute the linear and angular velocities to control the base
        geometry_msgs::Twist velocity = compute_target_velocity(current_pose, target_pose);

        // 3. Compute the next pose of mobile base after ideal control
        current_pose = fake_control_mobile_base(velocity, 0.1);

        // 4. Publish the new pose
        publish_current_pose(pose_publisher, current_pose);

        ros::spin();
    }

    return 0;
}

void set_target_pose_callback(const nav_msgs::Path::ConstPtr& _msg)
{
    if(_msg->poses.size() <= 0)
        return;

    // TODO: indexing can be different to the order of pose inserted by the path planning algorithm
    // We don't use the nearest target pose in order to move the mobile base fast
    if(_msg->poses.size() > 5)
        target_pose = _msg->poses.at(5).pose;
    else
        target_pose = _msg->poses.end()->pose;
}

geometry_msgs::Twist compute_target_velocity(geometry_msgs::Pose& _current_pose, geometry_msgs::Pose& _target_pose)
{

}

geometry_msgs::Pose fake_control_mobile_base(const geometry_msgs::Twist& _velocity, const double _timestep = 0.1)
{

}

void publish_current_pose(ros::Publisher& _pose_publisher, const geometry_msgs::Pose& _current_pose)
{
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "map";
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.pose = _current_pose;

    _pose_publisher.publish(pose_stamped);
}