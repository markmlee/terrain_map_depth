#include <ros/ros.h>

#include <hubo_planner/FootstepsStamped.h>

#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>

class FootstepSubscriber {
public:
    FootstepSubscriber() : nh() {
        // footstep_markerarray_subscriber = nh.subscribe("hubo_planner/footstep", 1, &FootstepSubscriber::subscribe_markerarray, this);
        footsteps_stamped_subscriber    = nh.subscribe("hubo_planner/footsteps_stamped", 1, &FootstepSubscriber::subscribe_foosteps_stamped, this);
    }

    void subscribe_markerarray(const visualization_msgs::MarkerArray::ConstPtr& _markerarray) {
        std::cout << "Marker Size: " << _markerarray->markers.size() << std::endl;
        for(unsigned int i = 0; i < _markerarray->markers.size(); i++) {
            const auto& marker = _markerarray->markers[i];
            if(marker.action == visualization_msgs::Marker::ADD) {
                std::cout << "ID: " << marker.id << std::endl;
                std::cout << "\tX: "  << marker.pose.position.x << " // ";
                std::cout << "Y: "  << marker.pose.position.y << " // ";

                tf::Quaternion q(
                        marker.pose.orientation.x,
                        marker.pose.orientation.y,
                        marker.pose.orientation.z,
                        marker.pose.orientation.w);
                tf::Matrix3x3 m(q);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);
                std::cout << "R: "  << yaw << std::endl;
            }
        }
    }

    void subscribe_foosteps_stamped(const hubo_planner::FootstepsStamped::ConstPtr& _footsteps_stamped) {
        std::cout << "Footstep Size: " << _footsteps_stamped->steps.size() << std::endl;
        for(unsigned int i = 0; i < _footsteps_stamped->steps.size(); i++) {
            std::cout << "\tID: " << i << "  " << "(" << (_footsteps_stamped->steps[i].is_right ? "Right" : "Left") << ")" << std::endl;
            std::cout << "\tX: "  << _footsteps_stamped->steps[i].pose.position.x << " // ";
            std::cout << "Y: "  << _footsteps_stamped->steps[i].pose.position.y << " // ";

            tf::Quaternion q(
                    _footsteps_stamped->steps[i].pose.orientation.x,
                    _footsteps_stamped->steps[i].pose.orientation.y,
                    _footsteps_stamped->steps[i].pose.orientation.z,
                    _footsteps_stamped->steps[i].pose.orientation.w);
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            std::cout << "R: "  << yaw << std::endl;
        }
    }


protected:
    ros::NodeHandle         nh;
    ros::Subscriber         footstep_markerarray_subscriber;
    ros::Subscriber         footsteps_stamped_subscriber;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "footstep_subscribe_example");
    ros::NodeHandle nh;

    FootstepSubscriber footstep_subscriber;

    try{
        while(true) {
            ros::spinOnce();
        }
    }
    catch (std::runtime_error& e){
        ROS_ERROR("Exception: %s", e.what());
        return -1;
    }

    return 0;
}

