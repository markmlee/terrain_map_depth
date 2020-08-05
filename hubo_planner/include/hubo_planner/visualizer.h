#ifndef HUBO_PLANNER_VISUALIZER_H
#define HUBO_PLANNER_VISUALIZER_H

#include <collision_detector/collision_detector.h>
#include <visualization_msgs/MarkerArray.h>

typedef OBB RobotModel;

namespace visualization {
    // OPTIONS =========================================================================================================
    const int    MAX_NUM_OF_MARKERS = 50;
    const double VIS_HEIGHT_MARKER  = 0.3;

    /*
     *
     */
    void robot_model_to_marker(const RobotModel& _robot_model, visualization_msgs::Marker& _marker,
                               const std::string& _FIXED_FRAME_ID = "map", const std::string& _color = "b",const ros::Time& _time = ros::Time::now(), const int& _id = 0) {
        // Message header
        _marker.header.frame_id = _FIXED_FRAME_ID;
        _marker.header.stamp = _time;

        _marker.ns = "robot_model";
        _marker.id = _id;
        _marker.type = visualization_msgs::Marker::LINE_STRIP;
        _marker.action = visualization_msgs::Marker::ADD;

        _marker.scale.x = 0.01;

        if(_color == "g"){
            _marker.color.r = 0.0;
            _marker.color.g = 1.0;
            _marker.color.b = 0.0;
            _marker.color.a = 1.0;
        }
        else if(_color == "r"){
            _marker.color.r = 1.0;
            _marker.color.g = 0.0;
            _marker.color.b = 0.0;
            _marker.color.a = 1.0;
        }
        else if(_color == "b"){
            _marker.color.r = 0.0;
            _marker.color.g = 0.0;
            _marker.color.b = 1.0;
            _marker.color.a = 1.0;
        }
        else if(_color == "w"){
            _marker.color.r = 1.0;
            _marker.color.g = 1.0;
            _marker.color.b = 1.0;
            _marker.color.a = 1.0;
        }
        else{
            _marker.color.r = 1.0;
            _marker.color.g = 1.0;
            _marker.color.b = 1.0;
            _marker.color.a = 1.0;
        }


        _marker.pose.position.x = 0.0;      _marker.pose.position.y = 0.0;      _marker.pose.position.z = 0.0;
        _marker.pose.orientation.x = 0.0;   _marker.pose.orientation.y = 0.0;   _marker.pose.orientation.z = 0.0;   _marker.pose.orientation.w = 1.0;

        std::array<quadmap::point2d, 4> vertices;
        _robot_model.get_vertices(vertices);

        _marker.points.clear();
        for(int i = 0; i < 5; i++) {
            int vertex_idx = i % 4;
            geometry_msgs::Point point;
            point.x = vertices[vertex_idx].x();
            point.y = vertices[vertex_idx].y();
            point.z = VIS_HEIGHT_MARKER;
            _marker.points.push_back(point);
        }
    }

    /*
     *
     */
    void quadtree_to_markerarray(const quadmap::QuadTree* _environment, const std_msgs::ColorRGBA& _color, visualization_msgs::MarkerArray& _markerarray,
                                 const std::string& _FIXED_FRAME_ID = "map", const ros::Time& _time = ros::Time::now()) {
        // Initialize a visualization message (visualization_msgs::MarkerArray)
        _markerarray.markers.resize(_environment->getTreeDepth()+1);

        // Insert occupancy data to the message
        const double VIS_CELL_HEIGHT = _environment->getResolution() * 0.5;
        for(auto leaf_it = _environment->begin_leafs(); leaf_it != _environment->end_leafs(); leaf_it++) {
            unsigned int marker_idx = leaf_it.getDepth();

            geometry_msgs::Point center;
            center.x = leaf_it.getX();
            center.y = leaf_it.getY();
            center.z = VIS_CELL_HEIGHT;

            _markerarray.markers[marker_idx].pose.orientation.x = 0.0;
            _markerarray.markers[marker_idx].pose.orientation.y = 0.0;
            _markerarray.markers[marker_idx].pose.orientation.z = 0.0;
            _markerarray.markers[marker_idx].pose.orientation.w = 1.0;

            _markerarray.markers[marker_idx].points.push_back(center);
            _markerarray.markers[marker_idx].colors.push_back(_color);
        }

        // Update the marker array
        for(unsigned int i = 0; i < _markerarray.markers.size(); i++) {
            double size = _environment->getNodeSize(i);

            _markerarray.markers[i].header.frame_id = _FIXED_FRAME_ID;
            _markerarray.markers[i].header.stamp = _time;
            _markerarray.markers[i].id = i;
            _markerarray.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
            _markerarray.markers[i].scale.x = size;
            _markerarray.markers[i].scale.y = size;
            _markerarray.markers[i].scale.z = _environment->getResolution();

            if(!_markerarray.markers[i].points.empty())
                _markerarray.markers[i].action = visualization_msgs::Marker::ADD;
            else
                _markerarray.markers[i].action = visualization_msgs::Marker::DELETE;
        }
    }

    /*
     *
     */
    void environment_to_markerarray(const quadmap::QuadTree* _environment, visualization_msgs::MarkerArray& _markerarray,
                                    const std::string& _FIXED_FRAME_ID = "map", const ros::Time& _time = ros::Time::now()) {
        std_msgs::ColorRGBA color;
        color.r = color.a = 1.0;
        color.g = color.b = 0.0;

        quadtree_to_markerarray(_environment, color, _markerarray, _FIXED_FRAME_ID, _time);
    }

    void stepping_stones_to_markerarray(const quadmap::QuadTree* _environment, visualization_msgs::MarkerArray& _markerarray,
                                    const std::string& _FIXED_FRAME_ID = "map", const ros::Time& _time = ros::Time::now()) {
        std_msgs::ColorRGBA color;
        color.g = color.a = 1.0;
        color.r = color.b = 0.0;

        quadtree_to_markerarray(_environment, color, _markerarray, _FIXED_FRAME_ID, _time);
    }

    /*
     *
     */
    void footstep_to_marker(const Configuration& _footstep_configuration, const quadmap::point2d& _footstep_size, visualization_msgs::Marker& _marker, bool _isRight,
                                   const std::string& _FIXED_FRAME_ID = "map", const ros::Time& _time = ros::Time::now(), const int& _id = 0) {
        // Message header
        _marker.header.frame_id = _FIXED_FRAME_ID;
        _marker.header.stamp = _time;

        _marker.ns = "footstep";
        _marker.id = _id;
        _marker.type = visualization_msgs::Marker::CUBE;
        _marker.action = visualization_msgs::Marker::ADD;

        _marker.scale.x = _footstep_size.x();
        _marker.scale.y = _footstep_size.y();
        _marker.scale.z = 0.01;

        if(_isRight) {
            _marker.color.r = 1.0;
            _marker.color.b = 0.0;
        }
        else {
            _marker.color.r = 0.0;
            _marker.color.b = 1.0;
        }
        _marker.color.g = 0.0;
        _marker.color.a = 1.0;

        _marker.pose.position.x = _footstep_configuration.x();
        _marker.pose.position.y = _footstep_configuration.y();
        _marker.pose.position.z = VIS_HEIGHT_MARKER;
        octomath::Quaternion q(0.0f, 0.0f, _footstep_configuration.r());
        _marker.pose.orientation.x = q.x();
        _marker.pose.orientation.y = q.y();
        _marker.pose.orientation.z = q.z();
        _marker.pose.orientation.w = q.u();
    }

    /*
     *
     */
    void footsteps_to_markerarray(const std::vector<Configuration>& _footsteps, const quadmap::point2d& _footstep_size, visualization_msgs::MarkerArray& _markerarray,
                                  const std::string& _FIXED_FRAME_ID = "map", const ros::Time& _time = ros::Time::now())
    {
        if(_footsteps.empty())
            return;

        _markerarray.markers.resize(MAX_NUM_OF_MARKERS);

        int footstep_idx = 0;
        int footstep_idx_step = ((int)(_footsteps.size() - 1) / MAX_NUM_OF_MARKERS) + 1;
        for(int i = 0; i < MAX_NUM_OF_MARKERS; i++) {
            if(footstep_idx < _footsteps.size()) {
                // visualization_msgs::Marker robot_model_marker;
                visualization_msgs::Marker& footstep_model_marker = _markerarray.markers[i];
                footstep_to_marker(_footsteps[footstep_idx], _footstep_size, footstep_model_marker, false, _FIXED_FRAME_ID, _time, i);

                // Set color
                double v = std::min(std::max(0.0, (double)footstep_idx / _footsteps.size()), 1.0);
                footstep_model_marker.color.r = v;
                footstep_model_marker.color.g = 0.0;
                footstep_model_marker.color.b = (1.0 - v);
            }
            else{
                visualization_msgs::Marker& footstep_model_marker = _markerarray.markers[i];
                footstep_to_marker(Configuration(), _footstep_size, footstep_model_marker, false, _FIXED_FRAME_ID, _time, i);
                footstep_model_marker.action = visualization_msgs::Marker::DELETE;
            }

            footstep_idx += footstep_idx_step;
        }
    }
};

#endif //HUBO_PLANNER_VISUALIZER_H
