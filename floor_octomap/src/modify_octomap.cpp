#include <modify_octomap.h>
#include <floor_octomap/OctoImage.h>

#include <octomap_msgs/conversions.h>
#include <octomap/ColorOcTree.h>
#include <octomap/OcTree.h>
#include <cstdint>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/Marker.h>


namespace octomodify
{
OctomapModify::OctomapModify(ros::NodeHandle &nh, ros::NodeHandle &nh_private, std::string tf_frame) : 
    nh_(nh),
    nh_private_(nh_private),
    octomap_sub_(nh, "/octomap_full", 1),
    projected_map_sub_(nh, "/projected_map", 1),
    synchronizer_(SyncPolicy(10), octomap_sub_, projected_map_sub_),
    tf_frame_(tf_frame)
{
    /* Start Tracking */
    /* In */
    connection_ = synchronizer_.registerCallback(boost::bind(&OctomapModify::octomapCallback, this, _1, _2));
    /* Out */
    octomap_pub_ = nh.advertise<octomap_msgs::Octomap>("/modified_octomap", 10);
    bounding_box_pub_ = nh.advertise<visualization_msgs::Marker>("/modifying_bounding_box", 10);
    height_image_pub_ = nh.advertise<sensor_msgs::Image>("/height_image", 10);
    height_result_pub_ = nh.advertise<sensor_msgs::Image>("/height_result", 10);

    cluster_service_ = nh.serviceClient<floor_octomap::OctoImage>("flatten_octomap");


    ROS_INFO("Started modify_octomap ...");
}

void OctomapModify::octomapCallback(const octomap_msgs::Octomap::ConstPtr &octomap_msg,
                                     const nav_msgs::OccupancyGrid::ConstPtr &occupancy_grid_msg)
{
    ROS_DEBUG_STREAM("Flattening Octomap");
    // http://docs.ros.org/kinetic/api/octomap_msgs/html/msg/Octomap.html
    octomap::AbstractOcTree *m_abstract_octomap = octomap_msgs::fullMsgToMap(*octomap_msg);
    octomap::OcTree *m_octomap = static_cast<octomap::OcTree *>(m_abstract_octomap);
    ROS_DEBUG_STREAM("Received tree with " << m_octomap->calcNumNodes() << " nodes");

    double min_Z = std::numeric_limits<double>::max();

    for (octomap::OcTree::leaf_iterator it = m_octomap->begin_leafs(), end = m_octomap->end_leafs(); it != end; ++it)
    {
        if (it.getZ() < min_Z)
        min_Z = it.getZ();
    }

    tf::StampedTransform transform;
    try
    {
        transform_listener_.lookupTransform("/world", tf_frame_,
                                            octomap_msg->header.stamp, transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
        return;
    }

    ROS_DEBUG_STREAM("Time Stamp: " << octomap_msg->header.stamp);

    tf::Vector3 v = transform.getOrigin();
    tf::Quaternion q = transform.getRotation();

    // Roll: Around X-Axis
    // Pitch: Around Y-Axis
    // Yaw: Around Z-Axis
    double roll, pitch, yaw;
    transform.getBasis().getRPY(roll, pitch, yaw);

    float camera_robot_height, box_width, box_front, box_back, box_up, box_down, image_height_base;
    int scale;
    nh_private_.getParam("camera_robot_height", camera_robot_height);
    nh_private_.getParam("box_width", box_width);
    nh_private_.getParam("box_front", box_front);
    nh_private_.getParam("box_back", box_back);
    nh_private_.getParam("box_up", box_up);
    nh_private_.getParam("box_down", box_down);
    nh_private_.getParam("image_height_base", image_height_base);
    nh_private_.getParam("pixel_height_scale", scale);

    tf::Vector3 z_axis(0., 0., 1.);
    tf::Vector3 pt1 = v + tf::Vector3(-box_back, box_width/2, 0.0).rotate(z_axis, yaw);
    tf::Vector3 pt2 = v + tf::Vector3(-box_back, -box_width/2, 0.0).rotate(z_axis, yaw);
    tf::Vector3 pt3 = v + tf::Vector3(box_front, box_width/2, 0.0).rotate(z_axis, yaw);
    tf::Vector3 pt4 = v + tf::Vector3(box_front, -box_width/2, 0.0).rotate(z_axis, yaw);
    
    double x_min = std::min({pt1.getX(),pt2.getX(),pt3.getX(),pt4.getX()});
    double x_max = std::max({pt1.getX(),pt2.getX(),pt3.getX(),pt4.getX()});
    double y_min = std::min({pt1.getY(),pt2.getY(),pt3.getY(),pt4.getY()});
    double y_max = std::max({pt1.getY(),pt2.getY(),pt3.getY(),pt4.getY()});

    octomap::point3d start_box(x_min, y_min, v.getZ() - camera_robot_height - box_down);
    octomap::point3d end_box(x_max, y_max, v.getZ() - camera_robot_height + box_up);

    ROS_DEBUG_STREAM("Bounding box: (" << x_min << ", " << y_min << ", " << v.getZ() - camera_robot_height - box_down << ") --> (" << x_max << ", " << y_max << ", " << v.getZ() - camera_robot_height + box_up << ")");

    publish_bounding_box(start_box, end_box, octomap_msg->header.stamp);

    /* Create data array */
    // TODO Check for correct frame direction
    double resolution = m_octomap->getResolution();
    int start_x = floor(start_box.x() / resolution);
    int start_y = floor(start_box.y() / resolution);
    int end_x = ceil(end_box.x() / resolution);
    int end_y = ceil(end_box.y() / resolution);

    uint32_t image_width = end_x - start_x;
    uint32_t image_height = end_y - start_y;
 
    ROS_DEBUG_STREAM("Image Width: " << image_width << " Image Height: " << image_height << " Res: " << resolution);

    // /* Taken from sensor_msgs/Images documentation: uint8[] data # actual matrix data, size is (step * rows) */
    std::vector<uint8_t> data(image_width * image_height, 0);
    int full_cnt = 0;
    int cnt = 0;
    for (octomap::OcTree::leaf_bbx_iterator it = m_octomap->begin_leafs_bbx(start_box, end_box), end = m_octomap->end_leafs_bbx(); it != end; ++it)
    {
        full_cnt ++;
        if (m_octomap->search(it.getCoordinate())->getOccupancy() < m_octomap->getOccupancyThres())
            continue;
        cnt ++;

        /*  Normalize Z Value in box and scale up to 255 
            We need to add the resolution as the centers might be above the camera */
        uint8_t z = octomap_to_image_height(it.getZ(), image_height_base, start_box.z(), scale);

        int oct_x = floor(it.getX() / resolution);
        int oct_y = floor(it.getY() / resolution);
        int siz = it.getSize() / resolution;
        if (siz == 1)
        {
            int img_x = oct_x - start_x;
            int img_y = oct_y - start_y;
            int image_idx = img_y * image_width + img_x;

            if (img_x < 0 || img_x >= image_width || img_y < 0 || img_y >= image_height){
                ROS_DEBUG_STREAM("Not chosen: " << img_x << " " << img_y);
                continue;
            }

            /* "+z" to actually print it (uint8_t is typedef char* --> no + results in as interpreting as a char*) */
            data[image_idx] = std::max(data[image_idx], z);
        }
        else
        {
            for (int i = -siz/2; i < siz/2; i += 1)
            {
                for (int j = -siz/2; j < siz/2; j += 1)
                {
                    int sub_x = oct_x + i;
                    int sub_y = oct_y + j;
                    int img_x = sub_x - start_x;
                    int img_y = sub_y - start_y;
                    int image_idx = img_y * image_width + img_x;

                    if (img_x < 0 || img_x >= image_width || img_y < 0 || img_y >= image_height)
                        continue;
                    
                    data[image_idx] = std::max(data[image_idx], z);
                }
            }
        }

        octomap::OcTreeKey key = it.getKey();
        /* Delete current node */
        m_octomap->deleteNode(key);
    }
    ROS_DEBUG_STREAM("Out of " << full_cnt << ", " << cnt << " chosen");

    if ((float) cnt / (image_width*image_height) > 0.2)
    {
        /*
        Header header        # Header timestamp should be acquisition time of image
                            # Header frame_id should be optical frame of camera
                            # origin of frame should be optical center of camera
                            # +x should point to the right in the image
                            # +y should point down in the image
                            # +z should point into to plane of the image
                            # If the frame_id here and the frame_id of the CameraInfo
                            # message associated with the image conflict
                            # the behavior is undefined

        uint32 height         # image height, that is, number of rows
        uint32 width          # image width, that is, number of columns

        # The legal values for encoding are in file src/image_encodings.cpp
        # If you want to standardize a new string format, join
        # ros-users@lists.sourceforge.net and send an email proposing a new encoding.

        string encoding       # Encoding of pixels -- channel meaning, ordering, size
                            # taken from the list of strings in include/sensor_msgs/image_encodings.h

        uint8 is_bigendian    # is this data bigendian?
        uint32 step           # Full row length in bytes
        uint8[] data          # actual matrix data, size is (step * rows)
        */
        sensor_msgs::Image service_input;
        service_input.header = octomap_msg->header;
        service_input.height = image_height;
        service_input.width = image_width;
        service_input.encoding = sensor_msgs::image_encodings::MONO8;
        service_input.is_bigendian = 0;
        service_input.step = image_width;
        service_input.data.assign(data.begin(), data.end());
        height_image_pub_.publish(service_input);


        // Send height_image to the service and get the new_image
        floor_octomap::OctoImage srv;

        srv.request.input = service_input;
        sensor_msgs::Image service_output;
        if (cluster_service_.call(srv))
        {
            service_output = srv.response.output;
            height_result_pub_.publish(service_output);
            ROS_DEBUG_STREAM("Service returned image of size " << service_output.height << " x " << service_output.width);
        }
        else
        {
            ROS_ERROR("Failed to call flattening service");
            return;
        }

        uint8_t min_img_z = std::numeric_limits<uint8_t>::max();
        for (auto const & it : service_output.data)
        {
            if (it < image_height_base)
                continue;
            min_img_z = std::min(it, min_img_z);
        }

        ROS_DEBUG_STREAM("Minimal Value in image above " << image_height_base << " is: " << +min_img_z);

        for (uint32_t image_y = 0; image_y < service_output.height; ++image_y)
        {
            for (uint32_t image_x = 0; image_x < service_output.width; ++image_x)
            {
                float x_coord = ((int)image_x + start_x + 0.5) * resolution;
                float y_coord = ((int)image_y + start_y + 0.5) * resolution;
                int image_idx = image_y * service_output.step + image_x;
                uint8_t image_z = service_output.data[image_idx];

                if (image_z < image_height_base)
                    continue;

                bool is_edge = false;

                for (int i = -1; i < 2; i++)
                {
                    for (int j = -1; j < 2; j++)
                    {
                        int nx = image_x+i;
                        int ny = image_y+j;
                        if (nx < 0 || nx >= service_output.width || ny < 0 || ny >= service_output.height) // point is at edge of image
                        {
                            is_edge = true;
                            break;
                        }
                        else if (service_output.data[ny * service_output.step + nx] < image_z) // neighbour point is lower than me
                        {
                            is_edge = true;
                            break;
                        }
                    }
                    if (is_edge)
                        break;
                    else
                    {
                        float z_coord = image_to_octomap_height(image_z, image_height_base, start_box.z(), scale);
                        if (z_coord < min_Z)
                            continue;
                        octomap::point3d coord(x_coord, y_coord, z_coord);
                        m_octomap->updateNode(coord, true, true);
                    }
                    
                }
                if (is_edge)
                {
                    for (uint8_t fill_z = min_img_z; fill_z <= image_z; fill_z += (uint8_t) scale)
                    {
                        float z_coord = image_to_octomap_height(fill_z, image_height_base, start_box.z(), scale);
                        
                        if (z_coord < min_Z)
                            continue;
                        
                        octomap::point3d coord(x_coord, y_coord, z_coord);
                        /* Fill in octomap */
                        m_octomap->updateNode(coord, true, true);
                    }
                }
            }
        }
    }
    else
    {
        ROS_INFO_STREAM("Not enough data in the image! Not calling flatten service");
    }

    m_octomap->updateInnerOccupancy();
    m_octomap->toMaxLikelihood();
    m_octomap->prune();
    /* Create Message */
    octomap_msgs::Octomap out_msg;
    octomap_msgs::fullMapToMsg(*m_octomap, out_msg);
    out_msg.header = octomap_msg->header;
    // out_msg.header.stamp = ros::Time::now(); // This creates the new octomap at the new timestamp

    /* Publish & Clean up */
    ROS_DEBUG_STREAM("Publishing tree with " << m_octomap->calcNumNodes() << " nodes");
    octomap_pub_.publish(out_msg);
    delete m_octomap;
}

void OctomapModify::publish_bounding_box(octomap::point3d start_box, octomap::point3d end_box, ros::Time time_stamp)
{
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/world";
    marker.header.stamp = time_stamp;

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "floor_octomap vizualization";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = (start_box.x() + end_box.x()) / 2;
    marker.pose.position.y = (start_box.y() + end_box.y()) / 2;
    marker.pose.position.z = (start_box.z() + end_box.z()) / 2;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = end_box.x() - start_box.x();
    marker.scale.y = end_box.y() - start_box.y();
    marker.scale.z = end_box.z() - start_box.z();

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.2;

    marker.lifetime = ros::Duration();

    bounding_box_pub_.publish(marker);
}

}