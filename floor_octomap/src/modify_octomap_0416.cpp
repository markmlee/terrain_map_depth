#include <modify_octomap_0416.h>
#include <floor_octomap/OctoImage.h>

#include <octomap_msgs/conversions.h>
#include <octomap/ColorOcTree.h>
#include <octomap/OcTree.h>
#include <cstdint>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/Marker.h>


namespace octomodify_0416
{
OctomapModify_0416::OctomapModify_0416(ros::NodeHandle &nh, ros::NodeHandle &nh_private, std::string tf_frame) : 
    nh_(nh),
    nh_private_(nh_private),
    octomap_sub_(nh, "/octomap_full", 1),
    projected_map_sub_(nh, "/projected_map", 1),
    synchronizer_(SyncPolicy(10), octomap_sub_, projected_map_sub_),
    tf_frame_(tf_frame)
{
    /* Start Tracking */
    /* In */
    connection_ = synchronizer_.registerCallback(boost::bind(&OctomapModify_0416::octomapCallback, this, _1, _2));
    /* Out */
    octomap_pub_ = nh.advertise<octomap_msgs::Octomap>("/modified_octomap", 10);
    bounding_box_pub_ = nh.advertise<visualization_msgs::Marker>("/modifying_bounding_box", 10);
    height_image_pub_ = nh.advertise<sensor_msgs::Image>("/height_image", 10);
    height_result_pub_ = nh.advertise<sensor_msgs::Image>("/height_result", 10);

    cluster_service_ = nh.serviceClient<floor_octomap::OctoImage>("flatten_octomap_0416");


    ROS_INFO("Started modify_octomap_0416 ...");
}

void OctomapModify_0416::octomapCallback(const octomap_msgs::Octomap::ConstPtr &octomap_msg,
                                     const nav_msgs::OccupancyGrid::ConstPtr &occupancy_grid_msg)
{
    ROS_DEBUG_STREAM("Flattening Octomap");
    // http://docs.ros.org/kinetic/api/octomap_msgs/html/msg/Octomap.html
    octomap::AbstractOcTree *m_abstract_octomap = octomap_msgs::fullMsgToMap(*octomap_msg);
    octomap::OcTree *m_octomap = static_cast<octomap::OcTree *>(m_abstract_octomap);
    ROS_DEBUG_STREAM("Received tree with " << m_octomap->calcNumNodes() << " nodes");

    tf::StampedTransform transform;
    try
    {
        transform_listener_.lookupTransform("/world", tf_frame_, octomap_msg->header.stamp, transform);
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

    double roll, pitch, yaw;
    transform.getBasis().getRPY(roll, pitch, yaw);

    float camera_robot_height, box_width, box_front, box_back, box_bottom, box_top, image_height_base;
    nh_private_.getParam("camera_robot_height", camera_robot_height);
    nh_private_.getParam("box_width", box_width);
    nh_private_.getParam("box_front", box_front);
    nh_private_.getParam("box_back", box_back);
    nh_private_.getParam("box_bottom", box_bottom);
    nh_private_.getParam("box_top", box_top);
    nh_private_.getParam("image_height_base", image_height_base);

    tf::Vector3 z_axis(0., 0., 1.);
    tf::Vector3 pt1 = v + tf::Vector3(-box_back, box_width/2, 0.0).rotate(z_axis, yaw);
    tf::Vector3 pt2 = v + tf::Vector3(-box_back, -box_width/2, 0.0).rotate(z_axis, yaw);
    tf::Vector3 pt3 = v + tf::Vector3(box_front, box_width/2, 0.0).rotate(z_axis, yaw);
    tf::Vector3 pt4 = v + tf::Vector3(box_front, -box_width/2, 0.0).rotate(z_axis, yaw);
    
    double x_min = std::min({pt1.getX(),pt2.getX(),pt3.getX(),pt4.getX()});
    double x_max = std::max({pt1.getX(),pt2.getX(),pt3.getX(),pt4.getX()});
    double y_min = std::min({pt1.getY(),pt2.getY(),pt3.getY(),pt4.getY()});
    double y_max = std::max({pt1.getY(),pt2.getY(),pt3.getY(),pt4.getY()});

    octomap::point3d start_box(x_min, y_min, box_bottom);
    octomap::point3d end_box(x_max, y_max, box_top);

    publish_bounding_box(start_box, end_box, octomap_msg->header.stamp);

    double min_Z = std::numeric_limits<double>::max();
    for (octomap::OcTree::leaf_bbx_iterator it = m_octomap->begin_leafs_bbx(start_box, end_box), end = m_octomap->end_leafs_bbx(); it != end; ++it)
    {
        if (it.getZ() < min_Z)
            min_Z = it.getZ();
    }

    /* Create data array */
    double resolution = m_octomap->getResolution();
    int start_x = floor(start_box.x() / resolution);
    int start_y = floor(start_box.y() / resolution);
    int end_x = ceil(end_box.x() / resolution);
    int end_y = ceil(end_box.y() / resolution);

    uint32_t image_width = end_x - start_x;
    uint32_t image_height = end_y - start_y;

    std::vector<uint8_t> data(image_width * image_height, 0);
    int cnt = 0;
    double halfres = resolution/2.0;
    
    for (octomap::OcTree::leaf_bbx_iterator it = m_octomap->begin_leafs_bbx(start_box, end_box), end = m_octomap->end_leafs_bbx(); it != end; ++it)
    {
        if (m_octomap->search(it.getCoordinate())->getOccupancy() < m_octomap->getOccupancyThres())
            continue;
        cnt ++;
        double size = it.getSize();

        if (size == resolution)
        {
            uint8_t z = (it.getZ() - box_bottom) / resolution + image_height_base;
            int oct_x = floor(it.getX() / resolution);
            int oct_y = floor(it.getY() / resolution);
            int img_x = oct_x - start_x;
            int img_y = oct_y - start_y;
            int image_idx = img_y * image_width + img_x;

            if (img_x >= 0 && img_x < image_width && img_y >= 0 && img_y < image_height)
                data[image_idx] = std::max(data[image_idx], z);
        }
        else
        {
            uint8_t z = ((it.getZ() + (size/2.0) - halfres) - box_bottom) / resolution + image_height_base;
            for (double i = -size/2.0; i < size/2.0; i += resolution)
            {
                for (double j = -size/2.0; j < size/2.0; j += resolution)
                {
                    int oct_x = floor((it.getX() + i + halfres) / resolution);
                    int oct_y = floor((it.getY() + j + halfres) / resolution);
                    int img_x = oct_x - start_x;
                    int img_y = oct_y - start_y;
                    int image_idx = img_y * image_width + img_x;

                    if (img_x >= 0 && img_x < image_width && img_y >= 0 && img_y < image_height)
                        data[image_idx] = std::max(data[image_idx], z);
                }
            }
        }

        octomap::OcTreeKey key = it.getKey();
        m_octomap->deleteNode(key);
    }
    delete m_octomap;

    if ((float) cnt / (image_width*image_height) < 0.2)
    {
        ROS_INFO_STREAM("Not enough data in the image! Not calling flatten service");
        return;
    }



    sensor_msgs::Image srv_in;
    srv_in.header = octomap_msg->header;
    srv_in.height = image_height;
    srv_in.width = image_width;
    srv_in.encoding = sensor_msgs::image_encodings::MONO8;
    srv_in.is_bigendian = 0;
    srv_in.step = image_width;
    srv_in.data.assign(data.begin(), data.end());
    height_image_pub_.publish(srv_in);

    floor_octomap::OctoImage srv;
    srv.request.input = srv_in;
    sensor_msgs::Image srv_out;
    
    if (cluster_service_.call(srv))
        srv_out = srv.response.output;
    else
        return;

    height_result_pub_.publish(srv_out);

    uint8_t min_img_z = std::numeric_limits<uint8_t>::max();
    for (auto const & it : srv_out.data)
    {
        if (it > 0)
            min_img_z = std::min(it, min_img_z);
    }

    octomap::OcTree *new_tree = new octomap::OcTree(resolution);

    for (uint32_t image_y = 0; image_y < srv_out.height; ++image_y)
    {
        for (uint32_t image_x = 0; image_x < srv_out.width; ++image_x)
        {
            float x_coord = ((int)image_x + start_x + 0.5) * resolution;
            float y_coord = ((int)image_y + start_y + 0.5) * resolution;
            int image_idx = image_y * srv_out.step + image_x;
            bool is_edge = false;
            uint8_t image_z = srv_out.data[image_idx];
            if (image_z == 0)
                continue;

            for (int i = -1; i < 2; i++)
            {
                for (int j = -1; j < 2; j++)
                {
                    int nx = image_x+i;
                    int ny = image_y+j;
                    if (nx < 0 || nx >= srv_out.width || ny < 0 || ny >= srv_out.height || (srv_out.data[ny * srv_out.step + nx] < image_z)) // point is at edge of image OR neighbour point is lower than me
                    {
                        is_edge = true;
                        break;
                    }
                }
                if (is_edge)
                    break;
            }
            if (is_edge)
            {
                for (uint8_t fill_z = min_img_z; fill_z <= image_z; fill_z += 1)
                {
                    float z_coord = (fill_z - image_height_base) * resolution + box_bottom + halfres;
                    octomap::point3d coord(x_coord, y_coord, z_coord);
                    new_tree->updateNode(coord, true, true);
                }
            }
            else
            {
                float z_coord = (image_z - image_height_base) * resolution + box_bottom + halfres;
                octomap::point3d coord(x_coord, y_coord, z_coord);
                new_tree->updateNode(coord, true, true);
            }
        }
    }

    new_tree->updateInnerOccupancy();
    new_tree->toMaxLikelihood();
    new_tree->prune();
    
    octomap_msgs::Octomap out_msg;
    octomap_msgs::fullMapToMsg(*new_tree, out_msg);
    out_msg.header = octomap_msg->header;

    ROS_DEBUG_STREAM("Publishing tree with " << new_tree->calcNumNodes() << " nodes");
    octomap_pub_.publish(out_msg);
    delete new_tree;

}

void OctomapModify_0416::publish_bounding_box(octomap::point3d start_box, octomap::point3d end_box, ros::Time time_stamp)
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