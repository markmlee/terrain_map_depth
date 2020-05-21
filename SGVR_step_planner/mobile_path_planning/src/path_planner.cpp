#include <mobile_path_planning/path_planner.h>

#include <algorithm>


AstarPlanner::AstarPlanner(const quadmap::QuadTree* _environment, const RobotModel& _robot)
        : environment(_environment)
{
    collision_checker = new CollisionChecker();
    robotmodel = _robot;

    // collision_map_voxel_size : If this value is low, the map become high resolution. This value is user definable
    double collision_map_voxel_size = _environment->getResolution() * 1.;
    // collision_map = new ConfigurationMap(_environment->getResolution());
    collision_map = new ConfigurationMap(collision_map_voxel_size);

    baseMatrix.setZero();

    baseMatrix(0, 0) = 1;
    baseMatrix(1, 0) = -3;
    baseMatrix(1, 1) = 3;
    baseMatrix(2, 0) = 3;
    baseMatrix(2, 1) = -6;
    baseMatrix(2, 2) = 3;
    baseMatrix(3, 0) = -1;
    baseMatrix(3, 1) = 3;
    baseMatrix(3, 2) = -3;
    baseMatrix(3, 3) = 1;

    //"nymParam":AstarPlanner`s memeber, type:int, value : 10
    paramMatrix = Eigen::MatrixXd(numParam, 4);

    std::vector<double> t = linspace<double>(0., 1., numParam);
    for(int i = 0; i < numParam; i++){
        paramMatrix(i, 0) = 1;
        paramMatrix(i, 1) = t.at(i);
        paramMatrix(i, 2) = pow(t.at(i), 2);
        paramMatrix(i, 3) = pow(t.at(i), 3);
    }
//"std::vector<ConfigurationKey> direction;" defined in path_planner.h
#ifdef USE_4_DIRECTION
    direction.resize(4);
    direction[0] = ConfigurationKey(1, 0);
    direction[1] = ConfigurationKey(0, 1);
    direction[2] = ConfigurationKey(-1, 0);
    direction[3] = ConfigurationKey(0, -1);
#else
    #ifdef USE_8_DIRECTION
    direction.resize(8);
    direction[0] = ConfigurationKey(1, 0);
    direction[1] = ConfigurationKey(1, 1);
    direction[2] = ConfigurationKey(0, 1);
    direction[3] = ConfigurationKey(-1, 1);
    direction[4] = ConfigurationKey(-1, 0);
    direction[5] = ConfigurationKey(-1, -1);
    direction[6] = ConfigurationKey(0, -1);
    direction[7] = ConfigurationKey(1, -1);
#endif
#endif
}

AstarPlanner::~AstarPlanner()
{
    if(collision_checker)
        delete collision_checker;

    if(collision_map)
        delete collision_map;
}

bool AstarPlanner::planning(const geometry_msgs::Pose& _start_pose, const geometry_msgs::Pose& _goal_pose, nav_msgs::Path& _nav_path)
{
    // Check inputs
    ConfigurationKey startKey = collision_map->coordToKey(_start_pose.position.x, _start_pose.position.y);
    ConfigurationKey goalKey = collision_map->coordToKey(_goal_pose.position.x, _goal_pose.position.y);
    if(startKey == goalKey)
        return true;

    // Check incorrect inputs
    if(isCollide(startKey) || isCollide(goalKey))
        return false;

    // Initialize
    AstarNode* current = nullptr;
    OpenSet openSet;
    ClosedSet closedSet;
    openSet.push(new AstarNode(startKey, 0, cost_to_go(startKey, goalKey), nullptr));

    // Try to find path
    while(!openSet.empty()){
        // Get current node
        current = openSet.top();

        // Found goal
        if(current->key == goalKey){
            break;
        }

        closedSet.insert(std::pair<ConfigurationKey, AstarNode*>(current->key, current));
        openSet.pop();

        // Check new candidates
        for(int i = 0; i < (int)direction.size(); ++i){
            ConfigurationKey newKey = current->key;
            newKey.k[0] += direction[i].k[0];
            newKey.k[1] += direction[i].k[1];

            if(isCollide(newKey) || closedSet.find(newKey) != closedSet.end())  //newKey is obstacle or not the end of closedset(previous node)
                continue;

            AstarNode* neighbor = openSet.find(newKey);

            double g_score = current->G + cost_to_come(current->key, newKey);//update G
            double h_score = g_score + cost_to_go(newKey, goalKey);//Act as F_score = G_score+H_score, *Smooth when use cost_to_go(newKey, newKey)

            if(neighbor == nullptr){  //openSet doesn`t include neighbor node
                neighbor = new AstarNode(newKey, g_score, h_score, current);
                openSet.push(neighbor);
            }
            else if(g_score < neighbor->G){ //neighbor node already exists in openSet but New g_score is smller than before
                neighbor->parent = current;
                neighbor->G = g_score;
                neighbor->H = h_score;
            }
        }
    }

    // Refine the path
    std::vector<Configuration> path;
    while(current != nullptr){
        path.push_back(collision_map->keyToCoord(current->key));
        current = current->parent;
    }
    refinePath(path, _nav_path);

    // Release memoriesopen-recent
    releaseNodes(openSet);
    releaseNodes(closedSet);

    return true;
}

bool AstarPlanner::planning(const geometry_msgs::Pose& _start_pose, const geometry_msgs::Pose& _goal_pose, const geometry_msgs::Pose* _err_pose, nav_msgs::Path& _nav_path, int& Max_err_pos)
{
    // Check inputs (Transform Coordinates to Key for idetification purpose)
    int cnt_err_pose = 0;
    ConfigurationKey startKey = collision_map->coordToKey(_start_pose.position.x, _start_pose.position.y);
    ConfigurationKey goalKey = collision_map->coordToKey(_goal_pose.position.x, _goal_pose.position.y);
    ConfigurationKey errkey;

    if(startKey == goalKey)
        return true;

    // Check incorrect inputs
    if(isCollide(startKey) || isCollide(goalKey))
        return false;

    // Initialize
    AstarNode* current = nullptr;
    OpenSet openSet;
    ClosedSet closedSet;
    openSet.push(new AstarNode(startKey, 0, cost_to_go(startKey, goalKey), nullptr));

    // Try to find path
    int CostIncrease = 1.0;



    while(!openSet.empty()){
        // Get current node
        current = openSet.top();

        // Found goal
        if(current->key == goalKey){
            break;
        }

        closedSet.insert(std::pair<ConfigurationKey, AstarNode*>(current->key, current));
        openSet.pop();

        if(CostIncrease != 1.0){//If there was error path position
            cnt_err_pose++;
            if(cnt_err_pose > Max_err_pos){
                cnt_err_pose = Max_err_pos;
            }
        }

        if(Max_err_pos == Prev_Max_err_pos && cnt_err_pose == Max_err_pos){
            this->Kernel_size += 2;
            if(this->Kernel_size > 11) this->Kernel_size = 11;
        }
        else this->Kernel_size = 3;

        errkey = collision_map->coordToKey(_err_pose[cnt_err_pose].position.x, _err_pose[cnt_err_pose].position.y);
        // Check new candidates
        for(int i = 0; i < (int)direction.size(); ++i){

            ConfigurationKey newKey = current->key;
            newKey.k[0] += direction[i].k[0];
            newKey.k[1] += direction[i].k[1];


            if(isCollide(newKey) || closedSet.find(newKey) != closedSet.end())  //newKey is obstacle or not the end of closedset(previous node)
                continue;

            AstarNode* neighbor = openSet.find(newKey);

            CostIncrease = 1.0;
            int metric = Kernel_size/2;
            //In case newKey is near the error path position(inside the kernel)
            for(int row = -metric; row <= metric; row++){
                for(int column = -metric; column <= metric; column++){
                    if((newKey.k[0] == errkey.k[0]+row) && (newKey.k[1] == errkey.k[1]+column)){
                        //Kernel, lookslike pyramid.......Form should be change
                        CostIncrease = -abs(row * column) + (Kernel_size*Kernel_size)/4;
                        if(row * column == 0){
                            CostIncrease -= abs(row + column);
                        }
                        CostIncrease/(-abs(metric * metric) + (Kernel_size*Kernel_size)/4);
                        //std::cout<<"CostIncrease : "<<CostIncrease<<std::endl;
                    }
                }
            }


            //"CostIncrease" increases the F_scroe -> Lower the chance to reach this area
            double g_score = (current->G + cost_to_come(current->key, newKey));//update G
            //double h_score = (g_score + cost_to_go(newKey, goalKey));//Act as F_score = G_score+H_score, *Smooth when use cost_to_go(newKey, newKey)
            double h_score = CostIncrease * (g_score + cost_to_go(newKey, newKey));//Simple global path...
            //std::cout<<h_score<<std::endl;
            if(neighbor == nullptr){  //openSet doesn`t include neighbor node
                neighbor = new AstarNode(newKey, g_score, h_score, current);
                openSet.push(neighbor);
            }
            else if(g_score < neighbor->G){ //neighbor node already exists in openSet but New g_score is smller than before
                neighbor->parent = current;
                neighbor->G = g_score;
                neighbor->H = h_score;
            }
        }
    }


    std::cout<<"ErrPathPoint["<<Max_err_pos<<"] : "<<_err_pose[Max_err_pos].position.x<<", "<<_err_pose[Max_err_pos].position.x;//<<std::endl;
    std::cout<<", Kernel_size : "<<Kernel_size<<std::endl;


    // Refine the path
    std::vector<Configuration> path;
    while(current != nullptr){
        path.push_back(collision_map->keyToCoord(current->key));
        current = current->parent;
    }
    refinePath(path, _nav_path);

    // Release memoriesopen-recent
    releaseNodes(openSet);
    releaseNodes(closedSet);

    this->Prev_Max_err_pos = Max_err_pos;
    return true;
}

geometry_msgs::Pose getMidPoint(geometry_msgs::Pose& after, geometry_msgs::Pose& before){
    float x = (after.position.x + before.position.x) / 2.f;
    float y = (after.position.y + before.position.y) / 2.f;

    geometry_msgs::Pose mid_point;
    mid_point.position.x = x;
    mid_point.position.y = y;

    return mid_point;
}

void AstarPlanner::refinePath(std::vector<Configuration>& _path, nav_msgs::Path& _refined_path)
{
    // Convert grid based path to nav_msgs::Path

    nav_msgs::Path tmp_path;
    tmp_path.header.frame_id = "world";
    tmp_path.header.stamp = ros::Time::now();
    _refined_path.header.frame_id = tmp_path.header.frame_id;
    _refined_path.header.stamp = tmp_path.header.stamp;

    for(int i = (int)_path.size() - 1; i > 0; i--){
        quadmath::Vector2 from((float)_path[i].x(), (float)_path[i].y());
        quadmath::Vector2 to((float)_path[i-1].x(), (float)_path[i-1].y());
        quadmath::Vector2 delta = to - from;

        geometry_msgs::PoseStamped path_node;
        path_node.header.frame_id = tmp_path.header.frame_id;
        path_node.header.stamp = tmp_path.header.stamp;

        path_node.pose.position.x = from.x();
        path_node.pose.position.y = from.y();
        path_node.pose.position.z = 10.0;
        //euler angle to quaternion
        double rotation = atan2(delta.y()/delta.norm(), delta.x()/delta.norm()); // acos(delta.norm() / delta.x());
        octomath::Quaternion q(0.0, 0.0, rotation);
        path_node.pose.orientation.x = q.x();
        path_node.pose.orientation.y = q.y();
        path_node.pose.orientation.z = q.z();
        path_node.pose.orientation.w = q.u();

        tmp_path.poses.push_back(path_node);
    }

    // Convert path to cubic bezier spline
    for(int i = 0; i < tmp_path.poses.size()/2; i++){
        geometry_msgs::Pose p0;
        geometry_msgs::Pose p1 = tmp_path.poses.at(2 * i).pose;
        if(i == 0){
            p0 = p1;
        }
        else{
            p0 = getMidPoint(tmp_path.poses.at(2 * (i-1) + 1).pose, p1);
        }
        geometry_msgs::Pose p2 = tmp_path.poses.at(2 * i + 1).pose;
        geometry_msgs::Pose p3;
        if(i == tmp_path.poses.size()/2 - 1){
            p3 = p2;
        }
        else{
            p3 = getMidPoint(p2, tmp_path.poses.at(2 * (i+1)).pose);
            p3.orientation = p2.orientation;
        }

        Eigen::MatrixXd pointMatrix(4, 2);
        pointMatrix << p0.position.x, p0.position.y, p1.position.x, p1.position.y, p2.position.x, p2.position.y, p3.position.x, p3.position.y;

        Eigen::MatrixXd bezier = paramMatrix * baseMatrix * pointMatrix; //beizier : 10x2 matrix

        for(int j = 0; j < numParam; j++){
            geometry_msgs::PoseStamped spline_node;
            spline_node.header.frame_id = tmp_path.header.frame_id;
            spline_node.header.stamp = tmp_path.header.stamp;

            spline_node.pose.position.x = bezier(j, 0);
            spline_node.pose.position.y = bezier(j, 1);
            spline_node.pose.position.z = 10.0;

            if(j != numParam-1){
                double yaw = atan2(bezier(j+1, 1) - bezier(j, 1), bezier(j+1, 0) - bezier(j, 0));

                tf2::Quaternion q;
                q.setRPY(0, 0, yaw);
                spline_node.pose.orientation.w = q.w();
                spline_node.pose.orientation.x = q.x();
                spline_node.pose.orientation.y = q.y();
                spline_node.pose.orientation.z = q.z();
            }

            _refined_path.poses.push_back(spline_node);
        }
    }
}
