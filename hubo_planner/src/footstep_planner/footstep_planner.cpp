#include <footstep_planner/footstep_planner.h>
/**
 * Footstep planning without goal position
 * @param _start_conf
 * @return
 */
bool FootstepPlanner::planning_maximum_length_optimal_solution(const Configuration &_start_conf){
    // Initialize the start configurations of footsteps
    FootstepNode start_left_footsteps(false);
    FootstepNode start_right_footsteps(true);

    set_initial_footsteps_from_pose(_start_conf, start_left_footsteps, start_right_footsteps);
    if(is_current_footstep_right)
        current_footstep_configuration = start_right_footsteps.step_conf;
    else
        current_footstep_configuration = start_left_footsteps.step_conf;

    FootstepNode* start_footstep = new FootstepNode(is_current_footstep_right, current_footstep_configuration);

    // Initialize the footstep_nodes with start_footstep
    std::vector<FootstepNode*> footstep_nodes;
    footstep_nodes.reserve(256);            // to prevent footstep_nodes from being reallocated.
    footstep_nodes.push_back(start_footstep);

    unsigned int max_length = 0;                 // max number of footsteps
    std::vector<FootstepNode*> last_footsteps;   // for optimization

    bool found = false;

    for(int i = 0; i < NUMBER_OF_TRIALS; i++) {
        // 1. Make a random footstep
        FootstepNode* next_footstep = make_new_sample(footstep_nodes);

        // 2. Check the availability of the new footstep
        if(!isAvailableFootStep(next_footstep)) {
            delete next_footstep;
            continue;
        }

        // 3. Check whether the number of footsteps is max length
        if(max_length < next_footstep->cnt_footsteps){
            max_length = next_footstep->cnt_footsteps;
            last_footsteps.clear();
            last_footsteps.push_back(next_footstep);
        }
        else if(max_length == next_footstep->cnt_footsteps){
            last_footsteps.push_back(next_footstep);
        }

        // 4. Prepare the continuous planning
        footstep_nodes.push_back(next_footstep);
    }

    // Make path
    float cost_min = 100000.0;

    footsteps.clear();
    if(max_length > 1) {
        found = true;
        std::vector<Configuration> footsteps_temp;
        footsteps_temp.reserve(8);

        for(int i = 0; i < last_footsteps.size(); i++){
            float cost = 0;
            FootstepNode* current = last_footsteps[i];
            while(current != nullptr){
                cost += calculateCost(current);
                footsteps_temp.push_back(current->step_conf);
                current = current->parent;
            }
            footsteps_temp.pop_back();
            //ROS_INFO("Cost : %d", cost);
            if(cost_min > cost){
                cost_min = cost;
                //ROS_INFO("Best Cost : %d", cost);
                footsteps = footsteps_temp;
            }
            footsteps_temp.clear();
        }

        // std::cout << "Number of nodes: " << footstep_nodes[0].size() + footstep_nodes[1].size() << std::endl;
        // std::cout << "Number of steps: " << footsteps.size() << std::endl;
    }
    else if(max_length == 1) {   // next footstep might be final footstep, so we should make pair footstep.
        float cost;

        for(int i = 0; i < footstep_nodes.size(); i++){
            FootstepNode* pair_footstep = make_pair_sample(footstep_nodes[i]);

            if(isAvailableFootStep(pair_footstep)){
                found = true;
                cost = calculateCost(footstep_nodes[i]) + calculateCost(pair_footstep);
                if(cost_min > cost){
                    cost_min = cost;
                    footsteps.clear();
                    footsteps.push_back(footstep_nodes[i]->step_conf);
                    footsteps.push_back(pair_footstep->step_conf);
                }
            }
            delete pair_footstep;
        }
        if(!found){
            footsteps.push_back(footstep_nodes[0]->step_conf);
        }
    }
    else {
        // std::cout << "Cannot find the path" << std::endl;
    }

    //ROS_INFO("footstep_nodes cap : %d", static_cast<int>(footstep_nodes.capacity()));
    ROS_INFO("Number of trials : %d", NUMBER_OF_TRIALS);
    ROS_INFO("Number of paths  : %d", static_cast<int>(last_footsteps.size()));
    ROS_INFO("Best Cost : %f", cost_min);


    // Release memories
    for(auto node : footstep_nodes)
        delete node;

    return found;
}
/**
 * Footstep planning using the largest number of footsteps w/o optimization
 * @param _start_conf
 * @return
 */
bool FootstepPlanner::planning_maximum_length(const Configuration &_start_conf){
    // Initialize the start configurations of footsteps
    FootstepNode start_left_footsteps(false);
    FootstepNode start_right_footsteps(true);

    set_initial_footsteps_from_pose(_start_conf, start_left_footsteps, start_right_footsteps);
    if(is_current_footstep_right)
        current_footstep_configuration = start_right_footsteps.step_conf;
    else
        current_footstep_configuration = start_left_footsteps.step_conf;

    FootstepNode* start_footstep = new FootstepNode(is_current_footstep_right, current_footstep_configuration);

    // Initialize the footstep_nodes with start_footstep
    std::vector<FootstepNode*> footstep_nodes;
    footstep_nodes.push_back(start_footstep);

    unsigned int max_length = 0;
    FootstepNode* last_footstep = nullptr;

    bool found = false;

    for(int i = 0; i < NUMBER_OF_TRIALS; i++) {
        // 1. Make a random footstep
        FootstepNode* next_footstep = make_new_sample(footstep_nodes);

        // 2. Check the availability of the new footstep
        if(!isAvailableFootStep(next_footstep)) {
            delete next_footstep;
            continue;
        }

        // 3.
        if(max_length < next_footstep->cnt_footsteps){
            max_length = next_footstep->cnt_footsteps;
            last_footstep = next_footstep;
        }

        // 4. Prepare the continuous planning
        footstep_nodes.push_back(next_footstep);
    }

    // Make path
    footsteps.clear();
    if(max_length > 1) {
        found = true;

        FootstepNode* current = last_footstep;
        while(current != nullptr){
            footsteps.push_back(current->step_conf);
            current = current->parent;
        }
        footsteps.pop_back();
        // std::cout << "Number of nodes: " << footstep_nodes[0].size() + footstep_nodes[1].size() << std::endl;
        // std::cout << "Number of steps: " << footsteps.size() << std::endl;
    }
    else if(max_length == 1) {   // next footstep might be final footstep, so we should make pair footstep.
        for(int i = 0; i < footstep_nodes.size(); i++){
            FootstepNode* pair_footstep = make_pair_sample(footstep_nodes[i]);

            if(isAvailableFootStep(pair_footstep)){
                found = true;
                footsteps.push_back(footstep_nodes[i]->step_conf);
                footsteps.push_back(pair_footstep->step_conf);
                delete pair_footstep;
                break;
            }
            delete pair_footstep;
        }
    }
    else {
        // std::cout << "Cannot find the path" << std::endl;
    }

    // Release memories
    for(auto node : footstep_nodes)
        delete node;

    return found;
}

/**
 * Footstep planning with given target number of footsteps
 *
 * @param _start_conf
 * @param target_num_footsteps
 * @return
 */
bool FootstepPlanner::planning(const Configuration& _start_conf, const int& target_num_footsteps)
{
    // Initialize the start configurations of footsteps
    FootstepNode start_left_footsteps(false);
    FootstepNode start_right_footsteps(true);

    set_initial_footsteps_from_pose(_start_conf, start_left_footsteps, start_right_footsteps);
    if(is_current_footstep_right)
        current_footstep_configuration = start_right_footsteps.step_conf;
    else
        current_footstep_configuration = start_left_footsteps.step_conf;

    FootstepNode* start_footstep = new FootstepNode(is_current_footstep_right, current_footstep_configuration);

    // Initialize the footstep_nodes with start_footstep
    std::vector<FootstepNode*> footstep_nodes;
    footstep_nodes.push_back(start_footstep);

    bool found = false;

//    FootstepNode* pair_footstep = nullptr;

    for(int i = 0; i < NUMBER_OF_TRIALS; i++) {
        // 1. Make a random footstep
        FootstepNode* next_footstep = make_new_sample(footstep_nodes);

        // 2. Check the availability of the new footstep
        if(!isAvailableFootStep(next_footstep)) {
            delete next_footstep;
            continue;
        }

        // 3. If the next footstep is target number of footsteps, stop the planning
        if(next_footstep->cnt_footsteps == target_num_footsteps){
//            pair_footstep = make_pair_sample(next_footstep);
//            if(isAvailableFootStep(pair_footstep))
              found = true;
//            else
//              delete pair_footstep;
        }

        // 4. Prepare the continuous planning
        footstep_nodes.push_back(next_footstep);

        if(found)
        {
//            footstep_nodes.push_back(pair_footstep);
            ROS_INFO("successful plan after trial :%i", i);
            break;
        }
    }

    // Make path
    footsteps.clear();
    if(found) {
        FootstepNode* current = footstep_nodes.back();   //The lastly saved footstep is goal footstep.
        while(current != nullptr){
            footsteps.push_back(current->step_conf);
            current = current->parent;
        }
        footsteps.pop_back();
        // std::cout << "Number of nodes: " << footstep_nodes[0].size() + footstep_nodes[1].size() << std::endl;
        // std::cout << "Number of steps: " << footsteps.size() << std::endl;
    }
    else {
        // std::cout << "Cannot find the path" << std::endl;
    }

    // Release memories
    for(auto node : footstep_nodes)
        delete node;

    return found;
}

bool FootstepPlanner::planning(const Configuration& _start_conf, const Configuration& _goal_conf)
{
    // Initialize the start configurations of footsteps
    FootstepNode start_left_footsteps(false);
    FootstepNode start_right_footsteps(true);

    set_initial_footsteps_from_pose(_start_conf, start_left_footsteps, start_right_footsteps);
    if(is_current_footstep_right)
        current_footstep_configuration = start_right_footsteps.step_conf;
    else
        current_footstep_configuration = start_left_footsteps.step_conf;

    return planning(_goal_conf);
}

bool FootstepPlanner::planning(const Configuration& _goal_conf)
{
    // Initialize the start configuration of footstep
    FootstepNode* start_footstep = new FootstepNode(is_current_footstep_right, current_footstep_configuration);
//    if(!isAvailableFootStep(start_footstep)) {
//        std::cout << "Start footstep is not available." << std::endl;
//        delete start_footstep;
//        return false;
//    }

    // Initialize the goal configurations of footsteps
    FootstepNode* goal_footsteps[2] = { new FootstepNode(false), new FootstepNode(true) };
    set_initial_footsteps_from_pose(_goal_conf, *(goal_footsteps[FOOT_LEFT]), *(goal_footsteps[FOOT_RIGHT]));

    bool goal_footsteps_availability[2] = {isAvailableFootStep(goal_footsteps[FOOT_LEFT]), isAvailableFootStep(goal_footsteps[FOOT_RIGHT])};
    if(!goal_footsteps_availability[FOOT_LEFT] && !goal_footsteps_availability[FOOT_RIGHT]) {
//        std::cout << "Goal footsteps are not available." << std::endl;
        delete start_footstep;
        delete goal_footsteps[FOOT_LEFT];
        delete goal_footsteps[FOOT_RIGHT];
        return false;
    }

    std::vector<FootstepNode*> footstep_nodes[2]; // TODO
    footstep_nodes[is_current_footstep_right ? FOOT_RIGHT : FOOT_LEFT].push_back(start_footstep);

    int footstep_stage = is_current_footstep_right ? FOOT_LEFT : FOOT_RIGHT;
    bool found = false;

    for(int i = 0; i < NUMBER_OF_TRIALS; i++) {
        // 1. Make a random footstep
        int support_stage = footstep_stage == FOOT_RIGHT ? FOOT_LEFT : FOOT_RIGHT;
        FootstepNode* next_footstep = make_new_sample(footstep_nodes[support_stage]);

        // 2. Check the availability of the new footstep
        if(!isAvailableFootStep(next_footstep)) {
            delete next_footstep;
            continue;
        }

        // 3. If the goal footstep is within the support region of the new footstep, stop the planning
        // support_footstep(support_foot -> next_footstep -> goal_footstep)
        if(goal_footsteps_availability[support_stage] && isLinkToGoalStep(next_footstep, goal_footsteps[support_stage])) {
            goal_footsteps[support_stage]->parent = next_footstep;
            found = true;
        }

        // 4. Prepare the continuous planning
        footstep_nodes[footstep_stage].push_back(next_footstep);
        footstep_stage = support_stage;

        if(found)
        {
          ROS_INFO("succesful plan after trial :%i", i);
          break;
          ROS_INFO("THIS SHOULDN'T PRINT");
        }

    }

    // Make path
    footsteps.clear();
    if(found) {
        FootstepNode* current = goal_footsteps[footstep_stage];
        while(current != nullptr){
            footsteps.push_back(current->step_conf);
            current = current->parent;
        }
        footsteps.pop_back();
        // std::cout << "Number of nodes: " << footstep_nodes[0].size() + footstep_nodes[1].size() << std::endl;
        // std::cout << "Number of steps: " << footsteps.size() << std::endl;
    }
    else {
        // std::cout << "Cannot find the path" << std::endl;
    }

    // Release memories
    for(auto node : footstep_nodes[0])
        delete node;
    for(auto node : footstep_nodes[1])
        delete node;

    for(int i = 0; i < 2; i++){
        delete goal_footsteps[i];
    }

    return found;
}

void FootstepPlanner::set_initial_footsteps_from_pose(const Configuration& _configuration, FootstepNode& _left_footstep, FootstepNode& _right_footstep)
{
    quadmap::point2d center_to_foot(-(float)(std::sin(_configuration.r()) * D), (float)(std::cos(_configuration.r()) * D));

    _left_footstep.step_conf.x() = _configuration.x() + center_to_foot.x();
    _left_footstep.step_conf.y() = _configuration.y() + center_to_foot.y();
    _left_footstep.step_conf.r() = _configuration.r();

    _right_footstep.step_conf.x() = _configuration.x() - center_to_foot.x();
    _right_footstep.step_conf.y() = _configuration.y() - center_to_foot.y();
    _right_footstep.step_conf.r() = _configuration.r();
}
FootstepNode* FootstepPlanner::make_pair_sample(FootstepNode* parent_footstep_node){
    Configuration pair_footstep_conf;

    if(parent_footstep_node->isRight){
        pair_footstep_conf = Configuration(parent_footstep_node->step_conf.x(), parent_footstep_node->step_conf.y() - (2*D + 0.05) , parent_footstep_node->step_conf.r());
    }
    else{
        pair_footstep_conf = Configuration(parent_footstep_node->step_conf.x(), parent_footstep_node->step_conf.y() + (2*D + 0.05), parent_footstep_node->step_conf.r());
    }

    return new FootstepNode(!(parent_footstep_node->isRight), pair_footstep_conf, parent_footstep_node, parent_footstep_node->cnt_footsteps + 1);
}
FootstepNode* FootstepPlanner::make_new_sample(const std::vector<FootstepNode*>& _footstep_nodes)
{
    // 1. Random selection of node index
    std::random_device generator;
    std::uniform_int_distribution<int> index_distribution(0, _footstep_nodes.size()-1);
    std::uniform_real_distribution<double> width_distribution(SUPPORT_REGION_MIN_X, SUPPORT_REGION_MAX_X);    // 0.05 : to prevent footsteps from overlapping
    std::uniform_real_distribution<double> height_distribution(SUPPORT_REGION_MIN_Y, SUPPORT_REGION_MAX_Y);
#ifdef USE_ROTATION
     std::uniform_real_distribution<double> rotation_distribution(-SUPPORT_REGION_ROTATION, SUPPORT_REGION_ROTATION);
//    std::normal_distribution<double> rotation_distribution(0.0, 0.1);
#endif
    int node_index = index_distribution(generator);
    FootstepNode* parent = _footstep_nodes[node_index];
    quadmap::point2d axis[2];
    double rotation = parent->step_conf.r();
    axis[0] = quadmap::point2d((float)std::cos(rotation), (float)std::sin(rotation)) * width_distribution(generator);
    axis[1] = quadmap::point2d(-(float)std::sin(rotation), (float)std::cos(rotation)) * height_distribution(generator);
    quadmap::point2d next_footstep_offset = parent->isRight ? axis[0] + axis[1] : axis[0] - axis[1];

//    auto next_footstep = new FootstepNode(!parent->isRight, parent->step_conf, parent);
    auto next_footstep = new FootstepNode(!parent->isRight, parent->step_conf, parent, parent->cnt_footsteps+1);
    next_footstep->step_conf.x() += next_footstep_offset.x();
    next_footstep->step_conf.y() += next_footstep_offset.y();
#ifdef USE_ROTATION
    next_footstep->step_conf.r() += (std::min(std::max(rotation_distribution(generator), -SUPPORT_REGION_ROTATION), SUPPORT_REGION_ROTATION));
#endif

    return next_footstep;
}

/**
 * Return cost of one footstep
 *
 * @param _footstep_node
 * @return cost
 */
float FootstepPlanner::calculateCost(const FootstepNode* _footstep_node) const{
    OBB footstep_model(quadmap::point2d(_footstep_node->step_conf.x(), _footstep_node->step_conf.y()), footstep_size, _footstep_node->step_conf.r());

    static std::array<quadmap::point2d, 32> vertices;  // points in order to calculate cost
    footstep_model.get_cost_vertices(vertices);

    float cost = 0.0;   // cost of one footstep.
    static const float direction_cost_factor[8] = {1.0, 0.9, 0.5, 0.3, 1.0, 0.9, 0.5, 0.3}; // {#4 for x/y direction , #4 for diagonal direction}
    static const float num_footstep_cost_factor[5] = {1.0, 0.6, 0.5, 0.4, 0.3}; // The closer footstep is more important than the far footstep.

    // index : counting footsteps
    // index_direction : selecting direction_cost_factor
    int index = 0,  index_direction = 0;

    quadmap::QuadTreeNode* node = nullptr;

    for(const auto& vertex : vertices) {
        node = stepping_stones->search(vertex);
        if(node == NULL || node->getOccupancy() <= 0.5)
            cost += direction_cost_factor[index_direction] * num_footstep_cost_factor[(index < 5) ? index : 4];

        index++;
        if(index % 4 == 0)
            index_direction++;
    }

    return cost;
}

bool FootstepPlanner::isOnSteppingStones(const OBB& _footstep_model)
{
    quadmap::QuadTreeNode* node = stepping_stones->search(_footstep_model.center);
    if(node == NULL || node->getOccupancy() <= 0.5)
        return false;

    std::array<quadmap::point2d, 12> vertices;
    _footstep_model.get_stepping_vertices(vertices);
    for(const auto& vertex : vertices) {
        node = stepping_stones->search(vertex);
        if(node == NULL || node->getOccupancy() <= 0.5)
            return false;
    }

    return true;
}

bool FootstepPlanner::isLinkToGoalStep(const FootstepNode* _next_footstep, const FootstepNode* _goal_footstep)  //next , goal
{
#ifdef USE_ROTATION
    double ROTATION = std::abs(_next_footstep->step_conf.r() - _goal_footstep->step_conf.r());
    if(ROTATION > SUPPORT_REGION_ROTATION)
        return false;
#endif

    quadmap::point2d transformed_center(_goal_footstep->step_conf.x() - _next_footstep->step_conf.x(), _goal_footstep->step_conf.y() - _next_footstep->step_conf.y());
//    transformed_center.rotate_IP(_next_footstep->step_conf.r());

    quadmap::point2d axis[2];
    double rotation = _next_footstep->step_conf.r();
    axis[0] = quadmap::point2d((float)std::cos(rotation), (float)std::sin(rotation));
    axis[1] = quadmap::point2d(-(float)std::sin(rotation), (float)std::cos(rotation));

    double determinant = axis[0].x() * axis[1].y() - axis[0].y() * axis[1].x();
    double factor_x = (axis[1].y() * transformed_center.x() - axis[1].x() * transformed_center.y()) / determinant;
    double factor_y = (-axis[0].y() * transformed_center.x() + axis[0].x() * transformed_center.y()) / determinant;

//    std::cout << "(" << factor_x << ", " << factor_y << ")" << std::endl;

    if(_next_footstep->isRight){     //if parent of goal is right
      //LMY fix that removed support. Y doesnt use min or max for goal
//              return factor_x > FOOTSTEP_WIDTH + SUPPORT_REGION_BIAS && factor_x < SUPPORT_REGION_MAX_X && factor_y > SUPPORT_REGION_BIAS && factor_y < SUPPORT_REGION_HEIGHT;   // 0.05 : to prevent footsteps from overlapping
      //fix that use min or max for goal
      return (factor_x > SUPPORT_REGION_MIN_X) && (factor_x < SUPPORT_REGION_MAX_X) && (factor_y > SUPPORT_REGION_MIN_Y) && (factor_y < SUPPORT_REGION_MAX_Y);
    }

    else{  //if parent of goal is left
      //LMY fix that removed support. Y doesnt use min or max for goal
//      return factor_x > FOOTSTEP_WIDTH + SUPPORT_REGION_BIAS && factor_x < SUPPORT_REGION_MAX_X && -factor_y > SUPPORT_REGION_BIAS && -factor_y < SUPPORT_REGION_HEIGHT;  // 0.05 : to prevent footsteps from overlapping
        return (factor_x > SUPPORT_REGION_MIN_X) && (factor_x < SUPPORT_REGION_MAX_X) && (-factor_y > SUPPORT_REGION_MIN_Y) && (-factor_y < SUPPORT_REGION_MAX_Y);

    }
}

bool FootstepPlanner::isAvailableFootStep(const FootstepNode* _footstep_node)
{
    OBB start_footstep_model(quadmap::point2d(_footstep_node->step_conf.x(), _footstep_node->step_conf.y()), footstep_size, _footstep_node->step_conf.r());

    if (CollisionChecker::doCollide(this->environment, start_footstep_model))
        return false;

    return isOnSteppingStones(start_footstep_model);
}
void FootstepPlanner::transform_footsteps(const tf::StampedTransform &tf_transform) {
    double roll, pitch, yaw;

    tf::Vector3 translation(tf_transform.getOrigin());
    tf_transform.getBasis().getRPY(roll, pitch, yaw);

    for(int i = (int)footsteps.size()-1; i >= 0; i--) {
        footsteps[i].transform(translation.x(), translation.y(), yaw);
    }
}
