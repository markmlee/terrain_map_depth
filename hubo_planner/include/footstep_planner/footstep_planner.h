#ifndef HUBO_PLANNER_FOOTSTEP_PLANNER_H
#define HUBO_PLANNER_FOOTSTEP_PLANNER_H

#include <quadmap/quadmap.h>
#include <tf/transform_listener.h>

#include <footstep_planner/footstep_planner_node.h>
#include <collision_detector/collision_detector.h>

#include <random>

//#define USE_ROTATION
#define USE_OPTIMIZATION

class FootstepPlanner {
public:
    // OPTIONS =========================================================================================================
  //THIS IS FOR NARROW PATH

  const double D = 0.09; // Displacement between center of body and center of foot
//    const double SUPPORT_REGION_WIDTH  = 0.09;   // maximum range of next footstep in forward direction [m]
    const double SUPPORT_REGION_HEIGHT = 0.30;   // maximum range of next footstep in side direction [m]
    const double SUPPORT_REGION_BIAS   = 0.02;  //

    const double SUPPORT_REGION_MIN_X  = 0.27;  // 0.27m = FOOTSET_WIDTH(=0.22m) + SUPPORT_REGION_BIAS(=0.05m)
    const double SUPPORT_REGION_MAX_X  = 0.30;  // 0.62m = SUPPORT_REGION_MIN_X(=0.27m) + SUPPORT_REGION_WIDTH(=0.35m)
    const double SUPPORT_REGION_MIN_Y  = 0.10;  // 0.05m = SUPPORT_REGION_BIAS(=0.05m)
    const double SUPPORT_REGION_MAX_Y  = 0.30;  // 0.30m = SUPPORT_REGION_MAX_X(=0.05m) + SUPPORT_REGION_HEIGHT(=0.30m)

    //THIS IS FOR STEPPING STONE
/*
    const double D = 0.09; // Displacement between center of body and center of foot
    /*
//    const double SUPPORT_REGION_WIDTH  = 0.09;   // maximum range of next footstep in forward direction [m]
    const double SUPPORT_REGION_HEIGHT = 0.30;   // maximum range of next footstep in side direction [m]
    const double SUPPORT_REGION_BIAS   = 0.05;  //

    const double SUPPORT_REGION_MIN_X  = 0.15;  // 0.27m = FOOTSET_WIDTH(=0.22m) + SUPPORT_REGION_BIAS(=0.05m)
    const double SUPPORT_REGION_MAX_X  = 0.32;  // 0.62m = SUPPORT_REGION_MIN_X(=0.27m) + SUPPORT_REGION_WIDTH(=0.35m)
    const double SUPPORT_REGION_MIN_Y  = 0.20;  // 0.05m = SUPPORT_REGION_BIAS(=0.05m)
    const double SUPPORT_REGION_MAX_Y  = 0.35;  // 0.30m = SUPPORT_REGION_MAX_X(=0.05m) + SUPPORT_REGION_HEIGHT(=0.30m)
*/

#ifdef USE_ROTATION
    const double SUPPORT_REGION_ROTATION = 0.0174533;//0.174533; // maximum range of next footstep in rotation [rad]
#endif

#ifdef USE_OPTIMIZATION

#endif
    const unsigned int NUMBER_OF_TRIALS = 1000; //narrowpath //stepping 5000

    enum {
        FOOT_LEFT  = 0,
        FOOT_RIGHT = 1
    };

public:
    FootstepPlanner(const double _footstep_width, const double _footstep_height)
    : environment(nullptr), stepping_stones(nullptr), FOOTSTEP_WIDTH(_footstep_width), FOOTSTEP_HEIGHT(_footstep_height)
    {
        footstep_size = quadmap::point2d((float)_footstep_width, (float)_footstep_height);
    }
    /*
     *
     */
    bool planning(const Configuration& _start_conf, const int& target_num_footsteps);
    /*
     *
     */
    bool planning(const Configuration& _start_conf, const Configuration& _goal_conf);

    /*
     *
     */
    bool planning_maximum_length_optimal_solution(const Configuration& _start_conf);
    /*
     *
     */
    bool planning_maximum_length(const Configuration& _start_conf);
    /*
     *
     */
    bool planning(const Configuration& _goal_conf);

    /*
     *
     */
    void set_environment(const quadmap::QuadTree* _environment) { this->environment = _environment; }

    /*
     *
     */
    void set_stepping_stones(const quadmap::QuadTree* _stepping_stones) { this->stepping_stones = _stepping_stones; }

    /*
     *
     */
    void transform_footsteps(const tf::StampedTransform& tf_transform);
    /*
     *
     */
    const std::vector<Configuration>& get_footsteps() const { return footsteps; }
    /*
     *
     */

    const Configuration& get_next_footstep() const { return footsteps[footsteps.size()-2]; }


    /*
     *
     */
    bool is_next_footstep_right() const { return !is_current_footstep_right; }

    /*
     *
     */
    void set_current_footstep_configuration(const Configuration& _configuration, bool _is_right) {
        current_footstep_configuration = _configuration;
        is_current_footstep_right = _is_right;
    }
    FootstepNode* get_current_footstep_node() {
        return new FootstepNode(is_current_footstep_right, current_footstep_configuration);
    }
    void set_current_footstep_configuration(const Configuration& _configuration) {
        current_footstep_configuration = _configuration;
        is_current_footstep_right = !is_current_footstep_right;
    }
    void set_current_footstep_configuration() {
        if(footsteps.size() < 2)
            return;
        current_footstep_configuration = footsteps[footsteps.size()-2];
        is_current_footstep_right = !is_current_footstep_right;
    }
    void set_current_footstep(bool is_right){
        is_current_footstep_right = is_right;
    }
     bool get_current_footstep(){
        return is_current_footstep_right;
    }




protected:
    const quadmap::QuadTree* environment;
    const quadmap::QuadTree* stepping_stones;

    Configuration current_footstep_configuration;
    bool          is_current_footstep_right = true;

    /*
     *
     */
    void set_initial_footsteps_from_pose(const Configuration& _configuration, FootstepNode& _left_footstep, FootstepNode& _right_footstep);

    /*
     *
     */

    FootstepNode* make_pair_sample(FootstepNode* parent_footstep_node);
    /*
     *
     */
    FootstepNode* make_new_sample(const std::vector<FootstepNode*>& _footstep_nodes);
    /*
     *
     */
    float calculateCost(const FootstepNode* _footstep_node) const;
    /*
     * Check whether the footstep is on the stepping stones.
     * The function checks the center of footstep and four vertices.
     * This computation can be inaccurate but computationally efficient.
     */

    inline bool isOnSteppingStones(const OBB& _footstep_model);

    /*
     *
     */
    inline bool isLinkToGoalStep(const FootstepNode* _A, const FootstepNode* _B);

    /*
     *
     */
    bool isAvailableFootStep(const FootstepNode* const _footstep_node);
    /*
     *
     */

    const double        FOOTSTEP_WIDTH;
    const double        FOOTSTEP_HEIGHT;

    quadmap::point2d    footstep_size;

    std::vector<Configuration> footsteps;   // 0-index: goal footstep, last index: start(previous) footstep
};

#endif //HUBO_PLANNER_FOOTSTEP_PLANNER_H
