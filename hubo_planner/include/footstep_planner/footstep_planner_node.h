#ifndef HUBO_PLANNER_FOOTSTEP_PLANNER_NODE_H
#define HUBO_PLANNER_FOOTSTEP_PLANNER_NODE_H

#include <hubo_planner/configuration.h>

struct FootstepNode {
    FootstepNode(bool _isRight) : isRight(_isRight), step_conf(), parent(nullptr), cnt_footsteps(0), cumulative_loss(0.0) {}
    FootstepNode(bool _isRight, const Configuration& _step_conf) : isRight(_isRight), step_conf(_step_conf), parent(nullptr), cnt_footsteps(0), cumulative_loss(0.0) {}
    FootstepNode(bool _isRight, const Configuration& _step_conf, FootstepNode* _parent) : isRight(_isRight), step_conf(_step_conf), parent(_parent), cnt_footsteps(_parent->cnt_footsteps+1), cumulative_loss(0.0) {}
    FootstepNode(bool _isRight, const Configuration& _step_conf, FootstepNode* _parent, const int& _cnt_footsteps) : isRight(_isRight), step_conf(_step_conf), parent(_parent), cnt_footsteps(_cnt_footsteps), cumulative_loss(0.0) {}
    FootstepNode(bool _isRight, const Configuration& _step_conf, FootstepNode* _parent, const int& _cnt_footsteps, const float& _cumulative_loss) : isRight(_isRight), step_conf(_step_conf), parent(_parent), cnt_footsteps(_cnt_footsteps), cumulative_loss(_cumulative_loss) {}
    FootstepNode(const FootstepNode&_other) : isRight(_other.isRight), step_conf(_other.step_conf), parent(_other.parent), cnt_footsteps(_other.cnt_footsteps), cumulative_loss(0.0) {}

    bool            isRight;
    Configuration   step_conf;  // configuration of footstep
    FootstepNode*   parent;
    int             cnt_footsteps;
    float           cumulative_loss;
};

#endif //HUBO_PLANNER_FOOTSTEP_PLANNER_NODE_H
