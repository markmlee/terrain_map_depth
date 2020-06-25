#ifndef HUBO_PLANNER_FOOTSTEP_PLANNER_NODE_H
#define HUBO_PLANNER_FOOTSTEP_PLANNER_NODE_H

#include <hubo_planner/configuration.h>

struct FootstepNode {
    FootstepNode(bool _isRight) : isRight(_isRight), step_conf(), parent(nullptr) {}
    FootstepNode(bool _isRight, const Configuration& _step_conf) : isRight(_isRight), step_conf(_step_conf), parent(nullptr) {}
    FootstepNode(bool _isRight, const Configuration& _step_conf, FootstepNode* _parent) : isRight(_isRight), step_conf(_step_conf), parent(_parent) {}
    FootstepNode(const FootstepNode&_other) : isRight(_other.isRight), step_conf(_other.step_conf), parent(_other.parent) {}

    bool            isRight;
    Configuration   step_conf;  // configuration of footstep
    FootstepNode*   parent;
};

#endif //HUBO_PLANNER_FOOTSTEP_PLANNER_NODE_H
