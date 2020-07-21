import random
import rospy

def run_ransac(data, estimate, is_inlier, sample_size, goal_inliers, max_iterations, stop_at_goal=True, random_seed=None):
    best_ic = 0
    best_model = None
    random.seed(random_seed)
    # random.sample cannot deal with "data" being a numpy array
    for i in range(max_iterations):
        s = random.sample(data, int(sample_size))
        m = estimate(s)
        ic = 0
        for j in range(len(data)):
            if is_inlier(m, data[j]):
                ic += 1
        if ic > best_ic:
            best_i = i+1
            best_ic = ic
            best_model = m
            if ic > goal_inliers and stop_at_goal:
                break
    rospy.logdebug('total: {}, at iteration: {}, explains: {} out of: {}'.format(i+1, best_i, best_ic, len(data)))
    return best_model, best_ic

def get_others(data, model, is_inlier):
    plane = []
    others = []
    for j in range(len(data)):
        if is_inlier(model, data[j]):
            plane.append(data[j])
        else:
            others.append(data[j])
    return plane, others

