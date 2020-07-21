#!/usr/bin/env python

from ransac_lib import *
from floor_octomap.srv import *
import rospy

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

from sklearn.cluster import AgglomerativeClustering 
from scipy.spatial import ConvexHull, Delaunay

def plot_plane(a, b, c, d, h, w):
    xx, yy = np.mgrid[:h, :w]
    return xx, yy, (-d - a * xx - b * yy) / c

def augment(points):
    aug_pts = np.ones((len(points), 4))
    aug_pts[:, :3] = points
    return aug_pts

def estimate(points):
    aug_pts = augment(points[:3])
    return np.linalg.svd(aug_pts)[-1][-1, :]

def is_inlier(coeffs, point, threshold):
    return np.abs(coeffs.dot(augment([point]).T)) < threshold

def in_hull(p, hull):
    if not isinstance(hull,Delaunay):
        hull = Delaunay(hull)

    return hull.find_simplex(p)>=0

def flatten(img):
    points = []
    h, w = np.shape(img)

    # Add all valid points to "points"
    for i in range(h):
        for j in range(w):
            if not img[i,j] == 0 and img[i,j] <= 40:
                points.append([i,j,img[i,j]])

    points = np.asarray(points)
    point_list = list(points)
    N = len(point_list)

    # Use RANSAC to get ground plane as "gnd_list" and the rest as "obj_list"
    max_iterations = 100
    goal_inliers = len(point_list) * 0.3
    m, _ = run_ransac(point_list, estimate, lambda x, y: is_inlier(x, y, 0.0005), 3, goal_inliers, max_iterations)
    list_1, list_2 = get_others(point_list, m, lambda x, y: is_inlier(x, y, 0.03))

    a, b, c, d = m
    _, _, zz = plot_plane(a, b, c, d, h, w)
    h1 = int(round(np.average(zz)))


    goal_inliers = len(list_2) * 0.3
    m, _ = run_ransac(list_2, estimate, lambda x, y: is_inlier(x, y, 0.0005), 3, goal_inliers, max_iterations)
    list_3, list_4 = get_others(list_2, m, lambda x, y: is_inlier(x, y, 0.03))

    a, b, c, d = m
    _, _, zz = plot_plane(a, b, c, d, h, w)
    h2 = int(round(np.average(zz)))
    
    if h1 < h2:
        gnd_list = list_1
        obj_list = list_2
        gnd_height = h1
        step_height = h2
    else:
        gnd_list = list_3
        obj_list = list_1 + list_4
        gnd_height = h2
        step_height = h1

    if len(obj_list):
        obj = np.asarray(obj_list)
        filt_obj_list = []
        obj_2d = obj[:,:2]

        if len(obj_list) > N * 0.1:
            topview = np.zeros((h,w,2))
            obj_img = np.zeros((h,w))
            for o in obj_list:
                obj_img[o[0],o[1]] = o[2]
            obj_img = obj_img.astype(np.uint8)
            obj_img = cv2.bilateralFilter(obj_img,5,75,75)
            for o in obj_list:
                if obj_img[o[0],o[1]] > gnd_height + 1:
                    filt_obj_list.append(o)
                else:
                    gnd_list.append([o[0],o[1],gnd_height])

        if len(filt_obj_list):
            filt_obj = np.asarray(filt_obj_list)
            filt_obj_2d = filt_obj[:,:2]

            # CLUSTER		
            estimator = AgglomerativeClustering(linkage='single')
            estimator.fit(obj_2d)
            labels = estimator.labels_
            cluster_list = [[] for c in range(estimator.n_clusters)]
            for idx, val in enumerate(obj_list):
                cluster_list[labels[idx]].append(val)

            for idx, cluster in enumerate(cluster_list):
                N_total = len(cluster)
                N_now = N_total
                cluster = np.asarray(cluster)
                h_cnt = {}
                for p in cluster:
                    if not p[2] in h_cnt:
                        h_cnt[p[2]] = 0
                    h_cnt[p[2]] += 1

                while h_cnt:
                    height = max(h_cnt)
                    height_count = h_cnt[height]
                    del h_cnt[height]
                    if height_count/float(N_total) < 0.1 or height_count < 10:
                        continue
                    plane = cluster[cluster[:,2] == height]
                    cluster = cluster[cluster[:,2] != height]

                    plane_2d = plane[:,:2]
                    N_now = len(cluster)

                    try:
                        hull = ConvexHull(plane_2d)
                    except Exception as e:
                        continue
                    edge_2d = np.array(plane_2d[hull.vertices])
                    c_x = sum([p[0] for p in plane_2d]) / len(plane_2d)
                    c_y = sum([p[1] for p in plane_2d]) / len(plane_2d)

                    tri = Delaunay(edge_2d)

                    fill = []
                    for i in range(min(edge_2d[:,0]),max(edge_2d[:,0])+1):
                        for j in range(min(edge_2d[:,1]),max(edge_2d[:,1])+1):
                            fill.append([i,j])
                    fill = np.array(fill)
                    mask = in_hull(fill, tri)
                    fill = fill[mask]
                    if topview[c_x,c_y,0] == 0 or abs(topview[c_x,c_y,1]-height) > 2:
                        for f in fill:
                            if topview[f[0],f[1],0] == 0:
                                img[f[0],f[1]] = height
                            topview[f[0],f[1],0] = height
                            topview[f[0],f[1],1] = height
                    else:
                        new_height = topview[c_x,c_y,0]
                        for f in fill:
                            if topview[f[0],f[1],0] == 0:
                                img[f[0],f[1]] = new_height
                            topview[f[0],f[1],0] = new_height
                            topview[f[0],f[1],1] = height
    
    if len(gnd_list):
        gnd = np.asarray(gnd_list)
        for g in gnd:
            img[g[0],g[1]] = 0

    img[img > step_height] = 0
    # print("Out")
    # print(h,w,img[0,0],img[0,w-1],img[h-1,0],img[h-1,w-1])
    return img

def flatten_2(img):
    points = []
    h, w = np.shape(img)

    # Add all valid points to "points"
    for i in range(h):
        for j in range(w):
            if not img[i,j] == 0 and img[i,j] <= 40:
                points.append([i,j,img[i,j]])

    points = np.asarray(points)
    point_list = list(points)
    N = len(point_list)

    # Use RANSAC to get ground plane as "gnd_list" and the rest as "obj_list"
    max_iterations = 100
    goal_inliers = len(point_list) * 0.3
    m, _ = run_ransac(point_list, estimate, lambda x, y: is_inlier(x, y, 0.0005), 3, goal_inliers, max_iterations)
    list_1, list_2 = get_others(point_list, m, lambda x, y: is_inlier(x, y, 0.03))

    a, b, c, d = m
    _, _, zz = plot_plane(a, b, c, d, h, w)
    h1 = int(round(np.average(zz)))


    goal_inliers = len(list_2) * 0.3
    m, _ = run_ransac(list_2, estimate, lambda x, y: is_inlier(x, y, 0.0005), 3, goal_inliers, max_iterations)
    list_3, list_4 = get_others(list_2, m, lambda x, y: is_inlier(x, y, 0.03))

    a, b, c, d = m
    _, _, zz = plot_plane(a, b, c, d, h, w)
    h2 = int(round(np.average(zz)))
    
    step_height = max(h1, h2)

    img[img == step_height+1] = step_height
    img[img == step_height-1] = step_height
    img[img != step_height] = 0

    return img

def flatten_3(img):
    global step_height
    if step_height < 0:        
        points = []
        h, w = np.shape(img)

        # Add all valid points to "points"
        for i in range(h):
            for j in range(w):
                if not img[i,j] == 0 and img[i,j] <= 40:
                    points.append([i,j,img[i,j]])

        points = np.asarray(points)
        point_list = list(points)
        N = len(point_list)

        # Use RANSAC to get ground plane as "gnd_list" and the rest as "obj_list"
        max_iterations = 100
        goal_inliers = len(point_list) * 0.4
        m, _ = run_ransac(point_list, estimate, lambda x, y: is_inlier(x, y, 0.0005), 3, goal_inliers, max_iterations)
        list_1, list_2 = get_others(point_list, m, lambda x, y: is_inlier(x, y, 0.03))

        a, b, c, d = m
        _, _, zz = plot_plane(a, b, c, d, h, w)
        h1 = int(round(np.average(zz)))

        max_iterations = 200
        goal_inliers = len(list_2) * 0.5
        m, _ = run_ransac(list_2, estimate, lambda x, y: is_inlier(x, y, 0.0005), 3, goal_inliers, max_iterations)
        list_3, list_4 = get_others(list_2, m, lambda x, y: is_inlier(x, y, 0.03))

        a, b, c, d = m
        _, _, zz = plot_plane(a, b, c, d, h, w)
        h2 = int(round(np.average(zz)))
        print(h1, h2, np.max(img))
        step_height = max(h1, h2)

    img[img == step_height+1] = step_height
    img[img == step_height-1] = step_height
    img[img != step_height] = 0

    return img
    

def call_flatten(req):
    # img = np.array(req.input.data).reshape(req.input.height, req.input.width)
    bridge = CvBridge()
    try:
        img = np.array(bridge.imgmsg_to_cv2(req.input, "mono8"))
    except CvBridgeError as e:
        print(e)
    # save_img(img)
    orig_img = np.array(img, copy=True)
    new_img = flatten_3(img)
    return OctoImageResponse(bridge.cv2_to_imgmsg(new_img, "mono8"))

# def save_img(img):
#     global tmp
#     if tmp >= 0:
#         np.save("/home/junyong/Desktop/floorslam/2020/{}.npy".format(tmp), img)
#         tmp+=1

step_height = -1
# tmp = -1
rospy.init_node('flatten_octomap_server_0416')
s = rospy.Service('flatten_octomap_0416', OctoImage, call_flatten)
rospy.loginfo("Starting flatten_octomap_server_0416")

rospy.spin()