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
            if not img[i,j] == 0:
                points.append([i,j,img[i,j]])

    points = np.asarray(points)
    point_list = list(points)

    # Use RANSAC to get ground plane as "gnd_list" and the rest as "obj_list"
    N = len(point_list)
    max_iterations = 100
    goal_inliers = N * 0.3
    m, ic = run_ransac(point_list, estimate, lambda x, y: is_inlier(x, y, 0.0005), 3, goal_inliers, max_iterations)
    gnd_list, obj_list = get_others(point_list, m, lambda x, y: is_inlier(x, y, 0.03))

    a, b, c, d = m
    xx, yy, zz = plot_plane(a, b, c, d, h, w)
    gnd_height = int(round(np.average(zz)))

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
            img[g[0],g[1]] = gnd_height
    return img

def flatten_2(img):
    GND_MAX = 46
    GND_VAL = 44
    STEP_MIN = 50
    STEP_MAX = 56
    STEP_VAL = 56
    h, w = np.shape(img)
    for i in range(h):
        for j in range(w):
            if not img[i,j] == 0:
                if img[i,j] <= GND_MAX:
                    img[i,j] = GND_VAL
                elif STEP_MIN <= img[i,j] <= STEP_MAX:
                    img[i,j] = STEP_VAL
    return img

def call_flatten(req):
    # img = np.array(req.input.data).reshape(req.input.height, req.input.width)
    bridge = CvBridge()
    try:
        img = np.array(bridge.imgmsg_to_cv2(req.input, "mono8"))
    except CvBridgeError as e:
        print(e)

    new_img = flatten(img)
    # new_img = flatten_2(img)
    return OctoImageResponse(bridge.cv2_to_imgmsg(new_img, "mono8"))

rospy.init_node('flatten_octomap_server')
s = rospy.Service('flatten_octomap', OctoImage, call_flatten)
rospy.loginfo("Starting flatten_octomap_server")

rospy.spin()