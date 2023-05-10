#!/usr/bin/env python
"""***********************************************************************************
     C++ Source Codes for "Autonomous Driving on Curvy Roads without Reliance on
     Frenet Frame: A Cartesian-based Trajectory Planning Method".
 ***********************************************************************************
     Copyright (C) 2022 Bai Li
     Users are suggested to cite the following article when they use the source codes.
     Bai Li et al., "Autonomous Driving on Curvy Roads without Reliance on
     Frenet Frame: A Cartesian-based Trajectory Planning Method",
     IEEE Transactions on Intelligent Transportation Systems, 2022.
***********************************************************************************"""
import bisect
import sys
import rospy
import numpy as np
import pickle
import os
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Path
from pjpo_curvature.msg import CenterLine, CenterLinePoint, Obstacles, DynamicObstacles, DynamicObstacle, \
    DynamicTrajectoryPoint, FrenetPath, FrenetPoint
from geometry_msgs.msg import Polygon, PoseArray
# import rosbag
import matplotlib
import matplotlib.pyplot as plt
import seaborn as sns
import math
import numpy
import statistics
from matplotlib import rcParams
import matplotlib.font_manager as fm
from matplotlib.font_manager import FontProperties


font_ = 15
config = {
    "font.family":'serif',
    "font.size": font_,
    "mathtext.fontset":'stix',
    "font.serif": ['TimesNewRoman'],
}
rcParams.update(config)
chinese_font = FontProperties(fname='/usr/share/matplotlib/mpl-data/fonts/ttf/FangSong.ttf')
english_font = FontProperties(fname='/usr/share/matplotlib/mpl-data/fonts/ttf/TimesNewRoman.ttf')
# display parameters
# kappa_limit = 0
kappa_limit_axis = []
kappa_limit_max_data = []
kappa_limit_min_data = []

reference_l_axis = []
reference_kappa_data = []
reference_dkappa_data = []
origin_l_axis = []
origin_kappa_data = []
origin_dkappa_data = []
new_l_axis = []
new_kappa_data = []
new_dkappa_data = []
new1_l_axis = []
new1_kappa_data = []
new1_dkappa_data = []

# display_iteration = True
display_iteration = False
class sl_path:
    def __init__(self):
        self.x = []
        self.y = []
        self.s = []
        self.l = []
        self.dl = []
        self.ddl = []
        self.dddl = []
        self.kappa = []
        self.dkappa = []
        self.left_bound = []
        self.right_bound = []
        self.left_bound_x = []
        self.left_bound_y = []
        self.right_bound_x = []
        self.right_bound_y = []
        self.mean = FrenetPoint()
        self.max = FrenetPoint()

    def record_path(self, input_path_):
        self.x = []
        self.y = []
        self.s = []
        self.l = []
        self.dl = []
        self.ddl = []
        self.dddl = []
        self.kappa = []
        self.dkappa = []
        self.left_bound = []
        self.right_bound = []
        self.left_bound_x = []
        self.left_bound_y = []
        self.right_bound_x = []
        self.right_bound_y = []
        # self.mean = FrenetPoint()
        # self.max = FrenetPoint()
        for point in input_path_:
            self.x.append(point.x)
            self.y.append(point.y)
            self.s.append(point.s)
            self.l.append(point.l)
            self.dl.append(point.dl)
            self.ddl.append(point.ddl)
            self.dddl.append(point.dddl)
            self.kappa.append(point.kappa)
            self.dkappa.append(point.dkappa)
            self.left_bound.append(point.left_bound)
            self.right_bound.append(point.right_bound)
            self.left_bound_x.append(point.left_bound_x)
            self.left_bound_y.append(point.left_bound_y)
            self.right_bound_x.append(point.right_bound_x)
            self.right_bound_y.append(point.right_bound_y)
        self.mean.l = np.mean(self.l)
        self.mean.dl = np.mean(self.dl)
        self.mean.ddl = np.mean(self.ddl)
        self.mean.kappa = np.mean(self.kappa)
        self.mean.dkappa = np.mean(self.dkappa)
        self.max.l = max(np.max(self.l), - np.min(self.l)) 
        self.max.dl = max(np.max(self.dl), - np.min(self.dl)) 
        self.max.ddl = max(np.max(self.ddl), - np.min(self.ddl)) 
        self.max.kappa = max(np.max(self.kappa), - np.min(self.kappa)) 
        self.max.dkappa = max(np.max(self.dkappa), - np.min(self.dkappa)) 

reference_path = sl_path()
origin_qp_path = sl_path()
proposed_qp_path = sl_path()
proposed_qp_path_1 = sl_path()


def path_callback(data):
    if data.id == "SmoothedReference":
        reference_path.record_path(data.path)
        print(1)
    if data.id == "originQP":
        origin_qp_path.record_path(data.path)
        print(2)
    if data.id == "newQP":
        proposed_qp_path.record_path(data.path)    
        print(3)  
    if data.id == "newQP_iteration4":
        proposed_qp_path_1.record_path(data.path)
    # print(len(proposed_qp_path_1.x))



def kappa_callback(data):
    if data.header.frame_id == "SmoothedReference":
        reference_l_axis.clear()
        reference_kappa_data.clear()
        reference_dkappa_data.clear()
        for pose in data.poses:
            reference_l_axis.append(pose.position.x)
            reference_kappa_data.append(pose.position.y)
            reference_dkappa_data.append(pose.position.z)
    if data.header.frame_id == "originQP":
        origin_l_axis.clear()
        origin_kappa_data.clear()
        origin_dkappa_data.clear()
        for pose in data.poses:
            origin_l_axis.append(pose.position.x)
            origin_kappa_data.append(pose.position.y)
            origin_dkappa_data.append(pose.position.z)
    if data.header.frame_id == "newQP":
        new_l_axis.clear()
        new_kappa_data.clear()
        new_dkappa_data.clear()
        for pose in data.poses:
            new_l_axis.append(pose.position.x)
            new_kappa_data.append(pose.position.y)
            new_dkappa_data.append(pose.position.z)
    if display_iteration == True and data.header.frame_id == "newQP_iteration4":
        new1_l_axis.clear()
        new1_kappa_data.clear()
        new1_dkappa_data.clear()
        for pose in data.poses:
            new1_l_axis.append(pose.position.x)
            new1_kappa_data.append(pose.position.y)
            new1_dkappa_data.append(pose.position.z)
    # if data.header.frame_id == "newQP_iteration2":
    #     new1_l_axis.clear()
    #     new1_kappa_data.clear()
    #     new1_dkappa_data.clear()
    #     for pose in data.poses:
    #         new1_l_axis.append(pose.position.x)
    #         new1_kappa_data.append(pose.position.y)
    #         new1_dkappa_data.append(pose.position.z)

    


def main():
    rospy.init_node('draw_kappa')
    rospy.Subscriber("kappa_variables", PoseArray, kappa_callback)
    rospy.Subscriber("sl_paths", FrenetPath, path_callback)
    kappa_limit = rospy.get_param('kappa_limit')
    print(matplotlib.__file__)
    # rospy.spin()
    print("test")
    # plt.figure(figsize=(10, 3))  # 设置画布的尺寸
    while not rospy.is_shutdown():
        if len(origin_l_axis) > 1 and len(new_l_axis) > 1:
            kappa_limit_axis.append(reference_l_axis[0])
            kappa_limit_axis.append(reference_l_axis[-1])
            kappa_limit_max_data.append(kappa_limit)
            kappa_limit_max_data.append(kappa_limit)
            kappa_limit_min_data.append(-kappa_limit)
            kappa_limit_min_data.append(-kappa_limit)
            if (display_iteration == True and len(new1_l_axis) > 1) or display_iteration == False:
                print("test")
                # plt.figure(1)
                plt.figure(1, figsize=(6, 4))  # 设置画布的尺寸
                # plt.subplot(2, 1, 1)
                # f1 = fm.FontProperties(fname='方正卡通简体.ttf', size=font_)
                # plt.xlabel('s(m)', fontdict={'family' : 'TimesNewRoman', 'size'   : font_})
                plt.xlabel('$s$ $(m)$', fontdict={'family' : 'TimesNewRoman', 'size'   : font_})
                plt.ylabel('$\kappa$', fontdict={'family' : 'TimesNewRoman', 'size'   : font_})
                plt.ylim(-0.25, 0.25)
                plt.yticks(fontproperties = english_font, size = font_)
                plt.xticks(fontproperties = english_font, size = font_)
                plt.plot(reference_l_axis, reference_kappa_data, label = 'Guide Line')
                plt.plot(origin_l_axis, origin_kappa_data, label = 'PJPO')
                plt.plot(new_l_axis, new_kappa_data, label = 'PJPO-C')
                if display_iteration == True:
                    plt.plot(new1_l_axis, new1_kappa_data, label = 'Proposed iteration')
                    # plt.legend(['参考线', 'PJPO算法', 'PJPO-C算法', 'Proposed iteration'])
                else:
                    plt.legend(['Guide Line', 'PJPO', 'PJPO-C'])
                plt.plot(kappa_limit_axis, kappa_limit_max_data,'k-.', label = 'kappa_limit_max')
                plt.plot(kappa_limit_axis, kappa_limit_min_data,'k-.', label = 'kappa_limit_min')
                # plt.title('$\kappa - s$',fontdict={'family' : 'TimesNewRoman', 'size'   : font_+2})
                plt.tight_layout()
                prefix = '/home/zza/Documents/english'
                filename = prefix + 'merge-k-s.pdf'
                plt.savefig(filename,format='pdf', bbox_inches='tight', pad_inches=0.03)
                # plt.grid()
                # plt.show()
                
                
                
                # plt.subplot(2, 1, 2)
                plt.figure(2, figsize=(6, 4))  # 设置画布的尺寸
                plt.xlabel('$s$ $(m)$', fontdict={'family' : 'TimesNewRoman', 'size'   : font_})
                plt.ylabel('$\kappa\prime$', fontdict={'family' : 'TimesNewRoman', 'size'   : font_})
                # plt.ylim(-0.5, 0.5)
                plt.yticks(fontproperties = english_font, size = font_)
                plt.xticks(fontproperties = english_font, size = font_)
                plt.plot(reference_l_axis, reference_dkappa_data, label = 'Guide Line')
                plt.plot(origin_l_axis, origin_dkappa_data, label = 'PJPO')
                plt.plot(new_l_axis, new_dkappa_data, label = 'PJPO-C')
                if display_iteration == True:
                    plt.plot(new1_l_axis, new1_dkappa_data, label = 'Proposed iteration')
                    # plt.legend(['参考线', 'PJPO算法', 'PJPO-C算法', 'Proposed iteration'])
                # else:
                #     plt.legend(['参考线', 'PJPO算法', 'PJPO-C算法'])
                # plt.title('$\kappa\prime-s$',fontdict={'family' : 'TimesNewRoman', 'size'   : font_+2})
                plt.tight_layout()
                filename = prefix + 'merge-dk-s.pdf'
                plt.savefig(filename,format='pdf', bbox_inches='tight', pad_inches=0.03)
                # plt.grid()
                # plt.show()


                # 
                plt.figure(3, figsize=(8, 5))  # 设置画布的尺寸
                plt.xlabel('$x$ $(m)$', fontdict={'family' : 'TimesNewRoman', 'size'   : font_})
                plt.ylabel('$y$ $(m)$', fontdict={'family' : 'TimesNewRoman', 'size'   : font_})
                # plt.ylim(-0.5, 0.5)
                plt.yticks(fontproperties = english_font, size = font_)
                plt.xticks(fontproperties = english_font, size = font_)
                plt.plot(reference_path.x, reference_path.y, label = 'Guide Line')
                plt.plot(origin_qp_path.x, origin_qp_path.y, label = 'PJPO')
                plt.plot(proposed_qp_path.x, proposed_qp_path.y, label = 'PJPO-C')
                if display_iteration == True:
                    plt.plot(proposed_qp_path_1.x, proposed_qp_path_1.y, label = 'Proposed iteration')
                    plt.legend(['Guide Line', 'PJPO', 'PJPO-C', 'Proposed iteration'])
                else:
                    plt.legend(['Guide Line', 'PJPO', 'PJPO-C'])
                plt.plot(proposed_qp_path.left_bound_x, proposed_qp_path.left_bound_y,'k-.')
                plt.plot(proposed_qp_path.right_bound_x, proposed_qp_path.right_bound_y,'k-.')
                # # plt.title(chr(954).lower()+'\' - s',fontdict={'family' : 'TimesNewRoman', 'size'   : font_+2})
                plt.tight_layout()
                filename = prefix + 'merge-x-y.pdf'
                plt.savefig(filename,format='pdf', bbox_inches='tight', pad_inches=0.03)
                # plt.grid()
                # plt.show()

                plt.figure(4, figsize=(8, 4))  # 设置画布的尺寸
                plt.xlabel('$s$ $(m)$', fontdict={'family' : 'TimesNewRoman', 'size'   : font_})
                plt.ylabel('$l$ $(m)$', fontdict={'family' : 'TimesNewRoman', 'size'   : font_})
                # plt.ylim(-0.5, 0.5)
                plt.yticks(fontproperties = english_font, size = font_)
                plt.xticks(fontproperties = english_font, size = font_)
                plt.plot(reference_path.s, reference_path.l, label = 'Guide Line')
                plt.plot(origin_qp_path.s, origin_qp_path.l, label = 'PJPO')
                plt.plot(proposed_qp_path.s, proposed_qp_path.l, label = 'PJPO-C')
                if display_iteration == True:
                    plt.plot(proposed_qp_path_1.s, proposed_qp_path_1.l, label = 'Proposed iteration')
                    plt.legend(['Guide Line', 'PJPO', 'PJPO-C', 'Proposed iteration'])
                else:
                    plt.legend(['Guide Line', 'PJPO', 'PJPO-C'])
                # plt.title('$l - s$',fontdict={'family' : 'TimesNewRoman', 'size'   : font_+2})
                plt.plot(proposed_qp_path.s, proposed_qp_path.left_bound,'k-.')
                plt.plot(proposed_qp_path.s, proposed_qp_path.right_bound,'k-.')
                plt.tight_layout()
                filename = prefix + 'merge-l-s.pdf'
                plt.savefig(filename,format='pdf', bbox_inches='tight', pad_inches=0.03)
                # plt.grid()

                plt.figure(5, figsize=(6, 4))  # 设置画布的尺寸
                plt.xlabel('$s$ $(m)$', fontdict={'family' : 'TimesNewRoman', 'size'   : font_})
                plt.ylabel('$l\prime$', fontdict={'family' : 'TimesNewRoman', 'size'   : font_})
                # plt.ylim(-0.5, 0.5)
                plt.yticks(fontproperties = english_font, size = font_)
                plt.xticks(fontproperties = english_font, size = font_)
                plt.plot(reference_path.s, reference_path.dl, label = 'Guide Line')
                plt.plot(origin_qp_path.s, origin_qp_path.dl, label = 'PJPO')
                plt.plot(proposed_qp_path.s, proposed_qp_path.dl, label = 'PJPO-C')
                if display_iteration == True:
                    plt.plot(proposed_qp_path_1.s, proposed_qp_path_1.dl, label = 'Proposed iteration')
                    # plt.legend(['Guide Line', 'PJPO', 'PJPO-C', 'Proposed iteration'])
                else:
                    plt.legend(['Guide Line', 'PJPO算法', 'PJPO-C'])
                # plt.title('$l\prime - s$',fontdict={'family' : 'TimesNewRoman', 'size'   : font_+2})
                plt.tight_layout()
                filename = prefix + 'merge-dl-s.pdf'
                plt.savefig(filename,format='pdf', bbox_inches='tight', pad_inches=0.03)
                # plt.grid()

                plt.figure(6, figsize=(8, 4))  # 设置画布的尺寸
                plt.xlabel('$s$ $(m)$', fontdict={'family' : 'TimesNewRoman', 'size'   : font_})
                plt.ylabel('$l{\prime\prime}$ $(m^{-1})$', fontdict={'family' : 'TimesNewRoman', 'size'   : font_})
                # plt.ylim(-0.5, 0.5)
                plt.yticks(fontproperties = english_font, size = font_)
                plt.xticks(fontproperties = english_font, size = font_)
                plt.plot(reference_path.s, reference_path.ddl, label = 'Guide Line')
                plt.plot(origin_qp_path.s, origin_qp_path.ddl, label = 'PJPO')
                plt.plot(proposed_qp_path.s, proposed_qp_path.ddl, label = 'PJPO-C')
                if display_iteration == True:
                    plt.plot(proposed_qp_path_1.s, proposed_qp_path_1.ddl, label = 'Proposed iteration')
                    plt.legend(['Guide Line', 'PJPO', 'PJPO-C'])
                else:
                    plt.legend(['Guide Line', 'PJPO', 'PJPO-C'])
                # plt.title('$l\prime\prime - s$',fontdict={'family' : 'TimesNewRoman', 'size'   : font_+2})
                plt.tight_layout()
                filename = prefix + 'merge-ddl-s.pdf'
                plt.savefig(filename,format='pdf', bbox_inches='tight', pad_inches=0.03)
                # plt.grid()

                # plt.figure(7, figsize=(8, 4))  # 设置画布的尺寸
                # plt.xlabel('$s$ $(m)$', fontdict={'family' : 'TimesNewRoman', 'size'   : font_})
                # plt.ylabel('$v^{max}$ $(m/s)$', fontdict={'family' : 'TimesNewRoman', 'size'   : font_})
                # # plt.ylim(-0.5, 0.5)
                # plt.yticks(fontproperties = english_font, size = font_)
                # plt.xticks(fontproperties = english_font, size = font_)
                # plt.plot(reference_path.s, reference_path.ddl, label = '参考线')
                # plt.plot(origin_qp_path.s, origin_qp_path.ddl, label = 'PJPO算法')
                # plt.plot(proposed_qp_path.s, proposed_qp_path.ddl, label = 'PJPO-C算法')
                # if display_iteration == True:
                #     plt.plot(proposed_qp_path_1.s, proposed_qp_path_1.ddl, label = 'Proposed iteration')
                #     plt.legend(['参考线', 'PJPO算法', 'PJPO-C算法'])
                # else:
                #     plt.legend(['参考线', 'PJPO算法', 'PJPO-C算法'])
                # # plt.title('$l\prime\prime - s$',fontdict={'family' : 'TimesNewRoman', 'size'   : font_+2})
                # plt.tight_layout()
                # filename = prefix + 's-wan-v-s.pdf'
                # plt.savefig(filename,format='pdf', bbox_inches='tight', pad_inches=0.03)
                # # plt.grid()

                plt.show()

if __name__ == '__main__':
    main()
