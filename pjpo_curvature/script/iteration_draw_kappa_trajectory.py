#!/usr/bin/env python
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
# from plotnine import *
import seaborn as sns
import math
from matplotlib import rcParams
import matplotlib.font_manager as fm
from matplotlib.font_manager import FontProperties

font_ = 15
config = {
    "font.family":'serif',
    "font.size": font_,
    "mathtext.fontset":'stix',
    "font.serif": ['FangSong'],
}
rcParams.update(config)
chinese_font = FontProperties(fname='/usr/share/matplotlib/mpl-data/fonts/ttf/FangSong.ttf')
english_font = FontProperties(fname='/usr/share/matplotlib/mpl-data/fonts/ttf/TimesNewRoman.ttf')
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

v_max_ref = 10.0
a_lat_max = 3.0


display_iteration = True
# display_iteration = False
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
        self.s_length = []
        self.v_max_from_a_lat = []
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
        self.s_length = []
        self.v_max_from_a_lat = []
        # self.mean = FrenetPoint()
        # self.max = FrenetPoint()
        for point in input_path_:
            self.x.append(point.x)
            self.y.append(point.y)
            self.s.append(point.s)
            if len(self.s_length) == 0:
                self.s_length.append(0)
            else :
                s_length = self.s_length[-1] + math.sqrt( (point.x - self.x[-2])*(point.x - self.x[-2]) + \
                                                            (point.y - self.y[-2])*(point.y - self.y[-2]))
                self.s_length.append(s_length)
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
            v_max = v_max_ref
            if abs(point.kappa) > 0.001:
                v_max = min( math.sqrt(a_lat_max / abs(point.kappa) ), v_max_ref)
            self.v_max_from_a_lat.append(v_max)
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

def print_mean_max_info(name, path, origin):
    print(name,
          "mean l: ", path.mean.l, "max l: ", path.max.l, 
          ",mean dl: ", path.mean.dl*10, ",max dl: ", path.max.dl*10,
          ",mean ddl: ", path.mean.ddl*100, ",max ddl: ", path.max.ddl*100,
          ",mean kappa: ", path.mean.kappa, ",max kappa: ", path.max.kappa,
          ",mean dkappa: ", path.mean.dkappa, ",max dkappa: ", path.max.dkappa)
    # print(name,"mean diff l: ", (path.mean.l - origin.mean.l)/origin.mean.l*100,
        #    ", dl: ", (path.mean.dl - origin.mean.dl)/origin.mean.dl*100, 
        #    ",ddl: ", (path.mean.ddl - origin.mean.ddl)/origin.mean.ddl*100, 
        #    ",kappa: ", (path.mean.kappa - origin.mean.kappa)/origin.mean.kappa*100,
            # ", dkappa: ", (path.mean.dkappa - origin.mean.dkappa)/origin.mean.dkappa*100)
    print(name,"max diff l: ", (path.max.l - origin.max.l)/origin.max.l*100,
           ",diff dl: ", (path.max.dl - origin.max.dl)/origin.max.dl*100, 
           ",diff ddl: ", (path.max.ddl - origin.max.ddl)/origin.max.ddl*100, 
           ",diff kappa: ", (path.max.kappa - origin.max.kappa)/origin.max.kappa*100,
            ",diff dkappa: ", (path.max.dkappa - origin.max.dkappa)/origin.max.dkappa*100)
    print(" ")


reference_path = sl_path()
origin_qp_path = sl_path()
proposed_qp_path = sl_path()
proposed_qp_path_1 = sl_path()
proposed_qp_path_2 = sl_path()
proposed_qp_path_3 = sl_path()
proposed_qp_path_4 = sl_path()
proposed_qp_path_5 = sl_path()
proposed_qp_path_6 = sl_path()
proposed_qp_path_7 = sl_path()


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
    if data.id == "newQP_iteration1":
        proposed_qp_path_1.record_path(data.path)
    if data.id == "newQP_iteration2":
        proposed_qp_path_2.record_path(data.path)
    if data.id == "newQP_iteration3":
        proposed_qp_path_3.record_path(data.path)
    if data.id == "newQP_iteration4":
        proposed_qp_path_4.record_path(data.path)
    if data.id == "newQP_iteration5":
        proposed_qp_path_5.record_path(data.path)
    if data.id == "newQP_iteration6":
        proposed_qp_path_6.record_path(data.path)
    if data.id == "newQP_iteration7":
        proposed_qp_path_7.record_path(data.path)


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


def main():
    rospy.init_node('draw_kappa')
    rospy.Subscriber("kappa_variables", PoseArray, kappa_callback)
    rospy.Subscriber("sl_paths", FrenetPath, path_callback)
    kappa_limit = rospy.get_param('kappa_limit')
    # rospy.spin()
    print("test")
    # plt.figure(figsize=(10, 3))  
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
                plt.figure(1, figsize=(8, 4))  
                # plt.subplot(2, 1, 1)
                plt.xlabel('$s$ $(m)$', fontdict={'family' : 'TimesNewRoman', 'size'   : font_})
                plt.ylabel('$\kappa$', fontdict={'family' : 'TimesNewRoman', 'size'   : font_})
                plt.ylim(-0.3, 0.3)
                plt.yticks(fontproperties = english_font, size = font_)
                plt.xticks(fontproperties = english_font, size = font_)
                if display_iteration == True:
                    plt.plot(reference_path.s, reference_path.kappa, label = 'Guide Line')
                    plt.plot(origin_qp_path.s, origin_qp_path.kappa, label = 'PJPO')
                    plt.plot(proposed_qp_path.s, proposed_qp_path.kappa, label = 'PJPO-C')
                    plt.plot(proposed_qp_path_4.s, proposed_qp_path_4.kappa, label = 'PJPO-CI')
                plt.plot(kappa_limit_axis, kappa_limit_max_data,'k-.', label = 'kappa_limit_max')
                plt.plot(kappa_limit_axis, kappa_limit_min_data,'k-.', label = 'kappa_limit_min')
                # # plt.title(chr(954).lower()+' - s',fontdict={'family' : 'Times New Roman', 'size'   : font_})
                plt.tight_layout()
                prefix = '/home/zza/Documents/'
                filename = prefix + 'iteration-final-k-s.pdf'
                plt.savefig(filename,format='pdf', bbox_inches='tight', pad_inches=0.03)
                # plt.grid()
                # plt.show()
                
                
                
                # plt.subplot(2, 1, 2)
                plt.figure(2, figsize=(5, 4))  
                plt.xlabel('$s$ $(m)$', fontdict={'family' : 'TimesNewRoman', 'size'   : font_})
                plt.ylabel('$\kappa\prime$', fontdict={'family' : 'TimesNewRoman', 'size'   : font_})
                # plt.ylim(-0.5, 0.5)
                plt.yticks(fontproperties = english_font, size = font_)
                plt.xticks(fontproperties = english_font, size = font_)
                if display_iteration == True:
                    plt.plot(reference_path.s, reference_path.dkappa, label = 'Guide Line')
                    plt.plot(origin_qp_path.s, origin_qp_path.dkappa, label = 'PJPO')
                    plt.plot(proposed_qp_path.s, proposed_qp_path.dkappa, label = 'PJPO-C')
                    plt.plot(proposed_qp_path_4.s, proposed_qp_path_4.dkappa, label = 'PJPO-CI')
                plt.tight_layout()
                filename = prefix + 'iteration-final-dk-s.pdf'
                plt.savefig(filename,format='pdf', bbox_inches='tight', pad_inches=0.03)
                # plt.grid()
                # plt.show()


                # 
                plt.figure(3, figsize=(64.0/9.0, 4))  
                plt.xlabel('$x$ $(m)$', fontdict={'family' : 'TimesNewRoman', 'size'   : font_})
                plt.ylabel('$y$ $(m)$', fontdict={'family' : 'TimesNewRoman', 'size'   : font_})
                # plt.ylim(-0.5, 0.5)
                plt.yticks(fontproperties = english_font, size = font_)
                plt.xticks(fontproperties = english_font, size = font_)
                if display_iteration == True:
                    plt.plot(reference_path.x, reference_path.y, label = 'Guide Line')
                    plt.plot(origin_qp_path.x, origin_qp_path.y, label = 'PJPO')
                    plt.plot(proposed_qp_path.x, proposed_qp_path.y, label = 'PJPO-C')
                    plt.plot(proposed_qp_path_4.x, proposed_qp_path_4.y, label = 'PJPO-CI')
                    plt.legend(['Guide Line', 'PJPO', 'PJPO-C','PJPO-CI'])
                plt.tight_layout()
                filename = prefix + 'iteration-final-x-y.pdf'
                plt.savefig(filename,format='pdf', bbox_inches='tight', pad_inches=0.03)
                # plt.grid()
                # plt.show()

                plt.figure(4, figsize=(5, 4))  
                plt.xlabel('$s$ $(m)$', fontdict={'family' : 'TimesNewRoman', 'size'   : font_})
                plt.ylabel('$l$ $(m)$', fontdict={'family' : 'TimesNewRoman', 'size'   : font_})
                # plt.ylim(-0.5, 0.5)
                plt.yticks(fontproperties = english_font, size = font_)
                plt.xticks(fontproperties = english_font, size = font_)
                if display_iteration == True:
                    plt.plot(reference_path.s, reference_path.l, label = 'Guide Line')
                    plt.plot(origin_qp_path.s, origin_qp_path.l, label = 'PJPO')
                    plt.plot(proposed_qp_path.s, proposed_qp_path.l, label = 'PJPO-C')
                    plt.plot(proposed_qp_path_4.s, proposed_qp_path_4.l, label = 'PJPO-CI')
                plt.tight_layout()
                filename = prefix + 'iteration-final-l-s.pdf'
                plt.savefig(filename,format='pdf', bbox_inches='tight', pad_inches=0.03)

                plt.figure(5, figsize=(5, 4))  
                plt.xlabel('$s$ $(m)$', fontdict={'family' : 'TimesNewRoman', 'size'   : font_})
                plt.ylabel('$l\prime$', fontdict={'family' : 'TimesNewRoman', 'size'   : font_})
               #  plt.xlim(0, 30)
                plt.yticks(fontproperties = english_font, size = font_)
                plt.xticks(fontproperties = english_font, size = font_)
                if display_iteration == True:
                    plt.plot(reference_path.s, reference_path.dl, label = 'Guide Line')
                    plt.plot(origin_qp_path.s, origin_qp_path.dl, label = 'PJPO')
                    plt.plot(proposed_qp_path.s, proposed_qp_path.dl, label = 'PJPO-C')
                    plt.plot(proposed_qp_path_4.s, proposed_qp_path_4.dl, label = 'PJPO-CI')
                plt.tight_layout()
                filename = prefix + 'iteration-final-dl-s.pdf'
                plt.savefig(filename,format='pdf', bbox_inches='tight', pad_inches=0.03)

                plt.figure(6, figsize=(5, 4))  
                plt.xlabel('$s$ $(m)$', fontdict={'family' : 'TimesNewRoman', 'size'   : font_})
                plt.ylabel('$l{\prime\prime}$ $(m^{-1})$', fontdict={'family' : 'TimesNewRoman', 'size'   : font_})
               #  plt.xlim(0, 30)
                plt.yticks(fontproperties = english_font, size = font_)
                plt.xticks(fontproperties = english_font, size = font_)
                if display_iteration == True:
                    plt.plot(reference_path.s, reference_path.ddl, label = 'Guide Line')
                    plt.plot(origin_qp_path.s, origin_qp_path.ddl, label = 'PJPO')
                    plt.plot(proposed_qp_path.s, proposed_qp_path.ddl, label = 'PJPO-C')
                    plt.plot(proposed_qp_path_4.s, proposed_qp_path_4.ddl, label = 'PJPO-CI')
                plt.tight_layout()
                filename = prefix + 'iteration-final-ddl-s.pdf'
                plt.savefig(filename,format='pdf', bbox_inches='tight', pad_inches=0.03)

                plt.figure(7, figsize=(5, 4))  
                plt.xlabel('$s$ $(m)$', fontdict={'family' : 'TimesNewRoman', 'size'   : font_})
                plt.ylabel('$v^{max}$ $(m/s)$', fontdict={'family' : 'TimesNewRoman', 'size'   : font_})
                #  plt.xlim(0, 30)
                plt.ylim(3.5, v_max_ref + .5)
                plt.yticks(fontproperties = english_font, size = font_)
                plt.xticks(fontproperties = english_font, size = font_)
                origin_min_v = np.min(origin_qp_path.v_max_from_a_lat)
                propose_min_v = np.min(proposed_qp_path.v_max_from_a_lat)
                propose_4_min_v = np.min(proposed_qp_path_4.v_max_from_a_lat)
                v_ref_array = [v_max_ref]*len(reference_path.s)
                v_origin_array = [origin_min_v]*len(reference_path.s)
                v_propose_array = [propose_min_v]*len(reference_path.s)
                v_propose_4_array = [propose_4_min_v]*len(reference_path.s)
                if display_iteration == True:
                    plt.plot(reference_path.s_length, v_ref_array, label = '参考线')
                    plt.plot(origin_qp_path.s_length, origin_qp_path.v_max_from_a_lat, label = 'PJPO算法')
                    plt.plot(proposed_qp_path.s_length, proposed_qp_path.v_max_from_a_lat, label = 'PJPO-C算法')
                    plt.plot(proposed_qp_path_4.s_length, proposed_qp_path_4.v_max_from_a_lat, label = 'PJPO-CI算法')
                    plt.plot(reference_path.s_length, v_origin_array, color = 'C1', linewidth=1.1, linestyle='--')
                    plt.plot(reference_path.s_length, v_propose_array, color = 'C2', linewidth=1.1, linestyle='--')
                    plt.plot(reference_path.s_length, v_propose_4_array, color = 'C3', linewidth=1.1, linestyle='--')
                plt.tight_layout()
                filename = prefix + 'iteration-final-v-s2.pdf'
                plt.savefig(filename,format='pdf', bbox_inches='tight', pad_inches=0.03)



                plt.figure(11, figsize=(5, 4))  
                plt.xlabel('$s$ $(m)$', fontdict={'family' : 'TimesNewRoman', 'size'   : font_})
                plt.ylabel('$\kappa$', fontdict={'family' : 'TimesNewRoman', 'size'   : font_})
                plt.xlim(0, 35)
                plt.ylim(-0.10, 0.10)
                plt.yticks(fontproperties = english_font, size = font_)
                plt.xticks(fontproperties = english_font, size = font_)
                
                if display_iteration == True:
                    plt.plot(origin_qp_path.s, origin_qp_path.kappa, label = 'PJPO算法')
                    plt.plot(proposed_qp_path.s, proposed_qp_path.kappa, label = 'Proposed')
                    plt.plot(proposed_qp_path_1.s, proposed_qp_path_1.kappa, label = 'Proposed iteration 1')
                    plt.plot(proposed_qp_path_2.s, proposed_qp_path_2.kappa, label = 'Proposed iteration 2')
                    plt.plot(proposed_qp_path_3.s, proposed_qp_path_3.kappa, label = 'Proposed iteration 3')
                    plt.plot(proposed_qp_path_4.s, proposed_qp_path_4.kappa, label = 'Proposed iteration 4')
                    # plt.plot(proposed_qp_path_5.s, proposed_qp_path_5.kappa, label = 'Proposed iteration 5')
                    # plt.legend(['Proposed', 'Proposed iteration 1', 'Proposed iteration 2', \
                    #     'Proposed iteration 3', 'Proposed iteration 4', 'Proposed iteration 5'], prop={'family' : 'Times New Roman', 'size'   : font_-1})
                # else:
                    # plt.legend(['Reference line', 'PJPO', 'Proposed'], prop={'family' : 'Times New Roman', 'size'   : font_-1})
                # # plt.title(chr(954).lower() + ' - s',fontdict={'family' : 'Times New Roman', 'size'   : font_})
                plt.tight_layout()
                filename = prefix + 'full-iteration-k-s.pdf'
                plt.savefig(filename,format='pdf', bbox_inches='tight', pad_inches=0.03)

                plt.figure(12, figsize=(5, 4))  
                plt.xlabel('$s$ $(m)$', fontdict={'family' : 'TimesNewRoman', 'size'   : font_})
                plt.ylabel('$\kappa\prime$', fontdict={'family' : 'TimesNewRoman', 'size'   : font_})
                # plt.ylim(-0.5, 0.5)
                plt.xlim(0, 35)
                plt.ylim(-0.21, 0.12)
                plt.yticks(fontproperties = english_font, size = font_)
                plt.xticks(fontproperties = english_font, size = font_)
                
                if display_iteration == True:
                    plt.plot(origin_qp_path.s, origin_qp_path.dkappa, label = 'PJPO算法')
                    plt.plot(proposed_qp_path.s, proposed_qp_path.dkappa, label = 'Proposed')
                    plt.plot(proposed_qp_path_1.s, proposed_qp_path_1.dkappa, label = 'Proposed iteration 1')
                    plt.plot(proposed_qp_path_2.s, proposed_qp_path_2.dkappa, label = 'Proposed iteration 2')
                    plt.plot(proposed_qp_path_3.s, proposed_qp_path_3.dkappa, label = 'Proposed iteration 3')
                    plt.plot(proposed_qp_path_4.s, proposed_qp_path_4.dkappa, label = 'Proposed iteration 4')
                    # plt.plot(proposed_qp_path_5.s, proposed_qp_path_5.dkappa, label = 'Proposed iteration 5')
                    # plt.legend(['Proposed', 'Proposed iteration 1', 'Proposed iteration 2', \
                    #     'Proposed iteration 3', 'Proposed iteration 4', 'Proposed iteration 5'], prop={'family' : 'Times New Roman', 'size'   : font_-1})
                # else:
                #     plt.legend(['Reference line', 'PJPO', 'Proposed'], prop={'family' : 'Times New Roman', 'size'   : font_-1})
                # # plt.title(chr(954).lower()+'\' - s',fontdict={'family' : 'Times New Roman', 'size'   : font_})
                plt.tight_layout()
                filename = prefix + 'full-iteration-dk-s.pdf'
                plt.savefig(filename,format='pdf', bbox_inches='tight', pad_inches=0.03)


                plt.figure(13, figsize=(5, 4))  
                plt.xlabel('$x$ $(m)$', fontdict={'family' : 'TimesNewRoman', 'size'   : font_})
                plt.ylabel('$y$ $(m)$', fontdict={'family' : 'TimesNewRoman', 'size'   : font_})
                plt.xlim(0, 35)
                # plt.ylim(-0.5, 2.6)
                plt.yticks(fontproperties = english_font, size = font_)
                plt.xticks(fontproperties = english_font, size = font_)
                if display_iteration == True:
                    plt.plot(origin_qp_path.x, origin_qp_path.y, label = 'PJPO')
                    plt.plot(proposed_qp_path.x, proposed_qp_path.y, label = 'PJPO-C')
                    plt.plot(proposed_qp_path_1.x, proposed_qp_path_1.y, label = 'PJPO-C iteration 1')
                    plt.plot(proposed_qp_path_2.x, proposed_qp_path_2.y, label = 'PJPO-C iteration  2')
                    plt.plot(proposed_qp_path_3.x, proposed_qp_path_3.y, label = 'PJPO-C iteration  3')
                    plt.plot(proposed_qp_path_4.x, proposed_qp_path_4.y, label = 'PJPO-C iteration  4')
                    # plt.plot(proposed_qp_path_5.x, proposed_qp_path_5.y, label = 'PJPO-C算法迭代  5')
                    plt.legend(['PJPO','PJPO-C', 'PJPO-C iteration 1', 'PJPO-C iteration 2', \
                        'PJPO-C iteration 3', 'PJPO-C iteration 4'
                        # , 'PJPO-C算法迭代 5'
                        ])
                # else:
                #     plt.plot(reference_path.s, reference_path.l, label = '参考线')
                #     plt.plot(origin_qp_path.s, origin_qp_path.l, label = 'PJPO算法')
                #     plt.plot(proposed_qp_path.s, proposed_qp_path.l, label = 'PJPO-C算法')
                #     plt.legend(['Reference line', 'PJPO', 'Proposed'])
                # plt.title('l - s',fontdict={'family' : 'Times New Roman', 'size'   : font_})
                plt.tight_layout()
                filename = prefix + 'full-iteration-x-y.pdf'
                plt.savefig(filename,format='pdf', bbox_inches='tight', pad_inches=0.03)
                # plt.grid()


                plt.figure(14, figsize=(5, 4))  
                plt.xlabel('$s$ $(m)$', fontdict={'family' : 'TimesNewRoman', 'size'   : font_})
                plt.ylabel('$l$ $(m)$', fontdict={'family' : 'TimesNewRoman', 'size'   : font_})
                plt.xlim(0, 35)
                # plt.ylim(-0.5, 2.6)
                plt.yticks(fontproperties = english_font, size = font_)
                plt.xticks(fontproperties = english_font, size = font_)
                if display_iteration == True:
                    plt.plot(origin_qp_path.s, origin_qp_path.l, label = 'PJPO算法')
                    plt.plot(proposed_qp_path.s, proposed_qp_path.l, label = 'PJPO-C算法')
                    plt.plot(proposed_qp_path_1.s, proposed_qp_path_1.l, label = 'PJPO-C算法迭代 1')
                    plt.plot(proposed_qp_path_2.s, proposed_qp_path_2.l, label = 'PJPO-C算法迭代  2')
                    plt.plot(proposed_qp_path_3.s, proposed_qp_path_3.l, label = 'PJPO-C算法迭代  3')
                    plt.plot(proposed_qp_path_4.s, proposed_qp_path_4.l, label = 'PJPO-C算法迭代  4')
                    # plt.plot(proposed_qp_path_5.s, proposed_qp_path_5.l, label = 'PJPO-C算法迭代  5')
                    plt.legend(['PJPO','PJPO-C', 'PJPO-C iteration 1', 'PJPO-C iteration 2', \
                        'PJPO-C iteration 3', 'PJPO-C iteration 4'
                        # , 'PJPO-C算法迭代 5'
                        ])
                else:
                    plt.plot(reference_path.s, reference_path.l, label = '参考线')
                    plt.plot(origin_qp_path.s, origin_qp_path.l, label = 'PJPO算法')
                    plt.plot(proposed_qp_path.s, proposed_qp_path.l, label = 'PJPO-C算法')
                    plt.legend(['Reference line', 'PJPO', 'Proposed'])
                # plt.title('l - s',fontdict={'family' : 'Times New Roman', 'size'   : font_})
                plt.tight_layout()
                filename = prefix + 'full-iteration-l-s.pdf'
                plt.savefig(filename,format='pdf', bbox_inches='tight', pad_inches=0.03)
                # plt.grid()

                plt.figure(15, figsize=(5, 4))  
                plt.xlabel('$s$ $(m)$', fontdict={'family' : 'TimesNewRoman', 'size'   : font_})
                plt.ylabel('$l\prime$', fontdict={'family' : 'TimesNewRoman', 'size'   : font_})
                plt.xlim(0, 35)
                plt.yticks(fontproperties = english_font, size = font_)
                plt.xticks(fontproperties = english_font, size = font_)
                if display_iteration == True:
                    plt.plot(origin_qp_path.s, origin_qp_path.dl, label = 'PJPO算法')
                    plt.plot(proposed_qp_path.s, proposed_qp_path.dl, label = 'PJPO-C算法')
                    plt.plot(proposed_qp_path_1.s, proposed_qp_path_1.dl, label = 'PJPO-C算法迭代 1')
                    plt.plot(proposed_qp_path_2.s, proposed_qp_path_2.dl, label = 'PJPO-C算法迭代  2')
                    plt.plot(proposed_qp_path_3.s, proposed_qp_path_3.dl, label = 'PJPO-C算法迭代  3')
                    plt.plot(proposed_qp_path_4.s, proposed_qp_path_4.dl, label = 'PJPO-C算法迭代  4')
                    # plt.plot(proposed_qp_path_5.s, proposed_qp_path_5.dl, label = 'PJPO-C算法迭代  5')
                    # plt.legend(['PJPO-C算法', 'PJPO-C算法迭代 1', 'PJPO-C算法迭代 2', \
                    #     'PJPO-C算法迭代 3', 'PJPO-C算法迭代 4', 'PJPO-C算法迭代 5'])
                # else:
                #     plt.legend(['Reference line', 'PJPO', 'Proposed'], prop={'family' : 'Times New Roman', 'size'   : font_-1})
                # plt.title('l\' - s',fontdict={'family' : 'Times New Roman', 'size'   : font_})
                plt.tight_layout()
                filename = prefix + 'full-iteration-dl-s.pdf'
                plt.savefig(filename,format='pdf', bbox_inches='tight', pad_inches=0.03)
                # plt.grid()

                plt.figure(16, figsize=(5, 4))  
                plt.xlabel('$s$ $(m)$', fontdict={'family' : 'TimesNewRoman', 'size'   : font_})
                plt.ylabel('$l\prime\prime$ $(m^{-1})$', fontdict={'family' : 'TimesNewRoman', 'size'   : font_})
                plt.xlim(0, 35)
                plt.yticks(fontproperties = english_font, size = font_)
                plt.xticks(fontproperties = english_font, size = font_)
                if display_iteration == True:
                    plt.plot(origin_qp_path.s, origin_qp_path.ddl, label = 'PJPO算法')
                    plt.plot(proposed_qp_path.s, proposed_qp_path.ddl, label = 'Proposed')
                    plt.plot(proposed_qp_path_1.s, proposed_qp_path_1.ddl, label = 'Proposed iteration 1')
                    plt.plot(proposed_qp_path_2.s, proposed_qp_path_2.ddl, label = 'Proposed iteration 2')
                    plt.plot(proposed_qp_path_3.s, proposed_qp_path_3.ddl, label = 'Proposed iteration 3')
                    plt.plot(proposed_qp_path_4.s, proposed_qp_path_4.ddl, label = 'Proposed iteration 4')
                    # plt.plot(proposed_qp_path_5.s, proposed_qp_path_5.ddl, label = 'Proposed iteration 5')
                #     plt.legend(['Proposed', 'Proposed iteration 1', 'Proposed iteration 2', \
                #         'Proposed iteration 3', 'Proposed iteration 4', 'Proposed iteration 5'], prop={'family' : 'Times New Roman', 'size'   : font_-1})
                # else:
                #     plt.legend(['Reference line', 'PJPO', 'Proposed'], prop={'family' : 'Times New Roman', 'size'   : font_-1})
                # # plt.title(chr(954).lower()+'\' - s',fontdict={'family' : 'Times New Roman', 'size'   : font_})
                plt.tight_layout()
                filename = prefix + 'full-iteration-ddl-s.pdf'
                plt.savefig(filename,format='pdf', bbox_inches='tight', pad_inches=0.03)
                # plt.grid()
                
                print_mean_max_info("origin QP",origin_qp_path,origin_qp_path)
                print_mean_max_info("New QP",proposed_qp_path,origin_qp_path)
                print_mean_max_info("Iteration QP",proposed_qp_path_4,origin_qp_path)
                
                
                plt.show()

if __name__ == '__main__':
    main()
