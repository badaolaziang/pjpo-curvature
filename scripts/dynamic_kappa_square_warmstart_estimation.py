from inspect import formatannotationrelativeto
# import rosbag
import numpy as np
import matplotlib.pyplot as plt
import sys
import os
import seaborn as sns

import math
import numpy.matlib
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

import seaborn as sns
import math
import numpy
from matplotlib import rcParams
import matplotlib.font_manager as fm
from matplotlib.font_manager import FontProperties

# display parameters
font_ = 13
config = {
    "font.family":'serif',
    "font.size": font_,
    "mathtext.fontset":'stix',
    "font.serif": ['FangSong'],
}
rcParams.update(config)
chinese_font = FontProperties(fname='/usr/share/matplotlib/mpl-data/fonts/ttf/FangSong.ttf')
english_font = FontProperties(fname='/usr/share/matplotlib/mpl-data/fonts/ttf/TimesNewRoman.ttf')

"""
variable name claim: xxx_k represents xxx^k
"""


def kappa_square_estimate_dynamic(
        l, dl, ddl,
        kappa_ref, dkappa_ref,
        dynamic_type, evaluation_range,
        l_diff, dl_diff, ddl_diff,
        resolution):
    """draw kappa^2 comparison with dynamic variables

    Args:
        l (float): l_0 of initial state
        dl (float): dl_0 of initial state
        ddl (float): ddl_0 of initial state
        kappa_ref (float): kappa value of reference line's corresponding point
        dkappa_ref (float): dkappa value of reference line's corresponding point
        dynamic_type (float): which variable is dynamic; 0: l; 1: dl; 2: ddl, evaluation_range: the change range of the variable
        l_diff (float): if stable, l_i = l_0 + l_diff; if dynamic, l_i belong to [l_0 - l_diff, l_0 + l_diff]
        dl_diff (float): if stable, dl_i = dl_0 + dl_diff; if dynamic, dl_i belong to [dl_0 - dl_diff, dl_0 + dl_diff]
        ddl_diff (float): if stable, ddl_i = ddl_0 + ddl_diff; if dynamic, ddl_i belong to [ddl_0 - ddl_diff, ddl_0 + ddl_diff]
        resolution (float): resolution of variable points
    """
    # set coefficient parameters
    theta = math.atan2(dl, 1-kappa_ref*l)
    # print(theta)
    kr_1 = kappa_ref
    kr_2 = math.pow(kr_1, 2)
    kr_3 = math.pow(kr_1, 3)
    dkr = dkappa_ref
    krl_1 = 1-kappa_ref*l
    krl_2 = math.pow(krl_1, 2)
    krl_3 = math.pow(krl_1, 3)
    krl_4 = math.pow(krl_1, 4)
    krl_5 = math.pow(krl_1, 5)
    c_1 = math.cos(theta)
    c_2 = math.pow(c_1, 2)
    c_3 = math.pow(c_1, 3)

    # real kappa value of the given initial state (l, dl, ddl)
    kappa_real = ((ddl+(dkr*l + kr_1*dl)*math.tan(theta)
                   * c_2 / krl_1 + kappa_ref)*c_1/krl_1)
    # get the kappa^2 of initial state
    kappa_real_2 = math.pow(kappa_real, 2)

    # initial the first order coefficient of l_i, dl_i, ddl_i
    C = np.array([[0.],
                  [0.],
                  [0.]])

    # initial the second order coefficient of l_i, dl_i, ddl_i
    A = np.array([[0., 0., 0.],
                  [0., 0., 0.],
                  [0., 0., 0.]])

    kappa_estimate_constant = c_1*kr_1/krl_1 + \
        (-c_1*kr_2*l)/krl_2 + (-c_1*kr_3*math.pow(l, 2))/krl_3

    # calculate the first ofder coefficient of l_i, dl_i, ddl_i
    C[0] = c_1*kr_2/krl_2 + (-2.0*c_1*kr_3*l)/krl_3
    C[1] = c_3*l*dkr/krl_3 + (-dkr*l*(c_3 + 2.0*c_3*l*kr_1))/krl_4
    C[2] = 2.0*c_3/krl_2 + (-2*c_3*kr_1*l)/krl_3 \
        + 3.0*c_2*kr_2*math.pow(l, 2)/krl_4

    # calculate the second ofder coefficient of l_i, dl_i, ddl_i
    A[0][0] = c_1*kr_3/krl_3
    A[1][1] = c_3*kr_1/krl_3 + (-3.0*c_3*kr_2*l)/krl_4 + \
        6.0*c_3*kr_3*math.pow(l, 2)/krl_5 + c_3*kr_1/krl_3
    A[0][1] = (dkr*(c_3 + 2.0*c_3*l*kr_1)/krl_4 + c_3*dkr/krl_3)/2
    A[1][0] = A[0][1]
    A[0][2] = (2*c_3*kr_1/krl_3 + (-6.0*c_3*kr_2*l/krl_4))/2
    A[2][0] = A[0][2]

    # initial state L_0 = [ l, dl, ddl]
    L_0 = np.array([[0.],
                    [0.],
                    [0.]])
    L_0[0] = l
    L_0[1] = dl
    L_0[2] = ddl

    # dynamic state L_i = [l_i, dl_i, ddl_i]
    # l_i = l + l_diff
    # dl_i = dl + dl_diff
    # ddl_i = ddl + ddl_diff
    l_i = l 
    dl_i = dl 
    ddl_i = ddl 

    L_i = np.array([[0.],
                    [0.],
                    [0.]])
    L_i[0] = l_i
    L_i[1] = dl_i
    L_i[2] = ddl_i

    const_1 = kappa_estimate_constant
    const_2 = math.pow(const_1, 2)
    # kappa_estimate_2 = const^2 + L^T(CC^T+2const*A)L + 2constC^TL
    # A_ = CC^T+2const*A
    # C_ = 2*const*C
    A_ = np.dot(C, C.T) + const_1*A
    C_ = 2*const_1*C
    kappa_estimate_zero_2 = const_2 + \
        np.dot((np.dot(L_0.T, A_)), L_0) + np.dot(C_.T, L_0)
    # get the stable-state error between the initial states's kappa^2 - kappa_estimate^2
    kappa_estimate_zero_2_error = kappa_real_2 - kappa_estimate_zero_2[0][0]

    # draw_data_record

    x_axis = []
    kappa_real_2_array = []
    kappa_ref_2_array = []
    kappa_estimate_2_array = []
    kappa_estimate_2_array_first_order = []
    kappa_2_diff_array = []
    kappa_2_diff_ratio_array = []
    kappa_2_diff_error_ratio_array = []

    # set new variables
    evaluation_range = abs(evaluation_range)
    if dynamic_type == 0:
        i_min = l_i - evaluation_range
        i_max = l_i + evaluation_range
    if dynamic_type == 1:
        i_min = dl_i - evaluation_range
        i_max = dl_i + evaluation_range
    if dynamic_type == 2:
        i_min = ddl_i - evaluation_range
        i_max = ddl_i + evaluation_range

    num = (i_max - i_min)/resolution
    for i in range(round(num) + 1):
        if dynamic_type == 0:
            l_i = i_min + resolution*i
            # update L_i[0]
            L_i[0] = l_i
            xlable_name = '$l$ $(m)$'
            variable_name = '$\kappa^2-l$'
        if dynamic_type == 1:
            dl_i = i_min + resolution*i
            # update L_i[1]
            L_i[1] = dl_i
            xlable_name = '$l^\prime$'
            variable_name = '$\kappa^2-l^\prime$'
        if dynamic_type == 2:
            ddl_i = i_min + resolution*i
            # update L_i[2]
            L_i[2] = ddl_i
            xlable_name = '$l^{\prime\prime}$ $(m^{-1})$'
            variable_name = '$\kappa^2-l^{\prime\prime}$'

        x_axis.append(i_min + resolution*i)

        # theta_i of new state
        # theta_i = math.atan2(dl_i, 1-kappa_ref*l_i)
        kappa_real = ((ddl_i+(dkr*l_i + kr_1*dl_i)*math.atan2(dl_i, 1-kappa_ref*l_i)
                       * c_2 / krl_1 + kappa_ref)*c_1/krl_1)
        kappa_real_2 = math.pow(kappa_real, 2)
        kappa_real_2_array.append(kappa_real_2)
        kappa_ref_2_array.append(kr_2)

        # kappa_estimate_2 = const^2 + L^T(CC^T+2const*A)L + 2constC^TL + stable-state error
        kappa_estimate_2 = const_2 + kappa_estimate_zero_2_error + \
            np.dot((np.dot(L_i.T, A_)), L_i) + np.dot(C_.T, L_i)
        kappa_estimate_2_first_order = const_2 + kappa_estimate_zero_2_error + np.dot(C_.T, L_i)
        kappa_estimate_2_array.append(kappa_estimate_2[0][0])
        kappa_estimate_2_array_first_order.append(kappa_estimate_2_first_order[0][0])
        kappa_2_diff_array.append(kappa_estimate_2[0][0] - kappa_real_2)
        kappa_2_diff_ratio_array.append(
            (kappa_estimate_2[0][0] - kappa_real_2)/kappa_real_2*100)
        # kappa_2_diff_error_ratio_array.append(
        #     (kappa_estimate_2[0][0] - kappa_real_2)/(kappa_real - kappa_ref)*100)
    plt.figure(figsize=(5, 4))  
    plt.xlabel(xlable_name, fontsize=font_)
    plt.ylabel('$\kappa^2$', fontsize=font_)
    # plt.title(variable_name, fontsize=font_)
    plt.xticks(fontproperties = english_font,fontsize=font_)
    plt.yticks(fontproperties = english_font,fontsize=font_)
    plt.xlim(i_min, i_max)
    plt.plot(x_axis, kappa_estimate_2_array_first_order, label='1st order estimation of $\kappa^2$')
    plt.plot(x_axis, kappa_estimate_2_array, label='2nd order estimation of $\kappa^2$')
    plt.plot(x_axis, kappa_real_2_array, label='actual $\kappa^2$')
    # plt.plot(x_axis, kappa_ref_2_array, label='kappa ref')
    if  dynamic_type == 0:
        plt.legend(['1st order estimation of $\kappa^2$','2nd order estimation of $\kappa^2$', 
                    'actual $\kappa^2$'],fontsize=font_)
    plt.grid()
    
    # plt.grid()

    # plt.subplot(2, 2, 2)
    # plt.xlabel(xlable_name, fontsize=font_+1)
    # plt.ylabel('diff between estimate^2 and real^2', fontsize=font_+1)
    # # plt.title(variable_name, fontsize=font_)
    # plt.xticks(fontsize=font_)
    # plt.yticks(fontsize=font_)
    # plt.xlim(i_min, i_max)
    # plt.plot(x_axis, kappa_2_diff_array,
    #          label='kappa^2 diff')
    # plt.legend(['kappa^2 diff'], fontsize=font_)
    # plt.grid()

    # plt.subplot(2, 2, 3)
    # plt.xlabel(xlable_name, fontsize=font_+1)
    # plt.ylabel('diff ratio of kappa_real&2 (%)', fontsize=font_+1)
    # # plt.title(variable_name, fontsize=font_)
    # plt.xticks(fontsize=font_)
    # plt.yticks(fontsize=font_)
    # plt.xlim(i_min, i_max)
    # plt.plot(x_axis, kappa_2_diff_ratio_array, label='kappa diff ratio')
    # plt.legend(['kappa^2 diff ratio'], fontsize=font_)
    # plt.grid()

    # plt.subplot(2, 2, 4)
    # plt.xlabel(xlable_name, fontsize=font_+1)
    # plt.ylabel('diff ratio of (kappa_real - kapap_ref) (%)',
    #            fontsize=font_+1)
    # # plt.title(variable_name, fontsize=font_)
    # plt.xticks(fontsize=font_)
    # plt.yticks(fontsize=font_)
    # plt.xlim(i_min, i_max)
    # plt.plot(x_axis, kappa_diff_error_ratio_array, label='kappa diff ratio')
    # plt.legend(['kappa diff ratio'], fontsize=font_)
    # plt.grid()
    # plt.title(variable_name,fontdict={'family' : 'FangSong', 'size'   : font_+2})
    plt.tight_layout()
    # plt.suptitle(variable_name)
    filename = ''
    if dynamic_type == 0:
        filename = 'squared_curvature_estimation_dynamic_l.pdf'
    if dynamic_type == 1:
        filename = 'squared_curvature_estimation_dynamic_dl.pdf'
    if dynamic_type == 2:
        filename = 'squared_curvature_estimation_dynamic_ddl.pdf'
    plt.savefig(filename,format='pdf', bbox_inches='tight', pad_inches=0.03)
    
    plt.show()


if __name__ == '__main__':
    kappa_square_estimate_dynamic(
        # parameter list:
        # l_0, dl_0, ddl_0, (initial state)
        0.5, 0.1, 0.02,
        # kappa_ref, dkappa_ref, (reference line information)
        0.04, 0.01,
        # dynamic_type and evaluation region d-range (0: dynamic l; 1: dynamic dl; 2: dynamic ddl)
            0, 11.0,
            # 1, 1.1,
            # 2, 0.2,
        # l_diff, dl_diff, ddl_diff
        0.0, 00.0, 0.000,
        # resolution of variable points
        0.0002)
