# -*- coding: utf-8 -*-
"""
Evalate the robust NID by IA approach
Author: Luke
Note: Line 88. Special operator for x_opt only to reduce the amount of the calculation.
Modified: Aug 1,2020
"""

from pyibex import pyibex
import numpy as np
from timeit import default_timer as timer

# define f.delta.lya
def project_wboxes_to_xboxes(w_boxes):
    """
    Project set w_boxes in state-control space along control space to state space
    :param  w_boxes:    a list of pyibex.IntervalVector
    :return x_boxes:    a list of pyibex.Interval
    """
    ww_boxes: list = w_boxes.copy()
    x_boxes = [pyibex.Interval(-0.05, 0.05)]

    while len(ww_boxes) != 0:
        w_box: pyibex.IntervalVector = ww_boxes.pop()
        x_w: pyibex.Interval = pyibex.Interval(w_box[0][0], w_box[0][1])

        xboxes_intersects_xw = []
        xboxes_disjoints_xw = []
        for x in x_boxes:
            if x_w.intersects(x):
                xboxes_intersects_xw.append(x)
            else:
                xboxes_disjoints_xw.append(x)
        x_boxes = xboxes_disjoints_xw.copy()
        xboxes_intersects_xw.append(x_w)
        x = xboxes_intersects_xw.pop()
        lb = x.lb()
        ub = x.ub()
        while len(xboxes_intersects_xw) != 0:
            x = xboxes_intersects_xw.pop()
            if lb >= x.lb():
                lb = x.lb()
            if ub <= x.ub():
                ub = x.ub()
        x_boxes.append(pyibex.Interval(lb, ub))

    return x_boxes


def is_subset(x_box: pyibex.Interval, x_boxes):
    """
    True iff x_box is a subset of x_boxes.
    :param x_box:       a pyibex.Interval
    :param x_boxes:     a list of pyibex.Interval
    :return flag:       True iff x_box is a subset of X
    """
    flag = False
    for x in x_boxes:
        if x_box.is_subset(x):
            flag = True
            break
    return flag


def is_disjoint(x_box: pyibex.Interval, x_boxes):
    """
    True iff x_box and X do not intersect.
    :param x_box:       a pyibex.Interval
    :param x_boxes:     a list of pyibex.Interval
    :return flag:       True iff x_box and X do not intersect
    """
    flag = True
    for x in x_boxes:
        flag = flag and x_box.is_disjoint(x)
    return flag


def I(w_boxes, f_plus,f_minus, eps):
    """
    Estimate the intersection of [f](W) and X with precision eps.
    :param w_boxes:     a list of pyibex.IntervalVector
    :param f:           a pyibex.Function
    :param eps:         a float, precision (boxes of size less than eps are not processed)
    :return :           a list of pyibex.IntervalVector approximating the intersection of [f](W) and X with precision eps
    """
    x_boxes = project_wboxes_to_xboxes(w_boxes)  #normal way
    # Special case for x_opt only
    #x_boxes = pyibex.IntervalVector([[-2, 2]])
    ##
    w_boxes_in = []
    w_boxes_out = []
    w_boxes_bou = []
    w_boxes_do = w_boxes.copy()

    while len(w_boxes_do) != 0:
        w_box = w_boxes_do.pop()
        xf_plus_box = f_plus.eval_vector(w_box)
        xf_plus_box = xf_plus_box[0]
        xf_minus_box = f_minus.eval_vector(w_box)
        xf_minus_box = xf_minus_box[0]
        xf_box = xf_minus_box | xf_plus_box

        if is_subset(xf_box, x_boxes):  # inner test
            w_boxes_in.append(w_box)
        elif is_disjoint(xf_box, x_boxes):  # outer test
            # w_boxes_out.append(w_box)
            pass
        elif w_box.max_diam() < eps:  # boundary test
            # w_boxes_bou.append(w_box)
            pass
        else:  # split the box
            i_diam = w_box.sort_indices(False)
            RL = w_box.bisect(i_diam[0])
            w_boxes_do.append(RL[0])
            w_boxes_do.append(RL[1])
    return w_boxes_in


allp = pyibex.Function("allinoneplusx2.txt") # allinoneplusx2 for x2
allm = pyibex.Function("allinoneminusx2.txt") # allinoneplusx2 for x2
#
w_box_0 = pyibex.IntervalVector([[-2, 2], [-2, 2]])
x_tempp = allp.eval(w_box_0)
x_tempm = allm.eval(w_box_0)
eps = 0.0001
w_boxes_in = []
w_boxes_out = []
w_boxes_bou = []

w_boxes_do = [w_box_0]
t0 = timer()
i = 0
while len(w_boxes_do) != 0:

    w_box = w_boxes_do.pop()
    x = w_box
    x_tempp = allp.eval(x)
    x_tempm = allm.eval(x)
    #
    x = w_box.tolist()
    if x_tempm[1] < 0 and x_tempp[1] <0: # dlya.ub() < 0 and robust <= 0:  # inner test
        w_boxes_in.append(w_box)
    elif x_tempm[0] > 0 or x_tempp[0] >0:  # outer test
        # w_boxes_out.append(w_box)
        pass
    elif w_box.max_diam() < eps:  # boundary test
        # w_boxes_bou.append(w_box)
        #w_boxes_in.append(w_box)
        pass
    else:  # split the box
        i_diam = w_box.sort_indices(False)
        w_boxes_splited = w_box.bisect(i_diam[0])
        w_boxes_do.append(w_boxes_splited[0])
        w_boxes_do.append(w_boxes_splited[1])
    #
    i = i+1

    if i % 1000000 == 0:
        print('Iter:', i, ':boxes in queue:', len(w_boxes_do),':num of inner boxes:',len(w_boxes_in))
    #
#
elapsed = timer() - t0
print('estimate_wboxes_ndd spends %.4f seconds' % elapsed)
##
# use mapping to evaluate invariant set
print('Invariant set evaluating')
w_boxes_ndd = w_boxes_in

w_boxes_hat = w_boxes_ndd.copy()
# plot2DBoxes([w_boxes_ndd], 'b', w_box_0)
f_plus = pyibex.Function("x", "u", "-sin(2*x) - x*u - 0.2*x - u^2 + u + (1-exp(-0.5*(x^2+u^2)))")
f_minus = pyibex.Function("x", "u", "-sin(2*x) - x*u - 0.2*x - u^2 + u - (1-exp(-0.5*(x^2+u^2)))")
delta = pyibex.Function("x", "u", "1-exp(-0.5*(x^2+u^2))")
w_boxes_ndd_inv = I(w_boxes_hat, f_plus, f_minus, eps)
i = 1
while len(w_boxes_ndd_inv) != len(w_boxes_hat):
    w_boxes_hat = w_boxes_ndd_inv.copy()
    w_boxes_ndd_inv = I(w_boxes_hat, f_plus, f_minus, eps)

    i = i + 1
    print('Mapping: ', i)
##

w_boxes_ndd_inv_robust = w_boxes_ndd_inv
lb = list([])
ub = list([])
left = list([])
right = list([])
for i in range(len(w_boxes_ndd_inv_robust)):
    lb.append(w_boxes_ndd_inv_robust[i - 1][1][0])
    ub.append(w_boxes_ndd_inv_robust[i - 1][1][1])
    left.append(w_boxes_ndd_inv_robust[i - 1][0][0])
    right.append(w_boxes_ndd_inv_robust[i - 1][0][1])

x_d = []
u_d = []
xf_d = []
data_array = np.array([lb, ub, left, right])
np.savetxt('w_box_ndd_inv_robust_array_1e4_allinonex2.csv', data_array)