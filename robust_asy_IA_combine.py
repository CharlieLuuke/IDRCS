# -*- coding: utf-8 -*-
"""
Estimate the robust NDD by IA approach
Author: Luke
Modified: Aug 1,2020
"""

from pyibex import pyibex
import numpy as np
from timeit import default_timer as timer
# define f.delta.lya
allp = pyibex.Function("allinoneplusx2.txt")
allm = pyibex.Function("allinoneminusx2.txt")
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
w_boxes_ndd_inv_robust = w_boxes_in
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
np.savetxt('w_box_ndd_robust_array_1e4_allinonex2.csv', data_array)