# -*- coding: utf-8 -*-
"""
Created on Fri Nov 11 01:19:51 2022

@author: ren tsai
"""
import numpy as np
from scipy import optimize
from scipy.optimize import least_squares
import sys, collections, time
from scipy.optimize import lsq_linear, root, minimize
import random
# import matplotlib.pyplot as plt
import numpy.matlib 
from itertools import product
from itertools import combinations
from collections import Counter
import numpy as np
# import matplotlib.pyplot as plt 
# from mpl_toolkits.mplot3d import Axes3D
import heapq
import random 
from sympy import *
import cmath
import math
import inspect
import sympy 

# 定义目标定位的误差函数
def error_function(target, anchors, distances):
    return np.sqrt(np.sum((np.linalg.norm(anchors - target, axis=1) - distances)**2))


def lsq_method(distances_to_anchors, anchor_positions, u):
    anchors = anchor_positions
    anchor_offset = anchor_positions[0]
    anchor_positions = anchor_positions[1:] - anchor_offset
    K = np.sum(np.square(anchor_positions), axis=1)   #ax=1 列加
    squared_distances_to_anchors = np.square(distances_to_anchors)
    squared_distances_to_anchors = (squared_distances_to_anchors - squared_distances_to_anchors[0])[1:]
    b = (K - squared_distances_to_anchors) / 2.
    det = u.T @ b
    #res = lsq_linear(anchor_positions, b, lsmr_tol='auto', verbose=0)
    # res = np.dot(np.dot(np.linalg.inv(np.dot(anchor_positions.T, anchor_positions)),(anchor_positions.T)), b)
    res = np.linalg.lstsq(anchor_positions, b, rcond=None)[0]
    result = res + anchor_offset

    
    return result, det, b

def lsq_method2(distances_to_anchors, anchor_positions, u):
    A = np.array([anchor_positions[1]-anchor_positions[0], anchor_positions[2]-anchor_positions[1], anchor_positions[3]-anchor_positions[2]])
    D_2 = np.square(distances_to_anchors)
    D_diff = np.array([D_2[0] - D_2[1], D_2[1] - D_2[2], D_2[2] - D_2[3]])
    K = np.sum(np.square(anchor_positions), axis=1)
    loc_diff = np.array([K[0]-K[1], K[1]-K[2], K[2]-K[3]])
    B = (D_diff-loc_diff)/ 2.
    res = np.dot(np.dot(np.linalg.inv(np.dot(A.T, A)),(A.T)), B)
    # res = np.linalg.lstsq(A, B, rcond=None)[0]
    return res, K, B

def triangle_haversine(a, b, c):
    s = (a + b + c) / 2
    area = (s * (s - a) * (s - b) * (s - c))**0.5

    return area

def find_two_largest_indices(values):
    # 初始化最大和次大值的索引
    max1_index = 0
    max2_index = 1 if values[1] < values[0] else 0

    # 遍歷剩餘的元素，更新最大和次大值的索引
    for i in range(2, len(values)):
        if values[i] > values[max1_index]:
            max2_index = max1_index
            max1_index = i
        elif values[i] > values[max2_index]:
            max2_index = i

    return max1_index, max2_index

def find_negative_value(data):
    # 檢查列表中是否存在任何一個負值
    for value in data:
        if value < 0:
            return 1
    return 0

def one_stage_svd(anchor_positions, distances_to_anchors):
    R12 = np.linalg.norm(anchor_positions[1] - anchor_positions[0])
    R13 = np.linalg.norm(anchor_positions[2] - anchor_positions[0])
    R14 = np.linalg.norm(anchor_positions[3] - anchor_positions[0])
    
    anchor_offset = anchor_positions[0]
    A = anchor_positions[1:] - anchor_offset
        
    
    r1 = distances_to_anchors[0]
    r2 = distances_to_anchors[1]
    r3 = distances_to_anchors[2]
    r4 = distances_to_anchors[3]
    
    b = np.array([1/2*(r1**2+R12**2-r2**2), 1/2*(r1**2+R13**2-r3**2), 1/2*(r1**2+R14**2-r4**2)]).reshape(3, 1)

    U, S, Vh = np.linalg.svd(A, full_matrices=True)
    # l1/l2
    conditional_num12 = S[0]/S[1]
    # l2/l3
    conditional_num23 = S[1]/S[2]
    # l1/l3
    conditional_num13 = S[0]/S[2]
    
    s_lambda_diagonal = np.diag(S)
    s_inv = np.linalg.inv(s_lambda_diagonal)

    if (conditional_num13 > 10):
        # s_inv[1][1] = 0
        # if (conditional_num13 > 10):
        s_inv[-1][-1] = 0
        # result = U@(s_inv@U.T@AT_b)
        result = (s_inv@U.T@b)
    else:
        # result = U@(s_inv@U.T@AT_b)
        result = (s_inv@U.T@b)
    
    return result
def choose_line(anchor_positions, distances_to_anchors):
    R12 = np.linalg.norm(anchor_positions[1] - anchor_positions[0])
    R13 = np.linalg.norm(anchor_positions[2] - anchor_positions[0])
    R14 = np.linalg.norm(anchor_positions[3] - anchor_positions[0])
    
    anchor_offset = anchor_positions[0]
    A = anchor_positions[1:] - anchor_offset
    
    a, b = A[:, 0], A[:, 1]
    
    r1 = distances_to_anchors[0]
    r2 = distances_to_anchors[1]
    r3 = distances_to_anchors[2]
    r4 = distances_to_anchors[3]
    
    test_triange2 = r1 + r2 - R12
    test_triange3 = r1 + r3 - R13
    test_triange4 = r1 + r4 - R14
    test_triange = [test_triange2, test_triange3, test_triange4]
    # print('triangle_area', test_triange)
    finding = find_negative_value(test_triange)
    # print('finding', finding)
    
    b_matrix = np.array([1/2*(r1**2+R12**2-r2**2), 1/2*(r1**2+R13**2-r3**2), 1/2*(r1**2+R14**2-r4**2)]).reshape(3, 1)

    # Select lines 1 and 2 (index 0 and 1) # r2r3
    A_selected = A[:2]
    b_selected = b_matrix[:2]
    solution, residuals, rank, singular_values = np.linalg.lstsq(A_selected, b_selected, rcond=None)
    X1 = solution
    # print(type(X1[0]))

    # Select lines 1 and 3 (index 0 and 2) # r2r4
    A_selected = np.array([A[0], A[2]])
    b_selected = np.array([b_matrix[0], b_matrix[2]])
    solution, residuals, rank, singular_values = np.linalg.lstsq(A_selected, b_selected, rcond=None)
    X2 = solution

    # Select lines 2 and 3 (index 1 and 2) # r3r4
    A_selected = A[1:]
    b_selected = b_matrix[1:]
    solution, residuals, rank, singular_values = np.linalg.lstsq(A_selected, b_selected, rcond=None)
    X3 = solution
    
    section_r2 = np.linalg.norm(X1 - X2)
    section_r3 = np.linalg.norm(X1 - X3)
    section_r4 = np.linalg.norm(X2 - X3)
    section_ls = [section_r2, section_r3, section_r4]
    max_section = max(section_ls)

    triangle_area = triangle_haversine(section_r2, section_r3, section_r4)
    

    # if(max_section > 0.3):
    if (finding == 1):
        index_1, index_2 = find_two_largest_indices(test_triange)
        filter_A = np.array([A[index_1], A[index_2]])
        filter_B = np.array([b_matrix[index_1], b_matrix[index_2]])
        res_solution, _, _, _ = np.linalg.lstsq(filter_A, filter_B, rcond=None)
        X = res_solution
        # else:
        #     X = one_stage_svd(A, b_matrix)
    else :
        X = one_stage_svd(anchor_positions, distances_to_anchors)

    return X

from sympy import symbols, expand
from sympy.parsing.sympy_parser import parse_expr
from collections import OrderedDict

def extract_coefficients(expression):
    z = symbols('z')
    expanded_expr = expand((expression))
    # print('expanded_expr', expanded_expr)
    coefficients = Poly(expanded_expr, z).all_coeffs()
    # print('coefficients', coefficients)

    return coefficients

def Cardano(a,b,c,d):
    
    complex_num = (-1+cmath.sqrt(3)*1j)/2
    complex_num_2 = complex_num**2
    z0=b/3/a
    a2,b2 = a*a,b*b
    p=-b2/3/a2 +c/a
    q=(b/27*(2*b2/a2-9*c/a)+d)/a
    D=-4*p*p*p-27*q*q
    delta = 18 * a * b * c * d - 4 * b**3 * d + b**2 * c**2 - 4 * a * c**3 - 27 * a**2 * d**2
    print('delta', delta)
    r=cmath.sqrt(-D/27+0j)
    u=((-q-r)/2)**0.33333333333333333333333
    v=((-q+r)/2)**0.33333333333333333333333

    
    z_candidate = [u+v-z0, u*complex_num + v *complex_num_2-z0, u*complex_num_2 + v*complex_num-z0]
    return z_candidate


def two_stage(distances_to_anchors, anchor_positions, u):
    tag_pos, det, b = lsq_method(distances_to_anchors, anchor_positions, u)
    
    z = symbols('z') #, real = True
    f = symbols('f', cls = Function)
    f = 0
    sum_delta, b_z, c_z, d_z = 0, 0, 0, 0
    for i in range(anchor_positions.shape[0]):
        delta = distances_to_anchors[i]**2 - ((tag_pos[0]- anchor_positions[i][0])**2 + (tag_pos[1]- anchor_positions[i][1])**2)
        f += 4 * ((z - anchor_positions[i][2]) ** 3 - delta*((z)-anchor_positions[i][2]))
    coeff = extract_coefficients(f)
    
    z_candidate = solve(f,z)
    # z_candidate = Cardano(coeff[0], coeff[1], coeff[2], coeff[3])
    # z_candidate = cardano_formula(coeff[0], coeff[1], coeff[2], coeff[3])
    # print('This is z candidate cardano ', z_candidate)

    z_candidate = np.array([complex(item) for item in z_candidate])
    # print('This is z candidate cardano', z_candidate)
    z_candidate = np.round(np.array([abs(z_candidate[0]), abs(z_candidate[1]), abs(z_candidate[2])]),5)

    result = list()
    check_ls = list()
    
        
    for i in range(z_candidate.shape[0]):
        check = abs(distances_to_anchors[0]**2 - (tag_pos[0] - anchor_positions[0][0])**2 - (tag_pos[1] - anchor_positions[0][1]) **2 - (z_candidate[i] - anchor_positions[0][2])**2)
        check_ls.append(check)
    index = check_ls.index(min(check_ls))
    # print('index', index)
    
    two_ans = np.array([tag_pos[0], tag_pos[1], z_candidate[index]])
    # print('two_ans', two_ans)
    result.append(two_ans)
    
    return np.array(result).astype(np.float32), det, b

def is_real_root(root, tolerance=1e-100):

    return abs(np.imag(root)) < tolerance


def cost_solve(distances_to_anchors, anchor_positions, u):
    distances_to_anchors, anchor_positions = np.array(distances_to_anchors), np.array(anchor_positions)
    
    tag_pos, det, b = lsq_method(distances_to_anchors, anchor_positions, u)
    
    anc_z_ls_mean = np.mean(np.array([i[2] for i in anchor_positions]) )  
    new_z = (np.array([i[2] for i in anchor_positions]) - anc_z_ls_mean).reshape(4, 1)
    new_anc_pos = np.concatenate((np.delete(anchor_positions, 2, axis = 1), new_z ), axis=1)
    new_disto_anc = np.sqrt(abs(distances_to_anchors[:]**2 - (tag_pos[0] - new_anc_pos[:,0])**2 - (tag_pos[1] - new_anc_pos[:,1])**2))
    new_z = new_z.reshape(4,)

    a = (np.sum(new_disto_anc[:]**2) - 3*np.sum(new_z[:]**2))/len(anchor_positions)
    b = (np.sum((new_disto_anc[:]**2) * (new_z[:])) - np.sum(new_z[:]**3))/len(anchor_positions)
    cost = lambda z: np.sum(((z - new_z[:])**4 - 2*(((new_disto_anc[:])*(z - new_z[:]))**2 ) + new_disto_anc[:]**4))/len(anchor_positions) 
    
    function = lambda z: z**3 - a*z + b
    ranges = (slice(0, 3, 0.01), )
    resbrute = optimize.brute(cost, ranges, full_output = True, finish = optimize.fmin)
    # print('resbrute: ', resbrute[0][0] + anc_z_ls_mean)
    new_tag_pos = np.array([tag_pos[0], tag_pos[1], abs(resbrute[0][0]) + anc_z_ls_mean])

    # new_tag_pos = np.array([tag_pos[0], tag_pos[1], newton_z + anc_z_ls_mean])

    return np.around(new_tag_pos, 2)

def opt_cost_solve(distances_to_anchors, anchor_positions, u):
    distances_to_anchors, anchor_positions = np.array(distances_to_anchors), np.array(anchor_positions)
    
    tag_pos, det, b = lsq_method(distances_to_anchors, anchor_positions, u)
    
    # anc_z_ls_mean = np.mean(np.array([i[2] for i in anchor_positions]) )  
    # new_z = (np.array([i[2] for i in anchor_positions]) - anc_z_ls_mean).reshape(4, 1)
    # new_anc_pos = np.concatenate((np.delete(anchor_positions, 2, axis = 1), new_z ), axis=1)
    # new_disto_anc = np.sqrt(abs(distances_to_anchors[:]**2 - (tag_pos[0] - new_anc_pos[:,0])**2 - (tag_pos[1] - new_anc_pos[:,1])**2))
    # new_z = new_z.reshape(4,)

    # a = (np.sum(new_disto_anc[:]**2) - 3*np.sum(new_z[:]**2))/len(anchor_positions)
    # b = (np.sum((new_disto_anc[:]**2) * (new_z[:])) - np.sum(new_z[:]**3))/len(anchor_positions)
    # cost = lambda z: np.sum(((z - new_z[:])**4 - 2*(((new_disto_anc[:])*(z - new_z[:]))**2 ) + new_disto_anc[:]**4))/len(anchor_positions) 
    new_disto_anc = (distances_to_anchors[:]**2 - (tag_pos[0] - anchor_positions[:,0])**2 - (tag_pos[1] - anchor_positions[:,1])**2)
    # print('new_disto_anc', new_disto_anc)
    # cost = lambda z: np.sum(((z - anchor_positions[:,2])**4 - 2*(((new_disto_anc[:])*(z - anchor_positions[:,2]))**2 ) + anchor_positions[:,2]**4)) 
    cost = lambda z: np.sum(((z - anchor_positions[:,2])**2 - new_disto_anc)**2)
    # 定义符号变量
    z = sympy.symbols('z')
    # 将 lambda 函数中的表达式转换为 SymPy 表达式
    cost_expression = cost(z)
    
    # 使用 SymPy 来展开表达式
    expanded_expression = sympy.expand(cost_expression)
    # print('expanded_expression', expanded_expression)
    
    start = -1
    end = 3
    number = end - start
    initial_guesses = np.linspace(start, end, num = end - start)  # 不同的初始猜测值
    
    # 存储局部最小值
    local_minima = collections.deque(maxlen = number)
    local_minima_z = collections.deque(maxlen = number)
    
    for initial_guess in initial_guesses:
        result = minimize(cost, initial_guess)
        optimal_params = result.x[0] 
        residue = abs(cost(optimal_params))
        
        local_minima.append(optimal_params)
        local_minima_z.append(residue)
    
    # print('local_minima',local_minima)  
    # print('local_minima_z',local_minima_z)  
    
    det_temp = min(local_minima_z)
    det_index = local_minima_z.index(det_temp)
    # print('det_temp', det_index)
    z_result = local_minima[det_index]
    
    result = minimize(cost, initial_guess)
    # 输出最优参数
    optimal_params = result.x 
    optimal_params = optimal_params[0]
    
    # print('optimal_params', optimal_params)
    # print('resbrute: ', resbrute[0][0] + anc_z_ls_mean)
    new_tag_pos = np.array([tag_pos[0], tag_pos[1], abs(z_result)])

    # new_tag_pos = np.array([tag_pos[0], tag_pos[1], newton_z + anc_z_ls_mean])

    return new_tag_pos, local_minima


def two_stage_solve(distances_to_anchors, anchor_positions, u):
    distances_to_anchors, anchor_positions = np.array(distances_to_anchors), np.array(anchor_positions)
    tag_pos, det, b = lsq_method(distances_to_anchors, anchor_positions, u)
    # tag_pos = choose_line(anchor_positions, distances_to_anchors).reshape(3)
    # tag_pos = one_stage_svd(anchor_positions, distances_to_anchors).reshape(3)

    # print('tag_pos_one stage', tag_pos)
    one_stage_z = tag_pos[2]
    # print('one_stage_z', one_stage_z)
    z = symbols('z') #, real = True
    f = symbols('f', cls = Function)
    f = 0
    sum_delta, b_z, c_z, d_z = 0, 0, 0, 0

    # print('truth', truth)
    delta_ls = []
    for i in range(anchor_positions.shape[0]):
        delta = distances_to_anchors[i]**2 - ((tag_pos[0]- anchor_positions[i][0])**2 + (tag_pos[1]- anchor_positions[i][1])**2)
        # delta = abs(delta)
        delta_ls.append(delta)
        # print('delta', delta)
        f += ((z - anchor_positions[i][2]) ** 3 - delta*((z)-anchor_positions[i][2]))
    

    z = symbols('z')
    expanded_expr = expand((f))
    coeff = extract_coefficients(f)
    
    # print('coeff', coeff)
    # z_candidate = solve(f,z)
    # z_candidate = solveset(f,z)
    # z_candidate = Cardano(coeff[0], coeff[1], coeff[2], coeff[3])
    z_candidate = np.roots(coeff)
    # print('z_candidate', z_candidate)
    a = coeff[0]
    b = coeff[1]
    c = coeff[2]
    d = coeff[3]
    # 计算判别式
    D0 = b**2 - 3*a*c
    D1 = 2*b**3 - 9*a*b*c + 27*a**2*d
    discriminant = D1**2 - 4*D0**3
    D = 18*a*b*c*d - 4*b**3*d + b**2*c**2 - 4*a*c**3 - 27*a**2*d**2
    discriminant = D
    # print('discriminant', discriminant)
    z_candidate = np.array([complex(item) for item in z_candidate])

    real_roots = collections.deque(maxlen = 3)
    imag_roots = collections.deque(maxlen = 2)
    tolerance = 1e-10
    
    if discriminant < 0:
        # 一實根
        # print('one real root')
        truth = 1

        if is_real_root(z_candidate[0], tolerance):
            real_roots.append(z_candidate[0])
        else:
            imag_roots.append(z_candidate[0])
        
        if is_real_root(z_candidate[1], tolerance):
            real_roots.append(z_candidate[1])
        else:
            imag_roots.append(z_candidate[1])
        
        if is_real_root(z_candidate[2], tolerance):
            real_roots.append(z_candidate[2])
        else:
            imag_roots.append(z_candidate[2])
        
        if len(real_roots) == 1:
            real_roots = [real_roots[0], real_roots[0], real_roots[0]]
            real_z_candidate = [real_roots[0], imag_roots[0], imag_roots[1]]
            abs_z_candidate = [real_roots[0], abs(imag_roots[0]), abs(imag_roots[1])]
            z_candidate = real_roots
    else:
        # 三個不同實根
        # print('three real root')
        truth = 0
        real_z_candidate = z_candidate
        z_candidate = z_candidate
        
    # print('This is z candidate solve', z_candidate)
    # z_candidate = np.round(np.a0'rray([abs(z_candidate[0]), abs(z_candidate[1]), abs(z_candidate[2])]),5)
    # print('This is z candidate', z_candidate_max)  
    z_candidate = np.array(z_candidate)
    

    local_minima = []
    new_disto_anc = (distances_to_anchors[:]**2 - (tag_pos[0] - anchor_positions[:,0])**2 - (tag_pos[1] - anchor_positions[:,1])**2)
    # print('new_disto_anc', new_disto_anc)
    # cost = lambda z: np.sum(((z - anchor_positions[:,2])**4 - 2*(((new_disto_anc[:])*(z - anchor_positions[:,2]))**2 ) + anchor_positions[:,2]**4)) 
    cost = lambda z: np.sum(((z - anchor_positions[:,2])**2 - new_disto_anc)**2)

    
    for z_candi in z_candidate:
        residue = cost(z_candi)
        local_minima.append(residue)
        
    det_temp = min(local_minima)
    det_index = local_minima.index(det_temp)
    # print('local_minima', local_minima)
    z_result = z_candidate[det_index]
    
    # result = list()
    check_ls = list()
    
    two_ans = np.array([tag_pos[0], tag_pos[1], z_candidate[0]])
    
    return np.array(two_ans).astype(np.float32),  z_candidate, local_minima

def two_stage_solve_trans(distances_to_anchors, anchor_positions, u):
    # 轉乘array
    distances_to_anchors, anchor_positions = np.array(distances_to_anchors), np.array(anchor_positions)
    # 透過在VH domain上，解出VHX的xy
    tag_pos = one_stage_svd(anchor_positions, distances_to_anchors).reshape(3)
    
    # 算出A Matrix
    anchor_offset = anchor_positions[0]
    A = anchor_positions[1:] - anchor_offset
    u, s, vh = np.linalg.svd(A, full_matrices=True)
    one_stage_z = tag_pos[2]

    # 二階段
    z = symbols('z') #, real = True
    f = symbols('f', cls = Function)
    f = 0
    sum_delta, b_z, c_z, d_z = 0, 0, 0, 0
    
    # VHA，把A轉去V domain
    anchor_positions = anchor_positions @vh.T

    # 二階段的cost function
    delta_ls = []
    for i in range(anchor_positions.shape[0]):
        delta = distances_to_anchors[i]**2 - ((tag_pos[0]- anchor_positions[i][0])**2 + (tag_pos[1]- anchor_positions[i][1])**2)
        delta_ls.append(delta)
        f += ((z - anchor_positions[i][2]) ** 3 - delta*((z)-anchor_positions[i][2]))
    

    z = symbols('z')
    expanded_expr = expand((f))
    coeff = extract_coefficients(f)
    
    # 用卡丹公式解
    z_candidate = np.roots(coeff)
    # print('z_candidate', z_candidate)
    # a = coeff[0]
    # b = coeff[1]
    # c = coeff[2]
    # d = coeff[3]
    
    # # 卡丹判別式
    # D0 = b**2 - 3*a*c
    # D1 = 2*b**3 - 9*a*b*c + 27*a**2*d
    # discriminant = D1**2 - 4*D0**3
    # D = 18*a*b*c*d - 4*b**3*d + b**2*c**2 - 4*a*c**3 - 27*a**2*d**2
    # discriminant = D
    z_candidate = np.array([complex(item) for item in z_candidate])

    # real_roots = collections.deque(maxlen = 3)
    # imag_roots = collections.deque(maxlen = 2)
    # tolerance = 1e-10
    
    # 透過discriminant判斷大於或小於0
    # if discriminant < 0:
    #     # 一實根
    #     # print('one real root')
    #     truth = 1

    #     if is_real_root(z_candidate[0], tolerance):
    #         real_roots.append(z_candidate[0])
    #     else:
    #         imag_roots.append(z_candidate[0])
        
    #     if is_real_root(z_candidate[1], tolerance):
    #         real_roots.append(z_candidate[1])
    #     else:
    #         imag_roots.append(z_candidate[1])
        
    #     if is_real_root(z_candidate[2], tolerance):
    #         real_roots.append(z_candidate[2])
    #     else:
    #         imag_roots.append(z_candidate[2])
        
    #     if len(real_roots) == 1:
    #         real_roots = [real_roots[0], real_roots[0], real_roots[0]]
    #         real_z_candidate = [real_roots[0], imag_roots[0], imag_roots[1]]
    #         abs_z_candidate = [real_roots[0], abs(imag_roots[0]), abs(imag_roots[1])]
    #         z_candidate = real_roots
    # else:
    #     # 三個不同實根
    #     # print('three real root')
    #     truth = 0
    #     real_z_candidate = z_candidate
    #     z_candidate = z_candidate
    
    z_candidate = np.array(z_candidate)
    local_minima = []
    new_disto_anc = (distances_to_anchors[:]**2 - (tag_pos[0] - anchor_positions[:,0])**2 - (tag_pos[1] - anchor_positions[:,1])**2)
    cost = lambda z: np.sum(((z - anchor_positions[:,2])**2 - new_disto_anc)**2)

    for z_candi in z_candidate:
        residue = cost(z_candi)
        local_minima.append(residue)
        
    det_temp = min(local_minima)
    det_index = local_minima.index(det_temp)
    z_result = z_candidate[det_index]
    check_ls = list()
    
    # VHX(x',y',z')
    two_ans = np.array([tag_pos[0], tag_pos[1], z_candidate[0]])
    # 再把VH Domain 轉回歐幾里得空間
    two_ans = vh.T@two_ans
    
    return np.array(two_ans).astype(np.float32)

from scipy.optimize import differential_evolution
def two_stage_solve_cost(distances_to_anchors, anchor_positions, u):
    distances_to_anchors, anchor_positions = np.array(distances_to_anchors), np.array(anchor_positions)
    
    tag_pos, check_ls, z_candidate, truth, expanded_expr, delta_ls, local_minima, discriminant, real_z_candidate, one_z = two_stage_solve(distances_to_anchors, anchor_positions, u)
    
    # print('z_candidate', z_candidate)
    # anc_z_ls_mean = np.mean(np.array([i[2] for i in anchor_positions]) )  
    # new_z = (np.array([i[2] for i in anchor_positions]) - anc_z_ls_mean).reshape(4, 1)
    # new_anc_pos = np.concatenate((np.delete(anchor_positions, 2, axis = 1), new_z ), axis=1)
    # new_disto_anc = np.sqrt(abs(distances_to_anchors[:]**2 - (tag_pos[0] - new_anc_pos[:,0])**2 - (tag_pos[1] - new_anc_pos[:,1])**2))
    # new_z = new_z.reshape(4,)

    # a = (np.sum(new_disto_anc[:]**2) - 3*np.sum(new_z[:]**2))/len(anchor_positions)
    # b = (np.sum((new_disto_anc[:]**2) * (new_z[:])) - np.sum(new_z[:]**3))/len(anchor_positions)
    # cost = lambda z: np.sum(((z - new_z[:])**4 - 2*(((new_disto_anc[:])*(z - new_z[:]))**2 ) + new_disto_anc[:]**4))/len(anchor_positions) 
    # cost = lambda z: np.sum(((z - new_z[:])**2 - (new_disto_anc[:])**2)**2)/len(anchor_positions) 
    
    new_disto_anc = (distances_to_anchors[:]**2 - (tag_pos[0] - anchor_positions[:,0])**2 - (tag_pos[1] - anchor_positions[:,1])**2)
    # print('new_disto_anc', new_disto_anc)
    # cost = lambda z: np.sum(((z - anchor_positions[:,2])**4 - 2*(((new_disto_anc[:])*(z - anchor_positions[:,2]))**2 ) + anchor_positions[:,2]**4)) 
    cost = lambda z: np.sum(((z - anchor_positions[:,2])**2 - new_disto_anc)**2)
    # function = lambda z: z**3 - a*z + b
    ranges = (slice(1.5, 2.5, 0.01), )
    resbrute = optimize.brute(cost, ranges, full_output = True, finish = optimize.fmin)
    bounds = [(1.5, 2)]
    # 使用 differential_evolution 进行全局优化搜索
    result_global = differential_evolution(cost, bounds)
    # 获取全局最小值的位置
    global_minimizer = result_global.x
    global_minimum = result_global.fun
    
    # 使用 minimize 进行局部优化（可选）
    result_local = minimize(cost, global_minimizer, method='BFGS')
    local_minimizer = result_local.x
    local_minimum = result_local.fun
    
    # print("全局最小值位置:", global_minimizer)
    # print("全局最小值:", global_minimum)
    # print("局部最小值位置:", local_minimizer)
    # print("局部最小值:", local_minimum)

    # print('resbrute: ', resbrute[0][0] + anc_z_ls_mean)
    # new_tag_pos = np.array([tag_pos[0], tag_pos[1], abs(resbrute[0][0]) + anc_z_ls_mean])
    new_tag_pos = np.array([tag_pos[0], tag_pos[1], abs(resbrute[0][0])])
    # print('value', resbrute[0])
    # print('Cost func value', resbrute[1])

    # new_tag_pos = np.array([tag_pos[0], tag_pos[1], newton_z + anc_z_ls_mean])

    return np.around(new_tag_pos, 2), global_minimizer, global_minimum, local_minimizer, local_minimum
