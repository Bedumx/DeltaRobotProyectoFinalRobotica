# -*- coding: utf-8 -*-
"""
Created on Sun May 16 18:44:42 2021

@author: brand
"""

import sympy as sp
from IPython.display import display
from sympy import init_printing
init_printing()

def local_trans(q,d,alpha,a):

    ct = sp.cos(q)
    st = sp.sin(q)
    ca = sp.cos(alpha)
    sa = sp.sin(alpha)

    # Denavit-Hartenberg estándar
    G = sp.Matrix([
                    [ct, -st*ca, st*sa, a*ct],
                    [st, ct*ca, -ct*sa, a*st],
                    [0, sa, ca, d],
                    [0, 0, 0, 1]])
    return G

def skew_sym(vector):
    w1,w2,w3 = vector
    W = sp.Matrix([
                    [ 0, -w3, w2],
                    [w3, 0, -w1],
                    [-w2, w1, 0]])
    return W


def geo_jacobian(G):
    
    dof = len(G)
    #end-effector position
    P_f = G[dof-1][0:3,3]
    
    Jg = sp.zeros(6,dof-1)
    i = 0
    for G_i in G[0:dof-1]:
        P = G_i[0:3,3];
        S = G_i[0:3,2];
        S_CPO = skew_sym(S);
        D = P_f - P;
        Jg[0:6,i] = sp.Matrix([S_CPO@D, S])
        i += 1
    
    Jg = sp.trigsimp(Jg)
    return Jg


SB, SP, L, l, h, WB, UB, WP, UP = sp.symbols("SB, SP, L, l, h, WB, UB, WP, UP")

#DISTANCIA DE CENTRO DE BASE A LADO
B_B1 = sp.Matrix([0,-WB,0])
B_B2 = sp.Matrix([(sp.sqrt(3)/2)*WB,(1/2)*WB,0])
B_B3 = sp.Matrix([-(sp.sqrt(3)/2)*WB,(1/2)*WB,0])

#DISTANCIA DE CENTRO DE PLATAFORMA A VERTICE
P_P1 = sp.Matrix([0,-UP,0])
P_P2 = sp.Matrix([SP/2,WP,0])
P_P3 = sp.Matrix([-SP/2,WP,0])

#DISTANCIA DE CENTRO DE BASE A VERTICE 
B_b1 = sp.Matrix([SB/2,-WB,0])
B_b2 = sp.Matrix([0,UB,0])
B_b3 = sp.Matrix([-SB/2,-WB,0])


dof = 6
q1, q2, q3= sp.symbols("q1, q2, q3")
#Vector de biceps
B_L1 = sp.Matrix([0,-L*sp.cos(q1),-L*sp.sin(q1)])
B_L2 = sp.Matrix([(sp.sqrt(3)/2)*L*sp.cos(q2),(1/2)*L*sp.cos(q2),-L*sp.sin(q2)])
B_L3 = sp.Matrix([(-sp.sqrt(3)/2)*L*sp.cos(q3),(1/2)*L*sp.cos(q3),-L*sp.sin(q3)])

a = WB - UP
b = (SP/2) - (sp.sqrt(3)/2)*WB
c = WP - (1/2)*WB    
x, y, z= sp.symbols("x, y, z")

B_P = sp.Matrix([x,y,z])

#de ecuacion li = [B_P+P_Pi-B_Bi-B_Li]

B_l1 = B_P+P_P1-B_B1-B_L1
B_l2 = B_P+P_P2-B_B2-B_L2
B_l3 = B_P+P_P3-B_B3-B_L3

#constrain eq.
c1 = sp.expand(2*L*(y+a)*sp.cos(q1)+2*z*L*sp.sin(q1)+x**2+y**2+z**2+a**2+L**2+2*y*a-l**2)
c11 = sp.trigsimp(B_l1[0]**2+sp.expand(B_l1[1]**2)+sp.expand(B_l1[2]**2)-l**2)
c2 = sp.trigsimp(B_l2[0]**2+sp.expand(B_l2[1]**2)+sp.expand(B_l2[2]**2)-l**2)
c3 = sp.trigsimp(B_l3[0]**2+sp.expand(B_l3[1]**2)+sp.expand(B_l3[2]**2)-l**2)

#absolute vector knee eq

B_A1=B_B1+B_L1
B_A2=B_B2+B_L2
B_A3=B_B3+B_L3



# dof = 6
# q1, q2, q3, q4, q5, q6 = sp.symbols("q1, q2, q3, q4, q5, q6")
# d1, d4, d5, d6 = sp.symbols("d1, d4, d5, d6")
# a2, a3 = sp.symbols("a2, a3")

# q = [q1, q2, q3, q4, q5, q6]
# d = [d1, 0, 0, d4, d5, d6]
# alpha  = [sp.pi/2,0,0,sp.pi/2,-sp.pi/2,0]
# a = [0, a2, a3, 0, 0, 0]

# kin_params = {'q' : q, 'd' : d, 'alpha' : alpha, 'a' : a}

# G_local = []
# for i in range(dof):
#     G_local.append(local_trans(kin_params['q'][i],
#                             kin_params['d'][i],
#                             kin_params['alpha'][i],
#                             kin_params['a'][i]))
    
# G = sp.eye(4)
# G_d = []
# G_d.append(G)
# for G_i in G_local:
#     G = G @ G_i
#     G_d.append(sp.trigsimp(G))