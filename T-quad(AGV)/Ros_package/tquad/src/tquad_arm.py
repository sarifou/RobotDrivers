#!/usr/bin/env python

# Import des bibiliothèques
#import rospy
from math import *

# Les dimensions globales du bras robotique
dim = {"d_1" : 75, "d_2" : 83.7, "l_0" : 5, "h_1" : 64.5, "l_1" : 15.1, "l_2" : 80, "l_3" : 80, "l_30": 35, "l_31":23.9, "l_4" : 80, "l_5":52, "d_5" : 5}


def inverse_kinematic(x,y,z):
    """
    Cinématique inverse du bras robotique : coordonnées articulaires 
    en fonction des coordonnées de l'effecteur.
    """

    # Calcul de l'angle q_1
    A = (x-dim["l_0"])/(-y)
    B_ = sqrt((x-dim["l_0"])**2 + y**2)
    B = dim["d_5"]/B_
    q_1 = degrees(atan(A)+asin(B))

    # Coordonnées de p_w
    p_wx = x + dim["l_1"]*sin(q_1) - dim["l_0"]
    p_wy = y - dim[""]*cos(q_1)
    p_wz = z

    q_2 = 0 
    q_3 =0
    return q_1, q_2, q_3

def forward_kinematic():
    """
    Cinématique direct du bras robotique : coordonnées de l'effecteur
    en fonction des coordonnées articulaires
    """

def publisher():
    """Publisher"""

if __name__ == '__main__':
    q_1, q_2, q_3 = inverse_kinematic(30,10,80)
    print("Angle Q1: " +str(q_1))
    print("Angle Q2: " +str(q_2))
    print("Angle Q3: " +str(q_3))
    print(dim["d_1"])