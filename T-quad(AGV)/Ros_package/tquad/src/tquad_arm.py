#!/usr/bin/env python

# Import des bibiliothèques
#import rospy
from math import *

# Les dimensions globales du bras robotique
dim = {"d_1" : 75, "d_2" : 83.7, "l_0" : 5, "h_1" : 64.5, "l_1" : 15.1, "l_2" : 80, "l_3" : 80, "l_30": 35, "l_31":23.9, "l_4" : 80, "l_5":52, "d_5" : 5}


def inverse_kinematic(target, params):
    """
    Cinématique inverse du bras robotique : coordonnées articulaires 
    en fonction des coordonnées de l'effecteur.
    target : dictionnaire contenant les coordonnées cibles
    params : dictionnaire contenant les dimensions globales du bras robotique
    """
    _target = target
    _params = params
    angle = {}
    pw = []
    # Calcul de l'angle q_1
    tan = atan2((_target["x"]-_params["l_0"]) , -_target["y"])
    den_sin = sqrt((_target["x"]-_params["l_0"])**2 + _target["y"]**2)
    sin_ = _params["d_5"]/ den_sin
    q_1 = tan +asin(sin_)
    angle["q_1"] = degrees(q_1) + 90

    # Coordonnées de p_w
    pw.append(_target["x"] + _params["l_5"]*sin(q_1) - _params["l_0"]) 
    pw.append(_target["y"] - _params["l_5"]*cos(q_1))
    pw.append(_target["z"])
    #print(pw)

    # Calcul de l'angle q_2
    r = sqrt(pw[0]**2 + pw[1]**2) - _params["l_1"]
    print(r)
    ze = pw[2] - _params["h_1"]
    print(ze)
    alpha = atan(ze/r)
    s = sqrt(r**2 + ze**2)
    gamma = acos((_params["l_2"]**2 + _params["l_3"]**2 - s**2) / (2*_params["l_2"]*_params["l_3"]) )
    beta = acos((s**2 + _params["l_2"]**2 - _params["l_3"]**2) / (2*s*_params["l_3"]))
    q_2 = pi - alpha -beta
    angle["q_2"] = degrees(q_2)
    q_30 = pi - gamma
   
    # Calcul de l'angle q_3
    e = sqrt((_params["l_30"]**2 + _params["l_2"]**2)-(2*_params["l_30"]*_params["l_2"]*cos(q_30)))
    phi = acos((e**2 + _params["l_31"]**2 - _params["l_4"]**2 ) / (2*e*_params["l_31"]))
    psi = asin((_params["l_30"]*sin(q_30)) / e)
    q_3 = psi + phi + (pi/2) - q_2
    angle["q_3"] = degrees(q_3)
    return angle

def forward_kinematic(angles, params):
    """
    Cinématique direct du bras robotique : coordonnées de l'effecteur
    en fonction des coordonnées articulaires.
    angle : dictionnaire contenant les trois des articulations.
    params : dictionnaire contenant les dimensions globales du bras robotique.
    """
    _angles = angles
    _params = params
    _angles["q_1"] = angles["q_1"] - 90
    # Conversion des angles en radian
    _angles["q_1"] = radians(_angles["q_1"])
    _angles["q_2"] = radians(_angles["q_2"])
    _angles["q_2"] = radians(_angles["q_2"])
    print(_angles["q_1"])
    target = {}
    phi = radians(_angles["q_2"] + _angles["q_3"] - 45)
    f = sqrt((_params["l_2"]**2 + _params["l_31"]**2) - (2*_params["l_31"]*_params["l_2"]*cos(phi)) )
    tau = asin((_params["l_31"]*sin(phi)) / f)
    mu = acos((f**2 + _params["l_30"]**2 - _params["l_4"]**2) / (2*f*_params["l_30"]))
    q_30 = tau + mu
    r = -_params["l_2"]*cos(_angles["q_2"]) - _params["l_3"]*cos(_angles["q_2"] + q_30)
    print(r)
    # Calcul de la coordonnée x
    px = _params["l_0"] + (r + _params["l_1"] + _params["l_5"])*sin(_angles["q_1"]) - _params["d_5"]*cos(_angles["q_1"])
    target["p_tx"] = px
    # Calcul de la coordonnée y
    py = -(r + _params["l_1"] + _params["l_5"])*cos(_angles["q_1"]) - _params["d_5"]*sin(_angles["q_1"])
    target["p_ty"] = py
    # Calcul de la coordonnée z
    pz = _params["h_1"] + _params["l_2"]*sin(_angles["q_2"]) + _params["l_3"]*sin(_angles["q_2"] + q_30)
    target["p_tz"] = pz
    return target

def publisher():
    """Publisher"""

if __name__ == '__main__':
    target = {"x" :14.6, "y" : -5.0, "z" : 142.8}
    config = {"q_1" : 180 , "q_2" : 65, "q_3" : 110}
    angles = inverse_kinematic(target, dim)
    print("Inverse kinematic angles  " + str(angles))
    position = forward_kinematic(config, dim)
    print("Forward kinematic position  " + str(position))
    