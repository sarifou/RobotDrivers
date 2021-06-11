#!/usr/bin/env python

# Import des bibiliothèques
#import rospy
from math import *

# Les dimensions globales du bras robotique
dim = {"d_1" : 75, "d_2" : 83.7, "l_0" : 5, "h_1" : 64.5, "l_1" : 15.1, "l_2" : 80, "l_3" : 80, "l_30": 35, "l_31":23.9, "l_4" : 80, "l_5":52, "d_5" : 5}

def acos_safe(alpha):
    """
    Calcul du cosinus d'un angle alpha en radian
    """
    _alpha = alpha
    if ((_alpha>=-1) & (_alpha<=1)):
        _alpha = acos(_alpha)
    elif (_alpha>1):
        _alpha = 0
    elif (_alpha<-1):
        _alpha = pi

    return _alpha

def asin_safe(alpha):
    """
    Cacul du sinus d'un angle alpha en radian
    """
    _alpha = alpha
    if ((_alpha>=-1) & (_alpha<=1)):
        _alpha = asin(_alpha)
    elif (_alpha>1):
        _alpha = pi/2
    elif (_alpha<-1):
        _alpha = -(pi/2)

    return _alpha

def inverse_kinematic(target, params):
    """
    Cinématique inverse du bras robotique : coordonnées articulaires 
    en fonction des coordonnées de l'effecteur.
    target : dictionnaire contenant les coordonnées cibles. Les coordonnées sont en mm.
    params : dictionnaire contenant les dimensions globales du bras robotique
    """
    _target = target
    _params = params
    angle = {}
    pw = []
    # Calcul de l'angle q_1
    tan_ = atan2((_target["x"]-_params["l_0"]) , -_target["y"])
    den_sin = sqrt((_target["x"]-_params["l_0"])**2 + _target["y"]**2)
    sin_ = _params["d_5"]/ den_sin
    q_1 = tan_ + asin_safe(sin_)
    angle["q_1"] = degrees(q_1) # correction de l'angle par rapport à la version du robot

    # Coordonnées de p_w
    pw.append(_target["x"] + _params["l_5"]*sin(q_1) - _params["l_0"]) 
    pw.append(_target["y"] - _params["l_5"]*cos(q_1))
    pw.append(_target["z"])
    #print(pw)

    # Calcul de l'angle q_2
    r = sqrt(pw[0]**2 + pw[1]**2) - _params["l_1"]
    ze = pw[2] - _params["h_1"]
    alpha = atan(ze/r)
    s = sqrt(r**2 + ze**2)
    gamma = acos_safe((_params["l_2"]**2 + _params["l_3"]**2 - s**2) / (2*_params["l_2"]*_params["l_3"]) )
    beta = acos_safe((s**2 + _params["l_2"]**2 - _params["l_3"]**2) / (2*s*_params["l_3"]))
    q_2 = pi - alpha -beta
    angle["q_2"] = degrees(q_2)
    q_30 = pi - gamma
   
    # Calcul de l'angle q_3
    e = sqrt((_params["l_30"]**2 + _params["l_2"]**2)-(2*_params["l_30"]*_params["l_2"]*cos(q_30)))
    phi = acos_safe((e**2 + _params["l_31"]**2 - _params["l_4"]**2 ) / (2*e*_params["l_31"]))
    psi = asin_safe((_params["l_30"]*sin(q_30)) / e)
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
    _angles["q_1"] = _angles["q_1"] # Correction de l'angle par rapport à la version du robot
    # Conversion des angles en radian
    _angles["q_1"] = radians(_angles["q_1"])
    _angles["q_2"] = radians(_angles["q_2"])
    _angles["q_3"] = radians(_angles["q_3"])
    target = {}
    phi = radians(_angles["q_2"] + _angles["q_3"] - 45)
    f = sqrt((_params["l_2"]**2 + _params["l_31"]**2) - (2*_params["l_31"]*_params["l_2"]*cos(phi)) )
    tau = asin_safe((_params["l_31"]*sin(phi)) / f)
    mu = acos_safe((f**2 + _params["l_30"]**2 - _params["l_4"]**2) / (2*f*_params["l_30"]))
    q_30 = tau + mu
    r = -_params["l_2"]*cos(_angles["q_2"]) - _params["l_3"]*cos(_angles["q_2"] + q_30)
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

def moveAbs():
    """
    """

def moveJ():
    """
    """

def moveL():
    """
    """


def publisher():
    """Publisher"""

if __name__ == '__main__':
    print(acos_safe(-1.4))
    target = {"x" : 152.09125732896138, "y" :-5.000000000000009, "z" : 145.68268801122028}
    config = {"q_1" : 90 , "q_2" : 90, "q_3" : 90}
    angles = inverse_kinematic(target, dim)
    print("Inverse kinematic angles  " + str(angles))
    position = forward_kinematic(config, dim)
    print("Forward kinematic position  " + str(position))
    